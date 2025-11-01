#![no_std]
#![no_main]
#![allow(non_snake_case)]

/*
    ToDo:
        - write measurement data to EEPROM
        - add function for user button and switch.
            - button (PB3)  -> briefly display Soc on the status LEDs?
            - switch (PA15) -> enable/disable data acq, put device to standby
        - SD card
        - SSR control (PB2)

        - HV PSU frequency, duty_cycle is placeholder
        - COMP1 should use interrupts instead of pooling.
          But it seems that comparators are not supported in Embassy, so have to do it "raw"
*/

use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Blocking;
use embassy_stm32::rtc::{DateTime, DayOfWeek, Rtc, RtcConfig};
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm;
use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

mod mc_24cs256;
mod sht40;
mod stc3315;
mod usb_connection;

type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, Blocking, i2c::Master>>;
type RtcShared = Mutex<NoopRawMutex, Rtc>;
type OutputShared<'a> = Mutex<NoopRawMutex, Output<'a>>;

static PULSE_COUNTER: AtomicU16 = AtomicU16::new(0u16);
static HV_PSU_ENABLE: AtomicBool = AtomicBool::new(false);

bind_interrupts!(struct Irqs {
    USB_DRD_FS => usb::InterruptHandler<peripherals::USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting up");

    let mut system_config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        system_config.rcc.hsi = true;
        system_config.rcc.ls = LsConfig::default_lse();
        system_config.rcc.pll = Some(Pll {
            source: PllSource::HSI, // 16 MHz
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL7,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2), // 56 MHz
        });
        system_config.rcc.sys = Sysclk::PLL1_R;
        system_config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        system_config.rcc.mux.clk48sel = mux::Clk48sel::HSI48; // USB uses ICLK
    }

    let p = embassy_stm32::init(system_config);

    // Shared I2C bus for tasks
    // Default is 100kHz, medium speed GPIO, no internal pull-ups
    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c::Config::default());
    static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    /*
        HV PSU:
            - comp1:
                + PA1
                - Vref
                0x4001_0200 - 0x4001_03FF COMP reg address
                COMP1_CSR: Comparator 1 control and status register, Address offset: 0x00
                     0000_0000_0000_0101 - medium speed, low hyst(??)
                     0000_0[010]_[0001]_0001 - INPSEL - INMSEL (Vref /2 )
            - comp2:
                + PA3
                - Vref
                COMP2_CSR: Same as COMP1_CSR, Address offset: 0x04
            - pwm_out: PB11
    */

    // There is no support for comparators in Embassy yet(??), so we do this the hard way
    unsafe {
        const COMP1_CSR: *mut u32 = 0x4001_0200 as *mut u32;
        //const COMP2_CSR: *mut u32 = (0x4001_0200 + 0x04) as *mut u32;
        core::ptr::write_volatile(COMP1_CSR, 0b0000_0000_0000_0101_0000_0010_0001_0001);
    }

    let tim2_ch4 = PwmPin::new(p.PB11, embassy_stm32::gpio::OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM2,
        None,
        None,
        None,
        Some(tim2_ch4),
        khz(10),
        Default::default(),
    );
    spawner.spawn(comparator_1(pwm)).unwrap();

    /*
        pulse_in (PC13) --> pulse_counter
    */

    let pulse_in = ExtiInput::new(p.PC13, p.EXTI13, Pull::None);
    spawner.spawn(pulse_detection(pulse_in)).unwrap();

    /*
        SSR control (PB2)
    */

    let ssr = Output::new(p.PB2, Level::Low, Speed::Low);
    static SSR_CTRL: StaticCell<OutputShared> = StaticCell::new();
    let ssr_ctrl = SSR_CTRL.init(Mutex::new(ssr));



    /*
        RTC
    */

    let mut rtc = Rtc::new(p.RTC, RtcConfig::default());
    let default_dt = DateTime::from(2025, 10, 25, DayOfWeek::Saturday, 12, 40, 00, 00).unwrap();
    rtc.set_datetime(default_dt).unwrap();
    static RTC_SHARED: StaticCell<RtcShared> = StaticCell::new();
    let rtc_shared = RTC_SHARED.init(Mutex::new(rtc));

    spawner.spawn(rtc_alarm(rtc_shared, i2c_bus)).unwrap();

    /*
        STC3315 + Status LEDs
    */

    // STC3315 initialization + start seq, sht20 soft reset
    {
        let mut i2c_locked = i2c_bus.lock().await;
        if let Err(e) = sht40::reset(&mut i2c_locked) {
            error!("SHT40 reset: {}", e);
        }

        let _ = stc3315::init(&mut i2c_locked);
        Timer::after_micros(100).await;
        let _ = stc3315::start(&mut i2c_locked);
        Timer::after_micros(100).await;
    }

    // 5 status LEDs for battery SoC display
    let status_LEDs = [
        Output::new(p.PA5, Level::Low, Speed::Low),
        Output::new(p.PA6, Level::Low, Speed::Low),
        Output::new(p.PA7, Level::Low, Speed::Low),
        Output::new(p.PB0, Level::Low, Speed::Low),
        Output::new(p.PB1, Level::Low, Speed::Low),
    ];

    // Is pulled LOW while charging the battery
    let charging_detection = Input::new(p.PB4, Pull::None);
    spawner
        .spawn(status_LED_control(status_LEDs, charging_detection, i2c_bus))
        .unwrap();

    /*
        USB initialization
    */

    // Create the driver, from the HAL.
    let driver = usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11);

    // Create embassy-usb Config
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    //usb_config.device_class = 0xFF; // Vendor Specific
    //usb_config.composite_with_iads = false;
    usb_config.max_packet_size_0 = 64;
    usb_config.manufacturer = Some("ShaOS");
    usb_config.product = Some("Radon Station");
    usb_config.max_power = 500;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = cdc_acm::State::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = cdc_acm::CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_future = usb.run();

    // Hnadling connections
    let class_future = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = usb_connection::handle_usb_connection(&mut class, i2c_bus, rtc_shared, ssr_ctrl).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    join(usb_future, class_future).await;

    // Will never reach here
}

#[embassy_executor::task]
async fn status_LED_control(
    mut status_LEDs: [Output<'static>; 5],
    charging_detection: Input<'static>,
    i2c_bus: &'static I2c1Bus,
) {
    loop {
        if charging_detection.is_low() {
            // Charging from USB
            // Check SoC
            if let Ok(soc) = stc3315::read_SOC(&mut *i2c_bus.lock().await) {
                //info!("Read: {}%", soc);
                let charging = charging_detection.is_low();

                // 5 LEDs
                // 15-34%, 35-54%, 55-74%, 75-94%, 94-100%
                let charge_level = ((soc + 5) / 20).min(5) as usize; // 0 to 5;

                for (i, led) in status_LEDs.iter_mut().enumerate() {
                    match i {
                        x if x < charge_level => {
                            led.set_high();
                        }
                        x if x == charge_level && charging => {
                            led.toggle();
                        }
                        _ => {
                            led.set_low();
                        }
                    }
                }
            } else {
                info!("Read failed!")
            }
            Timer::after_secs(1).await;
        } else {
            // Not charging
            for LED in &mut status_LEDs {
                LED.set_low();
            }

            Timer::after_secs(10).await;
        }
    }
}

#[embassy_executor::task]
async fn pulse_detection(mut pulse_in: ExtiInput<'static>) {
    loop {
        pulse_in.wait_for_rising_edge().await;
        let current_value = PULSE_COUNTER.load(Ordering::Relaxed);
        PULSE_COUNTER.store(current_value.wrapping_add(1), Ordering::Relaxed);
    }
}

// bit number 30 is the comparator output, 1 - high, 0 - low
fn read_comp_status(register: u32) -> bool {
    register & (1 << 30) != 0
}
/*
    Comparator status checking
    currently it uses pooling to achive this,
    but probably want to use interrupts.

    HV_PSU_ENABLE enables/disables the PSU regardles of the comp values.
    While developping, it starts of, and manually has to be turned on.
*/
#[embassy_executor::task]
async fn comparator_1(mut pwm: SimplePwm<'static, peripherals::TIM2>) {
    const COMP1_CSR: *mut u32 = 0x4001_0200 as *mut u32;
    let mut comp1_status_old = unsafe { read_comp_status(core::ptr::read_volatile(COMP1_CSR)) };

    let mut active_ch = pwm.ch4();
    active_ch.set_duty_cycle_percent(10); // placeholder

    let mut psu_enable_buf = HV_PSU_ENABLE.load(Ordering::Relaxed);

    loop {
        if HV_PSU_ENABLE.load(Ordering::Relaxed) {
            let comp1_status_new = unsafe { read_comp_status(core::ptr::read_volatile(COMP1_CSR)) };

            if comp1_status_new != comp1_status_old {
                comp1_status_old = comp1_status_new;

                if comp1_status_new {
                    active_ch.disable();
                } else {
                    active_ch.enable();
                }
            }

            if !psu_enable_buf {
                psu_enable_buf = true;
                info!("HV PSU: enabled by user");
                if !comp1_status_new {
                    active_ch.enable();
                }
            }

            Timer::after_micros(100).await;
        } else {
            if psu_enable_buf {
                psu_enable_buf = false;
                info!("HV PSU: disabled by user");
                active_ch.disable();
            }

            Timer::after_secs(1).await;
        }
    }
}

// RTC "alarm", uses pooling as proper alarms are not supported (?) yet
// Set to trigger every min while testing. Final product should trigger every hour
#[embassy_executor::task]
async fn rtc_alarm(rtc: &'static RtcShared, i2c_bus: &'static I2c1Bus) {
    let mut old = rtc.lock().await.now().unwrap();
    let mut now;
    loop {
        now = rtc.lock().await.now().unwrap();
        if now.minute() != old.minute() {
            old = now;

            info!("Pulse counter: {}", PULSE_COUNTER.load(Ordering::Relaxed));

            {
                let mut i2c = i2c_bus.lock().await;
                match stc3315::read_SOC(&mut i2c) {
                    Ok(soc) => info!("STC3315 - SoC: {}", soc),
                    Err(e) => error!("STC3315 error: {}", e),
                }

                match sht40::read(&mut i2c).await {
                    Ok(temp_and_rh) => info!(
                        "SHT40: {}, {}",
                        temp_and_rh.temperature, temp_and_rh.relative_humidity
                    ),
                    Err(e) => error!("SHT40 error: {}", e),
                }

                let r = match mc_24cs256::read_byte_random(&mut i2c, 0u16).await {
                    Ok(r) => {
                        info!("24CS256 read: {}", r);
                        r
                    }
                    Err(e) => {
                        error!("24CS256 read error: {}", e);
                        0
                    }
                };

                match mc_24cs256::write_byte(&mut i2c, 0u16, r.wrapping_add(1)).await {
                    Ok(_) => {}
                    Err(e) => error!("24CS256 write error: {}", e),
                }
            }
        }
        Timer::after_secs(10).await;
    }
}

