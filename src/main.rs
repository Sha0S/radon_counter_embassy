#![no_std]
#![no_main]
#![allow(non_snake_case)]

/*
    ToDo:
        - Use the RTC for timestamp while writing to the SD card
        - Implementing timeouts for the EEPROM operaions
        - LEDs: use them to display error codes?
        - SHT40 CRC check
*/

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering};

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::select::select3;
use embassy_stm32::crc::Crc;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::rtc::{Rtc, RtcConfig, RtcTimeProvider};
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::{Peri, bind_interrupts, interrupt, peripherals, usb};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm;

use embedded_sdmmc::{SdCard, VolumeIdx, VolumeManager};
use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

mod comparator;
mod mc_24cs256;
mod sht40;
mod stc3315;
mod usb_connection;

type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, Blocking, i2c::Master>>;
type RtcShared = Mutex<NoopRawMutex, Rtc>;
type OutputShared<'a> = Mutex<NoopRawMutex, Output<'a>>;
type ExtiInputShared<'a> = Mutex<NoopRawMutex, ExtiInput<'a>>;

static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<Blocking, Master>>>> = StaticCell::new();

static PULSE_COUNTER: AtomicU16 = AtomicU16::new(0u16);
static PULSE_COUNTER_TOTAL: AtomicU16 = AtomicU16::new(0u16);
static HV_PSU_ENABLE: AtomicBool = AtomicBool::new(false);
static USER_BUTTON: AtomicBool = AtomicBool::new(false);

// Tracking errors during runtime
const I2C_ERROR: u8 = 0u8;
const SD_ERROR: u8 = 1u8;
const USB_ERROR: u8 = 2u8;
const RTC_ERROR: u8 = 3u8;
const HV_PSU_ERROR: u8 = 4u8;
const LOGIC_ERROR: u8 = 7u8;
static ERROR_FLAGS: AtomicU8 = AtomicU8::new(0u8);

fn set_error_flag(error: u8) {
    if error > 7 {
        set_error_flag(LOGIC_ERROR);
        return;
    }

    ERROR_FLAGS.store(
        ERROR_FLAGS.load(Ordering::Relaxed) | (1u8 << error),
        Ordering::Relaxed,
    );
}

bind_interrupts!(struct Irqs {
    USB_DRD_FS => usb::InterruptHandler<peripherals::USB>;
    EXTI4_15 => exti::InterruptHandler<interrupt::typelevel::EXTI4_15>;
    EXTI2_3 => exti::InterruptHandler<interrupt::typelevel::EXTI2_3>;
});

/// Leave low power run mode, switch to HSI for the clock source + turn on HSI48 for USB
fn switch_to_hsi_2<'a>(
    rcc: &'a mut Peri<'a, peripherals::RCC>,
    pwm: &mut SimplePwmChannel<'_, TIM2>,
) {
    use embassy_stm32::pac::*;
    use embassy_stm32::rcc::*;
    info!("switch_to_hsi_2");

    // TODO: dirty hack, would have to reconfigure the whole TIM2 between the two(three?) modes.
    pwm.set_duty_cycle_percent(50);

    // leave low power mode
    PWR.cr1().write(|f| f.set_lpr(false));
    while PWR.sr2().read().reglpf() {}

    let mut rcc_config = Config::default();
    {
        rcc_config.hsi = true;
        rcc_config.ls = LsConfig::default_lse();
        rcc_config.msi = Some(MSIRange::RANGE2M);
        rcc_config.sys = Sysclk::HSI;
        rcc_config.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        rcc_config.mux.clk48sel = mux::Clk48sel::HSI48;
    }

    reinit(rcc_config, rcc);

    info!("switch_to_hsi_2 - done");
}

/// Disable HSI and HSI48, use MSI for clock source + enter low power run mode.
fn switch_to_msi_2<'a>(
    rcc: &'a mut Peri<'a, peripherals::RCC>,
    pwm: &mut SimplePwmChannel<'_, TIM2>,
) {
    use embassy_stm32::pac::*;
    use embassy_stm32::rcc::*;
    info!("switch_to_msi_2");

    // TODO: dirty hack, would have to reconfigure the whole TIM2 between the two(three?) modes.
    pwm.set_duty_cycle_percent(30);

    reinit(
        embassy_stm32::rcc::Config {
            hsi: false,
            ls: LsConfig::default_lse(),
            msi: Some(MSIRange::RANGE2M),
            sys: Sysclk::MSI,
            hsi48: None,
            ..Default::default()
        },
        rcc,
    );

    // enter low power mode
    PWR.cr1().write(|f| f.set_lpr(true));

    info!("switch_to_msi_2 - done");
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting up");

    /*
        I2C: requires APB 2  MHz (debatable)
        USB: requires APB 10 MHz (confirmed)
    */
    let mut system_config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        system_config.rcc.hsi = true;
        system_config.rcc.ls = LsConfig::default_lse();
        system_config.rcc.msi = Some(MSIRange::RANGE2M);
        system_config.rcc.sys = Sysclk::HSI;
        system_config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        system_config.rcc.mux.clk48sel = mux::Clk48sel::HSI48;
    }

    let mut p = embassy_stm32::init(system_config);
    let mut rcc = p.RCC;

    // enabling the charging of the super-cap connected to V_BAT
    embassy_stm32::pac::PWR.cr4().write(|f| f.set_vbrs(false)); // charge V_BAT  through a 5 kOhms resistor
    embassy_stm32::pac::PWR.cr4().write(|f| f.set_vbe(true)); // battery charging enable 

    // Shared I2C bus for tasks
    // Default is 100kHz, medium speed GPIO, no internal pull-ups
    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c::Config::default());
    static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    /*
        HV PSU:
    */

    // Setting up the comparators
    comparator::set_default_csr();
    // Wait for the COMPs to stabilize, or else they will generate a false high interrupt
    Timer::after_micros(100).await;
    comparator::enable_interrupts();
    // enable interrupts globally
    unsafe {
        cortex_m::interrupt::enable();
    }

    // PWM gen
    let tim2_ch4 = PwmPin::new(p.PB11, embassy_stm32::gpio::OutputType::PushPull);
    let mut pwm_controller = SimplePwm::new(
        p.TIM2,
        None,
        None,
        None,
        Some(tim2_ch4),
        khz(10),
        Default::default(),
    );
    let mut pwm = pwm_controller.ch4();
    pwm.set_duty_cycle_percent(30);

    if HV_PSU_ENABLE.load(Ordering::Relaxed) {
        pwm.enable();
    }

    /*
        pulse_in (PC13) --> pulse_counter
    */
    let pulse_in = ExtiInput::new(p.PC13, p.EXTI13, Pull::None, Irqs);
    spawner.spawn(pulse_detection(pulse_in).unwrap());

    /*
        SSR control (PB2)
    */

    let ssr = Output::new(p.PB2, Level::Low, Speed::Low);
    static SSR_CTRL: StaticCell<OutputShared> = StaticCell::new();
    let ssr_ctrl = SSR_CTRL.init(Mutex::new(ssr));

    /*
        User button (PB3)
    */

    let user_button = ExtiInput::new(p.PB3, p.EXTI3, Pull::Up, Irqs);
    spawner.spawn(user_button_fn(user_button).unwrap());

    /*
        Switch (PA15)
    */

    let switch = Input::new(p.PA15, Pull::None);

    /*
        CRC peripheral
    */

    let crc = Crc::new(p.CRC, {
        use embassy_stm32::crc::*;
        Config::new(
            InputReverseConfig::Byte,
            true,
            PolySize::Width32,
            0xFFFFFFFF,
            0x04C11DB7,
        )
        .unwrap()
    });

    /*
        SD card
        The embedded-sdmmc library excepts a embeded_hal SPI device,
        we use the embassy-embedded-hal library to convert the Embassy SPI device to it.
    */

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = embassy_stm32::time::Hertz(100_000);

    let spi = embassy_stm32::spi::Spi::new_blocking(p.SPI2, p.PB13, p.PB15, p.PB14, spi_config);
    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_BUS.init(spi_bus);

    let cs_pin1 = Output::new(p.PA8, Level::Low, Speed::Medium);
    let spi_dev1 = SpiDevice::new(spi_bus, cs_pin1);

    let sd_card = SdCard::new(spi_dev1, embassy_time::Delay);

    /* SD card detection, and power control */
    let sd_card_detect = Input::new(p.PB12, Pull::None);
    let sd_card_power = Output::new(p.PA9, Level::Low, Speed::Medium);

    /*
        RTC
    */

    let (rtc, rtc_time) = Rtc::new(p.RTC, RtcConfig::default());
    static RTC_SHARED: StaticCell<RtcShared> = StaticCell::new();
    let rtc_shared = RTC_SHARED.init(Mutex::new(rtc));

    spawner
        .spawn(rtc_alarm(
            rtc_time.clone(),
            i2c_bus,
            switch,
            crc,
            sd_card,
            sd_card_detect,
            sd_card_power,
        )
        .unwrap());

    /*
        STC3315 + Status LEDs
    */

    // STC3315 initialization + start seq, sht20 soft reset
    {
        let mut i2c_locked = i2c_bus.lock().await;
        if let Err(e) = sht40::reset(&mut i2c_locked) {
            set_error_flag(I2C_ERROR);
            error!("SHT40 reset: {}", e);
        }

        if stc3315::init(&mut i2c_locked).is_err() {
            set_error_flag(I2C_ERROR);
        }
        Timer::after_micros(100).await;
        if stc3315::start(&mut i2c_locked).is_err() {
            set_error_flag(I2C_ERROR);
        }
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
        .spawn(status_LED_control(status_LEDs, charging_detection, i2c_bus).unwrap());

    /*
        USB initialization
    */

    //Timer::after_secs(5).await;

    // USB detect pin: PA10
    let usb_detect_pin = ExtiInput::new(p.PA10, p.EXTI10, Pull::None, Irqs);
    if usb_detect_pin.is_low() {
        switch_to_msi_2(&mut rcc.reborrow(), &mut pwm);
    }

    static USB_DETECT: StaticCell<ExtiInputShared> = StaticCell::new();
    let usb_detect = USB_DETECT.init(Mutex::new(usb_detect_pin));

    loop {
        usb_detect.lock().await.wait_for_high().await;
        info!("USB detected");
        switch_to_hsi_2(&mut rcc.reborrow(), &mut pwm);

        Timer::after_secs(1).await;

        // Create the driver, from the HAL.
        let driver = usb::Driver::new(p.USB.reborrow(), Irqs, p.PA12.reborrow(), p.PA11.reborrow());

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

        let class_future = async {
            loop {
                class.wait_connection().await;
                info!("Class: Connected");
                let _ = usb_connection::handle_usb_connection(
                    &mut class, i2c_bus, rtc_shared, rtc_time.clone(), ssr_ctrl, &mut pwm,
                )
                .await;
                info!("Class: Disconnected");
            }
        };

        let usb_disconnection_detect = async {
            usb_detect.lock().await.wait_for_low().await;
        };

        // initialize USB peripheral
        let mut usb = builder.build();
        let usb_future = usb.run();

        // run them concurrently until usb_disconnection_detect exits
        select3(usb_future, class_future, usb_disconnection_detect).await;

        info!("USB disconnected");

        // deactivate USB peripheral
        usb.disable().await;

        Timer::after_secs(1).await;

        switch_to_msi_2(&mut rcc.reborrow(), &mut pwm);
    }
    // Will never reach here
}

#[embassy_executor::task]
async fn status_LED_control(
    mut status_LEDs: [Output<'static>; 5],
    charging_detection: Input<'static>,
    i2c_bus: &'static I2c1Bus,
) {
    loop {
        if charging_detection.is_low() || USER_BUTTON.load(Ordering::Relaxed) {
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
                set_error_flag(I2C_ERROR);
                error!("Read failed!")
            }
            Timer::after_secs(1).await;
        } else {
            // Not charging
            for LED in &mut status_LEDs {
                LED.set_low();
            }

            Timer::after_secs(1).await;
        }
    }
}

#[embassy_executor::task]
async fn pulse_detection(mut pulse_in: ExtiInput<'static>) {
    loop {
        pulse_in.wait_for_rising_edge().await;
        let current_value = PULSE_COUNTER.load(Ordering::Relaxed);
        PULSE_COUNTER.store(current_value.wrapping_add(1), Ordering::Relaxed);

        let total_value = PULSE_COUNTER_TOTAL.load(Ordering::Relaxed);
        PULSE_COUNTER_TOTAL.store(total_value.wrapping_add(1), Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn user_button_fn(mut button: ExtiInput<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        debug!("Button pressed!");
        USER_BUTTON.store(true, Ordering::Relaxed);
        Timer::after_secs(5).await;
        USER_BUTTON.store(false, Ordering::Relaxed);
    }
}

// RTC "alarm", uses pooling as proper alarms are not supported (?) yet
// Set to trigger every min while testing. Final product should trigger every hour
#[embassy_executor::task]
async fn rtc_alarm(
    rtc: RtcTimeProvider,
    i2c_bus: &'static I2c1Bus,
    switch: Input<'static>,
    mut crc: Crc<'static>,
    mut sd_card: SdCard<
        SpiDevice<'static, NoopRawMutex, Spi<'static, Blocking, Master>, Output<'static>>,
        embassy_time::Delay,
    >,
    sd_card_detect: Input<'static>,
    mut sd_card_power: Output<'static>,
) {
    let mut old = rtc.now().unwrap();
    let mut now;
    loop {
        now = rtc.now().unwrap();
        if now.hour() != old.hour() {
            //if now.minute() != old.minute() { // for debug purposes
            old = now;
            let pulse_counter = PULSE_COUNTER.load(Ordering::Relaxed);
            let pulse_counter_total = PULSE_COUNTER_TOTAL.load(Ordering::Relaxed);
            PULSE_COUNTER.store(0u16, Ordering::Relaxed);
            info!("Pulse counter: {} - {}", pulse_counter, pulse_counter_total);

            if switch.is_high() {
                let mut i2c = i2c_bus.lock().await;
                let soc = match stc3315::read_SOC(&mut i2c) {
                    Ok(soc) => {
                        info!("STC3315 - SoC: {}", soc);
                        soc
                    }
                    Err(e) => {
                        set_error_flag(I2C_ERROR);
                        error!("STC3315 error: {}", e);
                        0u8
                    }
                };

                let t_rh = match sht40::read_raw(&mut i2c).await {
                    Ok(temp_and_rh) => {
                        info!("SHT40: {}", temp_and_rh);
                        [
                            temp_and_rh[0],
                            temp_and_rh[1],
                            temp_and_rh[3],
                            temp_and_rh[4],
                        ]
                    }
                    Err(e) => {
                        set_error_flag(I2C_ERROR);
                        error!("SHT40 error: {}", e);
                        [0u8; 4]
                    }
                };

                /*
                    Data format:
                        timestamp: year - month - day - hour - minute -> 5 bytes
                        soc: 1 byte
                        temperature and rh: 6 bytes for the whole read, 4 bytes if we ignore the crc
                        pulse counter: 2 bytes, reset after each write

                    12 bytes, but we have to respect the page boundaries. It makes writing much easier
                    if no record overflows from one page to another. That means the the record size has to
                    be a (whole number) fraction of 64!
                    In the I used 16 bytes / record, where the last 4 bytes are reserved for later.
                    Maybe CRC?

                    The EEPROM has a capacity of 32kBytes, enough for 2048 entries or 85 days.

                    But, the first 16 bytes are reserved!
                    The first two bytes contain the next write address, then next 14 can be used to store config.
                */

                static RECORDS_START_ADDRESS: u16 = 0x10;
                static RECORDS_LENGTH: u16 = 0x10;

                let mut address = match mc_24cs256::read_u16_random(&mut i2c, 0u16).await {
                    Ok(r) => {
                        info!("24CS256 read: {}", r);
                        r
                    }
                    Err(e) => {
                        set_error_flag(I2C_ERROR);
                        error!("24CS256 read error: {}", e);
                        continue; // Don't proceed with the write operation if the read failed.
                    }
                };

                // check boundaries
                if address < RECORDS_START_ADDRESS
                    || address + RECORDS_LENGTH > mc_24cs256::EEPROM_END_ADDRESS
                {
                    address = RECORDS_START_ADDRESS;
                }

                // write next address
                if let Err(e) =
                    mc_24cs256::write_u16(&mut i2c, 0u16, address + RECORDS_LENGTH).await
                {
                    set_error_flag(I2C_ERROR);
                    error!("24CS256 address write error to 0x00: {}", e);
                    continue; // Don't proceed with the write operation
                }

                let pulse_bytes = pulse_counter.to_be_bytes();
                let data = [
                    (old.year() - 2000) as u8,
                    old.month(),
                    old.day(),
                    old.hour(),
                    old.minute(),
                    soc,
                    t_rh[0],
                    t_rh[1],
                    t_rh[2],
                    t_rh[3],
                    pulse_bytes[0],
                    pulse_bytes[1],
                ];

                crc.reset();
                let crc_value = crc.feed_bytes(&data);

                // write data
                if let Err(e) =
                    mc_24cs256::write_data_record(&mut i2c, address, data, crc_value).await
                {
                    set_error_flag(I2C_ERROR);
                    error!("24CS256 data write error to {}: {}", address, e);
                    continue;
                }

                info!("Write data: success! Address: {}", address);

                /*
                    Writing to SD card if it is present.
                    Pulls sd_card_detect pin low.
                */
                if sd_card_detect.is_low() {
                    (sd_card, sd_card_power) = write_to_sd(&data, sd_card, sd_card_power).await;
                }
            }
        }
        Timer::after_secs(10).await;
    }
}

async fn write_to_sd(
    data: &[u8],
    mut sd_card: SdCard<
        SpiDevice<'static, NoopRawMutex, Spi<'static, Blocking, Master>, Output<'static>>,
        embassy_time::Delay,
    >,
    mut sd_card_power: Output<'static>,
) -> (
    SdCard<
        SpiDevice<'static, NoopRawMutex, Spi<'static, Blocking, Master>, Output<'static>>,
        embassy_time::Delay,
    >,
    Output<'static>,
) {
    sd_card_power.set_high(); // Enable power to the SD card
    Timer::after_millis(100).await;

    if sd_card.num_bytes().is_err() {
        error!("SD card initialization failed!");
        set_error_flag(SD_ERROR);
    } // card size is not important, but this fn will trigger a re-initialization

    // Trying to mount the device
    let volume_mgr = VolumeManager::new(sd_card, DummyTimesource::default());
    if let Ok(volume0) = volume_mgr.open_volume(VolumeIdx(0))
        && let Ok(root_dir) = volume0.open_root_dir()
    {
        // Open file to append next dataset
        if let Ok(my_file) =
            root_dir.open_file_in_dir("RESULTS.DAT", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)
        {
            // Write to the opened file
            if my_file.write(data).is_err() {
                error!("Wirite to SD card failed!");
                set_error_flag(SD_ERROR);
            }
        } else {
            error!("Opening file failed!");
            set_error_flag(SD_ERROR);
        }
    } else {
        error!("Mounting volume failed!");
        set_error_flag(SD_ERROR);
    }

    (sd_card, _) = volume_mgr.free(); // return the sd_card device, so can use it the next loop
    sd_card_power.set_low(); // Disable power to the SD card

    (sd_card, sd_card_power)
}

#[derive(Default)]
pub struct DummyTimesource();

// ToDo: USE the RTC for timestamp
impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
