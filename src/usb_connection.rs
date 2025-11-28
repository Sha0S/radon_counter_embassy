use core::sync::atomic::Ordering;
use defmt::{error, info};
use embassy_stm32::{peripherals::TIM2, rtc::{DateTime, DayOfWeek}, timer::simple_pwm::SimplePwmChannel, usb};
use embassy_usb::{class::cdc_acm, driver::EndpointError};

use crate::{ERROR_FLAGS, HV_PSU_ENABLE, I2C_ERROR, I2c1Bus, OutputShared, PULSE_COUNTER, PULSE_COUNTER_TOTAL, RtcShared, USB_ERROR, mc_24cs256, sht40, stc3315};

/*
    USB COMMUNICATION
*/

pub struct Disconnected {}
impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => {
                crate::set_error_flag(USB_ERROR);
                panic!("Buffer overflow")
            },
            EndpointError::Disabled => Disconnected {},
        }
    }
}

// Accepted commands:
const USB_COMMAND_GET_ERROR_FLAGS: u8 = 0x02;   // returns the ERROR_FLAGS variable, u8
const USB_COMMAND_RESET_ERROR_FLAGS: u8 = 0x03;   // reset the ERROR_FLAGS variable

const USB_COMMAND_GET_PULSE_COUNTER: u8 = 0x04; // returns the pulse_counter variable, u16, [MSB, LSB]
const USB_COMMAND_GET_TEMP_AND_RH: u8 = 0x08;   // returns the whole 6 bytes from SHT40 response, no conversion
const USB_COMMAND_GET_BATTERY_SOC: u8 = 0x0C;   // returns the battery SoC from STC3315

const USB_COMMAND_SET_RTC: u8 = 0x10;           // sets the RTC to the time provided
const USB_COMMAND_GET_RTC: u8 = 0x12;           // reads the RTC

const USB_COMMAND_EEPROM_CLEAR: u8 = 0x20;      // clear the contents of the EEPROM
const USB_COMMAND_EEPROM_READ: u8 = 0x24;       // read the contents of the EEPROM

const USB_COMMAND_HV_PSU_ON: u8 = 0x30;
const USB_COMMAND_HV_PSU_OFF: u8 = 0x34;

const USB_COMMAND_SSR_ON: u8 = 0x40;
const USB_COMMAND_SSR_OFF: u8 = 0x44;

// Response:
const USB_RESPONSE_OK: u8 = 0x00;
const USB_RESPONSE_NOK: u8 = 0x01;

// The task handling the response to the incoming USB communication
pub async fn handle_usb_connection<'d, T: usb::Instance + 'd>(
    class: &mut cdc_acm::CdcAcmClass<'d, usb::Driver<'d, T>>,
    i2c_bus: &'static I2c1Bus,
    rtc: &'static RtcShared,
    ssr: &'static OutputShared<'static>,
    pwm: &mut SimplePwmChannel<'_, TIM2>
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        match data[0] {
            USB_COMMAND_GET_PULSE_COUNTER => {
                let pulse_counter_bytes = PULSE_COUNTER.load(Ordering::Relaxed).to_be_bytes();
                let pulse_total_bytes = PULSE_COUNTER_TOTAL.load(Ordering::Relaxed).to_be_bytes();
                let response = [pulse_counter_bytes[0], pulse_counter_bytes[1], pulse_total_bytes[0], pulse_total_bytes[1]];
                class.write_packet(&response).await?;
            }
            USB_COMMAND_GET_TEMP_AND_RH => {
                match sht40::read_raw(&mut *i2c_bus.lock().await).await {
                    Ok(response) => {
                        class.write_packet(&response).await?;
                    }
                    Err(e) => {
                        crate::set_error_flag(I2C_ERROR);
                        class.write_packet(&[USB_RESPONSE_NOK]).await?;
                        error!("SHT40: Raw read failed! {}", e);
                    }
                }
            }
            USB_COMMAND_GET_BATTERY_SOC => match stc3315::read_SOC(&mut *i2c_bus.lock().await) {
                Ok(response) => {
                    class.write_packet(&[response]).await?;
                }
                Err(e) => {
                    crate::set_error_flag(I2C_ERROR);
                    class.write_packet(&[USB_RESPONSE_NOK]).await?;
                    error!("STC3315: read SoC failed! {}", e);
                }
            },
            USB_COMMAND_SET_RTC => {
                if n != 8 {
                    crate::set_error_flag(USB_ERROR);
                    class.write_packet(&[USB_RESPONSE_NOK]).await?;
                    error!("Packet size incorrect for setting RTC");
                } else if let Ok(dow) = day_of_week_from_u8(buf[4])
                    && let Ok(new_time) = DateTime::from(
                        2000u16 + buf[1] as u16,
                        buf[2],
                        buf[3],
                        dow,
                        buf[5],
                        buf[6],
                        buf[7],
                        0,
                    )
                {
                    if rtc.lock().await.set_datetime(new_time).is_err() {
                        crate::set_error_flag(USB_ERROR);
                        class.write_packet(&[USB_RESPONSE_NOK]).await?;
                        error!("Failed to set RTC!");
                    } else {
                        class.write_packet(&[USB_RESPONSE_OK]).await?;
                    }
                } else {
                    crate::set_error_flag(USB_ERROR);
                    class.write_packet(&[USB_RESPONSE_NOK]).await?;
                    error!("Incorrect DateTime received!");
                }
            }
            USB_COMMAND_GET_RTC => {
                if let Ok(now) = rtc.lock().await.now() {
                    class.write_packet(&[
                        USB_RESPONSE_OK, 
                        (now.year() - 2000) as u8, 
                        now.month(), 
                        now.day(),
                        now.hour(),
                        now.minute()
                    ]).await?;
                } else {
                    crate::set_error_flag(crate::RTC_ERROR);
                    class.write_packet(&[USB_RESPONSE_NOK]).await?;
                    error!("Failed to get current time from RTC!");
                }
            }
            USB_COMMAND_EEPROM_CLEAR => match mc_24cs256::clear(&mut *i2c_bus.lock().await).await {
                Ok(_) => {
                    class.write_packet(&[USB_RESPONSE_OK]).await?;
                    info!("24CS256: Clear OK!");
                }
                Err(e) => {
                    crate::set_error_flag(I2C_ERROR);
                    class.write_packet(&[USB_RESPONSE_NOK]).await?;
                    error!("24CS256: Clear failed! {}", e);
                }
            },
            USB_COMMAND_EEPROM_READ => {
                let mut i2c = i2c_bus.lock().await; // locking the i2c bus for the duration

                /*
                    Tried to send a whole page (64 bytes) at once instead, as it is equal to the max packet size, but 
                    had issues with it, as it would not send the requested data. 32 bytes works.
                */


                // a) we receive a page number
                if n > 2 {
                    let half_page_number = ((data[1] as u16 ) << 8 ) + data[2] as u16;
                    info!("Requesting half-page #{} of EEPROM", half_page_number);
                    if half_page_number < 1024 {
                        if let Ok(page) = mc_24cs256::read_32_bytes(&mut i2c, half_page_number).await {
                            class.write_packet(&page).await?;
                        } else {
                            crate::set_error_flag(I2C_ERROR);
                            class.write_packet(&[USB_RESPONSE_NOK]).await?;
                        }
                    } else {
                        error!("Requested half-page number is outside of EEPROM capacity");
                        crate::set_error_flag(USB_ERROR);
                        class.write_packet(&[USB_RESPONSE_NOK]).await?;
                    }
                } else {
                 // b) we don't, and have to read 1024 32 byte pages
                 // This takes a while, less than 10 (4-5?) secs, but it still blocks the i2c bus in that time
                 // might cause issues
                    for i in 0..1024u16 {
                        if let Ok(page) = mc_24cs256::read_32_bytes(&mut i2c, i).await {
                            class.write_packet(&page).await?;
                        } else {
                            crate::set_error_flag(I2C_ERROR);
                            class.write_packet(&[USB_RESPONSE_NOK]).await?;
                        }
                    }
                }

                
            }
            USB_COMMAND_HV_PSU_ON => {
                HV_PSU_ENABLE.store(true, Ordering::Relaxed);
                pwm.enable();
                class.write_packet(&[USB_RESPONSE_OK]).await?;
            }
            USB_COMMAND_HV_PSU_OFF => {
                HV_PSU_ENABLE.store(false, Ordering::Relaxed);
                pwm.disable();
                class.write_packet(&[USB_RESPONSE_OK]).await?;
            }
            USB_COMMAND_SSR_ON => {
                ssr.lock().await.set_high();
                class.write_packet(&[USB_RESPONSE_OK]).await?;
            }
            USB_COMMAND_SSR_OFF => {
                ssr.lock().await.set_low();
                class.write_packet(&[USB_RESPONSE_OK]).await?;
            }
            USB_COMMAND_GET_ERROR_FLAGS => {
                let error_flags = ERROR_FLAGS.load(Ordering::Relaxed);
                class.write_packet(&[error_flags]).await?;
            }
            USB_COMMAND_RESET_ERROR_FLAGS => {
                ERROR_FLAGS.store(0u8, Ordering::Relaxed);
                class.write_packet(&[USB_RESPONSE_OK]).await?;
            }
            _ => {
                error!("Unknown command: {}", data);
                crate::set_error_flag(USB_ERROR);
                class.write_packet(data).await?;
            }
        }
    }
}

fn day_of_week_from_u8(v: u8) -> Result<DayOfWeek, embassy_stm32::rtc::DateTimeError> {
    Ok(match v {
        1 => DayOfWeek::Monday,
        2 => DayOfWeek::Tuesday,
        3 => DayOfWeek::Wednesday,
        4 => DayOfWeek::Thursday,
        5 => DayOfWeek::Friday,
        6 => DayOfWeek::Saturday,
        7 => DayOfWeek::Sunday,
        x => return Err(embassy_stm32::rtc::DateTimeError::InvalidDayOfWeek(x)),
    })
}
