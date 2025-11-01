use core::sync::atomic::Ordering;
use defmt::{error, info};
use embassy_stm32::{rtc::{DateTime, DayOfWeek}, usb};
use embassy_usb::{class::cdc_acm, driver::EndpointError};

use crate::{HV_PSU_ENABLE, I2c1Bus, OutputShared, PULSE_COUNTER, RtcShared, mc_24cs256, sht40, stc3315};

/*
    USB COMMUNICTAION
*/

pub struct Disconnected {}
impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

// Accepted commands:
const USB_COMMAND_GET_PULSE_COUNTER: u8 = 0x04; // returns the pulse_counter variable, u16, [MSB, LSB]
const USB_COMMAND_GET_TEMP_AND_RH: u8 = 0x08;   // returns the whole 6 bytes from SHT40 response, no conversion
const USB_COMMAND_GET_BATTERY_SOC: u8 = 0x0C;   // returns the battery SoC from STC3315

const USB_COMMAND_SET_RTC: u8 = 0x10;           // sets the RTC to the

const USB_COMMAND_EEPROM_CLEAR: u8 = 0x20;      // clear the contents of the EEPROM
const USB_COMMAND_EEPROM_READ: u8 = 0x24;       // read the contents of the EEPROM

const USB_COMMAND_HV_PSU_ON: u8 = 0x30;
const USB_COMMAND_HV_PSU_OFF: u8 = 0x34;

const USB_COMMAND_SSR_ON: u8 = 0x40;
const USB_COMMAND_SSR_OFF: u8 = 0x44;

// The task handling the response to the incomming USB communication
pub async fn handle_usb_connection<'d, T: usb::Instance + 'd>(
    class: &mut cdc_acm::CdcAcmClass<'d, usb::Driver<'d, T>>,
    i2c_bus: &'static I2c1Bus,
    rtc: &'static RtcShared,
    ssr: &'static OutputShared<'static>
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        match data[0] {
            USB_COMMAND_GET_PULSE_COUNTER => {
                let pulse_counter_bytes = PULSE_COUNTER.load(Ordering::Relaxed).to_be_bytes();
                let response = [pulse_counter_bytes[0], pulse_counter_bytes[1]];
                class.write_packet(&response).await?;
            }
            USB_COMMAND_GET_TEMP_AND_RH => {
                match sht40::read_raw(&mut *i2c_bus.lock().await).await {
                    Ok(response) => {
                        class.write_packet(&response).await?;
                    }
                    Err(e) => {
                        class.write_packet(&[0]).await?;
                        error!("SHT40: Raw read failed! {}", e);
                    }
                }
            }
            USB_COMMAND_GET_BATTERY_SOC => match stc3315::read_SOC(&mut *i2c_bus.lock().await) {
                Ok(response) => {
                    class.write_packet(&[response]).await?;
                }
                Err(e) => {
                    class.write_packet(&[0]).await?;
                    error!("SHT40: Raw read failed! {}", e);
                }
            },
            USB_COMMAND_SET_RTC => {
                if n != 8 {
                    class.write_packet(&[0]).await?;
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
                        error!("Failed to set RTC!");
                    }

                    class.write_packet(&[1]).await?;
                } else {
                    class.write_packet(&[0]).await?;
                    error!("Incorrect DateTime recived!");
                }
            }
            USB_COMMAND_EEPROM_CLEAR => match mc_24cs256::clear(&mut *i2c_bus.lock().await).await {
                Ok(_) => {
                    class.write_packet(&[1]).await?;
                    info!("24CS256: Clear OK!");
                }
                Err(e) => {
                    class.write_packet(&[0]).await?;
                    error!("24CS256: Clear failed! {}", e);
                }
            },
            USB_COMMAND_EEPROM_READ => {
                let mut i2c = i2c_bus.lock().await; // locking the i2c bus for the durration

                /*
                    Tried to send a whole page (64 bytes) at once instead, as it is equal to the max packet size, but 
                    had issues with it, as it would not send the requested data. 32 bytes works.
                */


                // a) we recive a page number
                if n > 2 {
                    let half_page_number = ((data[1] as u16 ) << 8 ) + data[2] as u16;
                    info!("Requesting half-page #{} of EEPROM", half_page_number);
                    if half_page_number < 1024 {
                        if let Ok(page) = mc_24cs256::read_32_bytes(&mut i2c, half_page_number).await {
                            class.write_packet(&page).await?;
                        } else {
                            class.write_packet(&[1]).await?;
                        }
                    } else {
                        error!("Requested half-page number is outside of EEPROM capacity");
                        class.write_packet(&[1]).await?;
                    }
                } else {
                 // b) we don't, and have to read 1024 32 byte pages
                 // This takes a while, less than 10 secs, but it still blocks the i2c bus in that time
                 // might cause issues
                    for i in 0..1024u16 {
                        if let Ok(page) = mc_24cs256::read_32_bytes(&mut i2c, i).await {
                            class.write_packet(&page).await?;
                        } else {
                            class.write_packet(&[1]).await?;
                        }
                    }
                }

                
            }
            USB_COMMAND_HV_PSU_ON => {
                HV_PSU_ENABLE.store(true, Ordering::Relaxed);
                class.write_packet(&[1]).await?;
            }
            USB_COMMAND_HV_PSU_OFF => {
                HV_PSU_ENABLE.store(false, Ordering::Relaxed);
                class.write_packet(&[1]).await?;
            }
            USB_COMMAND_SSR_ON => {
                ssr.lock().await.set_high();
                class.write_packet(&[1]).await?;
            }
            USB_COMMAND_SSR_OFF => {
                ssr.lock().await.set_low();
                class.write_packet(&[1]).await?;
            }
            _ => {
                error!("Unknown command: {}", data);
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
