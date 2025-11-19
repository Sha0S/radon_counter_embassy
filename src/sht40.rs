#![allow(unused)]
#![allow(non_camel_case_types)]
// SHT 40
// TODO: CRC check

// The address can be: 0x44, 0x45, 0x46
const I2C_ADDRESS: u8 = 0x46;

const SOFT_RESET: u8 = 0x94;
const READ_MEDIUM_PREC: u8 = 0xF6;


use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use defmt::{debug, info};
use embassy_stm32::{i2c::{self, Master}, mode::Blocking};
use embassy_time::{Delay, Timer};

pub struct SHT40_result {
    pub temperature: f32,
    pub relative_humidity: f32
}

// Should wait 20ms between reset and read
pub fn reset(i2c: &mut i2c::I2c<'_,Blocking,Master>) -> Result<(), embassy_stm32::i2c::Error> {
    i2c.blocking_write(I2C_ADDRESS, &[SOFT_RESET])?;

    debug!("SHT40 soft reset: Done");
    Ok(())
}

pub async fn read(i2c: &mut i2c::I2c<'_,Blocking,Master>) -> Result<SHT40_result, embassy_stm32::i2c::Error> {

    i2c.blocking_write(I2C_ADDRESS, &[READ_MEDIUM_PREC])?;

    Timer::after_millis(10).await; // 10ms wait

    let buf = &mut [0u8; 6];
    i2c.blocking_read(I2C_ADDRESS, buf)?;
    

    let t_raw: u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);
    let t = 175.0 * (t_raw as f32 / 65535.0) - 45.0;


    let rh_raw: u16 = ((buf[3] as u16) << 8) | (buf[4] as u16);
    let rh = 125.0 * (rh_raw as f32 / 65535.0) - 6.0;

    Ok(SHT40_result{
        temperature: t,
        relative_humidity: rh
    })
}

pub async fn read_raw(i2c: &mut i2c::I2c<'_,Blocking,Master>) -> Result<[u8;6], embassy_stm32::i2c::Error> {

    i2c.blocking_write(I2C_ADDRESS, &[READ_MEDIUM_PREC])?;

    Timer::after_millis(10).await; // 10ms wait

    let mut buf = [0u8; 6];
    i2c.blocking_read(I2C_ADDRESS, &mut buf)?;
    info!("SHT40 response: {:?}", buf);

    Ok(buf)
}