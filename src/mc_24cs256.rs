#![allow(unused)]
use defmt::{debug, info};
use embassy_stm32::{i2c::{self, I2c, Master}, mode::Blocking};
use embassy_time::Timer;

const I2C_ADDRESS_EEPROM: u8 = 0b101_0000; // SOT-23 package, last 3 bit is 0
const I2C_ADDRESS_REGISTERS: u8 = 0b101_1000; // SOT-23 package, last 3 bit is 0

pub const EEPROM_START_ADDRESS: u16 = 0x00;
pub const EEPROM_END_ADDRESS: u16 = 0x7FFF;

/*
    TO DO:
        Add a timeout to the write OPs!
*/

/*
Word address bytes:
15 bits
MSB (7 bits, A14-A8) then LSB (8 bits, A7-A0)

byte write or page write (64 bytes)

*/

// WRITE

pub async fn write_byte(i2c: &mut I2c<'_, Blocking, Master>, address: u16, data: u8) -> Result<(),i2c::Error> {
    let address_bytes = address.to_be_bytes();
    i2c.blocking_write(I2C_ADDRESS_EEPROM, &[address_bytes[0], address_bytes[1], data])?;

    // waiting for write to finish
    // TODO: add a timeout!
    while let Err(e) = i2c.blocking_write(I2C_ADDRESS_EEPROM, &[]) {
        if e != i2c::Error::Nack {
            return Err(e);
        }

        Timer::after_micros(10).await;
    }

    Ok(())
}

pub async fn write_u16(i2c: &mut I2c<'_, Blocking, Master>, address: u16, data: u16) -> Result<(),i2c::Error> {
    let address_bytes = address.to_be_bytes();
    let data_bytes = data.to_be_bytes();
    i2c.blocking_write(I2C_ADDRESS_EEPROM, &[address_bytes[0], address_bytes[1], data_bytes[0], data_bytes[1] ])?;

    // waiting for write to finish
    // TODO: add a timeout!
    while let Err(e) = i2c.blocking_write(I2C_ADDRESS_EEPROM, &[]) {
        if e != i2c::Error::Nack {
            return Err(e);
        }

        Timer::after_micros(10).await;
    }

    Ok(())
}

pub async fn write_raw_data(i2c: &mut I2c<'_, Blocking, Master>, data: &[u8]) -> Result<(),i2c::Error> {
    i2c.blocking_write(I2C_ADDRESS_EEPROM, data)?;

    // waiting for write to finish
    // TODO: add a timeout!
    while let Err(e) = i2c.blocking_write(I2C_ADDRESS_EEPROM, &[]) {
        if e != i2c::Error::Nack {
            return Err(e);
        }

        Timer::after_micros(10).await;
    }

    Ok(())
}

pub async fn write_data_record(i2c: &mut I2c<'_, Blocking, Master>, address: u16, record: [u8;12], crc: u32) -> Result<(),i2c::Error> {
    let address_bytes = address.to_be_bytes();
    let crc_bytes = crc.to_be_bytes();

    // This looks terrible, but there is no built-in method to join arrays
    let data: [u8; 18] = [
        address_bytes[0], address_bytes[1],
        record[0], record[1], record[2], record[3], record[4], record[5], record[6], record[7], record[8], record[9], record[10], record[11],
        crc_bytes[0], crc_bytes[1], crc_bytes[2], crc_bytes[3]
        ];

    i2c.blocking_write(I2C_ADDRESS_EEPROM, &data)?;

    // waiting for write to finish
    // TODO: add a timeout!
    while let Err(e) = i2c.blocking_write(I2C_ADDRESS_EEPROM, &[]) {
        if e != i2c::Error::Nack {
            return Err(e);
        }

        Timer::after_micros(10).await;
    }

    Ok(())
}

// READ

pub async fn read_byte_random(i2c: &mut I2c<'_, Blocking, Master>, address: u16) -> Result<u8,i2c::Error> {
    let address_bytes = address.to_be_bytes();
    i2c.blocking_write(I2C_ADDRESS_EEPROM, &[address_bytes[0], address_bytes[1]])?;

    let mut buf = [0u8];
    i2c.blocking_read(I2C_ADDRESS_EEPROM, &mut buf)?;

    Ok(buf[0])
}

pub async fn read_u16_random(i2c: &mut I2c<'_, Blocking, Master>, address: u16) -> Result<u16,i2c::Error> {
    let address_bytes = address.to_be_bytes();
    i2c.blocking_write(I2C_ADDRESS_EEPROM, &[address_bytes[0], address_bytes[1]])?;

    let mut buf = [0u8; 2];
    i2c.blocking_read(I2C_ADDRESS_EEPROM, &mut buf)?;

    Ok(((buf[0] as u16) << 8) + buf[1] as u16)
}


/*
    One page = 64 bytes
    Total of 512 pages
*/

pub async fn read_32_bytes(i2c: &mut I2c<'_, Blocking, Master>, half_page: u16) -> Result<[u8;32],i2c::Error> {
    let address_bytes = (EEPROM_START_ADDRESS+half_page*32).to_be_bytes();
    i2c.blocking_write(I2C_ADDRESS_EEPROM, &[address_bytes[0], address_bytes[1]])?;

    let mut buf = [0u8;32];
    i2c.blocking_read(I2C_ADDRESS_EEPROM, &mut buf)?;

    Ok(buf)
}

pub async fn clear(i2c: &mut I2c<'_, Blocking, Master>) -> Result<(),i2c::Error> {

    let mut write = [0u8;66];
    for address in (EEPROM_START_ADDRESS..EEPROM_END_ADDRESS).step_by(64) {
        let address_bytes = address.to_be_bytes();
        write[0] = address_bytes[0];
        write[1] = address_bytes[1];
        i2c.blocking_write(I2C_ADDRESS_EEPROM, &write)?;

        // waiting for write to finish
        // TODO: add a timeout!
        while let Err(e) = i2c.blocking_write(I2C_ADDRESS_EEPROM, &[]) {
            if e != i2c::Error::Nack {
                return Err(e);
            }

            Timer::after_micros(10).await;
        }
    }
    
    Ok(())
}

