#![allow(unused)]

use defmt::{debug, info};
use embassy_stm32::{i2c::{self, I2c, Master}, mode::Blocking};

const STC3115_ADDRESS: u8 = 0b111_0000;

// Registers:
const STC3115_REG_MODE: u8 = 0u8;
const STC3115_REG_CTRL: u8 = 1u8;           // ALM_SOC, ALM_VOLT bits
const STC3115_REG_SOC: u8 = 2u8;            // 2 bytes, 2-3
const STC3115_REG_COUNTER: u8 = 4u8;        // 2 bytes, 4-5
const STC3115_REG_CURRENT: u8 = 6u8;        // 2 bytes, 6-7
const STC3115_REG_VOLTAGE: u8 = 8u8;        // 2 bytes, 8-9
const STC3115_REG_OCV: u8 = 13u8;           // 2 bytes, 13-14
const STC3115_REG_CC_CNF: u8 = 15u8;        // 2 bytes, 15-16
const STC3115_REG_VM_CNF: u8 = 17u8;        // 2 bytes, 17-18
const STC3115_REG_ALARM_SOC: u8 = 19u8;     // 1 bytes + REG_ALARM_VOLTAGE is the next reg
const STC3115_REG_CURRENT_THRES: u8 = 21u8; // 1 bytes
const STC3115_REG_RELAX_MAX: u8 = 23u8;     // 1 bytes
const STC3115_REG_RAM_0: u8 = 32u8;         // 15 bytes, 32-47
const STC3115_REG_OCVTAB: u8 = 48u8;        // 16 bytes, 48-63

fn clear_STC3115_RAM(i2c: &mut I2c<'_, Blocking, Master>) -> Result<(),i2c::Error> {
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_RAM_0,        0b0000_0000, 0b0000_0000, 0b0000_0000 ])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_RAM_0+ 3u8,   0b0000_0000, 0b0000_0000, 0b0000_0000 ])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_RAM_0+ 6u8,   0b0000_0000, 0b0000_0000, 0b0000_0000 ])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_RAM_0+ 9u8,   0b0000_0000, 0b0000_0000, 0b0000_0000 ])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_RAM_0+ 12u8,  0b0000_0000, 0b0000_0000, 0b0000_0000 ])?;

    Ok(())
}

fn read_STC3115_RAM(i2c: &mut I2c<'_, Blocking, Master>) -> Result<(),i2c::Error> {
    let mut buf = [0u8,0u8,0u8];
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_RAM_0], &mut buf)?;
    info!("RAM reg 0: {:?}", buf);
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_RAM_0+3u8], &mut buf)?;
    info!("RAM reg 1: {:?}", buf);
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_RAM_0+6u8], &mut buf)?;
    info!("RAM reg 2: {:?}", buf);
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_RAM_0+9u8], &mut buf)?;
    info!("RAM reg 3: {:?}", buf);
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_RAM_0+12u8], &mut buf)?;
    info!("RAM reg 4: {:?}", buf);


    Ok(())
}

pub fn init(i2c: &mut I2c<'_, Blocking, Master>) -> Result<(),i2c::Error>  {
    // Reset RAM, write 0x00 at registers 0x20 to 0x2F (32-47, 15 regs)
    clear_STC3115_RAM(i2c)?;

    /* See: Appliction note - AN4324, model: STC3115AIQT */
    /*1. Read the OCV register. The first OCV measurement reflects the initial battery state of
    charge (SOC). Read it and save it into a temporary variable. */
    let mut ocv_raw = [0u8,0u8];
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_OCV], &mut ocv_raw )?;

    /* 2. Set the STC3115 parameters (ensuring first that the GG_RUN bit is set to 0). The
    REG_OCVTAB registers have to be filled with their previously calculated values as
    well as the values of the REG_CC_CNF and REG_VM_CNF registers. Then configure
    the application parameters by setting the REG_ALARM_SOC,
    REG_ALARM_VOLTAGE, REG_CURRENT_THRES, REG_RELAX_MAX,
    REG_ALARM_SOC, and REG_ALARM_VOLTAGE registers as required by the
    application to provide an HW interruption signal on the ALM pin when one of the
    measurements is detected as being below the defined threshold. The
    REG_CURRENT_THRES and REG_RELAX_MAX registers are used to configure the
    internal behavior of the STC3115. Default values are enough to provide good
    accuracy. */

    // 2.a)  Set GG_RUN bit in REG_MODE to 0
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_MODE, 0b000_0111])?;

    // 2.b) for REG_OCVTAB the default values (all 0) are reasonable for this application
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCVTAB,       0u8,0u8,0u8])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCVTAB + 3u8, 0u8,0u8,0u8])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCVTAB + 6u8, 0u8,0u8,0u8])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCVTAB + 9u8, 0u8,0u8,0u8])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCVTAB + 12u8, 0u8,0u8,0u8])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCVTAB + 15u8, 0u8])?;

    // 2.c) for REG_CC_CNF the default value assumes 10mOhm current sense resistor, and a 1957 mAh battery.
    //      the device uses a 10mOhm resistor, and a ~2000mAh battery, so the default (395) is fine
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_CC_CNF, 139u8, 1u8])?;

    // 2.d) REG_VM_CNF = Rinternal x Cnominal / 977.78
    //      it depends on the battery used. The default (321) should be fine, could double check if accuracy is a issue.
    //i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_VM_CNF], &mut buf )?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_VM_CNF, 65u8, 1u8])?;

    // 2.e) REG_ALARM_SOC, REG_ALARM_VOLTAGE
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_ALARM_SOC, 2u8, 170u8])?;

    // 2.f) REG_CURRENT_THRES, REG_RELAX_MAX
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_CURRENT_THRES, 10u8])?;
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_RELAX_MAX, 120u8])?;

    /* 3. From this point, the battery model characteristics and application threshold are
    initialized and battery tracking can start. To define the battery starting point, write back
    the OCV value into the REG_OCV register (16 bits) using the variable content. From
    this operation, the first battery SOC is available in the REG_SOC register 100 ms
    later. */

    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_OCV, ocv_raw[0], ocv_raw[1]])?;

    debug!("init_STC3115: Done");

    Ok(())
}

pub fn start(i2c: &mut I2c<'_, Blocking, Master>) -> Result<(),i2c::Error>  {
    i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_MODE, 0b001_0000])?;
    //i2c.blocking_write(STC3115_ADDRESS, &[STC3115_REG_CTRL, 0b000_0001])?;

    debug!("start_STC3115: Done");
    Ok(())
}

fn read_STC3115_settings(i2c: &mut I2c<'_, Blocking, Master>) -> Result<(),i2c::Error>  {
    let mut buf = [0u8,0u8];
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_MODE], &mut buf )?;
    info!("STC3115 MODE & CTRL: {:b}", buf);

    //i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_COUNTER], &mut buf )?;
    //info!("STC3115 COUNTER: {:?}", buf);

    //i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_CURRENT], &mut buf )?;
    //info!("STC3115 CURRENT: {:?}", buf);

    //i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_VOLTAGE], &mut buf )?;
    //info!("STC3115 VOLTAGE: {:?}", buf);

    Ok(())
}

pub fn read_SOC(i2c: &mut I2c<'_, Blocking, Master>) -> Result<u8,i2c::Error> {
    let mut buf = [0u8, 0u8];
    i2c.blocking_write_read(STC3115_ADDRESS, &[STC3115_REG_SOC], &mut buf )?;

    debug!("SoC raw result: {:?}", buf);

    // ROUGH ESTIMATE
    // it should be (MSB*256+LSB)/512
    // but the LSB is only 0.5% max, the reading being 1-2% of is acceptable
    // in this application
    Ok(buf[1]/2)
}