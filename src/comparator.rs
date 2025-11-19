/*
    There is no support for comparators in Embassy (and neither is it in the rust embedded HALs),
    so it has to be directly configured by writing to the registers.

    For interrupts it uses EXTI Line 17 and 18, and NVIC position 12. (ADC + COMP 1/2 uses the same interrupt)
*/

use core::sync::atomic::Ordering;

use defmt::{error, info};
use embassy_stm32::interrupt;

use crate::HV_PSU_ENABLE;

/// Comparator 1 control and status register
const COMP1_CSR: *mut u32 = 0x4001_0200u32 as *mut u32;
/// Comparator 2 control and status register
const COMP2_CSR: *mut u32 = (0x4001_0200u32 + 0x4u32) as *mut u32;

// NVIC ADC_COMP: ADC and COMP interrupts (ADC combined with EXTI lines 17 & 18)
/// NVIC BASE ADDRESS:
const NVIC_ISER0: *mut u32 = 0xE000_E100u32 as *mut u32;
const NVIC_ADC_COMP: u32 = 0x1u32 << 12;
const NVIC_ICPR0: *mut u32 = 0xE000E280u32 as *mut u32;

const COMP1_EXTI_LINE: u32 = 1u32 << 17;
const COMP2_EXTI_LINE: u32 = 1u32 << 18;

/// EXTI base address:
const EXTI: u32 = 0x4002_1800u32;
/// EXTI rising trigger selection register
const EXTI_RTSR1: *mut u32 = EXTI as *mut u32;
/// EXTI falling trigger selection register 1
const EXTI_FTSR1: *mut u32 = (EXTI + 0x004u32) as *mut u32;
/// EXTI rising edge pending register 1
const EXTI_RPR1: *mut u32 = (EXTI + 0x00Cu32) as *mut u32;
/// EXTI falling edge pending register 1
const EXTI_FPR1: *mut u32 = (EXTI + 0x010u32) as *mut u32;
/// EXTI CPU wake-up with interrupt mask register
const EXTI_IMR1: *mut u32 = (EXTI + 0x080u32) as *mut u32;
/// EXTI CPU wake-up with event mask register
const EXTI_EMR1: *mut u32 = (EXTI + 0x084u32) as *mut u32;

/*
    Dirty hack:
    The following registry value works, but it overwrites multiple reserved bits, whose values we where supposed to keep
    0b0000_0000_0000_0110_0000_0010_0010_0001
    - 0000_000 -> bit 31-30 is status reg (LOCK and VALUE), can't write. Next 5 are reserved.
    - BLANKSEL: 0_0000 - no blanking
    - PWRMODE:  01     - medium speed
    - HYST:     10     - medium hysteresis
    - polarity: 0      - non-inverted
    - WINOUT:   0      - window mode disabled
    - WINMODE:  000    - non-inverting input selector for window mode (not used)
    - INPSEL:   010    - input pin selection: PA1 and PA3
    - INMSEL:   0010   - Vref,int
    - enabled:  0001

    The same value works for both COMPs.
*/

/// Comparator control and status register default
const CSR_DEFAULT: u32 =   0b0000_0000_0000_0110_0000_0010_0011_0001;
const CSR_DEFAULT_2: u32 = 0b0000_0000_0000_1100_0000_0010_0011_0001;

/// Setting both of the comparator status registers to the CSR_DEFAULT value
pub fn set_default_csr() {
    unsafe {
        core::ptr::write_volatile(COMP1_CSR, CSR_DEFAULT);
        core::ptr::write_volatile(COMP2_CSR, CSR_DEFAULT_2);
    }
}

/// Sets EXTI_RTSR1 bit 17 & 18 to 1. Enabling rising edge interrupts for COMP 1 & 2;
pub fn enable_interrupts() {
    unsafe {
        let rtsr = core::ptr::read_volatile(EXTI_RTSR1) | COMP1_EXTI_LINE | COMP2_EXTI_LINE;
        core::ptr::write_volatile(EXTI_RTSR1, rtsr); // set rising edge detection
        core::ptr::write_volatile(EXTI_RPR1, COMP1_EXTI_LINE | COMP2_EXTI_LINE); // clear pending register

        core::ptr::write_volatile(EXTI_FTSR1, rtsr); // set falling edge detection
        core::ptr::write_volatile(EXTI_FPR1, COMP1_EXTI_LINE | COMP2_EXTI_LINE); // clear pending register

        let emr = core::ptr::read_volatile(EXTI_EMR1) & !(COMP1_EXTI_LINE | COMP2_EXTI_LINE);
        core::ptr::write_volatile(EXTI_EMR1, emr); // disable event mode

        let imr = core::ptr::read_volatile(EXTI_IMR1) | COMP1_EXTI_LINE | COMP2_EXTI_LINE;
        core::ptr::write_volatile(EXTI_IMR1, imr); // enable interrupt mode

        core::ptr::write_volatile(NVIC_ICPR0, NVIC_ADC_COMP); // clear any pending interrupts
        core::ptr::write_volatile(NVIC_ISER0, NVIC_ADC_COMP); // enable NVIC interrupt

        info!("COMP interrupts set!");
    }
}

// bit number 30 is the comparator output, 1 - high, 0 - low
fn read_comp_status(register: u32) -> bool {
    register & (1 << 30) != 0
}

/// Returns the output level of the comparator.
/// Low -> false, High -> true
pub fn read_comp1_value() -> bool {
    let csr = unsafe { core::ptr::read_volatile(COMP1_CSR) };

    read_comp_status(csr)
}

/// Returns the output level of the comparator.
/// Low -> false, High -> true
pub fn read_comp2_value() -> bool {
    let csr = unsafe { core::ptr::read_volatile(COMP2_CSR) };

    read_comp_status(csr)
}

/// TIM2 capture/compare enable register (TIM2_CCER)
/// 0x4000 0000 + 0x20
const TIM2_CCER: *mut u32 = 0x4000_0020u32 as *mut u32;
const CH4_ENABLE: u32 = 0x1u32 << 12;

/*
    Could also check which comparator has pending interrupt first,
    but it is kind of irrelevant
*/
#[interrupt]
unsafe fn ADC_COMP1_2() {
    let comp1 = read_comp1_value();
    let comp2 = read_comp2_value();

    unsafe {
        let ccer = core::ptr::read_volatile(TIM2_CCER);
        if HV_PSU_ENABLE.load(Ordering::Relaxed) {
                if comp2 { // Safety: Disable HV power gen
                    error!("COMP2 high: HV PSU disabled!");
                    HV_PSU_ENABLE.store(false, Ordering::Relaxed);
                    core::ptr::write_volatile(TIM2_CCER, ccer & !CH4_ENABLE); // disables PWM output
                } else if comp1 {
                    core::ptr::write_volatile(TIM2_CCER, ccer & !CH4_ENABLE); // disables PWM output
                } else {
                    core::ptr::write_volatile(TIM2_CCER, ccer | CH4_ENABLE); // enables PWM output
                }
        } else {
            core::ptr::write_volatile(TIM2_CCER, ccer & !CH4_ENABLE); // disables PWM output
        }

        // reset EXTI_RPR1, EXTI_FPR1, NVIC_ICPR0
        core::ptr::write_volatile(EXTI_RPR1, COMP1_EXTI_LINE | COMP2_EXTI_LINE);
        core::ptr::write_volatile(EXTI_FPR1, COMP1_EXTI_LINE | COMP2_EXTI_LINE);
        core::ptr::write_volatile(NVIC_ICPR0, NVIC_ADC_COMP);
    }
}
