#![feature(abi_avr_interrupt)]
#![feature(asm_experimental_arch)]
#![feature(lang_items)]

#![no_std]
#![no_main]

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }

#[no_mangle]
#[lang = "eh_personality"]
pub unsafe extern "C" fn rust_eh_personality() -> () {}

// extern crate avr_device;
// use avr_device::atmega32u4::Peripherals;

use core::ptr::{read_volatile, write_volatile};
use core::arch::asm;

// #[no_mangle]
// pub unsafe extern "avr-interrupt" fn timer() {
//     let p = Peripherals::take().unwrap_unchecked();
// }

pub const F_CPU: u32 = 16_000_000 as u32;

pub const DELAY_MS: u16 = (F_CPU / 4000) as u16;

pub const DDRB:  *mut u8 = 0x24 as *mut u8;
pub const PORTB:  *mut u8 = 0x25 as *mut u8;
pub const DDRD:  *mut u8 = 0x2a as *mut u8;
pub const PORTD:  *mut u8 = 0x2b as *mut u8;

// http://www.nongnu.org/avr-libc/user-manual/group__util__delay__basic.html
#[inline(always)]
pub fn _delay_loop_2(count: u16) {
    unsafe {
        asm!("1: sbiw {i}, 1",
             "brne 1b",
             i = inout(reg_iw) count => _,
        );
    }
}

pub fn delay_ms(ms: u16) {
    for _ in 0..ms {
        _delay_loop_2(DELAY_MS);
    }
}

pub fn enable(addr: *mut u8, bit: u8) {
    unsafe {
        write_volatile(addr, read_volatile(addr) | bit)
    }
}

pub fn disable(addr: *mut u8, bit: u8) {
    unsafe {
        write_volatile(addr, read_volatile(addr) & !bit)
    }
}


#[no_mangle]
pub unsafe extern "C" fn main() {
    enable(DDRD, 1 << 5);
    enable(DDRB, 1 << 0);

        // enable(PORTB, 1 << 0);
        // enable(PORTD, 1 << 5);

    loop {
        enable(PORTB, 1 << 0);
        disable(PORTD, 1 << 5);
        delay_ms(1000);

        disable(PORTB, 1 << 0);
        enable(PORTD, 1 << 5);
        delay_ms(1000);
    }

    // let p = Peripherals::take().unwrap_unchecked();

    // p.PORTC.ddrc.write(|w| w.pc7().set_bit());
    // p.PORTC.portc.write(|w| w.pc7().set_bit());

    // loop {
    //     avr_device::asm::sleep();
    // }
}
