// This file is part of hayashi-light-remote
//
// hayashi-light-remote is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//
// Copyright 2020 Ledger SAS, written by Olivier Hériveaux

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use stm32f2::stm32f215;

static mut USART1_QUEUE: heapless::spsc::Queue<u8, heapless::consts::U128> =
    heapless::spsc::Queue(heapless::i::Queue::new());

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    unsafe {
        // In case of panic, the peripherals may have already been taken, and we
        // cannot take it again... But this is panic, we can do dirty things and
        // call steal to use peripherals anyway!
        // Here we just blink the red LED forever to indicate there is a
        // problem.
        let peripherals = stm32f215::Peripherals::steal();
        set_led(&peripherals, false);
        loop {
            for _ in 0..50000 {
                set_led(&peripherals, true);
            }
            for _ in 0..50000 {
                set_led(&peripherals, false);
            }
        }
    }
}

pub extern "C" fn handler_default() { loop {}; }

/// USART interrupt handler. Called in case of data byte reception or overrun.
/// When a byte is received, it is pushed in the USART queue. If the queue is
/// full, the program will panic.
pub extern "C" fn handler_usart1() {
    unsafe {
        let mut producer = USART1_QUEUE.split().0;
        let peripherals = stm32f215::Peripherals::steal();
        if peripherals.USART1.sr.read().rxne().bit() {
            // If queue is full, panic!
            producer.enqueue(peripherals.USART1.dr.read().bits() as u8).unwrap();
        } else {
            // This is probably an overrun error.
            panic!();
        }
    }
}

#[link_section=".isr_vectors.reset"]
#[no_mangle]
pub static reset_vector: unsafe extern "C" fn() -> ! = _start;

#[link_section=".isr_vectors"]
#[no_mangle]
pub static interrupt_vectors: [unsafe extern "C" fn(); 95] = {
    let mut v: [unsafe extern "C" fn(); 95] = [handler_default; 95];
    v[51] = handler_usart1;
    v
};

/// Toggle the LED on or off.
///
/// # Arguments
///
/// * `peripherals` - This method needs to borrow the peripherals.
/// * `state` - true to turn on the LED, false to turn off.
fn set_led(peripherals: &stm32f215::Peripherals, state: bool) {
    peripherals.GPIOB.odr.modify(|_, w| { w.odr0().bit(state) });
}

/// Toggle the lamp on or off.
///
/// # Arguments
///
/// * `peripherals` - This method needs to borrow the peripherals.
/// * `state` - true to turn on the lamp, false to turn off.
fn set_lamp(peripherals: &stm32f215::Peripherals, state: bool) {
    peripherals.GPIOA.odr.modify(|_, w| { w.odr1().bit(!state) });
}

/// Return true if lamp is dead
fn lamp_burnout(peripherals: &stm32f215::Peripherals) -> bool {
    !peripherals.GPIOA.idr.read().idr0().bit()
}

/// Approximated delay function. Precise enought for what we need to do...
#[inline(never)]
fn delay_ms(duration: u32) {
    // Estimated duration for each loop: 7 clock cycles.
    assert!(duration <= 0xffffffff / 64000);
    let count: u32 = (duration * 64000) / 7;
    for _ in 0..count {
        cortex_m::asm::nop();
    }
}

/// Receives a byte from USART1. Blocks until data is available.
fn usart1_rx() -> u8 {
    unsafe {
        let mut producer = USART1_QUEUE.split().1;
        loop {
            if let Some(byte) = producer.dequeue() { return byte; }
        }
    }
}

/// Receive a 16-bits unsigned int from USART1. Blocks until all data is
/// available.
fn usart1_rx_u16() -> u16 {
    let h = usart1_rx();
    let l = usart1_rx();
    ((h as u16) << 8) + (l as u16)
}

/// Transmit a byte over USART1.
///
/// # Arguments
///
/// * `peripherals` - This method needs to borrow the peripherals.
/// * `value` - Byte to be transmitted.
fn usart1_tx(peripherals: &stm32f215::Peripherals, value: u8) {
    peripherals.USART1.dr.write(|w| { w.dr().bits(value as u16) });
    // Wait until byte is transferred into the shift-register.
    while !peripherals.USART1.sr.read().txe().bit() {};
}

/// Transmit a string over USART1.
///
/// # Arguments
///
/// * `peripherals` - This method needs to borrow the peripherals.
/// * `s` - String
fn usart1_tx_str(peripherals: &stm32f215::Peripherals, s: &str) {
    for c in s.bytes() {
        usart1_tx(&peripherals, c);
    }
}

/// Transmit 16 bits over SPI1
///
/// # Arguments
///
/// * `peripherals` - This method needs to borrow the peripherals.
/// * `value` - Value to be transmitted.
fn spi1_tx_u16(peripherals: &stm32f215::Peripherals, value: u16) {
    while !peripherals.SPI1.sr.read().txe().bit() {};
    peripherals.GPIOA.odr.modify(|_, w| { w.odr4().clear_bit() });
    peripherals.SPI1.dr.write(|w| { w.dr().bits(value) });
    while !peripherals.SPI1.sr.read().rxne().bit() {};
    peripherals.GPIOA.odr.modify(|_, w| { w.odr4().set_bit() });
    peripherals.SPI1.dr.read();
}

/// Configure internal Flash memory interface.
/// This changes the Flash latency to be compatible with PLL settings.
fn setup_flash(peripherals: &stm32f215::Peripherals){
    unsafe {
        peripherals.FLASH.acr.modify(|_, w| { w.latency().bits(2) });
    }
}

/// Configure PLL
fn setup_pll(peripherals: &stm32f215::Peripherals){
    let rcc = &peripherals.RCC;
    // Disable PLL
    rcc.cr.modify(|_, w| { w.pllon().clear_bit() });
    // HSI = 16 MHz
    // F = ((HSI (N / M) / P
    // Constraints to be respected:
    // 50 <= N <= 432
    // 2 <= M <= 63
    // Here the target frequency is 64 MHz
    unsafe {
        rcc.pllcfgr.modify(|_, w|
            { w.plln().bits(64).pllm().bits(8).pllp().div2() });
    }
    // Enable PLL and wait it to be locked.
    rcc.cr.modify(|_, w| { w.pllon().set_bit() });
    while !rcc.cr.read().pllrdy().bit() {}
    // Switch to PLL clock
    rcc.cfgr.modify(|_, w| { w.sw().pll() });
}

#[no_mangle]
pub extern "C" fn _start() -> ! {
    // Get .bss segment position for .bss initialization performed in _start.
    extern {
        static _bss: u32;
        static _ebss: u32;
    }
    // Clear RAM of .bss section before doing anything!
    unsafe {
        for i in ((&_bss as *const u32) as u32 .. (&_ebss as *const u32) as u32)
            .step_by(4) {
            core::ptr::write_volatile(i as *mut u32, 0u32);
        }
    }

    let peripherals = stm32f215::Peripherals::take().unwrap();
    setup_flash(&peripherals);
    setup_pll(&peripherals);

    peripherals.RCC.apb2enr.write(|w| {
        w.usart1en().set_bit()
        .spi1en().set_bit()
    });
    // PA0: LAMP_BURNOUT
    // PA1: LAMP_ON_OFF
    // PA4: SPI_NSS
    // PA5: SPI_SCK
    // PA6: SPI_MISO (not connected)
    // PA7: SPI_MOSI
    // PA9: USART1 TX
    // PA10: USART1 RX
    // PB0: LED
    // Enable clock for PORT A, and PORT B peripherals.
    peripherals.RCC.ahb1enr.write(
        |w| { w.gpioaen().set_bit().gpioben().set_bit() } );
    let gpioa = &peripherals.GPIOA;
    gpioa.moder.write(|w| {
        w.moder1().output()
        .moder4().output()
        .moder5().alternate()
        .moder7().alternate()
        .moder9().alternate()
        .moder10().alternate()
    });
    gpioa.ospeedr.write(|w| {
        w.ospeedr10().very_high_speed()
        .ospeedr9().very_high_speed()
        .ospeedr4().very_high_speed()
        .ospeedr5().very_high_speed()
        .ospeedr7().very_high_speed()
    });
    gpioa.odr.modify(|_, w| { w.odr4().set_bit() }); // SPI_NSS high by default
    gpioa.afrl.write(|w| { w.afrl5().af5().afrl7().af5() });
    gpioa.afrh.write(|w| { w.afrh9().af7().afrh10().af7() });
    peripherals.GPIOB.moder.modify(|_, w| { w.moder0().output() });

    // Configure UART1
    // UART Enable, Transmitter Enable, Receiver Enable
    peripherals.USART1.cr1.write(
        |w| { w.ue().set_bit().te().set_bit().re().set_bit() });
    peripherals.USART1.cr2.write(|w|{ w.stop().bits(2) });
    // Baudrate is Fck/(8*(2-OVER8)*DIV)
    // Fck = 64 MHz
    // OVER8 = 0
    // DIV = BRR / 16
    // Here we set 9600 bps
    let brr_value = 6666;
    peripherals.USART1.brr.write(
        |w| { w.div_mantissa().bits(brr_value >> 4)
        .div_fraction().bits((brr_value & 0x0f) as u8) });
    // Enable interrupt for USART1
    peripherals.USART1.cr1.modify(|_, w| { w.rxneie().set_bit() });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32f215::Interrupt::USART1);
    }

    // Configure SPI1
    let spi1 = &peripherals.SPI1;
    spi1.cr1.write(|w| {
        w.spe().set_bit()
        .ssi().set_bit()
        .ssm().set_bit()
        .br().bits(7)
        .mstr().set_bit()
        .dff().set_bit()
        .cpha().set_bit() });

    let mut lamp_state = false;
    let mut lamp_intensity = 0u16;
    set_led(&peripherals, lamp_state);
    set_lamp(&peripherals, lamp_state);
    spi1_tx_u16(&peripherals, lamp_intensity);
    // Give some time for the FT232 to boot-up.
    delay_ms(500);

    loop
    {
        let command_byte = usart1_rx();
        match command_byte {
            // Get version string
            0x01 => {
                usart1_tx_str(&peripherals, "hyshlc-v1.0");
                usart1_tx(&peripherals, 0);
            }
            // Switch light On/Off
            0x02 => {
                lamp_state = usart1_rx() != 0;
                set_lamp(&peripherals, lamp_state);
                set_led(&peripherals, lamp_state);
                usart1_tx(&peripherals, command_byte);
            }
            // Set light power (control the DAC)
            0x03 => {
                lamp_intensity = usart1_rx_u16();
                spi1_tx_u16(&peripherals, lamp_intensity);
                usart1_tx(&peripherals, command_byte);
            }
            // Get lamp state
            0x04 => {
                usart1_tx(&peripherals, command_byte);
                usart1_tx(&peripherals, lamp_state as u8);
            }
            // Get lamp intensity
            0x05 => {
                usart1_tx(&peripherals, command_byte);
                usart1_tx(&peripherals, (lamp_intensity >> 8) as u8);
                usart1_tx(&peripherals, (lamp_intensity & 0xff) as u8);
            }
            // Get lamp burnout indication
            0x06 => {
                usart1_tx(&peripherals, command_byte);
                usart1_tx(&peripherals, if lamp_burnout(&peripherals) { 1 } else { 0 });
            }
            _ => {
                // Unknown command. Panic!
                panic!();
            }
        }
    }
}
