// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the nRF52832 SPI, in host mode.
//!
//! This is the core logic, separated from the IPC server.
//!
//! The nRF52 has a very simple SPI compared to some other MCUs.
//!
//! # First, a note on DMA
//!
//! the nRF52 has a version of SPI that uses DMA. it's better than this. we
//! should use it. I got most of the way through implementing this using the
//! non-DMA interface so I'm going to keep going but it's worse than the DMA
//! version would be and requires the SPI task to do things after every byte
//! of transmission. The non-DMA version is also considered "deprecated", but
//! I don't know what that actually implies for Nordic stuff.
//!
//! # Clocking
//!
//! tick tock tick tock tick tock tick tock
//!
//! The FREQUENCY register allows the following pre-determined speeds, with
//! no further divisions:
//!
//! - 125KHz
//! - 250KHz
//! - 500KHz
//! - 1MHz
//! - 2MHz
//! - 4MHz
//! - 8MHz
//!
//!
//! # Double Buffering
//!
//! nRF52 SPI only allows transimitting 8 bits at a time. It's also
//! double-buffered, which for us means we get 8 SPI-clock cycles of
//! leeway in the code timing if we want to hit max throughput, which should
//! be plenty. However it also impacts how reading data works. At the start
//! of a transmission, the first byte of data received is delayed by one byte.
//! Basically, there's three scenarios
//! - You only want to write data. You just write as normal
//! - You only want to read data. At the start of transmission you write 2
//!   bytes without a read inbetween. At the end, you read 2 bytes without
//!   a write inbetween
//! - You want to read and write data. You delay reading until after the
//!   second byte is transmitted.
//! 
//! Note also that this affects how the READY event is fired. READY is based
//! on when valid data is available in the RXD register, so it will not fire
//! until after writing a second byte into TXD (prompting the first valid byte
//! to be moved out of the buffer and into RXD).
//!
//!
//!
//! # Interrupt Control
//!
//! Interrupts are turned on by writing 1 to INTENSET and turned off by
//! writing 1 to INTENCLR, which is a different register.
//!
//! 
//! # Pin Selection
//!
//! The SPI can use any pins 0-31 for any function. This differs from other
//! controllers, which often have specific pins tied to specific SPI devices.
//! Thus, the desired SPI pins are passed in as part of initialization
//!
//!
//! # Register Initialization
//!
//! The SPI shares some address space with other peripherals (like TWI).
//! Nordic recommends fully intializing all SPI-related registers, as values
//! are not reset when changing the peripheral mode.
//!
//!
//! # Why we use spi0 in the code
//!
//! We use spi0 for everything because its the only actual spi module. the pac
//! crate just re-exports spi0 as spi1/spi2 so you can use those as aliases.
//! The register block is really what determines which spi we're using.

#![no_std]

use nrf52832_pac as device;

pub struct Spi {
    reg: &'static device::spi0::RegisterBlock,
}

impl From<&'static device::spi0::RegisterBlock> for Spi {
    fn from(reg: &'static device::spi0::RegisterBlock) -> Self {
        Self { reg }
    }
}

impl Spi {

    // We take miso/mosi/sck as u32 instead of the respective WHATEVER_A
    // types because the WHATEVER_A enums only have the disconnected
    // value in them.
    pub fn initialize(
        &mut self,
        frequency: device::spi0::frequency::FREQUENCY_A,
        order: device::spi0::config::ORDER_A,
        cpha: device::spi0::config::CPHA_A,
        cpol: device::spi0::config::CPOL_A,
        miso_pin: u32,
        mosi_pin: u32,
        sck_pin: u32
    ) {
        // Expected preconditions:
        // - GPIOs configured to proper AF etc - we cannot do this, because we
        // cannot presume to have either direct GPIO access _or_ IPC access.
        // - Other peripherals sharing the SPI device's address space have
        // been turned off if they were previously in use. There is a finite
        // set of other peripherals that share the address space so technically
        // we could turn them all off here just in case, but we don't right now.

        // nRF52832 only has 32 pins, pin selection must be in range 0-31
        assert!(miso_pin <= 31);
        assert!(mosi_pin <= 31);
        assert!(sck_pin <= 31);


        // We need to initialize the whole register block
        // we clear events register in case there's stuff left over
        self.reg.intenclr.write(|w| w.ready().clear());
        self.reg.enable.write(|w| w.enable().disabled());

        // bits() calls are unsafe, but that's how you put the data in.
        self.reg.psel.miso.write(|w| unsafe { w.pselmiso().bits(miso_pin) });
        self.reg.psel.mosi.write(|w| unsafe { w.pselmosi().bits(mosi_pin) });
        self.reg.psel.sck.write(|w| unsafe { w.pselsck().bits(sck_pin) });

        self.reg.frequency.write(|w| w.frequency().variant(frequency));

        #[rustfmt::skip]
        self.reg.config.write(|w| {
            w
                .order().variant(order)
                .cpha().variant(cpha)
                .cpol().variant(cpol)
        });
    }

    pub fn enable(&mut self) {
        self.reg.enable.modify(|_, w| w.enable().enabled());
    }

    pub fn start(&mut self) {
        // Don't actually need to do anything.
    }

    pub fn is_read_ready(&self) -> bool {
        self.reg.events_ready.read().bits() != 0
    }

    /// Stuffs one byte of data into the SPI TX register.
    ///
    /// Preconditions:
    ///
    /// - There must be room for a byte in TXD. There's not really a way to
    ///   check this, so just, don't mess it up :).
    pub fn send8(&mut self, byte: u8) {
            // There's no "safe" way to put data into txd. thanks pac crate.
        self.reg.txd.write(|w| unsafe { w.txd().bits(byte) });
    }

    /// Pulls one byte of data from the SPI RX register. Also clears the
    /// ready event
    ///
    /// Preconditions:
    ///
    /// - There must be at least one byte of data in the receive register
    ///   (check is_read_ready). Otherwise you'll just get some undefined data
    pub fn recv8(&mut self) -> u8 {
        self.reg.rxd.read().rxd().bits()
    }

    pub fn end(&mut self) {
        // Turn off interrupt enables.
        self.reg.intenclr.write(|w| w.ready().clear());
    }

    pub fn enable_transfer_interrupts(&mut self) {
        self.reg.intenset.write(|w| w.ready().set());
    }

    pub fn disable_transfer_interrupts(&mut self) {
        self.reg.intenclr.write(|w| w.ready().clear());
    }
}
