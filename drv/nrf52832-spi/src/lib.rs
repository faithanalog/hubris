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
//! be plenty.
//!
//! This means that at the start of a transaction you'll have two writes
//! without a read in between, and at the end you'll have two reads without
//! a write.
//!
//! You don't actually have to do this though. You can transmit a byte and
//! then wait until you receive a byte before putting the next byte into the
//! transmit buffer. You won't sustain the full transfer rate doing this.
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
//! Pins must also be configured with the GPIO register such that
//! - miso is an input
//! - mosi is an output
//! - sck is an output
//! - all the pins are pushpull with no pull-up.
//! 
//! GPIO doesn't have a concept of alternate functions though, so that's all
//! you need to do.
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

    /// Initialize the SPI device to something reasonably. nRF wants us to fully configure
    /// peripherals before using them, so this function explicitly disables interrupts, and
    /// configures SPI mode 0 at the lowest frequency.
    pub fn initialize(&mut self) {
        self.reg.enable.write(|w| w.enable().disabled());
        self.reg.intenclr.write(|w| w.ready().clear());
        self.configure_transmission_parameters(
            device::spi0::frequency::FREQUENCY_A::K125,
            device::spi0::config::ORDER_A::MSBFIRST,
            device::spi0::config::CPHA_A::LEADING,
            device::spi0::config::CPOL_A::ACTIVEHIGH,
        );
    }

    /// Reconfigure the SPI pinout and enable it. nRF docs indicate that the
    /// pinout doesn't persist after the peripheral is disabled.
    pub fn enable(
        &mut self,
        miso_pin: u8,
        mosi_pin: u8,
        sck_pin: u8
    ) {
        // Expected preconditions:
        // - SPI is disabled.
        // - Other peripherals sharing the SPI device's address space have
        // been turned off if they were previously in use. There is a finite
        // set of other peripherals that share the address space so technically
        // we could turn them all off here just in case, but we don't right now.
        // - miso/mosi/sck have been correctly configured in GPIO
        //   - They all should be push-pull, with no pull-up
        //   - miso is an input
        //   - mosi/sck are outputs

        // nRF52832 only has 32 pins, pin selection must be in range 0-31
        assert!(miso_pin <= 31);
        assert!(mosi_pin <= 31);
        assert!(sck_pin <= 31);

        // bits() calls are unsafe, but that's how you put the data in.
        self.reg.psel.miso.write(|w| unsafe { w.pselmiso().bits(miso_pin as u32) });
        self.reg.psel.mosi.write(|w| unsafe { w.pselmosi().bits(mosi_pin as u32) });
        self.reg.psel.sck.write(|w| unsafe { w.pselsck().bits(sck_pin as u32) });

        self.reg.enable.write(|w| w.enable().enabled());
    }

    /// Disables the SPI device. Do not use it again without calling
    /// `reconfigure_and_enable`.
    pub fn disable(&mut self) {
        self.reg.enable.write(|w| w.enable().disabled());
    }

    /// Configure transmission parameters 
    pub fn configure_transmission_parameters(
        &mut self,
        frequency: device::spi0::frequency::FREQUENCY_A,
        order: device::spi0::config::ORDER_A,
        cpha: device::spi0::config::CPHA_A,
        cpol: device::spi0::config::CPOL_A,
    ) {
        self.reg.frequency.write(|w| w.frequency().variant(frequency));

        #[rustfmt::skip]
        self.reg.config.write(|w| {
            w
                .order().variant(order)
                .cpha().variant(cpha)
                .cpol().variant(cpol)
        });
    }

    /// Start a transaction. This just clears out the read buffer and the ready
    /// flag.
    pub fn start(&mut self) {
        // Read a byte just to clear the read event in case its set
        let _ = self.recv8();
    }

    /// Checks if the ready flag is set. The ready flag is set whenever the SPI
    /// peripheral provides a new byte in the RXD read-register, and remains set
    /// until we clear it. recv8 clears this.
    pub fn is_read_ready(&self) -> bool {
        self.reg.events_ready.read().bits() != 0
    }

    /// Stuffs one byte of data into the SPI TX register.
    ///
    /// SPI is double buffered, so you can write two bytes immediatley at the
    /// start of a transaction to keep stuff moving along smoothly. After that,
    /// wait for `is_read_ready()`, then call `recv8()` before sending another
    /// byte.
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
    ///   (check `is_read_ready()`). Otherwise you'll just get some undefined data
    pub fn recv8(&mut self) -> u8 {
        // the spec sheet is not terribly clear on whether you have to
        // manually zero the events_ready register. I (artemis) experimented
        // with it and found that you do in fact need to do this.
        self.reg.events_ready.write(|w| unsafe { w.bits(0) });
        let b = self.reg.rxd.read().rxd().bits();
        b
    }

    pub fn enable_transfer_interrupts(&mut self) {
        self.reg.intenset.write(|w| w.ready().set());
    }

    pub fn disable_transfer_interrupts(&mut self) {
        self.reg.intenclr.write(|w| w.ready().clear());
    }
}
