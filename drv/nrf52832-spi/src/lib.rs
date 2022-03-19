// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the nRF52832 SPI, in host mode.
//!
//! This is the core logic, separated from the IPC server.
//!
//! The nRF52 has a very simple SPI compared to some other MCUs.
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
//! # Transfer size
//!
//! nRF52 SPI only allows transimitting 8 bits at a time. It's also
//! double-buffered, which for us just means we get 8 SPI-clock cycles of
//! leeway in the code timing if we want to hit max throughput, which should
//! be plenty.
//!
//!
//! # Interrupt Control
//!
//! Interrupts are turned on by writing 1 to INTENSET and turned off by
//! writing 1 to INTENCLR, which is a different register. why? ask Nordic.
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
        miso_pin :: u32,
        mosi_pin :: u32,
        sck_pin :: u32
    ) {
        // Expected preconditions:
        // - GPIOs configured to proper AF etc - we cannot do this, because we
        // cannot presume to have either direct GPIO access _or_ IPC access.
        // - Other peripherals sharing the SPI device's address space have
        // been turned off if they were previously in use.

        // nRF52832 only has 32 pins, pin selection must be in range 0-31
        assert!(miso_pin >= 0 && miso_pin <= 31);
        assert!(mosi_pin >= 0 && mosi_pin <= 31);
        assert!(sck_pin >= 0 && sck_pin <= 31);


        // We need to initialize the whole register block
        // we clear events register in case there's stuff left over
        self.reg.events_ready.write(|w| w.bits(0));
        self.reg.intenclr.write(|w| w.intenclr().clear());
        self.reg.enable.write(|w| w.enable().disabled());
        self.reg.psel.miso.write(|w| w.pselsck().bits(miso_pin));
        self.reg.psel.mosi.write(|w| w.pselsck().bits(mosi_pin));
        self.reg.psel.sck.write(|w| w.pselsck().bits(sck_pin));
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
        // TODO anything else?
    }

    pub fn start(&mut self) {
        // I dont know if theres a difference between enabling and starting. maybe
        // interrupts?
        // we clear events register in case there's stuff left over. should we
        // do that here? i dunno.
        self.reg.events_ready.write(|w| w.bits(0));
        // TODO anything else?
    }

    pub fn can_rx_word(&self) -> bool {
        let sr = self.reg.sr.read();
        sr.rxwne().bit()
    }

    pub fn can_rx_byte(&self) -> bool {
        let sr = self.reg.sr.read();
        sr.rxwne().bit() || sr.rxplvl().bits() != 0
    }

    pub fn can_tx_frame(&self) -> bool {
        let sr = self.reg.sr.read();
        sr.txp().bit()
    }

    pub fn send32(&mut self, bytes: u32) {
        self.reg.txdr.write(|w| w.txdr().bits(bytes));
    }

    pub fn end_of_transmission(&self) -> bool {
        let sr = self.reg.sr.read();
        sr.eot().bit()
    }

    pub fn set_data_line_swap(&self, flag: bool) {
        self.reg.config.modify(|_, w| w.ioswp().bit(flag));
    }

    /// Stuffs one byte of data into the SPI TX FIFO.
    ///
    /// Preconditions:
    ///
    /// - There must be room for a byte in the TX FIFO (call `can_tx_frame` to
    ///   check, or call this in response to a TXP interrupt).
    pub fn send8(&mut self, byte: u8) {
        // The TXDR register can be accessed as a byte, halfword, or word. This
        // determines how many bytes are pushed in. stm32h7/svd2rust don't
        // understand this, and so we have to get a pointer to the byte portion
        // of the register manually and dereference it.

        // Because svd2rust didn't see this one coming, we cannot get a direct
        // reference to the VolatileCell within the wrapped Reg type of txdr,
        // nor will the Reg type agree to give us a pointer to its contents like
        // VolatileCell will, presumably to save us from ourselves. And thus we
        // must exploit the fact that VolatileCell is the only (non-zero-sized)
        // member of Reg, and in fact _must_ be for Reg to work correctly when
        // used to overlay registers in memory.

        // Safety: "Downcast" txdr to a pointer to its sole member, whose type
        // we know because of our unholy source-code-reading powers.
        let txdr: &vcell::VolatileCell<u32> =
            unsafe { core::mem::transmute(&self.reg.txdr) };
        // vcell is more pleasant and will happily give us the pointer we want.
        let txdr: *mut u32 = txdr.as_ptr();
        // As we are a little-endian machine it is sufficient to change the type
        // of the pointer to byte.
        let txdr8 = txdr as *mut u8;

        // Safety: we are dereferencing a pointer given to us by VolatileCell
        // (and thus UnsafeCell) using the same volatile access it would use.
        unsafe {
            txdr8.write_volatile(byte);
        }
    }

    pub fn recv32(&mut self) -> u32 {
        self.reg.rxdr.read().rxdr().bits()
    }

    /// Pulls one byte of data from the SPI RX FIFO.
    ///
    /// Preconditions:
    ///
    /// - There must be at least one byte of data in the FIFO (check using
    ///   `has_rx_byte` or call this in response to an RXP interrupt).
    ///
    /// - Frame size must be set to 8 bits or smaller. (Behavior if you write a
    ///   partial frame to the FIFO is not immediately clear from the
    ///   datasheet.)
    pub fn recv8(&mut self) -> u8 {
        // The RXDR register can be accessed as a byte, halfword, or word. This
        // determines how many bytes are pushed in. stm32h7/svd2rust don't
        // understand this, and so we have to get a pointer to the byte portion
        // of the register manually and dereference it.

        // See send8 for further rationale / ranting.

        // Safety: "Downcast" rxdr to a pointer to its sole member, whose type
        // we know because of our unholy source-code-reading powers.
        let rxdr: &vcell::VolatileCell<u32> =
            unsafe { core::mem::transmute(&self.reg.rxdr) };
        // vcell is more pleasant and will happily give us the pointer we want.
        let rxdr: *mut u32 = rxdr.as_ptr();
        // As we are a little-endian machine it is sufficient to change the type
        // of the pointer to byte.
        let rxdr8 = rxdr as *mut u8;

        // Safety: we are dereferencing a pointer given to us by VolatileCell
        // (and thus UnsafeCell) using the same volatile access it would use.
        unsafe { rxdr8.read_volatile() }
    }

    pub fn end(&mut self) {
        // Clear flags that tend to get set during transactions.
        self.reg.ifcr.write(|w| w.txtfc().set_bit());
        // Disable the transfer state machine.
        self.reg.cr1.modify(|_, w| w.spe().clear_bit());
        // Turn off interrupt enables.
        self.reg.ier.reset();

        // This is where we'd report errors (TODO). For now, just clear the
        // error flags, as they're sticky.
        self.reg.ifcr.write(|w| {
            w.ovrc()
                .set_bit()
                .udrc()
                .set_bit()
                .modfc()
                .set_bit()
                .tifrec()
                .set_bit()
        });
    }

    pub fn enable_transfer_interrupts(&mut self) {
        self.reg
            .ier
            .write(|w| w.txpie().set_bit().rxpie().set_bit().eotie().set_bit());
    }

    pub fn disable_can_tx_interrupt(&mut self) {
        self.reg.ier.modify(|_, w| w.txpie().clear_bit());
    }

    pub fn enable_can_tx_interrupt(&mut self) {
        self.reg.ier.modify(|_, w| w.txpie().set_bit());
    }

    pub fn check_eot(&self) -> bool {
        self.reg.sr.read().eot().is_completed()
    }

    pub fn clear_eot(&mut self) {
        self.reg.ifcr.write(|w| w.eotc().set_bit());
    }

    pub fn read_status(&self) -> u32 {
        self.reg.sr.read().bits()
    }

    pub fn check_overrun(&self) -> bool {
        self.reg.sr.read().ovr().is_overrun()
    }
}
