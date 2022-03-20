// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the STM32xx RCC and GPIO blocks, combined for compactness.

#![no_std]
#![no_main]

use drv_nrf52832_gpio_common::{server::get_gpio_regs, Port};
use drv_nrf52832_gpio_api::{GpioError, Group};
use idol_runtime::RequestError;
use userlib::*;

trait FlagsRegister {
    /// Sets bit `index` in the register, preserving other bits.
    ///
    /// # Safety
    ///
    /// This is unsafe because, in theory, you might be able to find a register
    /// where setting a bit can imperil memory safety. It is your responsibility
    /// not to use this on such registers.
    unsafe fn set_bit(&self, index: u8);

    /// Clears bit `index` in the register, preserving other bits.
    ///
    /// # Safety
    ///
    /// This is unsafe because, in theory, you might be able to find a register
    /// where clearing a bit can imperil memory safety. It is your
    /// responsibility not to use this on such registers.
    unsafe fn clear_bit(&self, index: u8);
}

impl<S> FlagsRegister for pac::Reg<S>
where
    S: pac::RegisterSpec<Ux = u32> + pac::Readable + pac::Writable,
{
    unsafe fn set_bit(&self, index: u8) {
        self.modify(|r, w| unsafe { w.bits(r.bits() | 1 << index) });
    }

    unsafe fn clear_bit(&self, index: u8) {
        self.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << index)) });
    }
}

#[export_name = "main"]
fn main() -> ! {
    // From thin air, pluck a pointer to the p0 register block.
    //
    // Safety: this is needlessly unsafe in the API. The p0 is essentially a
    // static, and we access it through a & reference so aliasing is not a
    // concern. Were it literally a static, we could just reference it.
    let p0 = unsafe { &*device::P0::ptr() };


    // Field messages.
    let mut buffer = [0u8; idl::INCOMING_SIZE];
    let mut server = ServerImpl { p0 };
    loop {
        idol_runtime::dispatch(&mut buffer, &mut server);
    }
}

struct ServerImpl<'a> {
    p0: &'a device::rcc::RegisterBlock,
}

impl ServerImpl<'_> {
    fn unpack_raw(raw: u32) -> Result<(Group, u8), RequestError<RccError>> {
        let bit: u8 = (raw & 0x1F) as u8;
        let bus =
            Group::from_u32(raw >> 5).ok_or(RccError::NoSuchPeripheral)?;
        // TODO: this lets people refer to bit indices that are not included in
        // the Peripheral enum, which is not great. Fixing this by deriving
        // FromPrimitive for Peripheral results in _really expensive_ checking
        // code. We could do better.
        Ok((bus, bit))
    }
}

impl idl::InOrderSysImpl for ServerImpl<'_> {
    fn gpio_configure_raw(
        &mut self,
        _: &RecvMessage,
        port: Port,
        pins: u16,
        packed_attributes: u16,
    ) -> Result<(), RequestError<GpioError>> {
        unsafe { get_gpio_regs(port) }.configure(pins, packed_attributes);
        Ok(())
    }

    fn gpio_set_reset(
        &mut self,
        _: &RecvMessage,
        set_pins: u16,
        reset_pins: u16,
    ) -> Result<(), RequestError<GpioError>> {
        unsafe { get_gpio_regs(port) }.set_reset(set_pins, reset_pins);
        Ok(())
    }

    fn gpio_toggle(
        &mut self,
        _: &RecvMessage,
        port: Port,
        pins: u16,
    ) -> Result<(), RequestError<GpioError>> {
        unsafe { get_gpio_regs(port) }.toggle(pins);
        Ok(())
    }

    fn gpio_read_input(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<GpioError>> {
        Ok(unsafe { get_gpio_regs(port) }.read())
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "family-stm32g0")] {
        fn enable_clock(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Iop => unsafe { rcc.iopenr.set_bit(bit) },
                Group::Ahb => unsafe { rcc.ahbenr.set_bit(bit) },
                Group::Apb1 => unsafe { rcc.apbenr1.set_bit(bit) },
                Group::Apb2 => unsafe { rcc.apbenr2.set_bit(bit) },
            }
        }

        fn disable_clock(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Iop => unsafe { rcc.iopenr.clear_bit(bit) },
                Group::Ahb => unsafe { rcc.ahbenr.clear_bit(bit) },
                Group::Apb1 => unsafe { rcc.apbenr1.clear_bit(bit) },
                Group::Apb2 => unsafe { rcc.apbenr2.clear_bit(bit) },
            }
        }

        fn enter_reset(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Iop => unsafe { rcc.ioprstr.set_bit(bit) },
                Group::Ahb => unsafe { rcc.ahbrstr.set_bit(bit) },
                Group::Apb1 => unsafe { rcc.apbrstr1.set_bit(bit) },
                Group::Apb2 => unsafe { rcc.apbrstr2.set_bit(bit) },
            }
        }

        fn leave_reset(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Iop => unsafe { rcc.ioprstr.clear_bit(bit) },
                Group::Ahb => unsafe { rcc.ahbrstr.clear_bit(bit) },
                Group::Apb1 => unsafe { rcc.apbrstr1.clear_bit(bit) },
                Group::Apb2 => unsafe { rcc.apbrstr2.clear_bit(bit) },
            }
        }

    } else if #[cfg(feature = "family-stm32h7")] {
        fn enable_clock(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Ahb1 => unsafe { rcc.ahb1enr.set_bit(bit) },
                Group::Ahb2 => unsafe { rcc.ahb2enr.set_bit(bit) },
                Group::Ahb3 => unsafe { rcc.ahb3enr.set_bit(bit) },
                Group::Ahb4 => unsafe { rcc.ahb4enr.set_bit(bit) },
                Group::Apb1L => unsafe { rcc.apb1lenr.set_bit(bit) },
                Group::Apb1H => unsafe { rcc.apb1henr.set_bit(bit) },
                Group::Apb2 => unsafe { rcc.apb2enr.set_bit(bit) },
                Group::Apb3 => unsafe { rcc.apb3enr.set_bit(bit) },
                Group::Apb4 => unsafe { rcc.apb4enr.set_bit(bit) },
            }
        }

        fn disable_clock(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Ahb1 => unsafe { rcc.ahb1enr.clear_bit(bit) },
                Group::Ahb2 => unsafe { rcc.ahb2enr.clear_bit(bit) },
                Group::Ahb3 => unsafe { rcc.ahb3enr.clear_bit(bit) },
                Group::Ahb4 => unsafe { rcc.ahb4enr.clear_bit(bit) },
                Group::Apb1L => unsafe { rcc.apb1lenr.clear_bit(bit) },
                Group::Apb1H => unsafe { rcc.apb1henr.clear_bit(bit) },
                Group::Apb2 => unsafe { rcc.apb2enr.clear_bit(bit) },
                Group::Apb3 => unsafe { rcc.apb3enr.clear_bit(bit) },
                Group::Apb4 => unsafe { rcc.apb4enr.clear_bit(bit) },
            }
        }

        fn enter_reset(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Ahb1 => unsafe { rcc.ahb1rstr.set_bit(bit) },
                Group::Ahb2 => unsafe { rcc.ahb2rstr.set_bit(bit) },
                Group::Ahb3 => unsafe { rcc.ahb3rstr.set_bit(bit) },
                Group::Ahb4 => unsafe { rcc.ahb4rstr.set_bit(bit) },
                Group::Apb1L => unsafe { rcc.apb1lrstr.set_bit(bit) },
                Group::Apb1H => unsafe { rcc.apb1hrstr.set_bit(bit) },
                Group::Apb2 => unsafe { rcc.apb2rstr.set_bit(bit) },
                Group::Apb3 => unsafe { rcc.apb3rstr.set_bit(bit) },
                Group::Apb4 => unsafe { rcc.apb4rstr.set_bit(bit) },
            }
        }

        fn leave_reset(
            rcc: &device::rcc::RegisterBlock,
            group: Group,
            bit: u8,
        ) {
            match group {
                Group::Ahb1 => unsafe { rcc.ahb1rstr.clear_bit(bit) },
                Group::Ahb2 => unsafe { rcc.ahb2rstr.clear_bit(bit) },
                Group::Ahb3 => unsafe { rcc.ahb3rstr.clear_bit(bit) },
                Group::Ahb4 => unsafe { rcc.ahb4rstr.clear_bit(bit) },
                Group::Apb1L => unsafe { rcc.apb1lrstr.clear_bit(bit) },
                Group::Apb1H => unsafe { rcc.apb1hrstr.clear_bit(bit) },
                Group::Apb2 => unsafe { rcc.apb2rstr.clear_bit(bit) },
                Group::Apb3 => unsafe { rcc.apb3rstr.clear_bit(bit) },
                Group::Apb4 => unsafe { rcc.apb4rstr.clear_bit(bit) },
            }
        }

    } else {
        compiler_error!("unsupported SoC family");
    }
}

mod idl {
    use super::{GpioError, Port, RccError};

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
