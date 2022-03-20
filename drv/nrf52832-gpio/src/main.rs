// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the nRF52832 GPIO

#![no_std]
#![no_main]

use drv_nrf52832_gpio_api::{GpioError};
use idol_runtime::RequestError;
use userlib::*;

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

impl idl::InOrderSysImpl for ServerImpl<'_> {

    /// Dumps a config into a pin's configuration register, no questions asked.
    fn gpio_configure_raw(
        &mut self,
        _: &RecvMessage,
        pin: u8,
        config: u32,
    ) -> Result<(), RequestError<GpioError>> {
        assert!(pin <= 31);
        self.p0.pin_cnf[pin].write(|w| unsafe { w.bits(config) });
        Ok(())
    }

    fn gpio_set_reset(
        &mut self,
        _: &RecvMessage,
        set_pins: u32,
        reset_pins: u32,
    ) -> Result<(), RequestError<GpioError>> {
        self.p0.outset.write(|w| unsafe { w.bits(set_pins) });
        self.p0.outclr.write(|w| unsafe { w.bits(reset_pins) });
        Ok(())
    }

    fn gpio_toggle(
        &mut self,
        _: &RecvMessage,
        pins: u32,
    ) -> Result<(), RequestError<GpioError>> {
        let pin_state = self.p0.out.read().bits();
        let new_state = pin_state ^ pins;
        self.p0.out.write(|w| unsafe { w.bits(new_state) });
        Ok(())
    }

    fn gpio_read_input(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<GpioError>> {
        Ok(self.p0.in_.read().bits())
    }
}

mod idl {
    use super::{GpioError, Port, RccError};

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
