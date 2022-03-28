// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the nRF52832 GPIO

#![no_std]
#![no_main]

use drv_nrf52832_gpio_api::{GpioError};
use idol_runtime::RequestError;
use userlib::*;
use nrf52832_pac as device;

use drv_nrf52832_gpio_common::{
    Mode, OutputType, Pull
};

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
    p0: &'a device::p0::RegisterBlock,
}

impl idl::InOrderGPIOImpl for ServerImpl<'_> {

    /// Dumps a config into a pin's configuration register, no questions asked.
    fn gpio_configure_raw(
        &mut self,
        _: &RecvMessage,
        pin: u8,
        config: u32,
    ) -> Result<(), RequestError<GpioError>> {
        assert!(pin <= 31);
        self.p0.pin_cnf[pin as usize].write(|w| unsafe { w.bits(config) });
        Ok(())
    }

    /// A delectable gpio configuration experience fit for royalty
    fn gpio_configure(
        &mut self,
        _: &RecvMessage,
        pin: u8,
        mode: Mode,
        output_type: OutputType,
        pull: Pull,
    ) -> Result<(), RequestError<GpioError>> {
        assert!(pin <= 31);
        let dir_a = match mode {
            Mode::Input | Mode::DisconnectedInput => device::p0::pin_cnf::DIR_A::INPUT,
            Mode::Output => device::p0::pin_cnf::DIR_A::OUTPUT
        };

        // I don't think this matters in output mode but might as well
        // be explicit about it
        let input_a = match mode {
            Mode::Input => device::p0::pin_cnf::INPUT_A::CONNECT,
            Mode::Output | Mode::DisconnectedInput => device::p0::pin_cnf::INPUT_A::DISCONNECT,
        };

        let pull_a = match pull {
            Pull::None => nrf52832_pac::p0::pin_cnf::PULL_A::DISABLED,
            Pull::Up => nrf52832_pac::p0::pin_cnf::PULL_A::PULLUP,
            Pull::Down => nrf52832_pac::p0::pin_cnf::PULL_A::PULLDOWN,
        };

        // We should make this more configurable later, but note that it can only handle three
        // high-drive pins at once so should ensure that somewhere
        let drive_a = match output_type {
            OutputType::PushPull => nrf52832_pac::p0::pin_cnf::DRIVE_A::S0S1,
            OutputType::OpenDrain => nrf52832_pac::p0::pin_cnf::DRIVE_A::S0D1
        };

        let sense_a = nrf52832_pac::p0::pin_cnf::SENSE_A::DISABLED;

        self.p0.pin_cnf[pin as usize].write(|w| {
            w
                .dir().variant(dir_a)
                .input().variant(input_a)
                .pull().variant(pull_a)
                .drive().variant(drive_a)
                .sense().variant(sense_a)
        });
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
    use super::{GpioError, Mode, OutputType, Pull};

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
