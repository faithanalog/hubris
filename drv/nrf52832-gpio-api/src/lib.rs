// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the STM32xx SYS server.

#![no_std]

use userlib::*;

pub use drv_nrf52832_gpio_common::{
    Mode, OutputType, Pull
};

#[derive(Copy, Clone, Debug)]
#[repr(u32)]
pub enum GpioError {
    BadArg = 2,
}

impl From<GpioError> for u32 {
    fn from(rc: GpioError) -> Self {
        rc as u32
    }
}

impl From<GpioError> for u16 {
    fn from(rc: GpioError) -> Self {
        rc as u16
    }
}

impl TryFrom<u32> for GpioError {
    type Error = ();

    fn try_from(x: u32) -> Result<Self, Self::Error> {
        match x {
            2 => Ok(GpioError::BadArg),
            _ => Err(()),
        }
    }
}

impl GPIO {
    /// Configures the pin as high-impedance digital input, with
    /// optional pull resistor.
    pub fn gpio_configure_input(
        &self,
        pin: u8,
        pull: Pull,
    ) -> Result<(), GpioError> {
        self.gpio_configure(
            pin,
            Mode::Input,
            OutputType::PushPull,
            pull
        )
    }

    /// Configures the pin as digital GPIO output, either
    /// push-pull or open-drain, with adjustable slew rate filtering and pull
    /// resistor.
    pub fn gpio_configure_output(
        &self,
        pin: u8,
        output_type: OutputType,
        pull: Pull,
    ) -> Result<(), GpioError> {
        self.gpio_configure(
            pin,
            Mode::Output,
            output_type,
            pull
        )
    }


    /// Configure the pin as a disconnected input. This is as close as you get
    /// to turning the pin off and saves power
    pub fn gpio_configure_disconnected(
        &self,
        pin: u8,
    ) -> Result<(), GpioError> {
        self.gpio_configure(
            pin,
            Mode::DisconnectedInput,
            OutputType::PushPull, // doesn't matter
            Pull::None
        )
    }


    /// Sets some pins high.
    pub fn gpio_set(&self, pinset: u32) -> Result<(), GpioError> {
        self.gpio_set_reset(pinset, 0)
    }

    /// Resets some pins low.
    pub fn gpio_reset(&self, pinset: u32) -> Result<(), GpioError> {
        self.gpio_set_reset(0, pinset)
    }

    /// Sets some pins based on `flag` -- high if `true`, low if `false`.
    pub fn gpio_set_to(
        &self,
        pinset: u32,
        flag: bool,
    ) -> Result<(), GpioError> {
        self.gpio_set_reset(
            if flag { pinset } else { 0 },
            if flag { 0 } else { pinset },
        )
    }

    pub fn gpio_read(&self, pinset: u32) -> Result<u32, GpioError> {
        Ok(self.gpio_read_input()? & pinset)
    }
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
