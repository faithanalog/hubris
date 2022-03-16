// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use super::*;
use drv_spi_api::{self as spi_api, SpiDevice, SpiError};
use drv_stm32xx_sys_api::{self as sys_api, GpioError, Sys};

/// Ecp5Spi is the simplest implementation of the Ecp5Impl interface using the
/// SPI and Sys APIs. It assumes the PROGRAM_N, INIT_N and DONE signals are
/// directly connected to GPIO pins.

pub struct Ecp5Spi {
    pub sys: Sys,
    pub spi: SpiDevice,
    pub done: sys_api::PinSet,
    pub init_n: sys_api::PinSet,
    pub program_n: sys_api::PinSet,
}

/// BSP Error type, with conversion from GpioError and SpiError.
#[derive(Copy, Clone, Debug)]
pub enum Ecp5SpiError {
    GpioError(GpioError),
    SpiError(SpiError),
}

impl From<GpioError> for Ecp5SpiError {
    fn from(e: GpioError) -> Self {
        Self::GpioError(e)
    }
}

impl From<SpiError> for Ecp5SpiError {
    fn from(e: SpiError) -> Self {
        Self::SpiError(e)
    }
}

impl From<Ecp5SpiError> for u8 {
    fn from(e: Ecp5SpiError) -> u8 {
        match e {
            Ecp5SpiError::GpioError(e) => match e {
                GpioError::BadArg => 2,
            },
            Ecp5SpiError::SpiError(e) => match e {
                SpiError::BadTransferSize => 3,
                SpiError::ServerRestarted => 4,
                SpiError::NothingToRelease => 5,
                SpiError::BadDevice => 6,
                SpiError::DataOverrun => 7,
            },
        }
    }
}

impl From<Ecp5SpiError> for Ecp5Error {
    fn from(e: Ecp5SpiError) -> Ecp5Error {
        Ecp5Error::ImplError(u8::from(e))
    }
}

/*
impl core::convert::TryFrom<u32> for Ecp5SpiError {
    type Error = ();
    fn try_from(x: u32) -> Result<Self, Self::Error> {
        match x {
            2 => Ok(Ecp5SpiError::GpioError(GpioError::BadArg))),
            3 => Ok(Ecp5SpiError::SpiError(SpiError::BadTransferSize))),
            4 => Ok(Ecp5SpiError::SpiError(SpiError::ServerRestarted))),
            5 => Ok(Ecp5SpiError::SpiError(SpiError::NothingToRelease))),
            6 => Ok(Ecp5SpiError::SpiError(SpiError::BadDevice))),
            7 => Ok(Ecp5SpiError::SpiError(SpiError::DataOverrun))),
            _ => Err(()),
        }
    }
}
*/

impl Ecp5Impl for Ecp5Spi {
    type Error = Ecp5SpiError;

    fn program_n(&self) -> Result<bool, Ecp5SpiError> {
        Ok(self.sys.gpio_read(self.program_n)? != 0)
    }

    fn set_program_n(&self, asserted: bool) -> Result<(), Ecp5SpiError> {
        Ok(self.sys.gpio_set_to(self.program_n, asserted)?)
    }

    fn init_n(&self) -> Result<bool, Ecp5SpiError> {
        Ok(self.sys.gpio_read(self.init_n)? != 0)
    }

    fn set_init_n(&self, asserted: bool) -> Result<(), Ecp5SpiError> {
        Ok(self.sys.gpio_set_to(self.init_n, !asserted)?)
    }

    fn done(&self) -> Result<bool, Ecp5SpiError> {
        Ok(self.sys.gpio_read(self.done)? != 0)
    }

    fn set_done(&self, asserted: bool) -> Result<(), Ecp5SpiError> {
        Ok(self.sys.gpio_set_to(self.done, asserted)?)
    }

    fn write_command(&self, c: Command) -> Result<(), Ecp5SpiError> {
        let buffer: [u8; 4] = [c as u8, 0, 0, 0];
        Ok(self.spi.write(&buffer)?)
    }

    fn read(&self, b: &mut [u8]) -> Result<(), Ecp5SpiError> {
        Ok(self.spi.read(b)?)
    }

    fn write(&self, b: &[u8]) -> Result<(), Ecp5SpiError> {
        Ok(self.spi.write(b)?)
    }

    fn lock(&self) -> Result<(), Ecp5SpiError> {
        Ok(self.spi.lock(spi_api::CsState::Asserted)?)
    }

    fn release(&self) -> Result<(), Ecp5SpiError> {
        Ok(self.spi.release()?)
    }
}

impl Ecp5Spi {
    pub fn configure_gpio(&self) {
        use sys_api::*;

        self.sys.gpio_set(self.done).unwrap();
        self.sys
            .gpio_configure_output(
                self.done,
                OutputType::OpenDrain,
                Speed::Low,
                Pull::Up,
            )
            .unwrap();

        self.sys.gpio_set(self.init_n).unwrap();
        self.sys
            .gpio_configure_output(
                self.init_n,
                OutputType::OpenDrain,
                Speed::Low,
                Pull::Up,
            )
            .unwrap();

        self.sys.gpio_set(self.program_n).unwrap();
        self.sys
            .gpio_configure_output(
                self.program_n,
                OutputType::OpenDrain,
                Speed::Low,
                Pull::None,
            )
            .unwrap();
    }
}
