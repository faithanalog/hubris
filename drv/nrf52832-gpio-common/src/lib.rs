// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! GPIO-related things for nRF
//!
//! Notably, nRF52 chips only have one GPIO bank, and it doesn't need to
//! be configured for alternate functions. In contrast with the stm32
//! code, we use u32 as the pinset directly and only need some cute
//! enums for mode/output type/speed here
//!
//! Also, this is very incomplete. We don't have any support for the
//! high-drive GPIO modes, for example, or detect/sense. feel free to add it
//! if you need it, but at the time of writing it's not my priority.

#![no_std]

use userlib::FromPrimitive;


/// Possible modes for a GPIO pin.
///
/// Currently doens't implement the analog-ish mode
#[derive(Copy, Clone, Debug, PartialEq, FromPrimitive)]
pub enum Mode {
    /// Digital input. This activates a Schmitt trigger on the pin, which is
    /// great for receiving digital signals, but can burn a lot of current if
    /// faced with signals intermediate between 1 and 0. As a result, to treat a
    /// pin as unused, set it to `Analog`.
    Input = 0b00,
    /// Software-controlled output. Values written to the corresponding bit of
    /// the ODR register will control the pin's driver.
    Output = 0b01,
    /// Input mode, but the input buffer is disconnected. This is as close as
    /// you get to turning the pin off, and is mainly for power savings
    DisconnectedInput = 0b10,
}

/// Drive modes for a GPIO pin.
#[derive(Copy, Clone, Debug, PartialEq, FromPrimitive)]
pub enum OutputType {
    /// DRIVE = S0S1 (0 = low, 1 = high)
    PushPull = 0,
    /// DRIVE = S0D1 (0 = low, 1 = disconnect)
    OpenDrain = 1,
}

/// Settings for the switchable weak pull resistors on GPIO pins.
///
/// Note that the pull resistors apply in all modes, so, you can apply these to
/// an input, and you will want to turn them off for `Analog`.
#[derive(Copy, Clone, Debug, PartialEq, FromPrimitive)]
pub enum Pull {
    /// Both resistors off.
    None = 0b00,
    /// Weak pull up.
    Up = 0b01,
    /// Weak pull down.
    Down = 0b10,
}

