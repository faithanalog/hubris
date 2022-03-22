// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use userlib::*;
use drv_nrf52832_gpio_api as iface;

task_slot!(GPIO, gpio);

#[export_name = "main"]
pub fn main() -> ! {
    const TIMER_NOTIFICATION: u32 = 1;
    const INTERVAL: u64 = 3000;
    const BACKLIGHT_HIGH: u8 = 23;

    let mut response: u32 = 0;

    let gpio = iface::Sys::from(GPIO.get_task_id());

    gpio.gpio_configure_output(BACKLIGHT_HIGH, iface::OutputType::PushPull, iface::Pull::None);

    let mut current = 0;
    let mut msg = [0; 16];
    let mut dl = INTERVAL;
    sys_set_timer(Some(dl), TIMER_NOTIFICATION);
    loop {
        let msginfo = sys_recv_open(&mut msg, TIMER_NOTIFICATION);
        
        gpio.gpio_toggle(1 << BACKLIGHT_HIGH);

        if msginfo.sender == TaskId::KERNEL {
            dl += INTERVAL;
            sys_set_timer(Some(dl), TIMER_NOTIFICATION);
        }
    }
}
