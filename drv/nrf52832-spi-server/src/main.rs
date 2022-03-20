// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Server task for the STM32H7 SPI peripheral.
//!
//! Currently this hardcodes the clock rate.
//!
//! See the `spi-api` crate for the protocol being implemented here.
//!
//! # Why is everything `spi1`
//!
//! As noted in the `stm32h7-spi` driver, the `stm32h7` PAC has decided that all
//! SPI types should be called `spi1`.

#![no_std]
#![no_main]

use drv_spi_api::*;
use idol_runtime::{
    LeaseBufReader, LeaseBufWriter, Leased, LenLimit, RequestError, R, W,
};
use ringbuf::*;

use nrf52832_pac as device;

use userlib::*;

use drv_nrf52832_spi as spi_core;

task_slot!(SYS, sys);

#[derive(Copy, Clone, PartialEq)]
enum Trace {
    Start(SpiOperation, (u16, u16)),
    Tx(u8),
    Rx(u8),
    WaitISR(u32),
    None,
}

ringbuf!(Trace, 64, Trace::None);

#[derive(Copy, Clone, Debug)]
struct LockState {
    task: TaskId,
    device_index: usize,
}

#[export_name = "main"]
fn main() -> ! {
    check_server_config();

    let sys = sys_api::Sys::from(SYS.get_task_id());

    let registers = unsafe { &*CONFIG.registers };

    let mut spi = spi_core::Spi::from(registers);

    spi.initialize(
        // TODO
    );

    // Configure all devices' CS pins to be deasserted (set).
    // We leave them in GPIO output mode from this point forward.
    // TODO
    for device in CONFIG.devices {
        sys.gpio_set_reset(device.cs.port, device.cs.pin_mask, 0)
            .unwrap();
        sys.gpio_configure(
            device.cs.port,
            device.cs.pin_mask,
            sys_api::Mode::Output,
            sys_api::OutputType::PushPull,
            sys_api::Speed::Low,
            sys_api::Pull::None,
            sys_api::Alternate::AF1, // doesn't matter in GPIO mode
        )
        .unwrap();
    }

    let mut server = ServerImpl {
        spi,
        sys,
        lock_holder: None,
        current_mux_index,
    };
    let mut incoming = [0u8; INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

struct ServerImpl {
    spi: spi_core::Spi,
    sys: sys_api::Sys,
    lock_holder: Option<LockState>,
    current_mux_index: usize,
}

impl InOrderSpiImpl for ServerImpl {
    fn recv_source(&self) -> Option<userlib::TaskId> {
        self.lock_holder.map(|s| s.task)
    }

    fn closed_recv_fail(&mut self) {
        // Welp, someone had asked us to lock and then died. Release the
        // lock.
        self.lock_holder = None;
    }

    fn read(
        &mut self,
        _: &RecvMessage,
        device_index: u8,
        dest: LenLimit<Leased<W, [u8]>, 65535>,
    ) -> Result<(), RequestError<SpiError>> {
        self.ready_writey(SpiOperation::read, device_index, None, Some(dest))
    }
    fn write(
        &mut self,
        _: &RecvMessage,
        device_index: u8,
        src: LenLimit<Leased<R, [u8]>, 65535>,
    ) -> Result<(), RequestError<SpiError>> {
        self.ready_writey(SpiOperation::write, device_index, Some(src), None)
    }
    fn exchange(
        &mut self,
        _: &RecvMessage,
        device_index: u8,
        src: LenLimit<Leased<R, [u8]>, 65535>,
        dest: LenLimit<Leased<W, [u8]>, 65535>,
    ) -> Result<(), RequestError<SpiError>> {
        self.ready_writey(
            SpiOperation::exchange,
            device_index,
            Some(src),
            Some(dest),
        )
    }

    /// Locks the spi device and asserts chip select
    fn lock(
        &mut self,
        rm: &RecvMessage,
        devidx: u8,
        cs_state: CsState,
    ) -> Result<(), RequestError<SpiError>> {
        let cs_asserted = cs_state == CsState::Asserted;
        let devidx = usize::from(devidx);

        // If we are locked there are more rules:
        if let Some(lockstate) = &self.lock_holder {
            // The fact that we received this message _at all_ means
            // that the sender matched our closed receive, but just
            // in case we have a server logic bug, let's check.
            assert!(lockstate.task == rm.sender);
            // The caller is not allowed to change the device index
            // once locked.
            if lockstate.device_index != devidx {
                return Err(SpiError::BadDevice.into());
            }
        }

        // OK! We are either (1) just locking now or (2) processing
        // a legal state change from the same sender.

        // Reject out-of-range devices.
        let device = CONFIG.devices.get(devidx).ok_or(SpiError::BadDevice)?;

        // If we're asserting CS, we want to *reset* the pin. If
        // we're not, we want to *set* it. Because CS is active low.
        let pin_mask = device.cs.pin_mask;
        self.sys
            .gpio_set_reset(
                device.cs.port,
                if cs_asserted { 0 } else { pin_mask },
                if cs_asserted { pin_mask } else { 0 },
            )
            .unwrap();
        self.lock_holder = Some(LockState {
            task: rm.sender,
            device_index: devidx,
        });
        Ok(())
    }

    /// Unlock the spi device and un-assert chip select
    fn release(
        &mut self,
        rm: &RecvMessage,
    ) -> Result<(), RequestError<SpiError>> {
        if let Some(lockstate) = &self.lock_holder {
            // The fact that we were able to receive this means we
            // should be locked by the sender...but double check.
            assert!(lockstate.task == rm.sender);

            let device = &CONFIG.devices[lockstate.device_index];

            // Deassert CS. If it wasn't asserted, this is a no-op.
            // If it was, this fixes that.
            self.sys
                .gpio_set_reset(device.cs.port, device.cs.pin_mask, 0)
                .unwrap();
            self.lock_holder = None;
            Ok(())
        } else {
            Err(SpiError::NothingToRelease.into())
        }
    }
}

impl ServerImpl {
    fn ready_writey(
        &mut self,
        op: SpiOperation,
        device_index: u8,
        data_src: Option<LenLimit<Leased<R, [u8]>, 65535>>,
        data_dest: Option<LenLimit<Leased<W, [u8]>, 65535>>,
    ) -> Result<(), RequestError<SpiError>> {
        let device_index = usize::from(device_index);

        // If we are locked, check that the caller isn't mistakenly
        // addressing the wrong device.
        if let Some(lockstate) = &self.lock_holder {
            if lockstate.device_index != device_index {
                return Err(SpiError::BadDevice.into());
            }
        }

        // Reject out-of-range devices.
        let device = CONFIG
            .devices
            .get(device_index)
            .ok_or(SpiError::BadDevice)?;

        // At least one lease must be provided. A failure here indicates that
        // the server stub calling this common routine is broken, not a client
        // mistake.
        if data_src.is_none() && data_dest.is_none() {
            panic!();
        }

        // Get the required transfer lengths in the src and dest directions.
        let src_len = data_src
            .as_ref()
            .map(|leased| LenLimit::len_as_u16(&leased))
            .unwrap_or(0);
        let dest_len = data_dest
            .as_ref()
            .map(|leased| LenLimit::len_as_u16(&leased))
            .unwrap_or(0);
        let transfer_size = src_len.max(dest_len);

        // Zero-byte SPI transactions don't make sense and we'll
        // decline them.
        if transfer_size == 0 {
            return Err(SpiError::BadTransferSize.into());
        }

        // We have a reasonable-looking request containing reasonable-looking
        // lease(s). This is our commit point.
        // arty - what does this mean
        ringbuf_entry!(Trace::Start(op, (src_len, dest_len)));

        // Vestigial from stm impl. these don't actually do anything but when
        // we switch to DMA maybe they will.
        self.spi.enable();
        self.spi.start();

        const BUFSIZ: usize = 16;

        let mut tx: Option<LeaseBufReader<_, BUFSIZ>> =
            data_src.map(|b| LeaseBufReader::from(b.into_inner()));
        let mut rx: Option<LeaseBufWriter<_, BUFSIZ>> =
            data_dest.map(|b| LeaseBufWriter::from(b.into_inner()));

        // We're doing this! Check if we need to control CS.
        let cs_override = self.lock_holder.is_some();
        if !cs_override {
            self.sys
                .gpio_set_reset(device.cs.port, 0, device.cs.pin_mask)
                .unwrap();
        }




        // Reading lags writing by one byte due to double buffering
        //
        // If the transaction is just one byte, we need to
        // - write 1 byte
        // - wait for ready event
        // - read the real byte.
        //
        // If the transaction is two or more bytes we need to
        // - write 2 bytes
        // - loop:
        //  - wait for ready event
        //  - read 1 byte
        //  - if bytes_written < transfer_size
        //      - write one byte
        //  - if bytes_read == transfer_size
        //      - break
        // 
        // Combined into one, this looks like
        // - write 1 byte
        // - loop:
        //  - if bytes_written < transfer_size
        //    - write 1 byte
        //  - wait for ready event
        //  - read 1 byte
        //  - if bytes_read == transfer_size
        //      - break
        // 
        // We don't use sleeps/interrupts because it doesn't make sense
        // right now. We're not using the DMA version of SPI, so we have
        // to do work after every byte to keep the transaction going. The
        // overhead of sleeping/interrupting for this just wouldn't make
        // sense at 8MHz transfer speed.
        //
        // It would make sense at slower speeds but at that point we should
        // just rewrite the underlying SPI implementation to use the DMA
        // interface at the same time so you don't have weird logic that
        // uses interrupts sometimes but not other times.


        let mut bytes_read = 0;
        let mut bytes_written = 0;

        let txbyte = if let Some(txbuf) = &mut tx {
            if let Some(b) = txbuf.read() {
                b
            } else {
                // We've hit the end of the lease. Stop checking.
                tx = None;
                0
            }
        } else {
            0
        };
        ringbuf_entry!(Trace::Tx(byte));
        self.spi.send8(byte);
        bytes_written++;

        while bytes_read < transfer_size {
            if bytes_written < transfer_size {
                let txbyte = if let Some(txbuf) = &mut tx {
                    if let Some(b) = txbuf.read() {
                        b
                    } else {
                        // We've hit the end of the lease. Stop checking.
                        tx = None;
                        0
                    }
                } else {
                    0
                };
                ringbuf_entry!(Trace::Tx(byte));
                self.spi.send8(byte);
                bytes_written++;
            }

            // spinloop wheeeee
            while !ready {
            }

            // read a byte
            let rxbyte = self.spi.recv8();
            ringbuf_entry!(Trace::Rx(b));
            bytes_read++;

            // Deposit the byte if we're still within the bounds of the
            // caller's incoming lease.
            if let Some(rx_reader) = &mut rx {
                if rx_reader.write(rxbyte).is_err() {
                    // We're off the end. Stop checking.
                    rx = None;
                }
            }
        }

        // Deassert (set) CS, if we asserted it in the first place.
        if !cs_override {
            self.sys
                .gpio_set_reset(device.cs.port, device.cs.pin_mask, 0)
                .unwrap();
        }

        Ok(())
    }
}


//////////////////////////////////////////////////////////////////////////////
// Board-peripheral-server configuration matrix
//
// The configurable bits for a given board and controller combination are in the
// ServerConfig struct. We use conditional compilation below to select _one_
// instance of this struct in a const called `CONFIG`.

/// Rolls up all the configuration options for this server on a given board and
/// controller.
#[derive(Copy, Clone)]
struct ServerConfig {
    /// Pointer to this controller's register block. Don't let the `spi1` fool
    /// you, they all have that type. This needs to match a peripheral in your
    /// task's `uses` list for this to work.
    registers: *const device::spi1::RegisterBlock,
    /// Name for the peripheral as far as the RCC is concerned.
    peripheral: sys_api::Peripheral,
    /// We allow for an individual SPI controller to be switched between several
    /// physical sets of pads. The mux options for a given server configuration
    /// are numbered from 0 and correspond to this slice.
    mux_options: &'static [SpiMuxOption],
    /// We keep track of a fixed set of devices per SPI controller, which each
    /// have an associated routing (from `mux_options`) and CS pin.
    devices: &'static [DeviceDescriptor],
}

/// A routing of the SPI controller onto pins.
#[derive(Copy, Clone, Debug)]
struct SpiMuxOption {
    /// A list of config changes to apply to activate the output pins of this
    /// mux option. This is a list because some mux options are spread across
    /// multiple ports, or (in at least one case) the pins in the same port
    /// require different AF numbers to work.
    ///
    /// To disable the mux, we'll force these pins low. This is correct for SPI
    /// mode 0/1 but not mode 2/3; fortunately we currently don't support mode
    /// 2/3, so we can simplify.
    outputs: &'static [(PinSet, sys_api::Alternate)],
    /// A list of config changes to apply to activate the input pins of this mux
    /// option. This is _not_ a list because there's only one such pin, CIPO.
    ///
    /// To disable the mux, we'll switch this pin to HiZ.
    input: (PinSet, sys_api::Alternate),
    /// Swap data lines?
    swap_data: bool,
}

#[derive(Copy, Clone, Debug)]
struct PinSet {
    port: sys_api::Port,
    pin_mask: u16,
}

/// Information about one device attached to the SPI controller.
#[derive(Copy, Clone, Debug)]
struct DeviceDescriptor {
    /// To reach this device, the SPI controller has to be muxed onto the
    /// correct physical circuit. This gives the index of the right choice in
    /// the server's configured `SpiMuxOption` array.
    mux_index: usize,
    /// Where the CS pin is. While this is a `PinSet`, it should only have one
    /// pin in it, and we check this at startup.
    cs: PinSet,
    /// Clock divider to apply while speaking with this device. Yes, this says
    /// spi1 no matter which SPI block we're in charge of.
    clock_divider: device::spi1::cfg1::MBR_A,
}

/// Any impl of ServerConfig for Server has to pass these tests at startup.
fn check_server_config() {
    // TODO some of this could potentially be moved into const fns for building
    // the tree, and thus to compile time ... if we could assert in const fns.
    //
    // That said, because this is analyzing constants, if the checks _pass_ this
    // should disappear at compilation.

    assert!(!CONFIG.registers.is_null()); // let's start off easy.

    // Mux options must be provided.
    assert!(!CONFIG.mux_options.is_empty());
    for muxopt in CONFIG.mux_options {
        // Each mux option must contain at least one output config record.
        assert!(!muxopt.outputs.is_empty());
        let mut total_pins = 0;
        for (pinset, _af) in muxopt.outputs {
            // Each config must apply to at least one pin.
            assert!(pinset.pin_mask != 0);
            // If this is the same port as the input pin, it must not _include_
            // the input pin.
            if pinset.port == muxopt.input.0.port {
                assert!(pinset.pin_mask & muxopt.input.0.pin_mask == 0);
            }
            // We're counting how many total pins are controlled here.
            total_pins += pinset.pin_mask.count_ones();
        }
        // There should be two affected output pins (COPI, SCK). This check
        // prevents people from being clever and trying to mux SPI to two
        // locations simultaneously, which Does Not Work. It also catches
        // mistakenly including CIPO in the outputs set.
        assert!(total_pins == 2);
        // There should be exactly one pin in the input set.
        assert!(muxopt.input.0.pin_mask.count_ones() == 1);
    }
    // At least one device must be defined.
    assert!(!CONFIG.devices.is_empty());
    for dev in CONFIG.devices {
        // Mux index must be valid.
        assert!(dev.mux_index < CONFIG.mux_options.len());
        // CS pin must designate _exactly one_ pin in its mask.
        assert!(dev.cs.pin_mask.is_power_of_two());
    }
}

include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
include!(concat!(env!("OUT_DIR"), "/spi_config.rs"));
