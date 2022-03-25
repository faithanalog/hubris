// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use indexmap::IndexMap;
use proc_macro2::TokenStream;
use quote::{ToTokens, TokenStreamExt};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    //build_util::expose_target_board();

    // where the fsck does hubris_task_config come from
    let task_config = build_util::task_config::<TaskConfig>()?;
    let global_config = build_util::config::<GlobalConfig>()?;
    check_spi_config(&global_config.spi, &task_config.spi)?;
    generate_spi_config(&global_config.spi, &task_config.spi)?;

    idol::server::build_server_support(
        "../../idl/spi.idol",
        "server_stub.rs",
        idol::server::ServerStyle::InOrder,
    )?;

    Ok(())
}

///////////////////////////////////////////////////////////////////////////////
// SPI config schema definition.
//
// There are two portions to this, task-level and global. Both are defined by
// the structs below using serde.
//
// Task-level simply provides a way (through `global_config`) to reference a key
// in the global.
//
// Global starts at `GlobalConfig`.

#[derive(Deserialize)]
struct TaskConfig {
    spi: SpiTaskConfig,
}

#[derive(Deserialize)]
struct SpiTaskConfig {
    global_config: String,
}

#[derive(Deserialize)]
struct GlobalConfig {
    spi: BTreeMap<String, SpiConfig>,
}

#[derive(Deserialize)]
struct SpiConfig {
    controller: usize,
    mux_options: BTreeMap<String, SpiMuxOptionConfig>,
    devices: IndexMap<String, DeviceDescriptorConfig>,
}

#[derive(Deserialize)]
struct SpiMuxOptionConfig {
    miso_pin: usize,
    mosi_pin: usize,
    sck_pin: usize
}

#[derive(Clone, Debug, Deserialize)]
struct DeviceDescriptorConfig {
    mux: String,
    frequency: FrequencyConfig,
    cs: usize,
}

#[derive(Copy, Clone, Debug, Deserialize)]
enum FrequencyConfig {
    K125,
    K250,
    K500,
    M1,
    M2,
    M4,
    M8,
}

///////////////////////////////////////////////////////////////////////////////
// SPI config code generation.
//
// Our config types, by design, map almost directly onto the structs that the
// SPI driver uses to configure itself. This means we can do the code generation
// process in a separable-and-composable fashion, by implementing
// `quote::ToTokens` for most of the config types.
//
// Each impl defines, in isolation, how code generation works for that part of
// the config. This is most of the code generation implementation; the
// `generate_spi_config` routine is just a wrapper.

fn generate_spi_config(
    config: &BTreeMap<String, SpiConfig>,
    task_config: &SpiTaskConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    let config = config.get(&task_config.global_config).ok_or_else(|| {
        format!(
            "reference to undefined spi config {}",
            task_config.global_config
        )
    })?;

    let out_dir = std::env::var("OUT_DIR")?;
    let dest_path = std::path::Path::new(&out_dir).join("spi_config.rs");

    let mut out = std::fs::File::create(&dest_path)?;

    writeln!(out, "{}", config.to_token_stream())?;

    drop(out);

    call_rustfmt::rustfmt(&dest_path)?;

    Ok(())
}

impl ToTokens for SpiConfig {
    fn to_tokens(&self, tokens: &mut TokenStream) {
        // Work out the mapping from mux names to indices so we can dereference
        // the mux names used in devices.
        let mux_indices: BTreeMap<_, usize> = self
            .mux_options
            .keys()
            .enumerate()
            .map(|(i, k)| (k, i))
            .collect();

        // The svd2rust PAC can't decide whether acronyms are words, so we get
        // to produce both identifiers.
        let devname: syn::Ident =
            syn::parse_str(&format!("SPI{}", self.controller)).unwrap();
        let pname: syn::Ident =
            syn::parse_str(&format!("Spi{}", self.controller)).unwrap();

        // We don't derive ToTokens for DeviceDescriptorConfig because it needs
        // extra knowledge (the mux_indices map) to do the conversion. Instead,
        // convert it here:
        let device_code = self.devices.values().map(|dev| {
            let mux_index = mux_indices[&dev.mux];
            let cs = dev.cs;
            let freq: syn::Ident =
                syn::parse_str(&format!("{:?}", dev.frequency)).unwrap();
            quote::quote! {
                DeviceDescriptor {
                    mux_index: #mux_index,
                    cs: #cs,
                    frequency: device::spi0::frequency::FREQUENCY_A::#freq,
                }
            }
        });

        let muxes = self.mux_options.values();

        tokens.append_all(quote::quote! {
            const CONFIG: ServerConfig = ServerConfig {
                registers: device::#devname::ptr(),
                mux_options: &[ #(#muxes),* ],
                devices: &[ #(#device_code),* ],
            };
        });
    }
}

impl ToTokens for SpiMuxOptionConfig {
    fn to_tokens(&self, tokens: &mut TokenStream) {
        let miso_pin = &self.miso_pin;
        let mosi_pin = &self.mosi_pin;
        let sck_pin = &self.sck_pin;
        tokens.append_all(quote::quote! {
            SpiMuxOption {
                miso_pin: #miso_pin,
                mosi_pin: #mosi_pin,
                sck_pin: #sck_pin,
            }
        });
    }
}


///////////////////////////////////////////////////////////////////////////////
// Check routines.

fn check_spi_config(
    config: &BTreeMap<String, SpiConfig>,
    task_config: &SpiTaskConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    // We only want to look at the subset of global configuration relevant to
    // this task, so that error reporting is more focused.
    let config = config.get(&task_config.global_config).ok_or_else(|| {
        format!(
            "reference to undefined spi config {}",
            task_config.global_config
        )
    })?;

    if config.controller > 2 {
        return Err(format!(
            "bad controller {}, valid values are 0 thru 2",
            config.controller
        )
        .into());
    }

    for mux in config.mux_options.values() {
        check_gpiopin(mux.miso_pin)?;
        check_gpiopin(mux.mosi_pin)?;
        check_gpiopin(mux.sck_pin)?;
    }

    for (devname, dev) in &config.devices {
        if !config.mux_options.contains_key(&dev.mux) {
            return Err(format!(
                "device {} names undefined mux {}",
                devname, dev.mux
            )
            .into());
        }
        check_gpiopin(dev.cs)?;
    }

    Ok(())
}

fn check_gpiopin(
    pin: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    if pin >= 32 {
        return Err(format!(
            "pin {} is invalid, pins are numbered 0-31",
            pin
        )
        .into());
    }
    Ok(())
}
