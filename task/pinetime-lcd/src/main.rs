// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use userlib::*;
use drv_nrf52832_gpio_api as gpio_api;
use drv_spi_api as spi_api;

task_slot!(GPIO, gpio);
task_slot!(SPI, spi);


const CMD_NOP: u8 =       0x00; // NOP
const CMD_SWRESET: u8 =   0x01; // Software Reset
const CMD_RDDID: u8 =     0x04; // Read Display ID
const CMD_RDDST: u8 =     0x09; // Read Display Status
const CMD_RDDPM: u8 =     0x0A; // Read Display Power Mode
const CMD_RDDMADCTL: u8 = 0x0B; // Read Display MADCTL
const CMD_RDDCOLMOD: u8 = 0x0C; // Read Display Pixel Format
const CMD_RDDIM: u8 =     0x0D; // Read Display Image Mode
const CMD_RDDSM: u8 =     0x0E; // Read Display Signal Mode
const CMD_RDDSDR: u8 =    0x0F; // Read Display Self-Diagnostic Result
const CMD_SLPIN: u8 =     0x10; // Sleep in
const CMD_SLPOUT: u8 =    0x11; // Sleep Out
const CMD_PTLON: u8 =     0x12; // Partial Display Mode On
const CMD_NORON: u8 =     0x13; // Normal Display Mode On
const CMD_INVOFF: u8 =    0x20; // Display Inversion Off
const CMD_INVON: u8 =     0x21; // Display Inversion On
const CMD_GAMSET: u8 =    0x26; // Gamma Set
const CMD_DISPOFF: u8 =   0x28; // Display Off
const CMD_DISPON: u8 =    0x29; // Display On
const CMD_CASET: u8 =     0x2A; // Column Address Set
const CMD_RASET: u8 =     0x2B; // Row Address Set
const CMD_RAMWR: u8 =     0x2C; // Memory Write
const CMD_RAMRD: u8 =     0x2E; // Memory Read
const CMD_PTLAR: u8 =     0x30; // Partial Area
const CMD_VSCRDEF: u8 =   0x33; // Vertical Scrolling Definition
const CMD_TEOFF: u8 =     0x34; // Tearing Effect Line OFF
const CMD_TEON: u8 =      0x35; // Tearing Effect Line On
const CMD_MADCTL: u8 =    0x36; // Memory Data Access Control
const CMD_VSCSAD: u8 =    0x37; // Vertical Scroll Start Address of RAM
const CMD_IDMOFF: u8 =    0x38; // Idle Mode Off
const CMD_IDMON: u8 =     0x39; // Idle mode on
const CMD_COLMOD: u8 =    0x3A; // Interface Pixel Format
const CMD_WRMEMC: u8 =    0x3C; // Write Memory Continue
const CMD_RDMEMC: u8 =    0x3E; // Read Memory Continue
const CMD_STE: u8 =       0x44; // Set Tear Scanline
const CMD_GSCAN: u8 =     0x45; // Get Scanline
const CMD_WRDISBV: u8 =   0x51; // Write Display Brightness
const CMD_RDDISBV: u8 =   0x52; // Read Display Brightness Value
const CMD_WRCTRLD: u8 =   0x53; // Write CTRL Display
const CMD_RDCTRLD: u8 =   0x54; // Read CTRL Value Display
const CMD_WRCACE: u8 =    0x55; // Write Content Adaptive Brightness Control and Color Enhancement
const CMD_RDCABC: u8 =    0x56; // Read Content Adaptive Brightness Control
const CMD_WRCABCMB: u8 =  0x5E; // Write CABC Minimum Brightness
const CMD_RDCABCMB: u8 =  0x5F; // Read CABC Minimum Brightness
const CMD_RDABCSDR: u8 =  0x68; // Read Automatic Brightness Control Self-Diagnostic Result
const CMD_RDID1: u8 =     0xDA; // Read ID1
const CMD_RDID2: u8 =     0xDB; // Read ID2
const CMD_RDID3: u8 =     0xDC; // Read ID3
const CMD_RAMCTRL: u8 =   0xB0; // RAM Control
const CMD_RGBCTRL: u8 =   0xB1; // RGB Interface Control
const CMD_PORCTRL: u8 =   0xB2; // Porch Setting
const CMD_FRCTRL1: u8 =   0xB3; // Frame Rate Control 1 0xIn partial mode/ idle colors)
const CMD_PARCTRL: u8 =   0xB5; // Partial Control
const CMD_GCTRL: u8 =     0xB7; // Gate Control
const CMD_GTADJ: u8 =     0xB8; // Gate On Timing Adjustment
const CMD_DGMEN: u8 =     0xBA; // Digital Gamma Enable
const CMD_VCOMS: u8 =     0xBB; // VCOM Setting
const CMD_LCMCTRL: u8 =   0xC0; // LCM Control
const CMD_IDSET: u8 =     0xC1; // ID Code Setting
const CMD_VDVVRHEN: u8 =  0xC2; // VDV and VRH Command Enable
const CMD_VRHS: u8 =      0xC3; // VRH Set
const CMD_VDVS: u8 =      0xC4; // VDV Set
const CMD_VCMOFSET: u8 =  0xC5; // VCOM Offset Set
const CMD_FRCTRL2: u8 =   0xC6; // Frame Rate Control in Normal Mode
const CMD_CABCCTRL: u8 =  0xC7; // CABC Control
const CMD_REGSEL1: u8 =   0xC8; // Register Value Selection 1
const CMD_REGSEL2: u8 =   0xCA; // Register Value Selection 2
const CMD_PWMFRSEL: u8 =  0xCC; // PWM Frequency Selection
const CMD_PWCTRL1: u8 =   0xD0; // Power Control 1
const CMD_VAPVANEN: u8 =  0xD2; // Enable VAP/VAN signal output
const CMD_CMD2EN: u8 =    0xDF; // Command 2 Enable
const CMD_PVGAMCTRL: u8 = 0xE0; // Positive Voltage Gamma Control
const CMD_NVGAMCTRL: u8 = 0xE1; // Negative Voltage Gamma Control
const CMD_DGMLUTR: u8 =   0xE2; // Digital Gamma Look-up Table for Red
const CMD_DGMLUTB: u8 =   0xE3; // Digital Gamma Look-up Table for Blue
const CMD_GATECTRL: u8 =  0xE4; // Gate Control
const CMD_SPI2EN: u8 =    0xE7; // SPI2 Enable
const CMD_PWCTRL2: u8 =   0xE8; // Power Control 2
const CMD_EQCTRL: u8 =    0xE9; // Equalize time control
const CMD_PROMCTRL: u8 =  0xEC; // Program Mode Control
const CMD_PROMEN: u8 =    0xFA; // Program Mode Enable
const CMD_NVMSET: u8 =    0xFC; // NVM Setting
const CMD_PROMACT: u8 =   0xFE; // Program action 
const BACKLIGHT_LOW: u8 = 14;
const BACKLIGHT_MED: u8 = 22;
const BACKLIGHT_HIGH: u8 = 23;
const CHIP_SELECT: u8 = 25;
const LCD_RESET: u8 = 26;
const LCD_COMMAND: u8 = 18;
const MISO: u8 = 4;
const MOSI: u8 = 3;
const SCK: u8 = 2;

#[export_name = "main"]
pub fn main() -> ! {
    const TIMER_NOTIFICATION: u32 = 1;
    const INTERVAL: u64 = 3000;

    let mut response: u32 = 0;

    let gpio = gpio_api::Sys::from(GPIO.get_task_id());
    let spi = spi_api::Spi::from(SPI.get_task_id());

    gpio.gpio_configure_output(BACKLIGHT_LOW, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(BACKLIGHT_MED, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(BACKLIGHT_HIGH, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_input(MISO, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(MOSI, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(SCK, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(CHIP_SELECT, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(LCD_RESET, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();
    gpio.gpio_configure_output(LCD_COMMAND, gpio_api::OutputType::PushPull, gpio_api::Pull::None).unwrap();


    gpio.gpio_set_reset(
        (1 << BACKLIGHT_LOW) | (1 << BACKLIGHT_MED) | (1 << CHIP_SELECT) | (1 << LCD_COMMAND) | (1 << LCD_RESET),
        1 << BACKLIGHT_HIGH,
    );


    let mut current = 0;
    let mut msg = [0; 16];
    let mut dl = INTERVAL;

    // TODO configurable device ID? Only matters of making this into a generic
    // this creates a second spi/gpio interface so i dont have to put pointers
    // in there or deal with moving the original ones in.
    let mut lcd = Lcd {
        spidev: 0,
        spi: spi_api::Spi::from(SPI.get_task_id()),
        gpio: gpio_api::Sys::from(GPIO.get_task_id())
    };

    gpio.gpio_reset(1 << LCD_RESET);

    sys_set_timer(Some(dl), TIMER_NOTIFICATION);
    let msginfo = sys_recv_open(&mut msg, TIMER_NOTIFICATION);
    dl += INTERVAL;

    gpio.gpio_set(1 << LCD_RESET);
    lcd.send_command(CMD_SWRESET);

    sys_set_timer(Some(dl), TIMER_NOTIFICATION);
    let msginfo = sys_recv_open(&mut msg, TIMER_NOTIFICATION);
    dl += INTERVAL;
    lcd.send_command(CMD_SLPOUT);

    lcd.send_command(CMD_NORON);
    lcd.send_command(CMD_INVON);

    lcd.set_pixel_format(PixelFormat::RGB565);
    lcd.put_color_rect(20, 20, 64, 64, 0xFFFF);
    lcd.put_color_rect(140, 20, 64, 64, 0xFFFF);
    lcd.put_color_rect(40, 160, 140, 20, 0x0F0F);

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

struct Lcd {
    spidev: u8,
    spi: spi_api::Spi,
    gpio: gpio_api::Sys,
}

// TODO make these things return errors instead of just unwrap
impl Lcd {
    fn send_command(&mut self, command: u8) {
        self.gpio.gpio_reset(1 << LCD_COMMAND).unwrap();
        self.spi.write(self.spidev, &[command]).unwrap();
    }

    fn send_data(&mut self, data: &[u8]) {
        self.gpio.gpio_set(1 << LCD_COMMAND).unwrap();
        self.spi.write(self.spidev, &data).unwrap();
    }

    fn send_data8(&mut self, data: u8) {
        self.send_data(&[data]);
    }

    fn send_data16(&mut self, data: u16) {
        self.send_data(&data.to_be_bytes());
    }

    fn set_window(&mut self, x: u8, y: u8, w: u8, h: u8) {
        self.send_command(CMD_CASET);
        self.send_data(&[
            0, x,
            0, (x + w - 1)
        ]);

        self.send_command(CMD_RASET);
        self.send_data(&[
            0, y,
            0, (y + h - 1)
        ]);
    }

    fn set_pixel_format(&mut self, fmt: PixelFormat) {
        let lcd_pix_fmt = fmt as u8;
        self.send_command(CMD_COLMOD);
        self.send_data(&[lcd_pix_fmt]);
    }

    /// Fill a rectangle with `color`. If display is in 12-bit mode, the
    /// low 12 bits of `color` will be used.
    fn put_color_rect(&mut self, x: u8, y: u8, w: u8, h: u8, color: u16) {

        // TODO 12 bit support
        self.set_window(x, y, w, h);
        self.send_command(CMD_RAMWR);
        
        // Transfer in up to 96-byte chunks (64 pixels in 12 bit, 48 in 16 bit)
        const TRANSFER_BLOCK: usize = 96;
        let mut pix_data = [0; TRANSFER_BLOCK];
        let repr = color.to_be_bytes();

        // There's got to be a better way to do this
        // TODO 12 bit support
        for i in 0..(TRANSFER_BLOCK / 2) {
            pix_data[i] = repr[0];
            pix_data[i+1] = repr[1];
        }

        // TODO 12 bit support
        let mut bytes_remaining = (w as usize) * (h as usize) * 2;

        while bytes_remaining > TRANSFER_BLOCK {
            self.send_data(&pix_data);
            bytes_remaining -= TRANSFER_BLOCK;
        }

        self.send_data(&pix_data[0 .. bytes_remaining]);
    }


    
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
// I'm not supporting RGB888 because this display is 16-bit and so its useless
pub enum PixelFormat {
    RGB444 = 0b0101_0011,
    RGB565 = 0b0101_0101,
}
