#![no_std]
#![no_main]

use userlib::*;
use drv_nrf52832_gpio_api as gpio_api;
//use drv_spi_api as spi_api;
use drv_nrf52832_spi as spi_core;
use nrf52832_pac as device;
use micromath::F32Ext;

task_slot!(GPIO, gpio);
//task_slot!(SPI, spi);


// Ignore the constant spam. this should be an enum later.
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

    let gpio = gpio_api::GPIO::from(GPIO.get_task_id());

    // dot unwrap
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
        (1 << BACKLIGHT_LOW) | (1 << BACKLIGHT_MED) | (1 << LCD_COMMAND) | (1 << LCD_RESET),
        (1 << BACKLIGHT_HIGH) | (1 << CHIP_SELECT),
    ).unwrap();


    let mut current = 0;
    let mut msg = [0; 16];
    let mut dl = INTERVAL;

    let registers = unsafe { &*device::SPI0::ptr() };
    let mut spi = spi_core::Spi::from(registers);
    spi.initialize();
    spi.configure_transmission_parameters(
            device::spi0::frequency::FREQUENCY_A::M8,
            device::spi0::config::ORDER_A::MSBFIRST,
            device::spi0::config::CPHA_A::TRAILING,
            device::spi0::config::CPOL_A::ACTIVELOW,
    );
    spi.enable(MISO, MOSI, SCK);

    let mut lcd = Lcd {
        //spi: spi_api::Spi::from(SPI.get_task_id()),
        spi,
        gpio: gpio_api::GPIO::from(GPIO.get_task_id()),
    };

    gpio.gpio_reset(1 << LCD_RESET).unwrap();

    sys_set_timer(Some(dl), TIMER_NOTIFICATION);
    let msginfo = sys_recv_open(&mut msg, TIMER_NOTIFICATION);
    dl += INTERVAL;

    gpio.gpio_set(1 << LCD_RESET).unwrap();
    lcd.send_command(CMD_SWRESET);

    sys_set_timer(Some(dl), TIMER_NOTIFICATION);
    let msginfo = sys_recv_open(&mut msg, TIMER_NOTIFICATION);
    dl += INTERVAL;
    lcd.send_command(CMD_SLPOUT);


    lcd.set_pixel_format(PixelFormat::RGB565);
    lcd.draw_color_rect(0, 0, 240, 320, 0x0808);
    lcd.draw_twister(0.0);

    lcd.send_command(CMD_DISPON);
    lcd.send_command(CMD_NORON);
    lcd.send_command(CMD_INVON);
    lcd.set_scroll_area(0, 320, 0);
    lcd.set_scroll_start(0);

    //sys_set_timer(Some(dl), TIMER_NOTIFICATION);
    let mut time = 0.0;
    let mut y = 160.0;
    let mut scroll = 1;
    loop {
        lcd.draw_twister(time);
        time += 4.0

        //let msginfo = sys_recv_open(&mut msg, TIMER_NOTIFICATION);
        
        //gpio.gpio_toggle(1 << BACKLIGHT_HIGH);

        //if msginfo.sender == TaskId::KERNEL {
            //dl += 20;
            //sys_set_timer(Some(dl), TIMER_NOTIFICATION);
        //}
    }
}

struct Lcd {
    spi: spi_core::Spi,
    gpio: gpio_api::GPIO,
}

// TODO make these things return errors instead of just unwrap
impl Lcd {
    fn spi_write(&mut self, data: &[u8]) {
        if data.len() == 0 {
            return;
        }
        self.spi.start();
        self.spi.send8(data[0]);
        for byte in &data[1..] {
            self.spi.send8(*byte);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
        }
        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();

        //self.spi.write(self.spidev, data).unwrap();
    }

    fn send_command(&mut self, command: u8) {
        self.gpio.gpio_reset(1 << LCD_COMMAND).unwrap();
        //self.spi.write(self.spidev, &[command]).unwrap();
        self.spi_write(&[command]);
    }

    fn send_data(&mut self, data: &[u8]) {
        self.gpio.gpio_set(1 << LCD_COMMAND).unwrap();
        //self.spi.write(self.spidev, &data).unwrap();
        self.spi_write(&data);
    }

    fn send_data8(&mut self, data: u8) {
        self.send_data(&[data]);
    }

    fn send_data16(&mut self, data: u16) {
        self.send_data(&data.to_be_bytes());
    }

    fn set_window(&mut self, x: u8, y: u16, w: u8, h: u16) {
        self.send_command(CMD_CASET);
        self.send_data(&[
            0, x,
            0, (x + w - 1)
        ]);

        let top = y.to_be_bytes();
        let bot = (y + h - 1).to_be_bytes();
        self.send_command(CMD_RASET);
        self.send_data(&[
            top[0], top[1],
            bot[0], bot[1]
        ]);
    }

    fn set_pixel_format(&mut self, fmt: PixelFormat) {
        let lcd_pix_fmt = fmt as u8;
        self.send_command(CMD_COLMOD);
        self.send_data(&[lcd_pix_fmt]);
    }

    /// Set the height of the top fixed area, scroll area, and bottom fixed area. The three values
    /// must sum to 320 or results are undefined.
    fn set_scroll_area(&mut self, top_fixed_height: u16, scroll_height: u16, bottom_fixed_height: u16) {
        let tfa = top_fixed_height.to_be_bytes();
        let vsa = scroll_height.to_be_bytes();
        let bfa = bottom_fixed_height.to_be_bytes();
        self.send_command(CMD_VSCRDEF);
        self.send_data(&[
            tfa[0], tfa[1],
            vsa[0], vsa[1],
            bfa[0], bfa[1]
        ]);
    }

    /// set VSCSAD register. When scanning out the scrolled area, the display starts at this
    /// row as the top row and then goes down from there. So steadily incrementing this number
    /// creates the effect of scrolling up, while decrementing scrolls down.
    ///
    /// The display controller has 320 rows of memory even though the display is only 240x240 so
    /// you can take advantage of this to draw stuff off-screen before scrolling it on screen.
    fn set_scroll_start(&mut self, row: u16) {
        self.send_command(CMD_VSCSAD);
        self.send_data(&row.to_be_bytes());
    }

    /// Fill a rectangle with `color`. No 12 bit support.
    fn draw_color_rect(&mut self, x: u8, y: u16, w: u8, h: u16, color: u16) {

        self.set_window(x, y, w, h);
        self.send_command(CMD_RAMWR);

        let repr = color.to_be_bytes();

        self.gpio.gpio_set(1 << LCD_COMMAND).unwrap();
        self.spi.start();
        self.spi.send8(repr[0]);
        self.spi.send8(repr[1]);
        for _ in 1..((w as u32) * (h as u32)) {
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(repr[0]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(repr[1]);
        }
        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();
        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();
    }

    fn draw_twister(&mut self, t: f32) {
        self.set_window(60, 0, 120, 240);
        self.send_command(CMD_RAMWR);
        self.gpio.gpio_set(1 << LCD_COMMAND).unwrap();
        for row in 0..120 {
            self.draw_twister_line(row as f32, t);
        }
    }


    fn draw_twister_line(&mut self, y: f32, t: f32) {
        const PI: f32 = 3.141592653589;
        const PI2: f32 = PI * 2.0;
        const FRAC_PI_2: f32 = PI * 0.5;


        // Only render to the center 120 pixels, at half res
        let mut pix_data = [0x08; 120];

        let a = (PI * (y / 2000.0 + t / 300.0)).cos();
        let a_pi = a * PI2;
        //let a = (PI * (y / 300.0)).cos() * PI;
        //let a = (y + t) / 40.0;
        let deg90 = FRAC_PI_2;
        let deg180 = PI;
        let deg270 = deg90 + deg180;

        //let xoff = (y / 20.0).sin() * 0.25 + (y / 10.0).cos() * 0.15;
        //let xoff = y / 100.0;
        let xoff = (((t / 80.0) - y + 20.0 * ((t / 20000.0 + a / (120.0 + 20.0 * ((t / 100.0 + y / 500.0) * PI2).sin())) * PI2).sin()) * PI2).cos() * 0.75;

        let x0 = a_pi.sin() + xoff;
        let x1 = (a_pi + deg90).sin() + xoff;
        let x2 = (a_pi + deg180).sin() + xoff;
        let x3 = (a_pi + deg270).sin() + xoff;

        let pairs = [
            TwisterPair{l: x0, r: x1, col: 0xF810_u16.to_be_bytes()},
            TwisterPair{l: x1, r: x2, col: 0x2104_u16.to_be_bytes()},
            TwisterPair{l: x2, r: x3, col: 0x061F_u16.to_be_bytes()},
            TwisterPair{l: x3, r: x0, col: 0xFE00_u16.to_be_bytes()},
        ];

        for pair in pairs {
            if pair.r > pair.l {
                let l = (((pair.l + 1.0) * 15.0 + 15.0) as usize) * 2;
                let r = (((pair.r + 1.0) * 15.0 + 15.0) as usize) * 2;
                for i in 0..4 {
                    pix_data[l + i - 2] = 0;
                    pix_data[r + i - 2] = 0;
                }
                for i in ((l + 2)..(r - 2)).step_by(2) {
                    pix_data[i] = pair.col[0];
                    pix_data[i + 1] = pair.col[1];
                }
            }
        }

        // Write out with pixel doubling
        self.spi.send8(pix_data[0]);
        self.spi.send8(pix_data[1]);
        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();
        self.spi.send8(pix_data[0]);
        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();
        self.spi.send8(pix_data[1]);
        for i in (2..120).step_by(2) {
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i + 1]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i + 1]);
        }

        // Second row
        for i in (0..120).step_by(2) {
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i + 1]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i]);
            while !self.spi.is_read_ready() {}
            let _ = self.spi.recv8();
            self.spi.send8(pix_data[i + 1]);
        }

        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();
        while !self.spi.is_read_ready() {}
        let _ = self.spi.recv8();
    }


    
}
struct TwisterPair {
    l: f32,
    r: f32,
    col: [u8; 2]
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
// I'm not supporting RGB888 because this display is 16-bit and so its useless
pub enum PixelFormat {
    RGB444 = 0b0101_0011,
    //RGB565 = 0b0101_0101,
    RGB565 = 0x65,
}
