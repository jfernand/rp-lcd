//! Example of graphics on the LCD of the Waveshare RP2040-LCD-0.96
//!
//! Draws a red and green line with a blue rectangle.
//! After that it fills the screen line for line, at the end it starts over with
//! another colour, RED, GREEN and BLUE.
#![no_std]
#![no_main]
#![feature(cell_update)]
#![feature(ascii_char)]

mod allocator;
mod init;
mod usb;

extern crate alloc;

use alloc::format;
use core::cell::{Cell};
use core::num;
use cortex_m::delay::Delay;
use cortex_m::interrupt::{free, Mutex};
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};
use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::text::Text;
use fugit::RateExtU32;
use num_traits::one;
use st7735_lcd::{Orientation, ST7735};
use panic_halt;
use rp2040_hal::clocks::ClocksManager;
use usb_device::{class_prelude::*};
use rp2040_hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio8};
use rp2040_hal::gpio::{FunctionSio, FunctionSpi, Pin, PullDown, SioOutput};
use rp2040_hal::pac::{CLOCKS, Peripherals, PLL_SYS, PLL_USB, RESETS, SPI1, XOSC};
use rp2040_hal::Spi;
use rp2040_hal::spi::Enabled;
use usb_device::prelude::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;
use waveshare_rp2040_lcd_0_96::{hal::{
    self,
    clocks::{Clock, init_clocks_and_plls},
    pio::PIOExt,
    Sio,
    watchdog::Watchdog,
}, pac, Pins, XOSC_CRYSTAL_FREQ};
use waveshare_rp2040_lcd_0_96::entry;
use crate::init::used_mem;
use crate::usb::{PARSE_ERROR_COUNT, INTERRUPT_COUNT, RX_COUNT, TX_COUNT, USB_BUS, USB_DEVICE, USB_SERIAL, ZERO_READ_COUNT, WOULD_BLOCK_ERROR_COUNT, BUFFER_OVERFLOW_ERROR_COUNT, ENDPOINT_OVERFLOW_ERROR_COUNT, ENDPOINT_MEMORY_OVERFLOW_ERROR_COUNT, INVALID_ENDPOINT_ERROR_COUNT, UNSUPPORTED_ERROR_COUNT, INVALID_STATE_ERROR_COUNT};

const LCD_WIDTH: u32 = 160;
const LCD_HEIGHT: u32 = 80;

#[entry]
fn main() -> ! {
    init::init_heap();
    let mut pac = Peripherals::take().unwrap();
    let Peripherals {
        ADC,
        BUSCTRL, CLOCKS, DMA, I2C0, I2C1, IO_BANK0, IO_QSPI, PADS_BANK0, PADS_QSPI, PIO0, PIO1, PLL_SYS, PLL_USB, PPB, PSM, PWM, RESETS: mut RESETS, ROSC, RTC, SIO, SPI0, SPI1,
        SYSCFG, SYSINFO, TBMAN, TIMER, UART0, UART1, USBCTRL_DPRAM,
        USBCTRL_REGS, VREG_AND_CHIP_RESET, WATCHDOG, XIP_CTRL, XIP_SSI, XOSC,
    } = pac;
    let (SPI1, USBCTRL_DPRAM, USBCTRL_REGS) = (SPI1, USBCTRL_DPRAM, USBCTRL_REGS);
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(WATCHDOG);

    let clocks = init_clocks(CLOCKS, PLL_SYS, PLL_USB, &mut RESETS, XOSC, &mut watchdog);

    let sio = Sio::new(SIO);
    let pins = Pins::new(
        IO_BANK0,
        PADS_BANK0,
        sio.gpio_bank0,
        &mut RESETS,
    );

    let (_pio, _sm0, _, _, _) = PIO0.split(&mut RESETS);

    // https://www.waveshare.com/wiki/RP2040-LCD-0.96
    // ST7735S LCD
    let lcd_dc = pins.gp8.into_push_pull_output();
    let lcd_clk = pins.gp10.into_function::<FunctionSpi>();
    let lcd_mosi = pins.gp11.into_function::<FunctionSpi>();
    let lcd_rst = pins
        .gp12
        .into_push_pull_output_in_state(hal::gpio::PinState::High);
    let _lcd_bl = pins
        .gp25
        .into_push_pull_output_in_state(hal::gpio::PinState::High);
    let spi = hal::Spi::<_, _, _, 8>::new(SPI1, (lcd_mosi, lcd_clk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        USBCTRL_REGS,
        USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }
    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }
    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Chinese Hacker Directory")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    };

    // init_usb(USBCTRL_DPRAM, USBCTRL_REGS, &mut RESETS, clocks);


    // LCD is a 65K IPS LCD 160x80, color order is BGR and a offset 1,26 pixel.
    // LCD controller can correct this by settings the order bit (bit 3) in MADCTL register.
    // Also the colours are inverted, LCD controller can also correct this by writing to INVON register with no paramters.
    // All this is handled by the ST7735 crate.
    let mut display = ST7735::new(spi, lcd_dc, lcd_rst, false, true, LCD_WIDTH, LCD_HEIGHT);

    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);
    display.init(&mut delay).unwrap();
    display.set_orientation(&Orientation::Landscape).unwrap();
    display.set_offset(1, 26);

    let mut c = Rgb565::RED;
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
    let _ = serial.write(format!("err: {}", PARSE_ERROR_COUNT.value()).as_ref());
    display.clear(Rgb565::BLACK).unwrap();
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();
    display.draw_text(2, 15, "int:", style);
    // display.draw_text(2, 15 + 18, "err:", style);
    // display.draw_text(2, 15 + 18 + 18, "tx:", style);
    // display.draw_text(2, 15 + 18 + 18 + 18, "kb:", style);
    // display.draw_text(2, 15 + 18 + 18 + 18 + 18, "kb:", style);
    // let _ = canvas.place_at(Point::zero()).draw(&mut display);
    loop {
        c = match c {
            Rgb565::RED => Rgb565::GREEN,
            Rgb565::GREEN => Rgb565::BLUE,
            Rgb565::BLUE => Rgb565::YELLOW,
            Rgb565::YELLOW => Rgb565::MAGENTA,
            Rgb565::MAGENTA => Rgb565::CYAN,
            Rgb565::CYAN => Rgb565::WHITE,
            Rgb565::WHITE => Rgb565::BLACK,
            _ => Rgb565::RED,
        };
        unsafe {
            display.draw_text(45, 15, &format!("{}", INTERRUPT_COUNT.value()), style);
            // display.draw_text(45, 15 + 18, &format!("{}", PARSE_ERROR_COUNT.value()), style);
            // display.draw_text(45, 15 + 18 + 18, &format!("{}", TX_COUNT.value()), style);
            // display.draw_text(45, 15 + 18 + 18 + 18, &format!("{}", RX_COUNT.value()), style);
            display.draw_text(
                2,
                15 + 18 + 18,
                &format!("z {} p{} w{} o{} eo{}\nem{} ie{} u{} is{}",
                         ZERO_READ_COUNT.value(),
                         PARSE_ERROR_COUNT.value(),
                         WOULD_BLOCK_ERROR_COUNT.value(),
                         BUFFER_OVERFLOW_ERROR_COUNT.value(),
                         ENDPOINT_OVERFLOW_ERROR_COUNT.value(),
                         ENDPOINT_MEMORY_OVERFLOW_ERROR_COUNT.value(),
                         INVALID_ENDPOINT_ERROR_COUNT.value(),
                         UNSUPPORTED_ERROR_COUNT.value(),
                         INVALID_STATE_ERROR_COUNT.value()
                ),
                style);
        }
    }
}

fn init_clocks(CLOCKS: CLOCKS, PLL_SYS: PLL_SYS, PLL_USB: PLL_USB, RESETS: &mut RESETS, XOSC: XOSC, watchdog: &mut Watchdog) -> ClocksManager {
    init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        XOSC,
        CLOCKS,
        PLL_SYS,
        PLL_USB,
        RESETS,
        watchdog,
    ).ok()
        .unwrap()
}

fn draw_text<D>(display: &mut D, x: i32, y: i32, text: &str, style: MonoTextStyle<Rgb565>)
where
    D: DrawTarget<Color=Rgb565>,
{
    let _ = Text::new(text, Point::new(x, y), style)
        .draw(display);
}

fn fill_screen(display: &mut ST7735<Spi<Enabled, SPI1, (Pin<Gpio11, FunctionSpi, PullDown>, Pin<Gpio10, FunctionSpi, PullDown>)>, Pin<Gpio8, FunctionSio<SioOutput>, PullDown>, Pin<Gpio12, FunctionSio<SioOutput>, PullDown>>, color: Rgb565) {
    let style = PrimitiveStyleBuilder::new()
        .fill_color(color)
        .build();

    Rectangle::with_corners(
        Point::new(1, 1),
        Point::new((LCD_WIDTH - 2) as i32, (LCD_HEIGHT - 2) as i32),
    )
        .into_styled(style)
        .draw(display)
        .unwrap();
}

trait Opposable<T: RgbColor> {
    fn opposite(&self) -> T;
}

impl Opposable<Rgb565> for Rgb565 {
    fn opposite(&self) -> Rgb565 {
        Rgb565::new(Rgb565::MAX_R - self.r(), Rgb565::MAX_G - self.g(), Rgb565::MAX_B - self.b())
    }
}

trait HasValue<T> {
    fn value(&self) -> T;
    fn update<F>(&self, f: F) -> T
    where
        F: FnOnce(T) -> T;
}

impl<T: Copy> HasValue<T> for Mutex<Cell<T>> {
    fn value(&self) -> T {
        free(|cs| self.borrow(cs).get())
    }

    fn update<F>(&self, f: F) -> T
    where
        F: FnOnce(T) -> T,
    {
        free(|cs| self.borrow(cs).update(f))
    }
}

trait Incrementable {
    fn inc(&self);
}

impl<T> Incrementable for Mutex<Cell<T>>
where
    T: num_traits::Num,T:Copy
{
    fn inc(&self)
    {
        free(|cs| self.borrow(cs).update(|x| x + one()));
    }
}


trait TextDrawer {
    fn draw_text(&mut self, x: i32, y: i32, text: &str, style: MonoTextStyle<Rgb565>);
}

impl TextDrawer for ST7735<Spi<Enabled, SPI1, (Pin<Gpio11, FunctionSpi, PullDown>, Pin<Gpio10, FunctionSpi, PullDown>)>, Pin<Gpio8, FunctionSio<SioOutput>, PullDown>, Pin<Gpio12, FunctionSio<SioOutput>, PullDown>> {
    fn draw_text(&mut self, x: i32, y: i32, text: &str, style: MonoTextStyle<Rgb565>) {
        draw_text(self, x, y, text, style)
    }
}




