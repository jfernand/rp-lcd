use alloc::boxed::Box;
use alloc::rc::Rc;
use core::ascii::Char;
use core::cell::{Cell, RefCell, RefMut};
use core::error::Error;
use core::str::from_utf8;
use crate::alloc::string::ToString;
use cortex_m::interrupt::{free, Mutex};
use heapless::Deque;
use rp2040_hal::clocks::ClocksManager;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usb_device::device::UsbDeviceState::Configured;
use usb_device::UsbError;
use waveshare_rp2040_lcd_0_96::{hal, pac};
use usbd_serial::SerialPort;
use waveshare_rp2040_lcd_0_96::pac::{interrupt, RESETS, USBCTRL_DPRAM, USBCTRL_REGS};
use crate::{HasValue, Incrementable};

/// The USB Bus Driver (shared with the interrupt).
pub static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
pub static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

/// Note whether we've already printed the "hello" message.
pub static INTERRUPT_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static PARSE_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static WOULD_BLOCK_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static BUFFER_OVERFLOW_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static ENDPOINT_OVERFLOW_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static ENDPOINT_MEMORY_OVERFLOW_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static INVALID_ENDPOINT_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static UNSUPPORTED_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static INVALID_STATE_ERROR_COUNT: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
pub static TX_COUNT: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

pub static RX_COUNT: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
pub static ZERO_READ_COUNT: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

pub static SAID_HELLO: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

pub static RING_BUFFER: Mutex<RefCell<Deque<Char, 256>>> = Mutex::new(RefCell::new(Deque::new()));

pub fn init_usb(usbctrl_dpram: USBCTRL_DPRAM, usbcrtl_regs: USBCTRL_REGS, resets: &mut RESETS, clocks: ClocksManager) {
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        usbcrtl_regs,
        usbctrl_dpram,
        clocks.usb_clock,
        true,
        resets,
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
    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    };
}


/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    free(|cs|
    INTERRUPT_COUNT.borrow(cs).update(|x| x + 1)
    );
    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();
    
    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) && usb_dev.state() == Configured {
        let mut buf = [0u8; 64];
        serial.dtr();
        match serial.read(&mut buf) {
            Err(e) => match e {
                UsbError::ParseError => PARSE_ERROR_COUNT.inc(),
                UsbError::WouldBlock => WOULD_BLOCK_ERROR_COUNT.inc(),
                UsbError::BufferOverflow => BUFFER_OVERFLOW_ERROR_COUNT.inc(),
                UsbError::EndpointOverflow => ENDPOINT_OVERFLOW_ERROR_COUNT.inc(),
                UsbError::EndpointMemoryOverflow => ENDPOINT_MEMORY_OVERFLOW_ERROR_COUNT.inc(),
                UsbError::InvalidEndpoint => INVALID_ENDPOINT_ERROR_COUNT.inc(),
                UsbError::Unsupported => UNSUPPORTED_ERROR_COUNT.inc(),
                UsbError::InvalidState => INVALID_STATE_ERROR_COUNT.inc(),
            },
            Ok(0) => {
                ZERO_READ_COUNT.update(|x| x + 1);
            }
            Ok(count) => {
                // Say hello exactly once on start-up
                if !SAID_HELLO.value() {
                    SAID_HELLO.update(|_| true);
                    let _ = serial.write(b"Hello, World!");
                }
                RX_COUNT.inc();
                // Convert to upper case
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });

                // Send back to the host
                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                        TX_COUNT.update(|x| x + len);
                    });
                }
                let wr_ptr = &buf[..count];
                free(|cs|
                {
                    let mut buffer = RING_BUFFER.borrow(cs).borrow_mut();
                    for &byte in wr_ptr {
                        let result = byte.as_ascii();
                        result.map(|char| buffer.push_front(char).ok());
                    }
                }
                );
            }
        }
    }
}
