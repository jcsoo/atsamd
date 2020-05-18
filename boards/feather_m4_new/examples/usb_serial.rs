#![no_std]
#![no_main]

/// Makes the feather_m4 appear as a USB serial port. The color of the
/// dotstar LED can be changed by sending bytes to the serial port.
///
/// Sending the characters R, G, and O set the LED red, green, and off
/// respectively. For example:
/// $> sudo stty -F /dev/ttyACM0 115200 raw -echo
/// $> sudo bash -c "echo 'R' > /dev/ttyACM0"
/// $> sudo bash -c "echo 'G' > /dev/ttyACM0"
/// $> sudo bash -c "echo 'O' > /dev/ttyACM0"
extern crate feather_m4 as hal;
extern crate panic_halt;

use ws2812_timer_delay as ws2812;
use hal::clock::GenericClockController;


use cortex_m::interrupt::free as disable_interrupts;
use cortex_m::peripheral::NVIC;
use hal::entry;
use hal::pac::{interrupt, CorePeripherals, Peripherals};

use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;

use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use hal::timer::SpinTimer;
use smart_leds::{hsv::RGB8, SmartLedsWrite};

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut pins = hal::Pins::new(peripherals.PORT).split();

    // (Re-)configure PB3 as output
    let ws_data_pin = pins.neopixel.ws_data.into_push_pull_output(&mut pins.port);
    // Create a spin timer whoes period will be 9 x 120MHz clock cycles (75ns)
    let timer = SpinTimer::new(9);
    let mut neopixel = ws2812::Ws2812::new(timer, ws_data_pin);

    // let mut rgb = hal::dotstar_bitbang(pins.dotstar, &mut pins.port, SpinTimer::new(12));
    // rgb.write([RGB8 { r: 0, g: 0, b: 0 }].iter().cloned())
    //     .unwrap();

    // uart_debug::wire_uart(uart(
    //     pins.uart,
    //     &mut clocks,
    //     Hertz(115200),
    //     peripherals.SERCOM5,
    //     &mut peripherals.MCLK,
    //     &mut pins.port,
    // ));
    // dbgprint!(
    //     "\n\n\n\n~========== STARTING {:?} ==========~\n",
    //     hal::serial_number()
    // );
    // dbgprint!("Last reset was from {:?}\n", hal::reset_cause(rstc));

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(pins.usb.init(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.MCLK,
            &mut pins.port,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_DEV = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB_OTHER, 1);
        core.NVIC.set_priority(interrupt::USB_TRCPT0, 1);
        core.NVIC.set_priority(interrupt::USB_TRCPT1, 1);
        NVIC::unmask(interrupt::USB_OTHER);
        NVIC::unmask(interrupt::USB_TRCPT0);
        NVIC::unmask(interrupt::USB_TRCPT1);
    }
 
    loop {
        let pending = disable_interrupts(|_| unsafe {
            let pending = PENDING_COLOR;
            PENDING_COLOR = None;
            pending
        });
        if let Some(colors) = pending {
            neopixel.write(colors.iter().cloned()).unwrap();
        }
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut PENDING_COLOR: Option<[RGB8; 1]> = None;

fn poll_usb() {
    unsafe {
        USB_DEV.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                usb_dev.poll(&mut [serial]);
                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    for (i, c) in buf.iter().enumerate() {
                        if i >= count {
                            break;
                        }
                        match c.clone() as char {
                            'R' | 'r' => {
                                PENDING_COLOR = Some([RGB8 { r: 120, g: 0, b: 0 }]);
                            }
                            'G' | 'g' => {
                                PENDING_COLOR = Some([RGB8 { r: 0, g: 120, b: 0 }]);
                            }
                            'B' | 'b' => {
                                PENDING_COLOR = Some([RGB8 { r: 0, g: 0, b: 120 }]);
                            }
                            'O' | 'o' => {
                                PENDING_COLOR = Some([RGB8 { r: 0, g: 0, b: 0 }]);
                            }
                            _ => {}
                        }
                    }
                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                };
            });
        });
    };
}

#[interrupt]
fn USB_OTHER() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT0() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT1() {
    poll_usb();
}
