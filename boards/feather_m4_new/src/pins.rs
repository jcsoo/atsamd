//! Feather M4 Express pins

use super::{hal, pac::MCLK, pac::SERCOM1, pac::SERCOM2, pac::SERCOM5, target_device};

use embedded_hal::timer::{CountDown, Periodic};
use hal::define_pins;
use hal::gpio::{self, *};
use hal::sercom::{I2CMaster2, PadPin, SPIMaster1, Sercom2Pad0, Sercom2Pad1, UART5};
use hal::time::Hertz;

use hal::clock::GenericClockController;
use super::pac::gclk::{genctrl::SRC_A, pchctrl::GEN_A};

use apa102_spi::Apa102;

#[cfg(feature = "usb")]
use hal::usb::usb_device::bus::UsbBusAllocator;
#[cfg(feature = "usb")]
pub use hal::usb::UsbBus;

define_pins!(
    /// Maps the pins to their arduino names and
    /// the numbers printed on the board.
    struct Pins,
    target_device: target_device,

    /// Analog pin 0.  Can act as a true analog output
    /// as it has a DAC (which is not currently supported
    /// by this hal) as well as input.
    pin a0 = a2,

    /// Analog Pin 1
    pin a1 = a5,
    /// Analog Pin 2
    pin a2 = b8,
    /// Analog Pin 3
    pin a3 = b9,
    /// Analog Pin 4
    pin a4 = a4,
    /// Analog Pin 5
    pin a5 = a6,

    /// Pin 0, rx
    pin d0 = b17,
    /// Pin 1, tx
    pin d1 = b16,
    /// Pin 4, PWM capable
    pin d4 = a14,

    /// Pin 5, PWM capable
    pin d5 = a16,
    /// Pin 6, PWM capable
    pin d6 = a18,
    /// Neopixel Pin
    pin neopixel = b3,
    /// Pin 9, PWM capable.  Also analog input (A7)
    pin d9 = a19,
    /// Pin 10, PWM capable
    pin d10 = a20,
    /// Pin 11, PWM capable
    pin d11 = a21,
    /// Pin 12, PWM capable
    pin d12 = a22,
    /// Pin 13, which is also attached to
    /// the red LED.  PWM capable.
    pin d13 = a23,

    /// The I2C data line
    pin i2c_sda = a12,
    /// The I2C clock line
    pin i2c_scl = a13,

    /// The SPI SCK
    pin sck = a17,
    /// The SPI MOSI
    pin mosi = b23,
    /// The SPI MISO
    pin miso = b22,

    /// The USB D- pad
    pin usb_dm = a24,
    /// The USB D+ pad
    pin usb_dp = a25,
);

impl Pins {
    /// Split the device pins into subsets
    pub fn split(self) -> Sets {
        let analog = Analog {
            a0: self.a0,
            a1: self.a1,
            a2: self.a2,
            a3: self.a3,
            a4: self.a4,
            a5: self.a5,
        };

        let led_pin = self.d13;

        let neopixel = Neopixel {
            ws_data: self.neopixel,
        };

        // let flash = QSPIFlash {
        //     sck: self.flash_sck,
        //     cs: self.flash_cs,
        //     d0: self.flash_d0,
        //     d1: self.flash_d1,
        //     d2: self.flash_d2,
        //     d3: self.flash_d3,
        // };

        let spi = SPI {
            sck: self.sck,
            mosi: self.mosi,
            miso: self.miso,
        };

        let i2c = I2C {
            sda: self.i2c_sda,
            scl: self.i2c_scl,
        };

        let usb = USB {
            dm: self.usb_dm,
            dp: self.usb_dp,
        };

        let uart = UART {
            rx: self.d0,
            tx: self.d1,
        };

        Sets {
            analog,
            led_pin,
            neopixel,
            spi,
            i2c,
            // flash,
            usb,
            uart,
            port: self.port,
        }
    }
}

/// Sets of pins split apart by category
pub struct Sets {
    /// Analog pins
    pub analog: Analog,

    /// Red Led
    pub led_pin: Pa23<Input<Floating>>,

    pub neopixel: Neopixel,

    /// SPI (external pinout) pins
    pub spi: SPI,

    /// I2C (external pinout) pins
    pub i2c: I2C,

    // /// QSPI Flash pins
    // pub flash: QSPIFlash,

    /// USB pins
    pub usb: USB,

    /// UART (external pinout) pins
    pub uart: UART,

    /// Port
    pub port: Port,
}

/// SPI pins
pub struct SPI {
    pub sck: gpio::Pa17<Input<Floating>>,
    pub mosi: gpio::Pb23<Input<Floating>>,
    pub miso: gpio::Pb22<Input<Floating>>,
}

impl SPI {
    /// Convenience for setting up the labelled pins to operate
    /// as an SPI master, running at the specified frequency.
    pub fn spi_master<F: Into<Hertz>>(
        self,
        clocks: &mut GenericClockController,
        bus_speed: F,
        sercom1: SERCOM1,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> SPIMaster1<
        hal::sercom::Sercom1Pad2<gpio::Pb22<gpio::PfC>>,
        hal::sercom::Sercom1Pad3<gpio::Pb23<gpio::PfC>>,
        hal::sercom::Sercom1Pad1<gpio::Pa17<gpio::PfC>>
> {
        let gclk0 = clocks.gclk0();
        SPIMaster1::new(
            &clocks.sercom1_core(&gclk0).unwrap(),
            bus_speed.into(),
            hal::hal::spi::Mode {
                phase: hal::hal::spi::Phase::CaptureOnFirstTransition,
                polarity: hal::hal::spi::Polarity::IdleLow,
            },
            sercom1,
            mclk,
            (
                self.miso.into_pad(port),
                self.mosi.into_pad(port),
                self.sck.into_pad(port),
            ),
        )
    }
}

/// I2C pins
pub struct I2C {
    pub sda: Pa12<Input<Floating>>,
    pub scl: Pa13<Input<Floating>>,
}

impl I2C {
    /// Convenience for setting up the labelled SDA, SCL pins to
    /// operate as an I2C master running at the specified frequency.
    pub fn i2c_master<F: Into<Hertz>>(
        self,
        clocks: &mut GenericClockController,
        bus_speed: F,
        sercom2: SERCOM2,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> I2CMaster2<Sercom2Pad0<Pa12<PfC>>, Sercom2Pad1<Pa13<PfC>>> {
        let gclk0 = clocks.gclk0();
        I2CMaster2::new(
            &clocks.sercom2_core(&gclk0).unwrap(),
            bus_speed.into(),
            sercom2,
            mclk,
            self.sda.into_pad(port),
            self.scl.into_pad(port),
        )
    }
}

/// USB pins
pub struct USB {
    pub dm: Pa24<Input<Floating>>,
    pub dp: Pa25<Input<Floating>>,
}


impl USB {
    #[cfg(feature = "usb")]
    pub fn usb_allocator(
        self,
        usb: super::pac::USB,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> UsbBusAllocator<UsbBus> {
        clocks.configure_gclk_divider_and_source(GEN_A::GCLK2, 1, SRC_A::DFLL, false);
        let usb_gclk = clocks.get_gclk(GEN_A::GCLK2).unwrap();
        let usb_clock = &clocks.usb(&usb_gclk).unwrap();

        UsbBusAllocator::new(UsbBus::new(
            usb_clock,
            mclk,
            self.dm.into_function(port),
            self.dp.into_function(port),
            usb,
        ))
    }

    #[cfg(feature = "usb")]
    /// Convenience for setting up the onboard usb port to operate
    /// as a USB device.
    pub fn init(
        self,
        usb: super::pac::USB,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> UsbBusAllocator<UsbBus> {
        clocks.configure_gclk_divider_and_source(GEN_A::GCLK2, 1, SRC_A::DFLL, false);
        let usb_gclk = clocks.get_gclk(GEN_A::GCLK2).unwrap();
        let usb_clock = &clocks.usb(&usb_gclk).unwrap();

        UsbBusAllocator::new(UsbBus::new(
            usb_clock,
            mclk,
            self.dm.into_function(port),
            self.dp.into_function(port),
            usb,
        ))
    }    
}

/// UART pins
pub struct UART {
    pub rx: Pb17<Input<Floating>>,
    pub tx: Pb16<Input<Floating>>,
}

impl UART {
    /// Convenience for setting up the labelled TX, RX pins in the
    /// to operate as a UART device at the specified baud rate.
    pub fn uart<F: Into<Hertz>>(
        self,
        clocks: &mut GenericClockController,
        baud: F,
        sercom5: SERCOM5,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> UART5<
        hal::sercom::Sercom5Pad1<gpio::Pb17<gpio::PfC>>,
        hal::sercom::Sercom5Pad0<gpio::Pb16<gpio::PfC>>,
        (),
        (),
    > {
        let gclk0 = clocks.gclk0();

        UART5::new(
            &clocks.sercom5_core(&gclk0).unwrap(),
            baud.into(),
            sercom5,
            mclk,
            (self.rx.into_pad(port), self.tx.into_pad(port)),
        )
    }
}

/// Analog pins
pub struct Analog {
    pub a0: Pa2<Input<Floating>>,
    pub a1: Pa5<Input<Floating>>,
    pub a2: Pb8<Input<Floating>>,
    pub a3: Pb9<Input<Floating>>,
    pub a4: Pa4<Input<Floating>>,
    pub a5: Pa6<Input<Floating>>,
}

pub struct Neopixel {
    pub ws_data: Pb3<Input<Floating>>,
}

// /// Dotstar pins
// pub struct Dotstar {
//     pub ci: Pb2<Input<Floating>>,
//     pub di: Pb3<Input<Floating>>,
//     // Pb0 is NC on the ItsyBitsy M4, so its safe to use
//     // as the MISO given the HAL requires it.
//     pub nc: Pb0<Input<Floating>>,
// }

// impl Dotstar {
//     pub fn dotstar<T: CountDown + Periodic>(
//         self,
//         port: &mut Port,
//         timer: T,
//     ) -> apa102_spi::Apa102<
//         bitbang_hal::spi::SPI<Pb0<Input<PullUp>>, Pb3<Output<PushPull>>, Pb2<Output<PushPull>>, T>,
//     > {
//         let di = self.di.into_push_pull_output(port);
//         let ci = self.ci.into_push_pull_output(port);
//         let nc = self.nc.into_pull_up_input(port);

//         let spi = bitbang_hal::spi::SPI::new(apa102_spi::MODE, nc, di, ci, timer);
//         Apa102::new_with_custom_postamble(spi, 4, false)
//     }
// }

// /// QSPI flash pins
// pub struct QSPIFlash {
//     pub sck: Pb10<Input<Floating>>,
//     pub cs: Pb11<Input<Floating>>,
//     pub d0: Pa8<Input<Floating>>,
//     pub d1: Pa9<Input<Floating>>,
//     pub d2: Pa10<Input<Floating>>,
//     pub d3: Pa11<Input<Floating>>,
// }
