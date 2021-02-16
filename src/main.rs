// #![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

mod protocol;

// use panic_halt as _;
use panic_rtt_target as _;
// use cortex_m_rt::entry;
use rtic::{app};

use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};

use heapless::{consts, Vec};
use nb::block;
use rtt_target::{rprintln, rtt_init_print};
use postcard::{flavors, from_bytes_cobs, serialize_with_flavor, Error};
use serde::{Deserialize, Serialize};

use stm32f4::stm32f446::{ADC1, TIM1};
use stm32f4xx_hal::{gpio, prelude::*, delay::Delay, i2c::I2c, serial, timer, adc::{
    Adc,
    config::AdcConfig,
    config::SampleTime,
    config::Sequence,
    config::Eoc,
    config::Scan,
    config::Clock,
    config::TriggerMode,
    config::ExternalTrigger,
}, stm32::interrupt, stm32};

use crate::protocol::Response;
use core::{cell::RefCell, borrow::{Borrow, BorrowMut}};

type Uart4Tx = gpio::gpioc::PC10<gpio::Alternate<gpio::AF8>>;
type Uart4Rx = gpio::gpioc::PC11<gpio::Alternate<gpio::AF8>>;
type Uart4 = serial::Serial<stm32f4::stm32f446::UART4, (Uart4Tx, Uart4Rx)>;

type Pa4 = gpio::gpioa::PA4<gpio::Analog>;
type Pa5 = gpio::gpioa::PA5<gpio::Analog>;
type Pa6 = gpio::gpioa::PA6<gpio::Analog>;
type Pa7 = gpio::gpioa::PA7<gpio::Analog>;

static ADC: Mutex<RefCell<Option<Adc<stm32::ADC1>>>> = Mutex::new(RefCell::new(None));

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct ModelArm {
    lower_axis    : u16,
    central_axis  : u16,
    upper_axis    : u16,
    rotation_axis : u16,
}

// impl ModelArm {
//     pub fn new() -> ModelArm
//     {
//         ModelArm {
//             angle1: 0,
//             angle2: 0,
//             angle3: 0,
//             angle4: 0,
//         }
//     }
//
//     pub fn convert_voltage(&self, adc: &mut Adc<ADC1>, adc_pins: &(Pa4, Pa5, Pa6, Pa7)) -> [u16; 4]
//     {
//             let voltages: [u16; 4] = [
//                 adc.convert(&adc_pins.0, SampleTime::Cycles_480),
//                 adc.convert(&adc_pins.1, SampleTime::Cycles_480),
//                 adc.convert(&adc_pins.2, SampleTime::Cycles_480),
//                 adc.convert(&adc_pins.3, SampleTime::Cycles_480),
//             ];
//
//         voltages
//     }
//
//     pub fn set_angles(&self, voltage: &[u16; 4]) -> [u16; 4]
//     {
//         let mut angles= [0; 4];
//
//         for (degree, volt) in angles.iter_mut().zip(voltage.iter())
//         {
//             let angle = (180_f32 / 3333_f32) * *volt as f32;
//             let mut angle = angle as u16;
//             if angle >= 180
//             {
//                 angle = 180;
//             }
//             *degree = angle;
//         }
//
//         angles
//     }
//
//     pub fn get_angles(&self, angles: [u16; 4]) -> ModelArm
//     {
//         ModelArm
//         {
//             angle1: angles[0],
//             angle2: angles[1],
//             angle3: angles[2],
//             angle4: angles[3],
//         }
//     }
// }

pub struct ArmAdc{
    adc: stm32f4xx_hal::adc::Adc<ADC1>
}

#[app(device = stm32f4::stm32f446, peripherals = true)]
const APP: () = {
    struct Resources {
        arm: ModelArm,
        uart4: Uart4,
        // adc_pins: (Pa4, Pa5, Pa6, Pa7),
    }

    #[init]
	fn init(context: init::Context) -> init::LateResources{
		rtt_init_print!();

        context
            .device
            .RCC
            .ahb1enr
            .modify(|_, w| w.dma1en().enabled());
        context.device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });

		let rcc = context.device.RCC.constrain();
		let clocks = rcc.cfgr.freeze();
        let gpioa = context.device.GPIOA.split();
        let gpioc = context.device.GPIOC.split();

        // Create a delay abstraction based on SysTick
        let mut _delay = Delay::new(context.core.SYST, clocks);

		let uart4_channels = (
			gpioc.pc10.into_alternate_af8(), // UART4_TX
			gpioc.pc11.into_alternate_af8(), // UART4_RX
		);

        let mut uart4 = serial::Serial::uart4(
			context.device.UART4,
			uart4_channels,
			serial::config::Config {
				baudrate: 9600.bps(),
				wordlength: serial::config::WordLength::DataBits8,
				parity: serial::config::Parity::ParityNone,
				stopbits: serial::config::StopBits::STOP1,
			},
			clocks,
		).unwrap();

        // uart4.listen(serial::Event::Rxne);

        // let config = AdcConfig::default().end_of_conversion_interrupt(Eoc::Conversion);

        let config = AdcConfig::default()
            //Set the trigger you want
            .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_1);
        let mut adc = Adc::adc1(context.device.ADC1, true, config);
        let adc_pins= (
            gpioa.pa4.into_analog(), // ADC pin for lower axis
            gpioa.pa5.into_analog(), // ADC pin for central axis
            gpioa.pa6.into_analog(), // ADC pin for upper axis
            gpioa.pa7.into_analog(), // ADC pin for rotation axis
        );

        adc.configure_channel(&adc_pins.0, Sequence::One, SampleTime::Cycles_112);
        adc.configure_channel(&adc_pins.1, Sequence::One, SampleTime::Cycles_112);
        adc.configure_channel(&adc_pins.2, Sequence::One, SampleTime::Cycles_112);
        adc.configure_channel(&adc_pins.3, Sequence::One, SampleTime::Cycles_112);

        // Make sure it's enabled but don't start the conversion
        adc.enable();

        //Configure the timer
        let mut timer = timer::Timer::tim6(context.device.TIM6, 1.hz(), clocks);

        // create a periodic timer to check the encoders periodically
        // and be sure to listen for its ticks.
        timer.listen(timer::Event::TimeOut);


        let arm = ModelArm{
            lower_axis: adc.convert(&adc_pins.0, SampleTime::Cycles_480),
            central_axis: adc.convert(&adc_pins.1, SampleTime::Cycles_480),
            upper_axis: adc.convert(&adc_pins.2, SampleTime::Cycles_480),
            rotation_axis: adc.convert(&adc_pins.3, SampleTime::Cycles_480),
        };

        init::LateResources {
            arm,
            uart4,
            // adc_pins,
        }
	}

    #[task(binds = TIM6_DAC, resources = [arm], priority = 3)]
    fn tim6_interrupt(context: tim6_interrupt::Context) {
        context.resources.arm.lock(|cs| {



            if let Some(ref mut adc) = ADC.borrow(cs).borrow_mut().deref_mut() {
                let sample = adc.convert(&pins, SampleTime::Cycles_480);
                // rprintln!("{}", sample);
            }
        });
    }

    // #[task(binds = UART4, resources = [uart4, arm], priority = 1)]
    // fn uart4_on_rxne (context: uart4_on_rxne::Context){
    //     let response =
    //         protocol::Response
    //         {
    //             status: protocol::Status::OK,
    //             data: Some(postcard::to_vec(&context.resources.arm).unwrap())
    //         };
    //     let buf: heapless::Vec<u8, heapless::consts::U1024> =
    //         postcard::to_vec_cobs(&response).unwrap();
    //     for byte in buf.iter() {
    //         block!(context.resources.uart4.write(*byte)).unwrap()
    //     }
    // }
};

// let i2c_channels = (
// 	gpiob.pb8.into_alternate_af4_open_drain(),
// 	gpiob.pb9.into_alternate_af4_open_drain(),
// );

// let mut i2c = I2c::i2c1 (
// 	context.device.I2C1,
// 	i2c_channels,
// 	100.khz(),
// 	clocks
// );