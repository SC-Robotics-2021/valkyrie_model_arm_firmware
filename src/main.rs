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
use stm32f4xx_hal::{gpio, prelude::*, serial, timer, adc::{
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

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct ModelArm {
    pub lower_axis: u16,
    pub central_axis: u16,
    pub upper_axis: u16,
    pub rotation_axis: u16,
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

pub struct ArmAdc {
    pub adc: stm32f4xx_hal::adc::Adc<ADC1>,
    pub pin0: Pa4,
    pub pin1: Pa5,
    pub pin2: Pa6,
    pub pin3: Pa7,
}

#[app(device = stm32f4::stm32f446, peripherals = true)]
const APP: () = {
    struct Resources {
        arm: ModelArm,
        arm_adc: ArmAdc,
        uart4: Uart4,
    }

    #[init]
    fn init(context: init::Context) -> init::LateResources {
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
        uart4.listen(serial::Event::Rxne);
        // Hello world!
        for byte in b"hello from STM32!".iter() {
            block!(uart4.write(*byte)).unwrap();
        }

        // let config = AdcConfig::default().end_of_conversion_interrupt(Eoc::Conversion);

        let config = AdcConfig::default()
            //Set the trigger you want
            .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_1);
        let adc = Adc::adc1(context.device.ADC1, true, config);
        let adc_pins = (
            gpioa.pa4.into_analog(), // ADC pin for lower axis
            gpioa.pa5.into_analog(), // ADC pin for central axis
            gpioa.pa6.into_analog(), // ADC pin for upper axis
            gpioa.pa7.into_analog(), // ADC pin for rotation axis
        );
        let mut arm_adc = ArmAdc {
            adc,
            pin0: adc_pins.0,
            pin1: adc_pins.1,
            pin2: adc_pins.2,
            pin3: adc_pins.3,
        };

        arm_adc.adc.configure_channel(&arm_adc.pin0, Sequence::One, SampleTime::Cycles_112);
        arm_adc.adc.configure_channel(&arm_adc.pin1, Sequence::One, SampleTime::Cycles_112);
        arm_adc.adc.configure_channel(&arm_adc.pin2, Sequence::One, SampleTime::Cycles_112);
        arm_adc.adc.configure_channel(&arm_adc.pin3, Sequence::One, SampleTime::Cycles_112);

        // Make sure it's enabled but don't start the conversion
        arm_adc.adc.enable();

        //Configure the timer
        let mut timer = timer::Timer::tim6(context.device.TIM6, 1.hz(), clocks);

        // create a periodic timer to check the encoders periodically
        // and be sure to listen for its ticks.
        timer.listen(timer::Event::TimeOut);


        let mut arm = ModelArm {
            lower_axis: arm_adc.adc.convert(&arm_adc.pin0, SampleTime::Cycles_480),
            central_axis: arm_adc.adc.convert(&arm_adc.pin1, SampleTime::Cycles_480),
            upper_axis: arm_adc.adc.convert(&arm_adc.pin2, SampleTime::Cycles_480),
            rotation_axis: arm_adc.adc.convert(&arm_adc.pin3, SampleTime::Cycles_480),
        };

        init::LateResources {
            arm,
            uart4,
            arm_adc,
        }
    }

    #[task(binds = TIM6_DAC, resources = [arm_adc, arm], priority = 3)]
    fn tim6_interrupt(context: tim6_interrupt::Context) {
        // arm adc critical section
        let mut arm_ptr = context.resources.arm;
        let mut arm_adc_ptr = context.resources.arm_adc;
        arm_ptr.lock(|arm_ptr| {
            arm_ptr.lower_axis = arm_adc_ptr.adc.convert(&mut arm_adc_ptr.pin0, SampleTime::Cycles_480);
            arm_ptr.central_axis = arm_adc_ptr.adc.convert(&mut arm_adc_ptr.pin1, SampleTime::Cycles_480);
            arm_ptr.upper_axis = arm_adc_ptr.adc.convert(&mut arm_adc_ptr.pin2, SampleTime::Cycles_480);
            arm_ptr.rotation_axis = arm_adc_ptr.adc.convert(&mut arm_adc_ptr.pin3, SampleTime::Cycles_480);
            // rprintln!("{:?}", arm_adc_ptr.adc.sample_to_millivolts(arm_ptr.lower_axis));
        });
    }

    #[task(binds = UART4, resources = [uart4, arm], priority = 10)]
    fn uart4_on_rxne (context: uart4_on_rxne::Context){
        let response =
            protocol::Response {
                status: protocol::Status::OK,
                data: Some(postcard::to_vec(context.resources.arm).unwrap())
            };
        rprintln!("{:?}", response);
        let buf: heapless::Vec<u8, heapless::consts::U1024> =
            postcard::to_vec_cobs(&response).unwrap();
        for byte in buf.iter() {
            block!(context.resources.uart4.write(*byte)).unwrap()
        }

    }
};