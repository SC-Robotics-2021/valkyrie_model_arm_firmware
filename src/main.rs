#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]


// use panic_halt as _;
use panic_rtt_target as _;
use rtic::app;

use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};

use heapless::{consts, Vec};
use nb::block;
use postcard::{flavors, from_bytes_cobs, serialize_with_flavor, Error};
use rtt_target::{rprintln, rtt_init_print};
use serde::{Deserialize, Serialize};

use rover_postcards::{KinematicArmPose, Request, RequestKind, Response, ResponseKind, Status};

use stm32f4::stm32f446::{ADC1};
use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, Clock, Eoc, ExternalTrigger, SampleTime, Scan, Sequence, TriggerMode},
        Adc,
    },
    gpio,
    prelude::*,
    serial, stm32,
    stm32::interrupt,
    timer,
};


type Uart4Tx = gpio::gpioc::PC10<gpio::Alternate<gpio::AF8>>;
type Uart4Rx = gpio::gpioc::PC11<gpio::Alternate<gpio::AF8>>;
type Uart4 = serial::Serial<stm32f4::stm32f446::UART4, (Uart4Tx, Uart4Rx)>;

type Pa4 = gpio::gpioa::PA4<gpio::Analog>;
type Pa5 = gpio::gpioa::PA5<gpio::Analog>;
type Pa6 = gpio::gpioa::PA6<gpio::Analog>;
type Pa7 = gpio::gpioa::PA7<gpio::Analog>;

trait converting {
    fn convert(&mut self);
}
impl converting for KinematicArmPose
{
     fn convert(&mut self) {
        self.lower_axis = to_scale(self.lower_axis);
        self.central_axis = to_scale(self.central_axis);
        self.upper_axis = to_scale(self.upper_axis);
        self.rotation_axis = to_scale(self.rotation_axis);
    }
}
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
        arm_pose: KinematicArmPose,
        arm_adc: ArmAdc,
        uart4: Uart4,
        rx_buffer: cobs_stream::CobsDecoder,
    }

    #[init]
    fn init(context: init::Context) -> init::LateResources {
        #[cfg(debug_assertions)]
        rtt_init_print!();

        #[cfg(debug_assertions)]
        rprintln!("hello, world!");
        /*
            This patch enables the debugger to behave correctly during a WFI
            See Errata: https://www.st.com/content/ccc/resource/technical/document/errata_sheet/c3/6b/f8/32/fc/01/48/6e/DM00155929.pdf/files/DM00155929.pdf/jcr:content/translations/en.DM00155929.pdf#%5B%7B%22num%22%3A37%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C67%2C724%2Cnull%5D
            See Also Github: https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
        */
        // enable the dma1 master
        context
            .device
            .RCC
            .ahb1enr
            .modify(|_, w| w.dma1en().enabled());
        // enable the debugger.
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
        )
            .unwrap();
        uart4.listen(serial::Event::Rxne);
        // Hello world!
        for byte in b"hello from STM32!".iter() {
            block!(uart4.write(*byte)).unwrap();
        }

        // let config = AdcConfig::default().end_of_conversion_interrupt(Eoc::Conversion);

        let config = AdcConfig::default()
            //Set the trigger you want
            .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_1)
            .continuous(stm32f4xx_hal::adc::config::Continuous::Continuous)
            ;
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

        arm_adc
            .adc
            .configure_channel(&arm_adc.pin0, Sequence::One, SampleTime::Cycles_112);
        arm_adc
            .adc
            .configure_channel(&arm_adc.pin1, Sequence::Two, SampleTime::Cycles_112);
        arm_adc
            .adc
            .configure_channel(&arm_adc.pin2, Sequence::Three, SampleTime::Cycles_112);
        arm_adc
            .adc
            .configure_channel(&arm_adc.pin3, Sequence::Four, SampleTime::Cycles_112);

        // Make sure it's enabled but don't start the conversion
        arm_adc.adc.enable();

        //Configure the timer
        // This timer is used for the ADC processing tick.
        let mut timer = timer::Timer::tim1(context.device.TIM1, 1.hz(), clocks);

        // create a periodic timer to check the encoders periodically
        // and be sure to listen for its ticks.
        timer.listen(timer::Event::TimeOut);

        let arm_pose = KinematicArmPose {
            lower_axis: arm_adc.adc.convert(&arm_adc.pin0, SampleTime::Cycles_480) as f32,
            central_axis: arm_adc.adc.convert(&arm_adc.pin1, SampleTime::Cycles_480) as f32,
            upper_axis: arm_adc.adc.convert(&arm_adc.pin2, SampleTime::Cycles_480) as f32,
            rotation_axis: arm_adc.adc.convert(&arm_adc.pin3, SampleTime::Cycles_480) as f32,
        };
        rprintln!("init RX buffer...");

        let mut rx_buffer = cobs_stream::CobsDecoder::new(cobs_stream::Buffer::new());
        rx_buffer
            .reset()
            .expect("Initialization of the RX buffer failed. this shouldn't happen.");
        rprintln!("returning late resources...");
        init::LateResources {
            arm_pose,
            uart4,
            arm_adc,
            rx_buffer,
        }
    }

    #[task(binds = TIM1_UP_TIM10, resources = [arm_adc, arm_pose], priority = 3)]
    fn tim6_interrupt(context: tim6_interrupt::Context) {
        // arm adc critical section
        let mut arm_ptr = context.resources.arm_pose;
        let arm_adc_ptr= context.resources.arm_adc;
        // arm_adc_ptr.sample_to_millivolts(arm_adc_ptr.read()
        arm_ptr.lock(|arm_ptr| {
            let lower = arm_adc_ptr
                .adc
                .convert(&mut arm_adc_ptr.pin0, SampleTime::Cycles_480);
            let center = arm_adc_ptr
                .adc
                .convert(&mut arm_adc_ptr.pin1, SampleTime::Cycles_480);
            let upper = arm_adc_ptr
                .adc
                .convert(&mut arm_adc_ptr.pin2, SampleTime::Cycles_480);
            let rotation = arm_adc_ptr
                .adc
                .convert(&mut arm_adc_ptr.pin3, SampleTime::Cycles_480);

            arm_ptr.lower_axis = arm_adc_ptr
                .adc
                .sample_to_millivolts(lower).into();
            arm_ptr.central_axis = arm_adc_ptr
                .adc
                .sample_to_millivolts(center).into();
            arm_ptr.upper_axis = arm_adc_ptr
                .adc
                .sample_to_millivolts(upper).into();
            arm_ptr.rotation_axis = arm_adc_ptr
                .adc
                .sample_to_millivolts(rotation).into();
            // #[cfg(debug_assertions)]
            // rprintln!("[pre-convert] observed state := {:?}", arm_ptr);
            arm_ptr.convert();

            #[cfg(debug_assertions)]
            rprintln!("observed state := {:?}", arm_ptr);
            // rprintln!("{:?}", arm_adc_ptr.adc.sample_to_millivolts(arm_ptr.lower_axis));
        });
    }

    #[task(binds = UART4, resources = [uart4, arm_pose, rx_buffer], priority = 10)]
    fn uart4_on_rxne(mut context: uart4_on_rxne::Context) {
        let connection: &mut Uart4 = context.resources.uart4;
        let decoder: &mut cobs_stream::CobsDecoder = &mut context.resources.rx_buffer;

        match connection.read() {
            Err(e) => {
                #[cfg(debug_assertions)]
                rprintln!("Failed to read byte due to {:?}... discarding buffer.", e);
                decoder.reset().expect("failed to reset stream decoder...");

                let response = rover_postcards::Response {
                    status: rover_postcards::Status::DecodeError,
                    state: -1,
                    data: None,
                };
                let buf: heapless::Vec<u8, heapless::consts::U32> =
                    postcard::to_vec_cobs(&response).unwrap();
                for byte in buf.iter() {
                    block!(context.resources.uart4.write(*byte)).unwrap();
                }

            }
            Ok(rx_byte) => {
                #[cfg(debug_assertions)]
                rprintln!("feeding byte {:?} ", rx_byte);
                match decoder.feed(rx_byte) {
                    Ok(None) => {} // needs more bytes, nothing to do.
                    Err(e) => {
                        #[cfg(debug_assertions)]
                        rprintln!("Decoding failure := {:?} !", e);
                        let response = rover_postcards::Response {
                            status: rover_postcards::Status::DecodeError,
                            state: -1,
                            data: None,
                        };
                        let buf: heapless::Vec<u8, heapless::consts::U32> =
                            postcard::to_vec_cobs(&response).unwrap();
                        for byte in buf.iter() {
                            block!(context.resources.uart4.write(*byte)).unwrap()
                        }
                    }
                    Ok(Some(_)) => {
                        let request: postcard::Result<rover_postcards::Request> =
                            postcard::from_bytes(&decoder.dest);

                        #[cfg(debug_assertions)]
                        rprintln!("recv'ed request {:?}", request);
                        match request {
                            Err(_) => {
                                let response = rover_postcards::Response {
                                    status: rover_postcards::Status::DecodeError,
                                    state: -1,
                                    data: None,
                                };
                                let buf: heapless::Vec<u8, heapless::consts::U32> =
                                    postcard::to_vec_cobs(&response).unwrap();
                                for byte in buf.iter() {
                                    block!(connection.write(*byte)).unwrap()
                                }
                            }
                            Ok(request) => {
                                let response = match request.kind {
                                    rover_postcards::RequestKind::GetKinematicArmPose => {
                                        rover_postcards::Response {
                                            status: rover_postcards::Status::OK,
                                            state: request.state,
                                            data: Some(ResponseKind::KinematicArmPose(
                                                *context.resources.arm_pose,
                                            )),
                                        }
                                    }

                                    _ => Response {
                                        status: rover_postcards::Status::Unimplemented,
                                        state: request.state,
                                        data: None,
                                    },
                                };

                                #[cfg(debug_assertions)]
                                rprintln!("writing response: {:?}", response);
                                let buf: heapless::Vec<u8, heapless::consts::U1024> =
                                    postcard::to_vec_cobs(&response).unwrap();
                                for byte in buf.iter() {
                                    block!(context.resources.uart4.write(*byte)).unwrap()
                                }
                            }
                        }

                        decoder.reset().expect("failed to reset stream decoder...");
                    }
                }
            }
        }
    }
};

pub fn to_scale(value: f32) -> f32 {
    2.0/3335.0 * value - 1.0
}
