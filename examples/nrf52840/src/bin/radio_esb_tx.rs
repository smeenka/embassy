#![no_std]
#![no_main]
use core::mem;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use defmt::{panic};
use embassy_nrf::gpio::{Level, Output, OutputDrive, Pin};

use embassy_nrf::{bind_interrupts, peripherals, usb};
use embassy_time::Timer;
use embassy_usb::driver::EndpointError;
use {defmt_rtt as _, panic_probe as _};
use embassy_nrf::radio;
use embassy_usb::Config;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_nrf::radio::esb::{EsbConfig, EsbRadio, EsbRadioCommand, EsbRadioEvent, ERadioEvent, ERadioCommand, EsbRadioAck};
use embassy_usb::Builder;
use embassy_nrf::usb::Driver;
use embassy_nrf::pac;
use embassy_executor::Spawner;
use embassy_nrf::radio::esb::esb_packet::EsbPacket;
use crate::radio::esb::ECrcSize;


bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
});

#[embassy_executor::task]
async fn alive_task() {
  Timer::after_secs(3).await;
  log::info!("Starting alive task");
    loop {
        log::info!("Hello World from Nrf Radio ESB test");
        Timer::after_secs(10).await;
    }
}
#[embassy_executor::task]
async fn test_commands_task(mut led: Output<'static> ) {
    for _i in 0..10 {
      led.set_low();
      Timer::after_millis(100).await;
      led.set_high();
      Timer::after_millis(400).await;
    }
    log::info!("Test send packet with ack");
    Timer::after_millis(100).await;
    let mut counter = 0;
    loop {
        led.set_low();
        log::info!("App: Sending # {}", counter);
        //let bytes = "Hello World Radio ESB ".as_bytes();
        let bytes = [0x41_u8, 0x62, 0x63];
        let packet = EsbPacket::tx_packet(&bytes, 2, false);
        EsbRadioCommand::send(ERadioCommand::Data(packet)).await; 
        Timer::after_millis(100).await;
        led.set_high();
        counter += 1;
        Timer::after_millis(2000).await;
    }
}

#[embassy_executor::task]
async fn test_events_task(mut led: Output<'static>) {
    let mut data0 = 0_u8;
    let mut counter = 0;
    Timer::after_secs(3).await;
    // fill the reuse channel with enough empty packets
    for _i in 0..5 {
      EsbRadioEvent::reuse_rx_packet(EsbPacket::empty());
    }
    for _ in 0..2 {
      for i in 0..5 {
        let mut data = [b'c'; 10];
        data[0] = b'a' + i;
        EsbRadioAck::send(EsbPacket::ack_packet(&data, i, counter)).await;
        counter += 1;
      }
    }
    // EsbRadioCommand::send(ERadioCommand::AckReporting(true)).await; // not yet functional. hangs!!

    log::info!("Waiting for incoming events");
    loop {
        let event = EsbRadioEvent::receive().await;
        led.set_low();
        match event {
          ERadioEvent::Data(packet) => {
            log::info!("Received packet:{:?}", packet);
            data0 += 1;
            if data0 == 128 {
              data0 = 32;
            }
            let mut data = [b'c'; 10];
            data[0] = data0;
            _ = EsbRadioAck::try_send(EsbPacket::ack_packet(&data, packet.pipe_nr(), counter));
            counter += 1;
            EsbRadioEvent::reuse_rx_packet(packet);
          }
          ERadioEvent::AckReporting(report) => {
            log::info!("Received reporting: {:?}", report)
          }
          
        }
        Timer::after_millis(50).await;
        led.set_high();
    }
}
#[embassy_executor::task]
async fn radio_driver_task(mut radio: EsbRadio<'static, peripherals::RADIO>, mut led: Output<'static>) {
    Timer::after_secs(5).await;
    log::info!("Starting the radio statemachine task inside the esb radio driver");
    loop {
        match radio.statemachine_runonce().await {
          Ok(_) => (),
          Err(e) => {
            log::info!("Error did happen: {:?}", e);
            led.set_low();
            Timer::after_millis(100);
            led.set_high();
          }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let led_blue = Output::new(p.P0_12.degrade(), Level::High, OutputDrive::Standard);
    let led_green = Output::new(p.P1_09.degrade(), Level::High, OutputDrive::Standard);
    let led_red = Output::new(p.P0_08.degrade(), Level::High, OutputDrive::Standard);
    let clock: pac::CLOCK = unsafe { mem::transmute(()) };

    log::info!("Enabling ext hfosc...");
    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while clock.events_hfclkstarted.read().bits() != 1 {}

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("NRF_Radio_ESB_TX_example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut logger_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);

    // Creates the logger and returns the logger future
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();
    
    let mut esb_radio: EsbRadio<'static, peripherals::RADIO> = EsbRadio::new(p.RADIO, Irqs);
    let mut config = EsbConfig::default();
    config.set_crc_size(ECrcSize::Size2);
    if let Err(e) =esb_radio.init(&config) {
      log::info!("Initialization error of esb radio: {:?}",e);
    }

    _ = spawner.spawn(test_commands_task(led_blue)); 
    _ = spawner.spawn(alive_task());
    _ = spawner.spawn(test_events_task(led_green));
    _ = spawner.spawn(radio_driver_task(esb_radio, led_red));

    // Run everything concurrently.
    join(usb_fut, log_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
