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
use embassy_nrf::radio::esb::EsbConfig;
use embassy_nrf::radio::esb::EsbRadio;
use embassy_usb::Builder;
use embassy_nrf::usb::Driver;
use embassy_nrf::pac;
use embassy_executor::Spawner;
use embassy_nrf::radio::esb::esb_packet::EsbPacket;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
});

#[embassy_executor::task]
async fn alive_task(mut led: Output<'static>) {
  log::info!("Starting alive task");
    let mut counter = 0;
    loop {
        led.set_low();
        // log::info!("Hello World from Nrf Radio ESB test {}", counter);
        counter += 1;
        Timer::after_millis(100).await;
        led.set_high();
        Timer::after_secs(1).await;
    }
}
#[embassy_executor::task]
async fn radio_tx_task(mut led: Output<'static> , mut esb_radio: EsbRadio<'static, peripherals::RADIO>) {
  log::info!("Starting radio tx  task");
    let mut counter = 0;
    for _i in 0..10 {
      led.set_low();
      Timer::after_millis(100).await;
      led.set_high();
      Timer::after_millis(400).await;
    }
    loop {
        led.set_low();
        log::info!("SendingHello World from Nrf Radio ESB test to remote {}", counter);
        let bytes = "Hello World Radio ESB ".as_bytes();
        let packet = EsbPacket::tx_packet(&bytes, 1, false);
        if let Err(e) = esb_radio.send_packet(packet).await {
          log::info!("Error while sending packet: {:?}",e);
        }
        counter += 1;
        Timer::after_millis(100).await;
        led.set_high();
        Timer::after_millis(550).await;
    }
}

async fn error_task(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after_millis(50).await;
        led.set_high();
        Timer::after_millis(200).await;
    }
}



#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut led_blue = Output::new(p.P0_12.degrade(), Level::High, OutputDrive::Standard);
    let mut led_green = Output::new(p.P1_09.degrade(), Level::High, OutputDrive::Standard);
    let mut led_red = Output::new(p.P0_08.degrade(), Level::High, OutputDrive::Standard);
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
    let config = EsbConfig::default();
    if let Err(e) =esb_radio.init(&config) {
      log::info!("Initialization error of esb radio: {:?}",e);
      // error_task(led_red).await
    }
    led_red.set_low();
    Timer::after_secs(1).await;
    led_red.set_high();
    led_green.set_low();
    Timer::after_secs(1).await;
    led_green.set_high();
    led_blue.set_low();
    Timer::after_secs(1).await;
    led_blue.set_high();
 

    _ = spawner.spawn(radio_tx_task(led_green, esb_radio)); 
    _ = spawner.spawn(alive_task(led_blue));
    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
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
