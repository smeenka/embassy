use core::future::poll_fn;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use super::esb_config::EsbConfig;
use super::esb_packet::EsbPacket;
use super::esb_state::PipeState;
use super::{ECrcSize, EDataRate, ERadioState, CHANNEL_RX_SIZE, MAX_NR_PIPES, MAX_PACKET_SIZE};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;
use crate::radio::{Error, Instance, InterruptHandler, RadioState};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use log;

fn bytewise_bit_swap(value: u32) -> u32 {
    value.reverse_bits().swap_bytes()
}

#[inline]
fn address_conversion(value: u32) -> u32 {
    value.reverse_bits()
}
const CRC_INIT: u32 = 0x0000_FFFF;
const CRC_POLY: u32 = 0x0001_1021;

#[derive()]
// MaxNrPipes is the maximum number of pipes to use. Note that each configured pipe will use 10 x MaxPacketSize of memory
// MaxPacketSize is the radio packet size. If 34 or less this EsbRadio will be able to communicate with legacy nrf24l01+ radios
// Note that MaxPacketSize must be 2 bytes bigger thant the max packet size of the radio, due to 2 meta fields (len and pid)
pub struct EsbRadio<'d, T: Instance> {
    radio: PeripheralRef<'d, T>,
    channel_rx: Channel<CriticalSectionRawMutex, EsbPacket, CHANNEL_RX_SIZE>,
    current_tx_packet: Option<EsbPacket>,
    pipes: [PipeState; MAX_NR_PIPES],
    radio_state: ERadioState,
}

impl<'d, T: Instance> EsbRadio<'d, T> {
    /// Create a new radio driver.
    pub fn new(
        radio: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        into_ref!(radio);
        // Disable NVIC interrupt
        T::Interrupt::unpend();
        Self {
            radio,
            channel_rx: Channel::new(),
            pipes: [
                PipeState::new(0),
                PipeState::new(1),
                PipeState::new(2),
                PipeState::new(3),
                PipeState::new(4),
            ],
            current_tx_packet: None,
            radio_state: ERadioState::Inactive,
        }
    }

    fn state(&self) -> RadioState {
        crate::radio::state(T::regs())
    }

    /// Configure the device
    /// set all its properties for proper operation and power it up.
    /// The device does remain in standby until self.listen() or self.send() is called.
    ///
    pub fn init(&mut self, config: &EsbConfig) -> Result<(), Error> {
        // Disables all interrupts
        let regs = T::regs();

        if MAX_NR_PIPES > 8 {
            return Err(Error::PipeNrTooHigh);
        }

        regs.intenclr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        match config.data_rate() {
            EDataRate::Dr2Mbps => regs.mode.write(|w| w.mode().nrf_2mbit()),
            EDataRate::Dr1Mbps => regs.mode.write(|w| w.mode().nrf_1mbit()),
        }
        let field_len_bits = if MAX_PACKET_SIZE <= 32 { 6 } else { 8 };
        // Convert addresses to remain compatible with nRF24L devices
        let base0 = address_conversion(u32::from_le_bytes(config.base0()));
        let base1 = address_conversion(u32::from_le_bytes(config.base1()));
        let prefix0 = bytewise_bit_swap(u32::from_le_bytes(config.prefixes0()));
        let prefix1 = bytewise_bit_swap(u32::from_le_bytes(config.prefixes1()));

        regs.shorts.write(|w| {
            w.ready_start()
                .enabled()
                .end_disable()
                .enabled()
                .address_rssistart()
                //.enabled()
                .disabled()
                .disabled_rssistop()
                .enabled()
        });

        regs.txpower.write(|w| w.txpower().variant(config.tx_power()));
        unsafe {
            regs.pcnf0.write(|w| w.lflen().bits(field_len_bits).s1len().bits(3));

            regs.pcnf1.write(|w| {
                w.maxlen()
                    .bits(MAX_PACKET_SIZE as u8)
                    // 4-Byte Base Address + 1-Byte Address Prefix
                    .balen()
                    .bits(4)
                    // Nordic's code doesn't use whitening, maybe enable in the future ?
                    //.whiteen()
                    //.set_bit()
                    .statlen()
                    .bits(0)
                    .endian()
                    .big()
            });

            regs.crcinit.write(|w| w.crcinit().bits(CRC_INIT & 0x00FF_FFFF));

            regs.crcpoly.write(|w| w.crcpoly().bits(CRC_POLY & 0x00FF_FFFF));

            match config.crc_size() {
                ECrcSize::Size1 => regs.crccnf.write(|w| w.len().one()),
                ECrcSize::Size2 => regs.crccnf.write(|w| w.len().two()),
            }
            regs.base0.write(|w| w.bits(base0));
            regs.base1.write(|w| w.bits(base1));

            regs.prefix0.write(|w| w.bits(prefix0));
            regs.prefix1.write(|w| w.bits(prefix1));

            // NOTE(unsafe) `rf_channel` was checked to be between 0 and 100 during the creation of
            // the `Adresses` object
            regs.frequency.write(|w| w.frequency().bits(config.rf_channel()));
        }
        // arm the radio interrupt
        T::Interrupt::pend();

        log::info!("init finished");
        Ok(())
    }

    // add packet for sending. In case packet cannot be added it will be returned in the Err
    // In case the radio is idle the packet will be send immediatly
    // In case the radio is busy, it will send the packet lateron
    pub async fn send_packet(&mut self, mut packet: EsbPacket) -> Result<(), EsbPacket> {
        if self.current_tx_packet.is_some() {
            return Err(packet);
        }
        let pid = self.pipes[packet.pipe_nr() as usize].inc_tx_pid();
        packet.set_pid(pid);

        self.current_tx_packet = Some(packet);
        match self.radio_state {
            ERadioState::Inactive | ERadioState::Receiving => {
                self.radio_state = ERadioState::Transmitting;

                _ = self.transmit();
                // make sure the state machine does make one loop
                self.statemachine_task().await;
            }
            _ => (),
        }
        Ok(())
    }

    pub fn try_add_ack_packet(&mut self, packet: EsbPacket) -> Result<(), Error> {
        let pipe_nr = packet.pipe_nr() as usize;
        match self.pipes[pipe_nr].channel_ack().try_send(packet) {
            Ok(_) => {
                self.pipes[pipe_nr].preload_ack();
                Ok(())
            }
            Err(_) => Err(Error::ChannelFull),
        }
    }
    pub async fn add_ack_packet(&mut self, packet: EsbPacket) {
        let pipe_nr = packet.pipe_nr() as usize;
        self.pipes[pipe_nr].channel_ack().send(packet).await;
        self.pipes[pipe_nr].preload_ack()
    }

    pub fn try_receive_rx_packet(&mut self) -> Result<EsbPacket, Error> {
        match self.channel_rx.try_receive() {
            Ok(p) => Ok(p),
            Err(_) => Err(Error::ChannelEmpty),
        }
    }
    pub async fn receive_rx_packet(&mut self) -> EsbPacket {
        self.channel_rx.receive().await
    }

    /// This state machine is awoken by the interrupt, and runs as an embassy task (NOT in interrupt context)
    ///
    async fn statemachine_task(&mut self) {
        // On poll check if interrupt happen
        let state = T::state();
        poll_fn(|cx| {
            state.event_waker.register(cx.waker());
            self.radio_state_machine()
        })
        .await;
        // compiler_fence(Ordering::SeqCst);
    }

    fn radio_state_machine(&mut self) -> Poll<()> {
        log::info!("State: {:?}", self.radio_state);
        let regs = T::regs();
        match self.radio_state {
            ERadioState::Inactive => (),
            // PTX and PRX role: the radio is requested to start a receing transaction.
            ERadioState::StartReceiving => (),
            // PTX and PRX role: The radio is listening for incoming packets.
            ERadioState::Receiving => (),
            // PTX and PRX role: the radio is requested to start a transmit transacktion.
            ERadioState::StartTransmitting => {
                _ = self.transmit();
                self.radio_state = ERadioState::Transmitting;
                // state machine will be awoken again by the interrupt
                return Poll::Ready(());
            }
            // PTX and PRX role: the radio is busy transmitting packets, until the tx channel is emtpy.
            ERadioState::Transmitting => {
                if regs.events_disabled.read().bits() == 1 {
                    // did send the packet successfull, drop packet
                    _ = self.current_tx_packet.take();
                    log::info!("Send Ready");
                    self.radio_state = ERadioState::Receiving;
                    return Poll::Ready(());
                }
                log::info!("Send Pending");
            }
            // the radio is busy transmitting the ack packet. Only for PRX role
            ERadioState::TransmittingAck => {}
            // PTX role: The radio did send a packet and is now listening for the incoming ack of the remote
            ERadioState::TransmitterWaitAck => {
                if regs.events_disabled.read().bits() == 1 {
                    // did receive the ack packet. Drop tx packet
                    _ = self.current_tx_packet.take();
                    // TODO:handling of the received packet
                    log::info!("Send fully Ready");
                    self.radio_state = ERadioState::Receiving;
                    return Poll::Ready(());
                }
            }
        }
        Poll::Pending
    }

    /// Transmit a packet. Setup interrupts and events
    fn transmit(&mut self) -> Result<(), Error> {
        let regs = T::regs();

        if let Some(tx_packet) = &self.current_tx_packet {
            if tx_packet.ack() {
                // Go to RX mode after the transmission
                regs.shorts.modify(|_, w| w.disabled_rxen().enabled());
            }
            // reset relevant events
            regs.events_disabled.reset();
            regs.events_ready.reset();
            regs.events_end.reset();
            regs.events_address.reset();
            regs.events_payload.reset();

            let pipe_nr = tx_packet.pipe_nr();
            unsafe {
                // Set the pipe number for this transmit
                regs.txaddress.write(|w| w.txaddress().bits(pipe_nr));
                // enable the receive pipe
                regs.rxaddresses.write(|w| w.bits(1 << pipe_nr));

                regs.packetptr.write(|w| w.bits(tx_packet.dma_pointer()));
            }

            // Enable interrupt for transmission end
            log::info!("Enabling int disabled");
            regs.intenset.write(|w| w.disabled().set_bit());

            compiler_fence(Ordering::SeqCst);
            // trigger the send
            unsafe {
                regs.tasks_txen.write(|w| w.bits(1));
            }
            log::info!("Transmit end");
        }
        Ok(())
    }
}
