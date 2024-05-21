use core::future::poll_fn;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;
use embassy_time::{Duration, Instant, Timer};

use super::esb_config::EsbConfig;
use super::esb_packet::EsbPacket;
use super::esb_state::PipeState;
use super::{ECrcSize, EDataRate, ERadioState, CHANNEL_RX_SIZE, MAX_NR_PIPES, MAX_PACKET_SIZE};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::radio::RegisterBlock;
use crate::radio::State;
use crate::radio::{Error, Instance, InterruptHandler};
use embassy_futures::select::{select, Either};
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
const CRC_INIT2: u32 = 0x0000_FFFF;
const CRC_POLY2: u32 = 0x0001_1021;
const CRC_INIT1: u32 = 0x0000_00FF;
const CRC_POLY1: u32 = 0x0000_0107;

static CHANNEL_RX: Channel<CriticalSectionRawMutex, EsbPacket, CHANNEL_RX_SIZE> = Channel::new();

#[derive()]
// MaxNrPipes is the maximum number of pipes to use. Note that each configured pipe will use 10 x MaxPacketSize of memory
// MaxPacketSize is the radio packet size. If 34 or less this EsbRadio will be able to communicate with legacy nrf24l01+ radios
// Note that MaxPacketSize must be 2 bytes bigger thant the max packet size of the radio, due to 2 meta fields (len and pid)
pub struct EsbRadio<'d, T: Instance> {
    _radio: PeripheralRef<'d, T>,
    current_tx_packet: Option<EsbPacket>,
    pipes: [PipeState; MAX_NR_PIPES],
    timer_timeout: Instant,
    max_retries: u8,    //
    retry_delay: usize, // with 250 us resolution
    retry_counter: u8,
}

impl<'d, T: Instance> EsbRadio<'d, T> {
    /// Create a new radio driver.
    pub fn new(
        radio: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        into_ref!(radio);
        Self {
            _radio: radio,
            pipes: [
                PipeState::new(0),
                PipeState::new(1),
                PipeState::new(2),
                PipeState::new(3),
                PipeState::new(4),
            ],
            current_tx_packet: None,
            timer_timeout: Instant::MAX,
            max_retries: 15, //
            retry_delay: 4,  // with 250 us resolution
            retry_counter: 0,
        }
    }
    /// Configure the device
    /// set all its properties for proper operation and power it up.
    /// The device does remain in standby until self.listen() or self.send() is called.
    ///
    pub fn init(&mut self, config: &EsbConfig) -> Result<(), Error> {
        let regs = T::regs();

        if MAX_NR_PIPES > 8 {
            return Err(Error::PipeNrTooHigh);
        }
        self.max_retries = config.max_retries();
        self.retry_delay = config.retry_delay() as usize;

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
                .enabled()
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

            match config.crc_size() {
                ECrcSize::Size1 => {
                    regs.crccnf.write(|w| w.len().one());
                    regs.crcinit.write(|w| w.crcinit().bits(CRC_INIT1 & 0x00FF_FFFF));
                    regs.crcpoly.write(|w| w.crcpoly().bits(CRC_POLY1 & 0x00FF_FFFF));
                }
                ECrcSize::Size2 => {
                    regs.crccnf.write(|w| w.len().two());
                    regs.crcinit.write(|w| w.crcinit().bits(CRC_INIT2 & 0x00FF_FFFF));
                    regs.crcpoly.write(|w| w.crcpoly().bits(CRC_POLY2 & 0x00FF_FFFF));
                }
            }
            regs.base0.write(|w| w.bits(base0));
            regs.base1.write(|w| w.bits(base1));

            regs.prefix0.write(|w| w.bits(prefix0));
            regs.prefix1.write(|w| w.bits(prefix1));

            // NOTE(unsafe) `rf_channel` was checked to be between 0 and 100 during the creation of
            // the `Adresses` object
            regs.frequency.write(|w| w.frequency().bits(config.rf_channel()));
        }
        // Enable NVIC interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        log::info!("init finished");
        Ok(())
    }
    // add packet for sending. In case packet cannot be added it will be returned in the Err
    // In case the radio is idle the packet will be send immediatly
    // In case the radio is busy, it will send the packet lateron
    pub async fn send_packet(&mut self, mut packet: EsbPacket) -> Result<(), Error> {
        let pid = self.pipes[packet.pipe_nr() as usize].inc_tx_pid();
        packet.set_pid(pid);
        self.current_tx_packet = Some(packet);
        match self.transmit() {
            Ok(state) => self.set_radio_state(state),
            Err(e) => {
                log::info!("Error: {:?}", e);
                return Err(e);
            }
        }
        self.retry_counter = 0;

        self.statemachine_task().await?;
        if self.current_tx_packet.is_some() {
            log::info!("Error: Packet should be consumed now!")
        }
        // drop current packet
        self.current_tx_packet.take();
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

    /// This state machine is awoken by the interrupt, and runs as an embassy task (NOT in interrupt context)
    ///
    async fn statemachine_task(&mut self) -> Result<(), Error> {
        loop {
            let timeout = self.timer_timeout;
            let new_radio_state = match select(self.statemachine_poll(), Timer::at(timeout)).await {
                Either::First(new_radio_state) => new_radio_state,
                Either::Second(_) => self.timeout_handler(),
            };
            match new_radio_state {
                Ok(state) => {
                    self.set_radio_state(state);
                    if let ERadioState::Idle = state {
                        return Ok(());
                    }
                }
                Err(e) => {
                    self.set_radio_state(ERadioState::Idle);
                    return Err(e);
                }
            }
        }
    }
    async fn statemachine_poll(&mut self) -> Result<ERadioState, Error> {
        // After each interrupt this code will runonce due to the waker in the interrupt
        let state = T::state();
        poll_fn(|cx| {
            state.event_waker.register(cx.waker());
            self.radio_state_machine()
        })
        .await
    }

    // Take the radio state from the interrupt with a mutex lock
    // Return the updated radio state in the result
    fn radio_state_machine(&mut self) -> Poll<Result<ERadioState, Error>> {
        log::info!("S");
        // update the radio state as it can be changed in the interrupt
        let mut radio_state = ERadioState::Idle;
        T::state().mutex.lock(|f| {
            let state_m = f.borrow();
            radio_state = state_m.radio_state;
        });
        let regs = T::regs();
        // log::info!("State: {:?} ", radio_state);
        match radio_state {
            // PTX and PRX role: the radio is busy transmitting packets, until the tx channel is emtpy.
            ERadioState::TransmitAckFinished => {
                if regs.crcstatus.read().crcstatus().is_crcok() {
                    // tx successfull, drop packet
                    if let Some(mut ack_data) = self.current_tx_packet.take() {
                        // the tx packet contains the ack message,as the dma pointer did not change
                        ack_data.set_rssi(regs.rssisample.read().rssisample().bits());
                        if let Err(_) = CHANNEL_RX.try_send(ack_data) {
                            return Poll::Ready(Err(Error::ChannelFull));
                        }
                    }
                    Poll::Ready(Ok(ERadioState::Idle))
                } else {
                    Poll::Ready(Err(Error::CrcFailed(regs.crcstatus.read().bits() as u16)))
                }
            }
            ERadioState::TransmitNoAckFinished => {
                // tx successfull, drop packet
                _ = self.current_tx_packet.take();
                log::info!("Transmit done no ack");
                Poll::Ready(Ok(ERadioState::Idle))
            }
            ERadioState::Idle => Poll::Ready(Ok(ERadioState::Idle)),
            _ => Poll::Pending,
        }
    }

    fn timeout_handler(&mut self) -> Result<ERadioState, Error> {
        // disarm current timeout
        self.timer_timeout = Instant::MAX;

        let regs = T::regs();
        // update the radio state as it can be changed in the interrupt
        let mut radio_state = ERadioState::Idle;
        T::state().mutex.lock(|f| {
            let state_m = f.borrow();
            radio_state = state_m.radio_state;
        });
        log::info!("R:{:?}", self.retry_counter);
        self.retry_counter += 1;
        if self.retry_counter >= self.max_retries {
            let packet = self.current_tx_packet.take();
            return Err(Error::NoAckResponse(packet));
        }
        match radio_state {
            ERadioState::TransmitAckWait | ERadioState::TransmitAck => {
                // disarm the interrupt
                unsafe {
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF));
                }
                // disarm the radio
                regs.tasks_disable.write(|w| unsafe { w.bits(1) });
                // return the new state as got from the transmit
                return self.transmit();
            }
            _ => (),
        }
        Ok(radio_state)
    }

    /// Transmit a packet. Setup interrupts and events
    /// Change the radio state.
    fn transmit(&mut self) -> Result<ERadioState, Error> {
        let regs = T::regs();
        let mut new_radio_state = ERadioState::Idle;

        if let Some(tx_packet) = &self.current_tx_packet {
            let pipe_nr = tx_packet.pipe_nr();

            if tx_packet.ack() {
                // Go to RX mode after the transmission
                regs.shorts.modify(|_, w| w.disabled_rxen().enabled());
                // note that we are going to reuse the tx packet for the content of the ack!
                // the dma pointer for easy dma is not changed during the switching from tx to rx
                new_radio_state = ERadioState::TransmitAck;
                // arm the no ack received timer
                self.timer_timeout = Instant::now() + Duration::from_micros((self.retry_delay * 250) as u64);
            } else {
                new_radio_state = ERadioState::TransmitNoAck;
            }

            // reset relevant events
            regs.events_disabled.reset();
            regs.events_end.reset();

            unsafe {
                // Set the pipe number for this transmit
                regs.txaddress.write(|w| w.txaddress().bits(pipe_nr));
                // enable the receive pipe
                regs.rxaddresses.write(|w| w.bits(1 << pipe_nr));
                regs.packetptr.write(|w| w.bits(tx_packet.dma_pointer()));
            }
            // Enable interrupt for transmission end
            regs.intenset.write(|w| w.disabled().set());
            // be sure no instruction reordering after next fence
            compiler_fence(Ordering::SeqCst);
            // trigger the send
            unsafe {
                regs.tasks_txen.write(|w| w.bits(1));
            }
        }
        Ok(new_radio_state)
    }
    // write the new radio state to the interrupt state, with a mutex lock
    fn set_radio_state(&mut self, radio_state: ERadioState) {
        T::state().mutex.lock(|f| {
            let mut state_m = f.borrow_mut();
            state_m.radio_state = radio_state;
        });
    }
}

#[inline]
pub(crate) fn on_interrupt(regs: &RegisterBlock, state: &State) {
    // reset the cause of this interrupt
    regs.events_disabled.reset();
    regs.events_end.reset();

    state.mutex.lock(|f| {
        let mut state_m = f.borrow_mut();
        state_m.timestamp = Instant::now();
        match state_m.radio_state {
            ERadioState::Idle => (),
            ERadioState::Receiving(dma_rx_pointer) => {
                // remove the shortcut rx->disabled to tx enable for sending the ack
                regs.shorts.modify(|_, w| w.disabled_rxen().disabled());
                compiler_fence(Ordering::Release);
                // dangerout step: no protection at all from Rust!
                regs.packetptr.write(|w| unsafe { w.bits(dma_rx_pointer) });
                state_m.radio_state = ERadioState::ReceiveTransmitAck;
            }
            ERadioState::ReceiveTransmitAck => {
                unsafe {
                    // disable IRQ the rest handled in driver context
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF));
                }
                state.event_waker.wake();
            }
            ERadioState::TransmitAck => {
                state_m.radio_state = ERadioState::TransmitAckWait;
                compiler_fence(Ordering::Release);
                // remove the shortcut rx->disabled to rx enable for sending ack with optional payload
                regs.shorts.modify(|_, w| w.disabled_rxen().disabled());
            }
            ERadioState::TransmitAckWait => {
                state_m.radio_state = ERadioState::TransmitAckFinished;
                unsafe {
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF)); // disable IRQ, the rest handled in driver context
                }
                // and then wake the driver statemachine
                state.event_waker.wake();
            }
            ERadioState::TransmitNoAck => {
                state_m.radio_state = ERadioState::TransmitNoAckFinished;
                unsafe {
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF)); // disable IRQ, the rest handled in driver context
                }
                state.event_waker.wake();
            }
            _ => (),
        }
        log::info!("-I-");
    }); // end of mutex
}

pub struct EsbRadioRx {}

impl EsbRadioRx {
    // static functions
    pub async fn rx_receive() -> EsbPacket {
        CHANNEL_RX.receive().await
    }
    pub fn rx_try_receive() -> Result<EsbPacket, Error> {
        match CHANNEL_RX.try_receive() {
            Ok(packet) => Ok(packet),
            Err(_) => Err(Error::ChannelEmpty),
        }
    }
}
