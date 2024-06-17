use core::future::poll_fn;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use super::esb_config::EsbConfig;
use super::esb_packet::EsbPacket;
use super::esb_state::{AckReporting, PipeState};
use super::{
    ECrcSize, EDataRate, ERadioCommand, ERadioEvent, ERadioState, CHANNEL_ACK_SIZE, CHANNEL_RX_SIZE, CHANNEL_TX_SIZE,
    MAX_NR_PIPES, MAX_PACKET_SIZE, RECEIVE_MASK,
};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::radio::RegisterBlock;
use crate::radio::State;
use crate::radio::{Error, Instance, InterruptHandler};
use embassy_futures::select::{select3, Either3};
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

static CHANNEL_EVENTS: Channel<CriticalSectionRawMutex, ERadioEvent, CHANNEL_RX_SIZE> = Channel::new();
static CHANNEL_COMMANDS: Channel<CriticalSectionRawMutex, ERadioCommand, CHANNEL_TX_SIZE> = Channel::new();
static CHANNEL_REUSE: Channel<CriticalSectionRawMutex, EsbPacket, CHANNEL_TX_SIZE> = Channel::new();
static SIGNAL_TX: Signal<CriticalSectionRawMutex, usize> = Signal::new();
static PIPES: [Channel<CriticalSectionRawMutex, EsbPacket, CHANNEL_ACK_SIZE>; MAX_NR_PIPES] = [
    Channel::new(),
    Channel::new(),
    Channel::new(),
    Channel::new(),
    Channel::new(),
];

#[derive()]
// MaxNrPipes is the maximum number of pipes to use. Note that each configured pipe will use 10 x MaxPacketSize of memory
// MaxPacketSize is the radio packet size. If 32 or less this EsbRadio will be able to communicate with legacy nrf24l01+ radios
// Note that MaxPacketSize must be 2 bytes bigger thant the max packet size of the radio, due to 2 meta fields (len and pid)
pub struct EsbRadio<'d, T: Instance> {
    _radio: PeripheralRef<'d, T>,
    current_tx_packet: Option<EsbPacket>,
    pipes: [PipeState; MAX_NR_PIPES],
    timer_timeout: Instant,
    max_retries: u8,    //
    retry_delay: usize, // with 250 us resolution
    retry_counter: u8,
    last_tx_pid: u8,
    last_rx_pid: u8,
    last_checksum: u16,
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
            last_rx_pid: 0,
            last_tx_pid: 0,
            last_checksum: 0,
            max_retries: 15, //
            retry_delay: 4,  // with 250 us resolution
            retry_counter: 0,
        }
    }
    pub fn set_rx_pid(&mut self, pid: u8) {
        self.last_rx_pid = pid
    }
    pub fn rx_pid(&mut self) -> u8 {
        self.last_rx_pid
    }
    pub fn inc_tx_pid(&mut self) -> u8 {
        self.last_tx_pid = (self.last_tx_pid + 1) & 0x3;
        self.last_tx_pid
    }
    pub fn tx_pid(&self) -> u8 {
        self.last_tx_pid
    }
    pub fn set_checksum(&mut self, cs: u16) {
        self.last_checksum = cs
    }
    pub fn checksum(&mut self) -> u16 {
        self.last_checksum
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
                .txready_start()
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
        self.set_radio_state(ERadioState::ReceivePrepare);
        // Enable NVIC interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        log::info!("init finished");
        Ok(())
    }

    /// This state machine is awoken by the interrupt, and runs as an embassy task (NOT in interrupt context)
    ///
    /// this task should be run by the top level application in a loop.
    /// The top level applicaion is the owner of the radio
    /// On each iteration an error can be returnned
    pub async fn statemachine_runonce(&mut self) -> Result<(), Error> {
        let timeout = self.timer_timeout;
        Timer::after_millis(0).await; // needed to make the funtion decent async. Select alone will hang
        let new_radio_state = match select3(self.statemachine_poll(), Timer::at(timeout), SIGNAL_TX.wait()).await {
            Either3::First(new_radio_state) => new_radio_state,
            Either3::Second(_) => self.timeout_handler(),
            Either3::Third(_) => self.try_send_packet(),
        };
        match new_radio_state {
            Ok(state) => {
                self.set_radio_state(state);
                Ok(())
            }
            Err(e) => {
                self.set_radio_state(ERadioState::TransmitCheck);
                Err(e)
            }
        }
    }

    // The state machine is supposed to run in a loop in an application task
    // If you want to make the statemachine a step return Poll::Ready(Result)
    // Returning Poll::Pending will let the code sleep, and wait until the interrupt does call waker.wake
    async fn statemachine_poll(&mut self) -> Result<ERadioState, Error> {
        // After each interrupt this code will runonce due to the waker in the interrupt
        let state = T::state();
        poll_fn(|cx| {
            state.event_waker.register(cx.waker());
            self.radio_state_machine()
        })
        .await
    }

    // Start with the radio_state as in the interrupt state
    // Return the updated radio state in the result
    fn radio_state_machine(&mut self) -> Poll<Result<ERadioState, Error>> {
        // update the radio state as it can be changed in the interrupt
        let mut radio_state = ERadioState::Idle;
        T::state().mutex.lock(|f| {
            let state_m = f.borrow();
            radio_state = state_m.radio_state;
        });
        let regs = T::regs();
        //log::info!("{:?}", &radio_state);
        match radio_state {
            ERadioState::Idle => Poll::Pending,
            ERadioState::ReceivePrepare => match self.receive_prepare() {
                Ok(new_state) => Poll::Ready(Ok(new_state)),
                Err(e) => Poll::Ready(Err(e)),
            },
            ERadioState::Receiving => Poll::Pending, // wait for interrupt
            ERadioState::ReceiveFinished => {
                let mut rx_packet = None;
                T::state().mutex.lock(|f| {
                    let mut state_m = f.borrow_mut();
                    rx_packet = state_m.current_rx_packet.take();
                });

                if let Some(mut p) = rx_packet {
                    let crc = regs.rxcrc.read().rxcrc().bits() as u16;
                    let pipe_nr = regs.rxmatch.read().rxmatch().bits() as usize;
                    if regs.crcstatus.read().crcstatus().is_crcerror() {
                        // treat crc as if no packet was received. Increment retry count and go again!
                        EsbRadioEvent::reuse_rx_packet(p);
                        self.pipes[pipe_nr].inc_last_rx_retry();
                        return Poll::Ready(Ok(ERadioState::ReceivePrepare));
                    }
                    if pipe_nr >= MAX_NR_PIPES {
                        EsbRadioEvent::reuse_rx_packet(p);
                        return Poll::Ready(Err(Error::PipeNrTooHigh));
                    }
                    let pid = p.pid();
                    if pid == self.rx_pid() && crc == self.checksum() {
                        // log::info!("Repeated packet. Not adding to queue");
                        self.pipes[pipe_nr].inc_last_rx_retry();
                        EsbRadioEvent::reuse_rx_packet(p);
                    } else {
                        // update pipe admin
                        self.set_rx_pid(pid);
                        self.set_checksum(crc);
                        // update packet
                        if p.ack() {
                            // a packet was send. The tx pid was incremented in interrupt state, but not yet updated here
                            _ = self.inc_tx_pid();
                        }
                        p.set_rssi(regs.rssisample.read().rssisample().bits());
                        p.set_pipe_nr(pipe_nr as u8);
                        p.set_retry(self.pipes[pipe_nr].last_rx_retry());
                        // send to application
                        if let Err(_) = CHANNEL_EVENTS.try_send(ERadioEvent::Data(p)) {
                            return Poll::Ready(Err(Error::ChannelFull));
                        }
                    }
                    Poll::Ready(Ok(ERadioState::TransmitCheck))
                } else {
                    Poll::Ready(Err(Error::ChannelEmpty))
                }
            }
            ERadioState::TransmitCheck => {
                let new_state = self.check_commands();
                Poll::Ready(new_state)
            }
            ERadioState::TransmitAckFinished => {
                // disarm timeout timer
                self.timer_timeout = Instant::MAX;
                if regs.crcstatus.read().crcstatus().is_crcok() {
                    // tx successfull, drop packet
                    self.current_tx_packet.take();

                    // fetch rx packet from interrupt context
                    let mut some_ack_packet = None;
                    T::state().mutex.lock(|f| {
                        let mut state_m = f.borrow_mut();
                        some_ack_packet = state_m.current_ack_packet.take();
                    });
                    if let Some(mut ack_data) = some_ack_packet {
                        if ack_data.len() > 0 {
                            ack_data.set_rssi(regs.rssisample.read().rssisample().bits());
                            ack_data.set_retry(self.retry_counter);
                            if let Err(_) = CHANNEL_EVENTS.try_send(ERadioEvent::Data(ack_data)) {
                                return Poll::Ready(Err(Error::ChannelFull));
                            }
                        }
                    }
                    Poll::Ready(Ok(ERadioState::TransmitCheck))
                } else {
                    // treat the CRC error as a normal timeout, no ack received. Force the timeout handler to fire
                    self.timer_timeout = Instant::MIN;
                    Poll::Ready(Ok(ERadioState::TransmitAckWait))
                }
            }
            ERadioState::TransmitNoAckFinished => {
                // tx successfull, drop packet
                _ = self.current_tx_packet.take();
                log::info!("Transmit done no ack");
                Poll::Ready(Ok(ERadioState::TransmitCheck))
            }
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
        match radio_state {
            ERadioState::TransmitAckWait | ERadioState::TransmitAck => {
                // log::info!("R:{:?}", self.retry_counter);
                self.retry_counter += 1;
                if self.retry_counter >= self.max_retries {
                    let packet = self.current_tx_packet.take();
                    return Err(Error::NoAckResponse(packet));
                }
                // disarm the interrupt
                unsafe {
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF));
                }
                // return the new state as got from the transmit
                return self.transmit();
            }
            _ => (),
        }
        Ok(radio_state)
    }

    /// If the radio is idle, and there is a packet to send, send this packet.
    /// Else do nothing, the radio state machine should pick up this tx packet when idle
    /// Errors are added to CHANNEL_RX queue
    pub fn try_send_packet(&mut self) -> Result<ERadioState, Error> {
        let mut state = ERadioState::Idle;
        // lock the mutex as short as possible
        T::state().mutex.lock(|f| {
            let state_m = f.borrow();
            state = state_m.radio_state;
        });
        log::info!("Try Send {:?}", state);
        match state {
            ERadioState::Idle => self.check_commands(),
            ERadioState::Receiving => self.check_commands(),
            _ => Ok(state), // do not change the state
        }
    }

    /// Check the CHANNEL_COMMANDS. If emtpy check the receive state state
    /// Check the current_packet. If None, take one from the CHANNEL_TX
    /// Assumed is that the radio is idle, and can send the packet
    /// Return the new radio state in the result
    pub fn check_commands(&mut self) -> Result<ERadioState, Error> {
        if self.current_tx_packet.is_none() {
            if CHANNEL_COMMANDS.is_empty() {
                return Ok(ERadioState::ReceivePrepare);
            } else {
                if let Ok(command) = CHANNEL_COMMANDS.try_receive() {
                    match command {
                        ERadioCommand::Data(mut p) => {
                            let pid = self.inc_tx_pid();
                            p.set_pid(pid);
                            self.current_tx_packet = Some(p);
                        }
                        ERadioCommand::AckReporting(reporting) => {
                            log::info!("Setting ack reporting to {}", reporting);
                            T::state().mutex.lock(|f| {
                                let mut state_m = f.borrow_mut();
                                state_m.ack_reporting = reporting
                            })
                        }
                    }
                }
            }
        }
        self.retry_counter = 0;
        self.transmit()
    }
    #[inline]
    fn receive_prepare(&mut self) -> Result<ERadioState, Error> {
        // update the rx packet in the interrupt context
        let regs = T::regs();
        T::state().mutex.lock(|f| {
            let mut state_m = f.borrow_mut();
            if state_m.current_rx_packet.is_none() {
                match CHANNEL_REUSE.try_receive() {
                    Ok(p) => state_m.current_rx_packet = Some(p),
                    _ => state_m.current_rx_packet = Some(EsbPacket::empty()),
                }
            }
            // Note the reference is crucial for the DMA to work!
            if let Some(p) = &state_m.current_rx_packet {
                unsafe {
                    regs.packetptr.write(|w| w.bits(p.dma_pointer()));
                }
            }
        });
        unsafe {
            // Go to TX mode after the reception for sending the ACK
            regs.shorts.modify(|_, w| w.disabled_txen().enabled());
            // enable the receive pipes
            regs.rxaddresses.write(|w| w.bits(RECEIVE_MASK as u32));
            // Enable interrupt for receive disabled
            regs.intenset.write(|w| w.disabled().set());
            // be sure no instruction reordering after next fence
            compiler_fence(Ordering::SeqCst);
            // trigger the listen task
            regs.tasks_rxen.write(|w| w.bits(1));
            Ok(ERadioState::Receiving)
        }
    }

    /// Transmit a packet. Setup interrupts and events
    /// Change the radio state.
    fn transmit(&mut self) -> Result<ERadioState, Error> {
        let regs = T::regs();
        unsafe {
            // disable IRQ (could be armed due to receive state)
            regs.intenclr.write(|w| w.bits(0xFFFF_FFFF));
        }
        // disble radio and wait until actually disabled
        regs.shorts
            .modify(|_, w| w.disabled_rxen().disabled().disabled_txen().disabled());

        regs.tasks_disable.write(|w| unsafe { w.bits(1) });
        while regs.events_disabled.read().bits() == 0 {}
        compiler_fence(Ordering::SeqCst);

        let mut new_radio_state = ERadioState::Idle;

        if let Some(tx_packet) = &self.current_tx_packet {
            // Note the reference is crucial for the DMA to work!
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
            // Enable interrupt for transmission disabled
            regs.intenset.write(|w| w.disabled().set());

            T::state().mutex.lock(|f| {
                let mut state_m = f.borrow_mut();
                // to be sure the state is set before the interrupt can happen, set the state here
                // possible duplicate, but for sure
                state_m.radio_state = new_radio_state;

                if tx_packet.ack() {
                    // prepare receive packet for the ack response possible containing data
                    if state_m.current_ack_packet.is_none() {
                        let mut packet = EsbPacket::empty();
                        packet.set_pipe_nr(pipe_nr);
                        state_m.current_ack_packet = Some(packet);
                    }
                }
            });
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
        //log::info!("I{:?}", state_m.radio_state);
        match state_m.radio_state {
            ERadioState::Idle => (),
            ERadioState::Receiving => {
                let mut previous_packet = None;
                if regs.crcstatus.read().crcstatus().is_crcerror() {
                    // treat error as if no packet was received. Let the driver code decide what to do
                    // log::info!("Checksum error");
                    state_m.radio_state = ERadioState::ReceiveFinished;
                    unsafe {
                        // disable IRQ the rest handled in driver context
                        regs.intenclr.write(|w| w.bits(0xFFFF_FFFF));
                    }
                    regs.shorts.modify(|_, w| w.disabled_txen().disabled());
                    compiler_fence(Ordering::Release);
                    state.event_waker.wake();
                    return;
                }
                if let Some(p) = state_m.current_rx_packet {
                    // check if the remote node does want to receive an ACK
                    if p.ack() {
                        let pipe_nr = regs.rxmatch.read().rxmatch().bits() as usize;
                        let crc = regs.rxcrc.read().rxcrc().bits() as u16;
                        let pid = p.pid();
                        if pid == state_m.last_rx_pid && crc == state_m.last_rx_checksum {
                            // log::info!("Repeated packet. do nothing here");
                        } else {
                            // remember state previous packet
                            state_m.last_rx_pid = pid;
                            state_m.last_rx_checksum = crc;
                            // drop previous packet
                            previous_packet = state_m.current_ack_packet.take();
                        }
                        if state_m.current_ack_packet.is_none() {
                            if pipe_nr < MAX_NR_PIPES {
                                match PIPES[pipe_nr].try_receive() {
                                    Ok(p) => state_m.current_ack_packet = Some(p),
                                    Err(_) => state_m.current_ack_packet = Some(EsbPacket::empty()),
                                }
                            } else {
                                // fallback code, will never (??) execute
                                state_m.current_ack_packet = Some(EsbPacket::empty());
                            }
                        }
                        compiler_fence(Ordering::Release);
                        // Note the reference is crucial for the DMA to work!
                        if let Some(ack) = &mut state_m.current_ack_packet {
                            // Note that the next step is very time critical, it must be done before the radio turnaround
                            // from RX to TX is finished (130 us). So it can only be done in interrupt context
                            unsafe {
                                regs.packetptr.write(|w| w.bits(ack.dma_pointer()));
                                regs.txaddress.write(|w| w.txaddress().bits(pipe_nr as u8));
                            }
                        }
                        compiler_fence(Ordering::Release);
                        // handle option ack reporting after the dma pointer is set, so a bit extra time in the interrupt does not hurt
                        if state_m.ack_reporting {
                            if let Some(p) = previous_packet {
                                let report = AckReporting::new(p.counter());
                                _ = CHANNEL_EVENTS.try_send(ERadioEvent::AckReporting(report));
                            }
                        }
                        state_m.radio_state = ERadioState::ReceiveTransmitAck;
                    } else {
                        state_m.radio_state = ERadioState::ReceiveFinished
                    }
                }
                // remove the shortcut rx->disabled to tx enable for sending the ack
                regs.shorts.modify(|_, w| w.disabled_txen().disabled());
            }
            ERadioState::ReceiveTransmitAck => {
                unsafe {
                    // disable IRQ the rest handled in driver context
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF));
                }
                state_m.radio_state = ERadioState::ReceiveFinished;
                compiler_fence(Ordering::Release);
                state.event_waker.wake();
            }
            ERadioState::TransmitAck => {
                state_m.radio_state = ERadioState::TransmitAckWait;
                // remove the shortcut rx->disabled to rx enable for sending ack with optional payload
                regs.shorts.modify(|_, w| w.disabled_rxen().disabled());
                // Note the reference is crucial for the DMA to work!
                if let Some(ack) = &mut state_m.current_ack_packet {
                    // Note that the next step is very time critical, it must be done before the radio turnaround
                    // from TX to RX is finished (130 us). So it can only be done in interrupt context
                    unsafe {
                        regs.packetptr.write(|w| w.bits(ack.dma_pointer()));
                    }
                }
                compiler_fence(Ordering::Release);
            }
            ERadioState::TransmitAckWait => {
                state_m.radio_state = ERadioState::TransmitAckFinished;
                unsafe {
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF)); // disable IRQ, the rest handled in driver context
                }
                // and then wake the driver statemachine
                compiler_fence(Ordering::Release);
                state.event_waker.wake();
            }
            ERadioState::TransmitNoAck => {
                state_m.radio_state = ERadioState::TransmitNoAckFinished;
                unsafe {
                    regs.intenclr.write(|w| w.bits(0xFFFF_FFFF)); // disable IRQ, the rest handled in driver context
                }
                compiler_fence(Ordering::Release);
                state.event_waker.wake();
            }
            _ => (),
        }
    }); // end of mutex
}

pub struct EsbRadioEvent {}

impl EsbRadioEvent {
    // static functions
    pub async fn receive() -> ERadioEvent {
        CHANNEL_EVENTS.receive().await
    }
    pub fn try_receive() -> Result<ERadioEvent, Error> {
        match CHANNEL_EVENTS.try_receive() {
            Ok(event) => Ok(event),
            Err(_) => Err(Error::ChannelEmpty),
        }
    }
    // the reuse channel does contain the packets to be used for receiving
    // After receive the packet should be returned to this queue
    pub fn reuse_rx_packet(mut packet: EsbPacket) {
        packet.reset();
        _ = CHANNEL_REUSE.try_send(packet); // discard error channel full
    }
}
pub struct EsbRadioCommand {}

impl EsbRadioCommand {
    // static functions
    pub async fn send(command: ERadioCommand) {
        CHANNEL_COMMANDS.send(command).await;
        SIGNAL_TX.signal(0) // actual value does not matter
    }
    pub fn try_send(command: ERadioCommand) -> Result<(), Error> {
        match CHANNEL_COMMANDS.try_send(command) {
            Ok(_) => {
                SIGNAL_TX.signal(0); // acutal value does not matter
                Ok(())
            }
            Err(_) => Err(Error::ChannelFull),
        }
    }
}
pub struct EsbRadioAck {}

impl EsbRadioAck {
    // static functions
    pub async fn send(packet: EsbPacket) -> Result<(), Error> {
        let pipe_nr = packet.pipe_nr() as usize;
        if pipe_nr < MAX_NR_PIPES {
            PIPES[pipe_nr].send(packet).await;
            Ok(())
        } else {
            log::info!("Incorrect pipe");
            Err(Error::PipeNrTooHigh)
        }
    }
    pub fn try_send(packet: EsbPacket) -> Result<(), Error> {
        let pipe_nr = packet.pipe_nr() as usize;
        if pipe_nr < MAX_NR_PIPES {
            match PIPES[pipe_nr].try_send(packet) {
                Ok(_) => Ok(()),
                Err(_) => Err(Error::ChannelFull),
            }
        } else {
            log::info!("Incorrect pipe");
            Err(Error::PipeNrTooHigh)
        }
    }
}
