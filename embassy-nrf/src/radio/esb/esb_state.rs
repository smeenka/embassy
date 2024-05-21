use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Sender},
};
use embassy_time::Instant;

use super::{esb_packet::EsbPacket, ERadioState, CHANNEL_ACK_SIZE};

pub(crate) struct EsbState {
    pub(crate) radio_state: ERadioState,
    pub(crate) timestamp: Instant,
}

impl EsbState {
    pub(crate) const fn new() -> Self {
        EsbState {
            radio_state: ERadioState::Idle,
            timestamp: Instant::MIN,
        }
    }
}

pub(crate) struct PipeState {
    channel_ack: Channel<CriticalSectionRawMutex, EsbPacket, CHANNEL_ACK_SIZE>,
    last_checksum: u16,
    last_tx_pid: u8,
    last_rx_pid: u8,
    pipe_nr: u8,
    ack_packet: Option<EsbPacket>,
}
impl PipeState {
    pub(crate) const fn new(pipe_nr: u8) -> Self {
        Self {
            pipe_nr,
            channel_ack: Channel::new(),
            last_checksum: 0,
            last_rx_pid: 0,
            last_tx_pid: 0,
            ack_packet: None,
        }
    }

    pub(crate) fn preload_ack(&mut self) {
        if self.ack_packet.is_none() && !self.channel_ack.is_empty() {
            self.ack_packet = Some(self.channel_ack.try_receive().unwrap());
        }
    }
    pub(crate) fn pipe_nr(&self) -> u8 {
        self.pipe_nr
    }
    /// retreive the sender for this ack channel. With the sender the user can send ack packets
    pub fn sender(&self) -> Sender<'_, CriticalSectionRawMutex, EsbPacket, CHANNEL_ACK_SIZE> {
        self.channel_ack.sender()
    }
    pub(crate) fn channel_ack(&self) -> &Channel<CriticalSectionRawMutex, EsbPacket, CHANNEL_ACK_SIZE> {
        &self.channel_ack
    }
    pub fn inc_tx_pid(&mut self) -> u8 {
        self.last_tx_pid = (self.last_tx_pid + 1) & 0x3;
        self.last_tx_pid
    }
    pub fn ack_packet(&self) -> &Option<EsbPacket> {
        &self.ack_packet
    }
    // set the ack packet and return the dma pointer to this packet
    pub fn set_ack_packet(&mut self, ack_packet: EsbPacket) -> u32 {
        let dma_pointer = ack_packet.dma_pointer();
        self.ack_packet = Some(ack_packet);
        dma_pointer
    }
}
