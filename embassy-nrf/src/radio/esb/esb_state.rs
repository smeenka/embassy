use embassy_time::Instant;

use super::{esb_packet::EsbPacket, ERadioState};

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
            last_checksum: 0,
            last_rx_pid: 0,
            last_tx_pid: 0,
            ack_packet: None,
        }
    }

    pub(crate) fn pipe_nr(&self) -> u8 {
        self.pipe_nr
    }

    pub fn inc_tx_pid(&mut self) -> u8 {
        self.last_tx_pid = (self.last_tx_pid + 1) & 0x3;
        self.last_tx_pid
    }
    pub fn ack_packet(&self) -> &Option<EsbPacket> {
        &self.ack_packet
    }
}
