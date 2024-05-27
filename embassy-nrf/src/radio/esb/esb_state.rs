use embassy_time::Instant;

use super::{esb_packet::EsbPacket, ERadioState};

pub(crate) struct EsbIrqState {
    pub(crate) radio_state: ERadioState,
    pub(crate) timestamp: Instant,
    pub(crate) current_rx_packet: Option<EsbPacket>,
    pub(crate) current_ack_packet: Option<EsbPacket>,
    pub(crate) last_rx_pid: u8,
    pub(crate) last_rx_checksum: u16,
    pub(crate) ack_reporting: bool,

}

impl EsbIrqState {
    pub(crate) const fn new() -> Self {
        EsbIrqState {
            radio_state: ERadioState::Idle,
            timestamp: Instant::MIN,
            current_rx_packet: None,
            current_ack_packet: None,
            last_rx_pid: 0,
            last_rx_checksum: 0,
            ack_reporting: false,

        }
    }
}
#[derive(Clone, Copy)]
pub(crate) struct PipeState {
    last_rx_retry: u8,
}
impl PipeState {
    pub(crate) const fn new(pipe_nr: u8) -> Self {
        Self { last_rx_retry: 0 }
    }
    // return last_rx_retry and reset
    pub fn last_rx_retry(&mut self) -> u8 {
        let result = self.last_rx_retry;
        self.last_rx_retry = 0;
        result
    }
    pub fn inc_last_rx_retry(&mut self) {
        if self.last_rx_retry < 255 {
            self.last_rx_retry += 1
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AckReporting {
    timestamp: Instant,
    counter: usize,
}

impl AckReporting {
    pub fn new(counter: usize) -> Self {
        AckReporting {
            timestamp: Instant::now(),
            counter,
        }
    }
    pub fn timestamp(&self) -> Instant {
        self.timestamp
    }
    pub fn counter(&self) -> usize {
        self.counter
    }
}
