use crate::pac::radio::txpower::TXPOWER_A as TxPower;

use super::{ECrcSize, EDataRate};

#[derive(Debug)]
pub struct EsbConfig {
    data_rate: EDataRate,
    /// Base address for pipe 0
    base0: [u8; 4],
    /// Base address for pipe 1-7
    base1: [u8; 4],
    /// Prefixes for pipes 0-3
    prefixes0: [u8; 4],
    /// `prefixes1` - Prefixes for pipes 4-7
    prefixes1: [u8; 4],
    /// Channel to be used by the radio hardware (must be between 0 and 100)
    rf_channel: u8,
    tx_power: TxPower,
    max_retries: u8, // [0, 15]
    retry_delay: u8, // [0, 15]
    crc_size: ECrcSize,
}

impl Default for EsbConfig {
    fn default() -> Self {
        EsbConfig {
            data_rate: EDataRate::Dr1Mbps,
            crc_size: ECrcSize::Size1,
            tx_power: TxPower::_0D_BM,
            base0: [b't', b'x', b'c', b'h'], // "txch" for tx channel
            base1: [b'p', b'i', b'p', b'e'], // "pipe"
            prefixes0: [0, 1, 2, 3],
            prefixes1: [4, 5, 6, 7],
            rf_channel: 90,
            max_retries: 8, // [0, 15]
            retry_delay: 8, // 1 ms between retries
        }
    }
}

impl EsbConfig {
    pub fn set_data_rate(&mut self, data_rate: EDataRate) {
        self.data_rate = data_rate
    }
    pub fn set_base0(&mut self, base: [u8; 4]) {
        self.base0 = base
    }
    pub fn set_base1(&mut self, base: [u8; 4]) {
        self.base1 = base
    }
    pub fn set_prefixes0(&mut self, prefixes: [u8; 4]) {
        self.prefixes0 = prefixes
    }
    pub fn set_prefixes1(&mut self, prefixes: [u8; 4]) {
        self.prefixes1 = prefixes
    }
    pub fn set_rf_channel(&mut self, channel: u8) {
        self.rf_channel = if channel >= 100 { 100 } else { channel }
    }
    pub fn set_tx_power(&mut self, power: TxPower) {
        self.tx_power = power
    }
    pub fn set_max_retries(&mut self, retries: u8) {
        self.max_retries = retries
    }
    pub fn set_retry_delay(&mut self, delay: u8) {
        self.retry_delay = delay
    }
    pub fn set_crc_size(&mut self, size: ECrcSize) {
        self.crc_size = size
    }
    pub fn data_rate(&self) -> EDataRate {
        self.data_rate
    }
    pub fn base0(&self) -> [u8; 4] {
        self.base0
    }
    pub fn base1(&self) -> [u8; 4] {
        self.base1
    }
    pub fn prefixes0(&self) -> [u8; 4] {
        self.prefixes0
    }
    pub fn prefixes1(&self) -> [u8; 4] {
        self.prefixes1
    }
    pub fn rf_channel(&self) -> u8 {
        self.rf_channel
    }
    pub fn tx_power(&self) -> TxPower {
        self.tx_power
    }
    pub fn max_retries(&self) -> u8 {
        self.max_retries
    }
    pub fn retry_delay(&self) -> u8 {
        self.retry_delay
    }
    pub fn crc_size(&self) -> ECrcSize {
        self.crc_size
    }
}
