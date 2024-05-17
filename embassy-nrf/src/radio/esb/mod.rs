pub use esb_config::EsbConfig;
pub use esb_radio::EsbRadio;

/// Configuration of the Enhanced Shockburst Radio.
pub mod esb_config;
/// Packet definitions for the Enhanced Shockburst Radio
pub mod esb_packet;
/// The main Enhanced Shockburst Radio module
pub mod esb_radio;
/// The internal state of the Enhanced Shockburst radio
pub(crate) mod esb_state;

#[derive(Debug, Clone, Copy)]
/// Possible air datarates for Esb
pub enum EDataRate {
    /// 1 megabit per second
    Dr1Mbps,
    /// 2 megabit per second
    Dr2Mbps,
}
pub const CHANNEL_RX_SIZE: usize = 10;
pub const CHANNEL_ACK_SIZE: usize = 10;
pub const MAX_PACKET_SIZE: usize = 32;
pub const MAX_NR_PIPES: usize = 5;

impl Default for EDataRate {
    fn default() -> EDataRate {
        EDataRate::Dr1Mbps
    }
}

#[derive(Debug, Clone, Copy)]
/// Crc Size for a esb packet.
pub enum ECrcSize {
    /// Crc size 1 byte
    Size1,
    /// Crc size 2 byte
    Size2,
}

/// The ERadioState is the state shared between the driver and the interrupt. It lives in the interrupt state
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum ERadioState {
    /// PTX and PRX role: The radio is inactive
    Inactive,
    /// PTX and PRX role: the radio is requested to start a receing transaction.
    StartReceiving,
    /// PTX and PRX role: The radio is listening for incoming packets.
    Receiving,
    /// PTX and PRX role: the radio is requested to start a transmit transacktion.
    StartTransmitting,
    /// PTX and PRX role: the radio is busy transmitting packets, until the tx channel is emtpy. 
    Transmitting,
    /// PRX role. The radio is busy transmitting the ack packet
    TransmittingAck,
    /// PTX role: The radio did send a packet and is now listening for the incoming ack of the remote
    TransmitterWaitAck,

}
