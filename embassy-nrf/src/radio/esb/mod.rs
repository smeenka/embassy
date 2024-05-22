pub use esb_config::EsbConfig;
pub use esb_radio::EsbRadio;
pub use esb_radio::EsbRadioAck;
pub use esb_radio::EsbRadioRx;
pub use esb_radio::EsbRadioTx;

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
pub const CHANNEL_TX_SIZE: usize = 10;
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

/// The ERadioState is the state only in the embassy driver context. It is not shared with the interrupt
#[derive(PartialEq, Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ERadioState {
    /// The radio is inactive
    Idle,
    /// The radio is listening for incoming packets.
    Receiving(u32), // parameter is the dma buffer pointer for the tx buffer for sending the ack response,
    /// Packet is received and the ack is send
    ReceiveTransmitAck,
    ReceiveFinished, // irq will be disarmed
    TransmitCheck,   // Check if there is a packet to send
    /// the radio should send the packet, and does nnot need to wait for an ack
    TransmitNoAck,
    TransmitNoAckFinished,
    /// The radio should send a packet and be prepared to receive the ack response
    TransmitAck,
    // packet is transmitted, and now waiting for ack received
    TransmitAckWait,
    TransmitAckFinished,
}
