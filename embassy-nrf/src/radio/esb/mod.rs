pub use esb_config::EsbConfig;
pub use esb_radio::EsbRadio;
pub use esb_radio::EsbRadioAck;
pub use esb_radio::EsbRadioCommand;
pub use esb_radio::EsbRadioEvent;

use self::esb_packet::EsbPacket;
use self::esb_state::AckReporting;

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
pub const CHANNEL_RX_SIZE: usize = 20;
pub const CHANNEL_TX_SIZE: usize = 10;
pub const CHANNEL_ACK_SIZE: usize = 10;
pub const MAX_PACKET_SIZE: usize = 32;
pub const MAX_NR_PIPES: usize = 5;
pub const RECEIVE_MASK: usize = 0x1F; // 5 channels

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
    ReceivePrepare,
    /// The radio is listening for incoming packets.
    Receiving,
    /// A packet was received and Ack is transmitting now
    ReceiveTransmitAck,
    /// Packet is received and the ack is send
    ReceiveFinished,
    TransmitCheck, // Check if there is a packet to send
    /// the radio should send the packet, and does nnot need to wait for an ack
    TransmitNoAck,
    TransmitNoAckFinished,
    /// The radio should send a packet and be prepared to receive the ack response
    TransmitAck,
    // packet is transmitted, and now waiting for ack received
    TransmitAckWait,
    TransmitAckFinished,
}

/// The ERadioCommand is via a channel send from the application to the radio driver
#[derive(PartialEq, Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ERadioCommand {
    /// Send data to the remote node
    Data(EsbPacket),
    /// Ack reporting on/off. Default off.
    /// Reporting on will generate for each consumed Ack packet an reporting event, with the timestamp and the counter value
    /// Reporting makes it easy to maintain the connection state with the remote node with timestamp field
    /// And ack queue management with the counter value
    AckReporting(bool),
    // More commands for changing parameters in the driver can be added here ...
}
/// The ERadioEvent is sent from the driver to the application via a channel
#[derive(PartialEq, Debug, Clone)]
pub enum ERadioEvent {
    /// Data as received from a remote node, send to application
    Data(EsbPacket),
    /// Ack reporting on/off. Default off.
    /// Reporting on will generate for each consumed Ack packet an reporting event, with the timestamp and the counter value
    /// Reporting makes it easy to maintain the connection state with the remote node with timestamp field
    /// And ack queue management with the counter value
    AckReporting(AckReporting),
    // More events can be added here ...
}
