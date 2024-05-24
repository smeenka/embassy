use super::MAX_PACKET_SIZE;

const DMA_PACKET_SIZE: usize = MAX_PACKET_SIZE + 2;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]

pub struct EsbPacket {
    rssi: Option<u8>,
    pipe_nr: u8,
    /// Note this is the Dma buffer which will be send or received via EasyDma.
    /// The buffer is 2 bytes longer than the MaxPacketSize
    /// Byte 0 contains the size of the data
    /// Byte 1 contains the 3 metabits for Esb: bit 0-1 packet counter, bit 2: NoAck
    data: [u8; DMA_PACKET_SIZE],
}

impl EsbPacket {
    pub fn empty() -> Self {
        let data = [0; DMA_PACKET_SIZE];
        EsbPacket {
            rssi: None,
            pipe_nr: 0,
            data,
        }
    }
    /// In user context, create an ack  packet for sending in an ack response. Pid will be added to the packet in the driver
    pub fn ack_packet(tx_data: &[u8], pipe_nr: u8) -> Self {
        EsbPacket::tx_packet(tx_data, pipe_nr, false)
    }

    /// In user context, create a packet for sending. Pid will be added to the packet in the driver
    pub fn tx_packet(tx_data: &[u8], pipe_nr: u8, ack: bool) -> Self {
        let no_ack = if ack { 0 } else { 1 };
        let mut data = [0; DMA_PACKET_SIZE];
        let len = if tx_data.len() >= MAX_PACKET_SIZE {
            MAX_PACKET_SIZE
        } else {
            tx_data.len()
        };
        for i in 0..len {
            data[i + 2] = tx_data[i];
        }
        data[0] = len as u8;
        data[1] = no_ack;
        EsbPacket {
            rssi: None,
            pipe_nr,
            data,
        }
    }
    /// In driver context (not interrupt!), create a packet for receiving.
    pub fn rx_packet(pipe_nr: u8) -> Self {
        let data = [0; DMA_PACKET_SIZE];
        EsbPacket {
            rssi: None,
            pipe_nr,
            data,
        }
    }
    pub(crate) fn dma_pointer(&self) -> u32 {
        (self.data.as_ptr() as *const u8) as u32
    }

    pub fn reset(&mut self) {
        self.rssi = None;
        self.pipe_nr = 0;
        for i in 0..DMA_PACKET_SIZE {
            self.data[i] = 0;
        }
    }
    /// get the lenght of the data in the packet. Only valid if the packet contains valid data
    pub fn len(&self) -> u8 {
        self.data[0]
    }
    pub fn data(&self) -> &[u8] {
        &self.data
    }
    /// get the ack bit from the data in the packet. Only valid after the packet contains valid data
    pub fn pid(&self) -> u8 {
        (self.data[1] >> 1) & 0x3
    }
    /// get the ack bit from the data in the packet. Only valid after the packet is received and contains valid data
    pub fn ack(&self) -> bool {
        if self.data[1] & 0x1 == 1 {
            false
        } else {
            true
        }
    }
    pub fn rssi(&self) -> Option<u8> {
        self.rssi
    }
    pub fn pipe_nr(&self) -> u8 {
        self.pipe_nr
    }
    pub fn set_pipe_nr(&mut self, pipe_nr: u8) {
        self.pipe_nr = pipe_nr
    }
    pub fn set_rssi(&mut self, rssi: u8) {
        self.rssi = Some(rssi)
    }
    pub(crate) fn set_pid(&mut self, pid: u8) {
        let no_ack = self.data[1] & 0x1;
        let pid = pid & 0x3;
        self.data[1] = pid << 1 | no_ack;
    }
}
