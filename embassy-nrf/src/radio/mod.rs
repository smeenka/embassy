//! Integrated 2.4 GHz Radio
//!
//! The 2.4 GHz radio transceiver is compatible with multiple radio standards
//! such as 1Mbps, 2Mbps and Long Range Bluetooth Low Energy.

#![macro_use]

/// todo remove
 use log;

/// Bluetooth Low Energy Radio driver.
pub mod ble;
#[cfg(any(
    feature = "nrf52811",
    feature = "nrf52820",
    feature = "nrf52833",
    feature = "nrf52840",
    feature = "_nrf5340-net"
))]
/// IEEE 802.15.4
pub mod ieee802154;

#[cfg(feature = "nrf-esb")]
pub mod esb;

use core::{cell::RefCell, marker::PhantomData};

use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    waitqueue::AtomicWaker,
};
use pac::radio::state::STATE_A as RadioState;
pub use pac::radio::txpower::TXPOWER_A as TxPower;

use self::esb::esb_state::EsbState;
use crate::{interrupt, pac, Peripheral};

/// RADIO error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Buffer was too long.
    BufferTooLong,
    /// Buffer was too short.
    BufferTooShort,
    /// The buffer is not in data RAM. It's most likely in flash, and nRF's DMA cannot access flash.
    BufferNotInRAM,
    /// Clear channel assessment reported channel in use
    ChannelInUse,
    /// CRC check failed
    CrcFailed(u16),
    #[cfg(feature = "nrf-esb")]
    ChannelFull,
    #[cfg(feature = "nrf-esb")]
    ChannelEmpty,
    #[cfg(feature = "nrf-esb")]
    PipeNrTooHigh,
}

/// Interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();
        // clear all interrupts
        r.intenclr.write(|w| w.bits(0xffff_ffff));
        log::info!("Ixx");

        s.event_waker.wake();
    }
}

pub(crate) struct State {
    /// end packet transmission or reception
    event_waker: AtomicWaker,
    #[cfg(feature = "nrf-esb")]
    pub(crate) mutex: Mutex<CriticalSectionRawMutex, RefCell<EsbState>>,
}
impl State {
    pub(crate) const fn new() -> Self {
        Self {
            event_waker: AtomicWaker::new(),
            #[cfg(feature = "nrf-esb")]
            // inner mutability pattern
            mutex: Mutex::new(RefCell::new(EsbState::new())),
        }
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> &'static crate::pac::radio::RegisterBlock;
    fn state() -> &'static State;
}

macro_rules! impl_radio {
    ($type:ident, $pac_type:ident, $irq:ident) => {
        impl crate::radio::SealedInstance for peripherals::$type {
            fn regs() -> &'static pac::radio::RegisterBlock {
                unsafe { &*pac::$pac_type::ptr() }
            }

            fn state() -> &'static crate::radio::State {
                static STATE: crate::radio::State = crate::radio::State::new();
                &STATE
            }
        }
        impl crate::radio::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

/// Radio peripheral instance.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// Get the state of the radio
pub(crate) fn state(radio: &pac::radio::RegisterBlock) -> RadioState {
    match radio.state.read().state().variant() {
        Some(state) => state,
        None => unreachable!(),
    }
}
