use core::ptr::NonNull;

use log::debug;

use crate::{
    descriptor::{AdvRxDesc, AdvTxDesc},
    err::IgbError,
    mac::*,
    phy::Phy,
    ring::{Ring, DEFAULT_RING_SIZE},
};

pub struct Igb {
    mac: Mac,
    tx_ring: Ring<AdvTxDesc>,
    rx_ring: Ring<AdvRxDesc>,
    phy: Phy,
}

impl Igb {
    pub fn new(bar0: NonNull<u8>) -> Result<Self, IgbError> {
        let mac = Mac::from(bar0);

        let tx_ring = Ring::new(mac, DEFAULT_RING_SIZE)?;
        let rx_ring = Ring::new(mac, DEFAULT_RING_SIZE)?;

        Ok(Self {
            mac,
            tx_ring,
            rx_ring,
            phy: Phy::new(mac),
        })
    }

    pub fn open(&mut self) -> Result<(), IgbError> {
        self.mac.disable_interrupts();

        self.mac.reset()?;

        self.mac.disable_interrupts();

        debug!("reset done");

        self.setup_phy_and_the_link()?;

        self.init_stat();

        self.init_rx();
        self.init_tx();

        self.enable_interrupts();

        self.mac.set_link_up();

        Ok(())
    }

    fn init_stat(&mut self) {
        //TODO
    }
    /// 4.5.9 Receive Initialization
    fn init_rx(&mut self) {
        // disable rx when configing.
        // self.reg.write_reg(RCTL::empty());

        self.rx_ring.init();

        // self.reg.write_reg(RCTL::RXEN | RCTL::SZ_4096);
    }

    fn init_tx(&mut self) {
        // self.reg.write_reg(TCTL::empty());

        // self.tx_ring.init();

        // self.reg.write_reg(TCTL::EN);
    }

    fn setup_phy_and_the_link(&mut self) -> Result<(), IgbError> {
        self.phy.power_up()?;
        Ok(())
    }

    pub fn mac(&self) -> [u8; 6] {
        self.mac.read_mac()
    }

    fn enable_interrupts(&self) {
        //TODO
    }

    pub fn status(&self) -> MacStatus {
        self.mac.status()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Speed {
    Mb10,
    Mb100,
    Mb1000,
}
