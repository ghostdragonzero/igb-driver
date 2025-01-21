use crate::{
    err::IgbError,
    mac::{Mac, SyncFlags},
};

const PHY_CONTROL: u32 = 0;
const MII_CR_POWER_DOWN: u16 = 0x0800;

pub struct Phy {
    mac: Mac,
    addr: u32,
}

impl Phy {
    pub fn new(mac: Mac) -> Self {
        Self { mac, addr: 1 }
    }

    pub fn read_mdic(&self, offset: u32) -> Result<u16, IgbError> {
        self.mac.read_mdic(self.addr, offset)
    }

    pub fn write_mdic(&self, offset: u32, data: u16) -> Result<(), IgbError> {
        self.mac.write_mdic(self.addr, offset, data)
    }

    pub fn aquire_sync(&self, flags: SyncFlags) -> Result<Synced, IgbError> {
        Synced::new(self.mac, flags)
    }

    pub fn power_up(&self) -> Result<(), IgbError> {
        let mut mii_reg = self.read_mdic(PHY_CONTROL)?;
        mii_reg &= !MII_CR_POWER_DOWN;
        self.write_mdic(PHY_CONTROL, mii_reg)
    }
}

pub struct Synced {
    mac: Mac,
    mask: u32,
}

impl Synced {
    pub fn new(mac: Mac, flags: SyncFlags) -> Result<Self, IgbError> {
        let semaphore = Semaphore::new(mac)?;
        let mask = mac.software_sync_aquire(flags)?;
        drop(semaphore);
        Ok(Self { mac, mask })
    }
}

impl Drop for Synced {
    fn drop(&mut self) {
        let semaphore = Semaphore::new(self.mac).unwrap();
        self.mac.software_sync_release(self.mask);
        drop(semaphore);
    }
}

pub struct Semaphore {
    mac: Mac,
}

impl Semaphore {
    pub fn new(mac: Mac) -> Result<Self, IgbError> {
        mac.software_semaphore_aquire()?;
        Ok(Self { mac })
    }
}

impl Drop for Semaphore {
    fn drop(&mut self) {
        self.mac.software_semaphore_release();
    }
}
