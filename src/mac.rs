#![allow(non_camel_case_types)]

use core::{
    ops::Deref,
    ptr::NonNull,
    sync::atomic::{fence, Ordering},
    time::Duration,
    u32,
};

use bitflags::{bitflags, Flags};
use log::error;
use tock_registers::{
    fields::FieldValue,
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::*,
};

use crate::{err::IgbError, sleep, wait_for, Speed};

pub const EIMS: u32 = 0x01524;
pub const EIMC: u32 = 0x01528;
pub const EICR: u32 = 0x01580;

#[derive(Clone, Copy)]
pub(crate) struct Mac {
    reg: NonNull<MacRegister>,
}

impl From<NonNull<u8>> for Mac {
    fn from(value: NonNull<u8>) -> Self {
        Self { reg: value.cast() }
    }
}

impl Mac {
    pub fn disable_interrupts(&mut self) {
        self.reg_mut().eims.set(0);
        self.clear_interrupts();
    }

    pub fn enable_interrupts(&mut self) {
        //TODO
    }

    /// Clear all interrupt masks for all queues.
    pub fn clear_interrupts(&mut self) {
        // Clear interrupt mask
        self.reg_mut().eimc.set(u32::MAX);
        self.reg().eicr.get();
    }

    pub fn reset(&mut self) -> Result<(), IgbError> {
        self.reg_mut().ctrl.modify(CTRL::RST::Reset);

        wait_for(
            || self.reg().ctrl.matches_any(&[CTRL::RST::Normal]),
            Duration::from_millis(1),
            Some(1000),
        )
    }

    pub fn set_link_up(&mut self) {
        self.reg_mut().ctrl.modify(
            CTRL::SLU::SET
                + CTRL::FD::SET
                + CTRL::SPEED::Speed1000
                + CTRL::FRCSPD::SET
                + CTRL::FRCDPLX::SET,
        );
    }

    pub fn write_mdic(&self, phys_addr: u32, offset: u32, data: u16) -> Result<(), IgbError> {
        self.reg().mdic.write(
            MDIC::REGADDR.val(offset)
                + MDIC::PHY_ADDR.val(phys_addr)
                + MDIC::DATA.val(data as _)
                + MDIC::OP::Write,
        );
        fence(Ordering::SeqCst);
        loop {
            let mdic = self.reg().mdic.extract();

            if mdic.is_set(MDIC::READY) {
                break;
            }
            if mdic.is_set(MDIC::E) {
                error!("MDIC read error");
                return Err(IgbError::Unknown);
            }
        }

        Ok(())
    }

    pub fn read_mdic(&self, phys_addr: u32, offset: u32) -> Result<u16, IgbError> {
        self.reg()
            .mdic
            .write(MDIC::REGADDR.val(offset) + MDIC::PHY_ADDR.val(phys_addr) + MDIC::OP::Read);
        fence(Ordering::SeqCst);
        loop {
            let mdic = self.reg().mdic.extract();
            if mdic.is_set(MDIC::READY) {
                return Ok(mdic.read(MDIC::DATA) as _);
            }
            if mdic.is_set(MDIC::E) {
                error!("MDIC read error");
                return Err(IgbError::Unknown);
            }
        }
    }

    pub fn software_semaphore_aquire(&self) -> Result<(), IgbError> {
        loop {
            self.reg().swsm.modify(SWSM::SWESMBI::SET);
            fence(Ordering::SeqCst);

            if self.reg().swsm.is_set(SWSM::SWESMBI) {
                return Ok(());
            }
        }
    }

    pub fn software_semaphore_release(&self) {
        self.reg()
            .swsm
            .modify(SWSM::SWESMBI::CLEAR + SWSM::SMBI::CLEAR);
    }

    pub fn software_sync_aquire(&self, flags: SyncFlags) -> Result<u32, IgbError> {
        let swmask = flags.bits();
        let fwmask = swmask << 16;

        let mut value;
        loop {
            value = self.reg().sw_fw_sync.get();

            if (value & (swmask | fwmask)) == 0 {
                break;
            }
        }

        value |= swmask;
        self.reg().sw_fw_sync.set(value);

        Ok(swmask)
    }

    pub fn software_sync_release(&self, swmask: u32) {
        let raw = self.reg().sw_fw_sync.get();
        self.reg().sw_fw_sync.set(raw & !swmask);
    }

    fn ral(&self, i: usize) -> u32 {
        if i <= 15 {
            self.reg().ralh_0_15[i * 2].get()
        } else {
            self.reg().ralh_16_23[i * 2].get()
        }
    }

    fn rah(&self, i: usize) -> u32 {
        if i <= 15 {
            self.reg().ralh_0_15[i * 2 + 1].get()
        } else {
            self.reg().ralh_16_23[i * 2 + 1].get()
        }
    }

    pub fn read_mac(&self) -> [u8; 6] {
        let low = self.ral(0);
        let high = self.rah(0);

        [
            (low & 0xff) as u8,
            ((low >> 8) & 0xff) as u8,
            ((low >> 16) & 0xff) as u8,
            (low >> 24) as u8,
            (high & 0xff) as u8,
            ((high >> 8) & 0xff) as u8,
        ]
    }

    pub fn status(&self) -> MacStatus {
        let status = self.reg().status.extract();
        let speed = match status.read_as_enum(STATUS::SPEED) {
            Some(STATUS::SPEED::Value::Speed1000) => Speed::Mb1000,
            Some(STATUS::SPEED::Value::Speed100) => Speed::Mb100,
            _ => Speed::Mb10,
        };
        let full_duplex = status.is_set(STATUS::FD);
        let link_up = status.is_set(STATUS::LU);
        let phy_reset_asserted = status.is_set(STATUS::PHYRA);

        MacStatus {
            full_duplex,
            link_up,
            speed,
            phy_reset_asserted,
        }
    }

    fn reg(&self) -> &MacRegister {
        unsafe { self.reg.as_ref() }
    }
    fn reg_mut(&mut self) -> &mut MacRegister {
        unsafe { self.reg.as_mut() }
    }
}

#[derive(Debug, Clone)]
pub struct MacStatus {
    pub full_duplex: bool,
    pub link_up: bool,
    pub speed: Speed,
    pub phy_reset_asserted: bool,
}

register_structs! {
    pub MacRegister {
        (0x0 => ctrl: ReadWrite<u32, CTRL::Register>),
        (0x4 => _rsv1),
        (0x8 => status: ReadOnly<u32, STATUS::Register>),
        (0xC => _rsv2),
        (0x18 => ctrl_ext: ReadWrite<u32>),
        (0x1c => _rsv3),
        (0x20 => mdic: ReadWrite<u32, MDIC::Register>),
        (0x24 => _rsv4),
        (0x28 => rctl: ReadWrite<u32>),
        (0x2c => _rsv7),
        (0x01524 => eims: ReadWrite<u32>),
        (0x01528 => eimc: ReadWrite<u32>),
        (0x0152c => eiac: ReadWrite<u32>),
        (0x01530 => eiam: ReadWrite<u32>),
        (0x01534 => _rsv5),
        (0x01580 => eicr: ReadWrite<u32>),
        (0x01584 => _rsv6),
        (0x5400 => ralh_0_15: [ReadWrite<u32>; 32]),
        (0x5480 => _rsv8),
        (0x54e0 => ralh_16_23: [ReadWrite<u32>;32]),
        (0x5560 => _rsv9),
        (0x5B50 => swsm: ReadWrite<u32, SWSM::Register>),
        (0x5B54 => fwsm: ReadWrite<u32>),
        (0x5B58 => _rsv10),
        (0x5B5C => sw_fw_sync: ReadWrite<u32>),
        (0x5B60 => _rsv11),

        // The end of the struct is marked as follows.
        (0xBFFF => @END),
    }
}

register_bitfields! [
    // First parameter is the register width. Can be u8, u16, u32, or u64.
    u32,

    CTRL [
        FD OFFSET(0) NUMBITS(1)[
            HalfDuplex = 0,
            FullDuplex = 1,
        ],
        SLU OFFSET(6) NUMBITS(1)[],
        SPEED OFFSET(8) NUMBITS(2)[
            Speed10 = 0,
            Speed100 = 1,
            Speed1000 = 0b10,
        ],
        FRCSPD OFFSET(11) NUMBITS(1)[],
        FRCDPLX OFFSET(12) NUMBITS(1)[],
        RST OFFSET(26) NUMBITS(1)[
            Normal = 0,
            Reset = 1,
        ],
    ],
    STATUS [
        FD OFFSET(0) NUMBITS(1)[
            HalfDuplex = 0,
            FullDuplex = 1,
        ],
        LU OFFSET(1) NUMBITS(1)[],
        SPEED OFFSET(6) NUMBITS(2)[
            Speed10 = 0,
            Speed100 = 1,
            Speed1000 = 0b10,
        ],
         PHYRA OFFSET(10) NUMBITS(1)[],
    ],

    MDIC [
        DATA OFFSET(0) NUMBITS(16)[],
        REGADDR OFFSET(16) NUMBITS(5)[],
        PHY_ADDR OFFSET(21) NUMBITS(5)[],
        OP OFFSET(26) NUMBITS(1)[
            Read = 1,
            Write = 0b10,
        ],
        READY OFFSET(28) NUMBITS(1)[],
        I OFFSET(29) NUMBITS(1)[],
        E OFFSET(30) NUMBITS(1)[
            NoError = 0,
            Error = 1,
        ],
        Destination OFFSET(31) NUMBITS(1)[
            Internal = 0,
            External = 1,
        ]
    ],

    SWSM [
        SMBI OFFSET(0) NUMBITS(1)[],
        SWESMBI OFFSET(1) NUMBITS(1)[],
        WMNG OFFSET(2) NUMBITS(1)[],
        EEUR OFFSET(3) NUMBITS(1)[],
    ],

    SW_FW_SYNC [
        SW_EEP_SM OFFSET(0) NUMBITS(1)[],
        SW_PHY_SM0 OFFSET(1) NUMBITS(1)[],
        SW_PHY_SM1 OFFSET(2) NUMBITS(1)[],
        SW_MAC_CSR_SM OFFSET(3) NUMBITS(1)[],
        SW_FLASH_SM OFFSET(4) NUMBITS(1)[],

        FW_EEP_SM OFFSET(16) NUMBITS(1)[],
        FW_PHY_SM0 OFFSET(17) NUMBITS(1)[],
        FW_PHY_SM1 OFFSET(18) NUMBITS(1)[],
        FW_MAC_CSR_SM OFFSET(19) NUMBITS(1)[],
        FW_FLASH_SM OFFSET(20) NUMBITS(1)[],
    ]
];

/* Multicast Table Array - 128 entries */
fn mta(i: u32) -> u32 {
    0x05200 + i * 4
}

bitflags! {
    pub struct SyncFlags: u32 {
        const EEPROM = 1 << 0;
        const PHY0 = 1 << 1;
        const PHY1 = 1 << 2;
        const MAC_CSR = 1 << 3;
        const FLASH = 1 << 4;
    }
}
