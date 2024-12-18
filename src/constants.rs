#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(clippy::all)]

pub const IGB_CTRL: u32 = 0x00000;
pub const IGB_STATUS: u32 = 0x00008;
pub const IGB_CTRL_EXT: u32 = 0x00018;

/* CTRL Bit Masks */
pub const IGB_CTRL_LNK_RST: u32 = 1 << 3; /* Rset Link */
pub const IGB_CTRL_SLU: u32 = 1 << 6; /* Set Link Up */
pub const IGB_CTRL_DEV_RST: u32 = 1 << 26; /* Rest Device. */
pub const IGB_CTRL_PHY_RST: u32 = 1 << 31; /* Reset (PHY) */
pub const IGB_CTRL_RST_MASK: u32 = IGB_CTRL_LNK_RST | IGB_CTRL_DEV_RST;
pub const IGB_CTRL_START:u32 = 1 | 0x00000200 | IGB_CTRL_SLU |0x00000200 |0x00001000;

/*CTRL EXRT Masks */
pub const IGB_CTRL_EXT_DRV_LOAD:u32 = 1 << 28; /* Drv loaded bit for FW */
/*RX reg */
pub const IGB_RXCTL: u32 = 0x00100;
pub const IGB_RXPBS: u32 = 0x02404;//defaut is 64k

/*RX Mask */
pub const IGB_RCTL_EN:u32 = 0x00000002;
pub const IGB_RCTL_SBP:u32 = 0x00000004;
pub const IGB_RCTL_UPE:u32 = 0x00000008;
pub const IGB_RCTL_MPE:u32 = 0x00000010;
pub const IGB_RCTL_LPE:u32 = 0x00000020;
pub const IGB_RCTL_VFE:u32 = 0x00040000;
pub const IGB_RCTL_BAM:u32 = 0x00008000;

pub const  E1000_RCTL_LBM_NO:u32 =	0x00000000; /* no loopback mode */
pub const  E1000_RCTL_LBM_MAC:u32 = 	0x00000040; /* MAC loopback mode */
pub const  E1000_RCTL_LBM_TCVR:u32 = 	0x000000C0; /* tcvr loopback mode */

pub const  E1000_RCTL_SECRC:u32 = 	0x04000000; /* Strip Ethernet CRC */


pub fn IGB_RDBAL(i: u32) -> u32 {
    if i < 16 {
        0x0C000 + i * 0x40
    } else {
        0x0D004 + ((i - 64) * 0x40)
    }
}

pub fn IGB_RDBAH(i: u32) -> u32 {
    if i < 16 {
        0x0C004 + i * 0x40
    } else {
        0x0D008 + ((i - 64) * 0x40)
    }
}
pub fn IGB_RDLEN(i: u32) -> u32 {
    if i < 16 {
        0x0C008 + i * 0x40
    } else {
        0x0D008 + ((i - 64) * 0x40)
    }
}
pub fn IGB_RDH(i: u32) -> u32 {
    if i < 64 {
        0x0C010 + i * 0x40
    } else {
        0x0D010 + ((i - 64) * 0x40)
    }
}
pub fn IGB_RDT(i: u32) -> u32 {
    if i < 64 {
        0x0C018 + i * 0x40
    } else {
        0x0D018 + ((i - 64) * 0x40)
    }
}

pub fn IGB_RXDCTL(i: u32) -> u32 {
    if i < 15 {
        0x0C028 + i * 0x40
    } else {
        0x0D018 + ((i - 64) * 0x40)
    }
}
pub const IGB_RXDCTL_ENABLE:u32 = 0x01000000;
/*
 * Split and Replication Receive Control Registers
 * 00-15 : 0x0C00c + 0x40*n;
 */
pub const IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF: u32 = 0x02000000;
pub const IXGBE_SRRCTL_DESCTYPE_MASK: u32 = 0x0E000000;

pub const IXGBE_SRRCTL_DROP_EN: u32 = 0x02000000;

pub fn IGB_SRRCTL(i: u32) -> u32 {
    if i <= 15 {
        0x0C00C + i * 0x40
    } else if i < 64 {
        0x01014 + i * 0x40
    } else {
        0x0D014 + ((i - 64) * 0x40)
    }
}



/*Rx Bit Masks */

pub const IGB_RXCTL_RXEN: u32 = 1 << 1;


/*Tx reg */
pub const IGB_TXCTL:u32 = 0x00400;
pub fn IGB_TXDCTL(i: u32) -> u32 {
    if i < 15 {
        0x0E028 + i * 0x40
    } else {
        0x0D018 + ((i - 64) * 0x40)
    }
}

/*Tx mask */
pub const IGB_TCTRL_PSP:u32 =  0x0000ff0;
pub const IGB_TCTRL_CT:u32 =  0x0000ff0;
pub const IGB_TCTRL_RTLC:u32 =  0x0000ff0;
/* Collision related configuration parameters ????*/

pub const  E1000_CT_SHIFT:u32 = 			4;
pub const  E1000_COLLISION_THRESHOLD:u32 = 	15;





/* Receive Address page 528*/
pub const IGB_ADDR_L:u32 = 0x05400;
pub const IGB_ADDR_H:u32 = 0x05404;

/*EEROM */
pub const IGB_EEC:u32 = 0x00010;


/*intrrupt */
pub const IGB_EIMS:u32 = 0x1524;
pub const IGB_EIMC:u32 = 0x1528;
pub const IGB_EICR:u32 = 0x1580;




/*MDIC reg */
pub const IGB_MDIC:u32 = 0x00020;

/*MDIC mask */
pub const MDIC_READ:u32 =  0x08000000;
pub const MDIC_WRITE:u32 = 0x04000000;
pub const MDIC_READY:u32 = 1 << 28;
pub const MDIC_ERROR:u32 = 1 << 30;

