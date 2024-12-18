use core::ptr::{self, NonNull};
use core::u32;
use alloc::boxed::Box;
use alloc::collections::vec_deque::VecDeque;
use alloc::vec::Vec;
use core::mem;
use log::{debug, error, info};
use crate::constants::*;
use alloc::sync::Arc;
use dma_api;
use crate::memory::MemPool;
use crate::memory::Dma;
use crate::descriptor::{AdvancedRxDescriptor, AdvancedTxDescriptor, RX_STATUS_DD, RX_STATUS_EOP};





use crate::{IgbError, IgbResult};

const IGB_DEFUAULT_TXD: u32 = 256;
const IGB_DEFUAULT_RXD: u32 = 256;

//use pool size
const MEM_POOL: usize = 4096;
const MEM_POOL_ENTRY_SIZE: usize = 2048;


pub struct Igb {
    addr:*mut u8,
    rx_ring_count:u32,
    //rx queue number
    tx_ring_count:u32,
    rx_queues: Vec<IxgbeRxQueue>,
    tx_queues: Vec<IxgbeTxQueue>,
}

struct IxgbeRxQueue {
    descriptors: Box<[NonNull<AdvancedRxDescriptor>]>,
    num_descriptors: usize,
    pool: Arc<MemPool>,
    bufs_in_use: Vec<usize>,
    rx_index: usize,
}
impl IxgbeRxQueue {
    fn can_recv(&self) -> bool {
        let rx_index = self.rx_index;

        let desc = unsafe { self.descriptors[rx_index].as_ref() };
        let status = desc.get_ext_status() as u8;
        status & RX_STATUS_DD != 0
    }
}

struct IxgbeTxQueue {
    descriptors: Box<[NonNull<AdvancedTxDescriptor>]>,
    num_descriptors: usize,
    pool: Option<Arc<MemPool>>,
    bufs_in_use: VecDeque<usize>,
    clean_index: usize,
    tx_index: usize,
}

fn wrap_ring(index: usize, ring_size: usize) -> usize {
    (index + 1) & (ring_size - 1)
}

impl IxgbeTxQueue {
    fn can_send(&self) -> bool {
        let next_tx_index = wrap_ring(self.tx_index, self.num_descriptors);
        next_tx_index != self.clean_index
    }
}




impl Igb {
    pub fn new(bar0: NonNull<u8>) -> Self {
        info!("bar0 = {:?}", bar0);
        let bra0 = unsafe {
            bar0.as_ptr()
        };
                // initialize RX and TX queue
        let QN:usize = 1;
        //QS = 1024
        let rx_queues = Vec::with_capacity(QN);
        let tx_queues = Vec::with_capacity(QN);
       // let mut bra0 = bar0.pointer ;
        Self {
            addr:bra0,
            rx_ring_count:QN as u32,
            tx_ring_count:QN as u32,
            rx_queues,
            tx_queues,
        }

    }
    pub fn init(& mut self){
        self.wait_clear_reg32(IGB_CTRL, IGB_CTRL_DEV_RST);
        //do reset 
        info!("reset success");
       // self.set_flags32(IGB_CTRL, IGB_CTRL_LNK_RST);
        info!("link mode is defaut 00");
        self.set_flags32(IGB_CTRL_EXT, IGB_CTRL_EXT_DRV_LOAD);
        
        //igb get hw control
        info!("set lsu is 1");
        /* */
        let mac = self.get_mac_addr();

        info!(
            "mac address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        );
        let mut mii_reg = self.phy_read(0);
        mii_reg |= 1<<9;
        info!("rs_atu write_mii{:b}", mii_reg);
        let status = self.get_reg32(IGB_STATUS);
        info!("reset end status {:b}", status);
        self.phy_write(0, mii_reg);


        self.set_flags32(IGB_CTRL, IGB_CTRL_START);
        //FRCSPD defaut is 0 FRCDPLX
        
        info!("set SLU ok");
        loop{
            let status = self.get_reg32(IGB_STATUS);
            //info!("status {:b}", status);
            if (status &(1 << 1)) == (1<<1){
                break;
            }
        }
        info!("link up ok");
        //self.wait_set_reg32(IGB_EEC, IXGBE_EEC_ARD);
        //igb可能不需要
        //RO 位置 应该只是利用这个函数等待为1 说明读取完毕
        /* 
        self.setup_rx_mode();
        self.setup_tctl();
        self.setup_rctl();
        self.init_rx();
        */

    }

    fn get_mac_addr(&self) -> [u8; 6] {
        let low = self.get_reg32(IGB_ADDR_L);
        let high = self.get_reg32(IGB_ADDR_H);

        [
            (low & 0xff) as u8,
            (low >> 8 & 0xff) as u8,
            (low >> 16 & 0xff) as u8,
            (low >> 24) as u8,
            (high & 0xff) as u8,
            (high >> 8 & 0xff) as u8,
        ]
    }

    fn disable_interrupts(&self) {
        self.set_reg32(IGB_EIMS, 0);
        //mask interrupt
        self.clean_interrupt();
    }

    fn clean_interrupt(&self) {
        self.set_reg32(IGB_EIMC, u32::MAX);
        self.get_reg32(IGB_EICR);
    }

    fn setup_rx_mode(&mut self) {
        let mut rx_ctrl = self.get_reg32(IGB_RXCTL);
        rx_ctrl |= (IGB_RCTL_MPE | IGB_RCTL_UPE);
        self.set_reg32(IGB_RXCTL, rx_ctrl);
        //igb_set_rx_mode
    }
    fn setup_tctl(&mut self) {
        self.set_reg32(IGB_TXDCTL(0), 0);
        let mut tx_ctrl = self.get_reg32(IGB_TXCTL);
        tx_ctrl &= !IGB_TCTRL_CT;
        tx_ctrl |= IGB_TCTRL_PSP | IGB_TCTRL_RTLC | (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);
        tx_ctrl |= IGB_TCTRL_CT;
        self.set_reg32(IGB_TXCTL, tx_ctrl);
        //igb_setup_tctl
    }
    fn setup_rctl(&mut self) {
        let mut rx_ctrl = self.get_reg32(IGB_RXCTL);
        rx_ctrl &= !(E1000_RCTL_LBM_TCVR | E1000_RCTL_LBM_MAC);
        rx_ctrl |= IGB_RCTL_EN | IGB_RCTL_BAM | E1000_RCTL_SECRC;
        rx_ctrl |= IGB_RCTL_LPE;
        self.set_reg32(IGB_RXDCTL(0), 0);
        self.set_reg32(IGB_RXCTL, rx_ctrl);
    }


        // sections 4.6.7
    /// Initializes the rx queues of this device.
    #[allow(clippy::needless_range_loop)]
    fn init_rx(&mut self) -> bool {
        // disable rx while re-configuring it
        let pool =  MemPool::allocate(MEM_POOL, MEM_POOL_ENTRY_SIZE).unwrap();
        const  QS:usize = 1024;
        //self.wait_clear_reg32(IGB_RXCTL, IGB_RXCTL_RXEN);
        self.set_reg32(IGB_RXDCTL(0), 0);

        // section 4.6.11.3.4 - allocate all queues and traffic to PB0
        /*self.set_reg32(IXGBE_RXPBSIZE(0), IXGBE_RXPBSIZE_128KB);
        for i in 1..8 {
            self.set_reg32(IXGBE_RXPBSIZE(i), 0);
        }
        */  //use default 64k
        for i in 0..self.rx_ring_count {
           
            info!("initializing rx queue {}", i);
            assert_eq!(mem::size_of::<AdvancedRxDescriptor>(), 16);
            let ring_size_bytes = QS * mem::size_of::<AdvancedRxDescriptor>();
            //QS number of descrip in one queue
            let dma: Dma<AdvancedRxDescriptor> = Dma::allocate(ring_size_bytes, true).unwrap();
            let mut descriptors: [NonNull<AdvancedRxDescriptor>; QS] = [NonNull::dangling(); QS];

            unsafe {
                for desc_id in 0..QS {
                    descriptors[desc_id] = NonNull::new(dma.virt.add(desc_id)).unwrap();
                    descriptors[desc_id].as_mut().init();
                    //set all data 0
                }
            }
            self.set_reg32(IGB_RDBAL(u32::from(i)), (dma.phys as u64 & 0xffff_ffff) as u32);
            info!("{}", self.get_reg32(IGB_RDBAL(u32::from(i))));
            self.set_reg32(IGB_RDBAH(i as u32), (dma.phys as u64 >>32) as u32);
            self.set_reg32(IGB_RDLEN( i as u32), ring_size_bytes as u32);

            info!("rx ring {} phys addr: {:#x}", i, dma.phys);
            info!("rx ring {} virt addr: {:p}", i, dma.virt);

            self.set_reg32(IGB_RDH(u32::from(i)), 0);
            self.set_reg32(IGB_RDT(u32::from(i)), 0);

            /* 
            self.set_reg32(IGB_SRRCTL(u32::from(i)), 
            (self.get_reg32(IGB_SRRCTL(u32::from(i))) & !IXGBE_SRRCTL_DESCTYPE_MASK)
                   | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
            //clearn DESCTYPE and set 010
            */
            self.set_flags32(IGB_SRRCTL(u32::from(i)), IXGBE_SRRCTL_DROP_EN);
            // Program SRRCTL of the queue according to the size of the buffers and the required header 
            info!("SRRCTL{}", self.get_reg32(IGB_SRRCTL(u32::from(i))));
            self.wait_set_reg32(IGB_RXDCTL(0), IGB_RXDCTL_ENABLE);
            info!("RXDCTL OK");

            let rx_queue = IxgbeRxQueue {
                descriptors: Box::new(descriptors),
                pool: Arc::clone(&pool),
                num_descriptors: QS,
                rx_index: 0,
                bufs_in_use: Vec::with_capacity(QS),
            };

            self.rx_queues.push(rx_queue);
            
        }

        // enable CRC offloading

        self.set_reg32(IGB_RXCTL, IGB_RXCTL_RXEN);
       

        true
    }

    fn get_reg32(&self, reg: u32) -> u32 {
        //assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe { ptr::read_volatile((self.addr as usize + reg as usize) as *mut u32) }
    }

    /// Sets the register at `self.addr` + `reg` to `value`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn set_reg32(&self, reg: u32, value: u32) {
        //assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe {
            ptr::write_volatile((self.addr as usize + reg as usize) as *mut u32, value);
        }
    }

    /// Sets the `flags` at `self.addr` + `reg`.
    fn set_flags32(&self, reg: u32, flags: u32) {
        self.set_reg32(reg, self.get_reg32(reg) | flags);
    }

    /// Clears the `flags` at `self.addr` + `reg`.
    fn clear_flags32(&self, reg: u32, flags: u32) {
        self.set_reg32(reg, self.get_reg32(reg) & !flags);
    }

    /// Waits for `self.addr` + `reg` to clear `value`.
    fn wait_clear_reg32(&self, reg: u32, value: u32) {
        loop {
            let current = self.get_reg32(reg);
            if (current & value) == 0 {
                break;
            }
            // `thread::sleep(Duration::from_millis(100));`
            // let _ = H::wait_ms(100);
            //let _ = H::wait_until(Duration::from_millis(100));
        }
    }

    /// Waits for `self.addr` + `reg` to set `value`.
    fn wait_set_reg32(&self, reg: u32, value: u32) {
        loop {
            let current = self.get_reg32(reg);
            if (current & value) == value {
                break;
            }
            //let _ = H::wait_until(Duration::from_millis(100));
        }
    }

    fn phy_read(&mut self, offset: u32) -> u32{
        let mdic_info = self.get_reg32(IGB_MDIC);
        info!("mdic_info {:b}", mdic_info);
        let mut mdic_cmd = offset << 16 | 1 << 21 | MDIC_READ;
        self.set_reg32(IGB_MDIC, mdic_cmd);

        loop {
            mdic_cmd = self.get_reg32(IGB_MDIC);
            if (mdic_cmd & MDIC_READ) == MDIC_READ{
                break;
            }
            if (mdic_cmd & MDIC_ERROR) == MDIC_ERROR{
                error!("read error");
                return 0;
            }
        }
        return mdic_cmd;
    }

    fn phy_write(&mut self, offset: u32, data:u32) -> bool{
        let mdic_info = self.get_reg32(IGB_MDIC);
        info!("mdic_info {:b}", mdic_info);
        let mut mdic_cmd = offset << 16 | 1 << 21 | data | MDIC_WRITE;
        info!("phy write cmd {:b}", mdic_cmd);
        self.set_reg32(IGB_MDIC, mdic_cmd);

        loop {
            mdic_cmd = self.get_reg32(IGB_MDIC);
            if (mdic_cmd & MDIC_READ) == MDIC_READ{
                info!("write ok");
                break;
            }
            if (mdic_cmd & MDIC_ERROR) == MDIC_READ{
                error!("read error");
                return false;
            }
        }
        return true;
    }

}

