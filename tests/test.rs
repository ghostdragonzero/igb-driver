#![no_std]
#![no_main]
#![feature(custom_test_frameworks)]
#![test_runner(bare_test::test_runner)]
#![reexport_test_harness_main = "test_main"]

extern crate alloc;

use core::time::Duration;

use bare_test::{
    driver::device_tree::get_device_tree, fdt::PciSpace, mem::mmu::iomap, println, time::delay,
};
use igb_driver::Igb;
use log::{debug, info};
use pcie::*;

bare_test::test_setup!();

#[test_case]
fn test_work() {
    let mut igb = get_igb();

    debug!("igb init");

    igb.open().unwrap();

    let mac = igb.mac();

    debug!("igb opened");

    debug!("mac: {:x?}", mac);

    info!("status: {:?}", igb.status());

    while !igb.status().link_up {
        sleep(Duration::from_millis(10));
    }

    info!("status: {:?}", igb.status());
}

fn sleep(duration: Duration) {
    spin_on::spin_on(delay(duration));
}

struct KernelImpl;

impl igb_driver::Kernel for KernelImpl {
    fn sleep(duration: Duration) {
        sleep(duration);
    }
}

igb_driver::set_impl!(KernelImpl);

fn get_igb() -> Igb {
    let fdt = get_device_tree().unwrap();
    let pcie = fdt
        .find_compatible(&["pci-host-ecam-generic"])
        .next()
        .unwrap()
        .into_pci()
        .unwrap();

    let mut pcie_regs = alloc::vec![];

    println!("pcie: {}", pcie.node.name);

    for reg in pcie.node.reg().unwrap() {
        println!("pcie reg: {:#x}", reg.address);
        pcie_regs.push(iomap((reg.address as usize).into(), reg.size.unwrap()));
    }

    let mut bar_alloc = SimpleBarAllocator::default();

    for range in pcie.ranges().unwrap() {
        info!("pcie range: {:?}", range);

        match range.space {
            PciSpace::Memory32 => bar_alloc.set_mem32(range.cpu_address as _, range.size as _),
            PciSpace::Memory64 => bar_alloc.set_mem64(range.cpu_address, range.size),
            _ => {}
        }
    }

    let base_vaddr = pcie_regs[0];

    info!("Init PCIE @{:?}", base_vaddr);

    let mut root = RootComplexGeneric::new(base_vaddr);

    for elem in root.enumerate(None, Some(bar_alloc)) {
        debug!("PCI {}", elem);

        if let Header::Endpoint(ep) = elem.header {
            ep.update_command(elem.root, |cmd| {
                cmd | CommandRegister::IO_ENABLE
                    | CommandRegister::MEMORY_ENABLE
                    | CommandRegister::BUS_MASTER_ENABLE
            });

            if ep.vendor_id == 0x8086 && (ep.device_id == 0x10C9 || ep.device_id == 0x1533) {
                let bar_addr;
                let bar_size;
                match ep.bar {
                    pcie::BarVec::Memory32(bar_vec_t) => {
                        let bar0 = bar_vec_t[0].as_ref().unwrap();
                        bar_addr = bar0.address as usize;
                        bar_size = bar0.size as usize;
                    }
                    pcie::BarVec::Memory64(bar_vec_t) => {
                        let bar0 = bar_vec_t[0].as_ref().unwrap();
                        bar_addr = bar0.address as usize;
                        bar_size = bar0.size as usize;
                    }
                    pcie::BarVec::Io(_bar_vec_t) => todo!(),
                };

                println!("bar0: {:#x}", bar_addr);

                let addr = iomap(bar_addr.into(), bar_size);

                let igb = Igb::new(addr).unwrap();
                return igb;
            }
        }
    }
    panic!("no igb found");
}
