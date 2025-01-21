#![no_std]

extern crate alloc;

mod descriptor;
mod err;
mod igb;
mod mac;
mod phy;
mod ring;

use core::time::Duration;

use err::IgbError;
pub use igb::*;

pub trait Kernel {
    fn sleep(duration: Duration);
}

pub(crate) fn sleep(duration: Duration) {
    extern "Rust" {
        fn _igb_driver_sleep(duration: Duration);
    }

    unsafe {
        _igb_driver_sleep(duration);
    }
}

#[macro_export]
macro_rules! set_impl {
    ($t: ty) => {
        #[no_mangle]
        unsafe fn _igb_driver_sleep(duration: core::time::Duration) {
            <$t as $crate::Kernel>::sleep(duration)
        }
    };
}

fn wait_for<F: Fn() -> bool>(
    f: F,
    interval: Duration,
    try_count: Option<usize>,
) -> Result<(), IgbError> {
    for _ in 0..try_count.unwrap_or(usize::MAX) {
        if f() {
            return Ok(());
        }

        sleep(interval);
    }
    Err(IgbError::Timeout)
}
