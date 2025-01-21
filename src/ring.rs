use dma_api::{DVec, Direction};

use crate::{descriptor::Descriptor, err::IgbError, mac::Mac};

pub const DEFAULT_RING_SIZE: usize = 256;

pub struct Ring<D: Descriptor> {
    pub descriptors: DVec<D>,
    mac: Mac,
}

impl<D: Descriptor> Ring<D> {
    pub fn new(mac: Mac, size: usize) -> Result<Self, IgbError> {
        let descriptors =
            DVec::zeros(size, 4096, Direction::Bidirectional).ok_or(IgbError::NoMemory)?;

        Ok(Self { descriptors, mac })
    }

    pub fn init(&mut self) {}
}
