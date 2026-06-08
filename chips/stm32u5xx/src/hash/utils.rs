// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026.

use core::cell::Cell;

use kernel::hil::digest;
use kernel::utilities::cells::MapCell;
use kernel::utilities::leasable_buffer::{SubSlice, SubSliceMut};
use kernel::ErrorCode;

const MAX_HMAC_KEY_LEN: usize = 512 / 8;

#[derive(Clone, Copy)]
pub enum DataWidth {
    _32bitData,
    _16bitData,
    _8bitData,
    _1bitData,
}

#[derive(Clone, Copy)]
pub enum Mode {
    MD5,
    SHA1,
    SHA2_224,
    SHA2_256,
}

impl Mode {
    pub fn get_digest_len(&self) -> usize {
        match self {
            Mode::MD5 => 4,
            Mode::SHA1 => 5,
            Mode::SHA2_224 => 7,
            Mode::SHA2_256 => 8,
        }
    }
}

#[derive(Clone, Copy)]
pub enum State {
    Add,
    Run,
    HmacInit,
    HmacPreAuth,
    HmacPostAuth,
    HmacFinalize,
}

#[derive(Clone, Copy)]
pub enum HashClient<'a, const DIGEST_LEN: usize> {
    Single(
        Option<&'a dyn digest::ClientData<DIGEST_LEN>>,
        Option<&'a dyn digest::ClientHash<DIGEST_LEN>>,
        Option<&'a dyn digest::ClientVerify<DIGEST_LEN>>,
    ),
    DoubleHash(&'a dyn digest::ClientDataHash<DIGEST_LEN>),
    DoubleVerify(&'a dyn digest::ClientDataVerify<DIGEST_LEN>),
    Full(&'a dyn digest::Client<DIGEST_LEN>),
}

impl<const DIGEST_LEN: usize> HashClient<'_, DIGEST_LEN> {
    pub fn add_data_done(&self, result: Result<(), ErrorCode>, data: SubSlice<'static, u8>) {
        match self {
            Self::Single(client_data, _, _) => {
                client_data.map(|c| c.add_data_done(result, data));
            }
            Self::DoubleHash(client) => client.add_data_done(result, data),
            Self::DoubleVerify(client) => client.add_data_done(result, data),
            Self::Full(client) => client.add_data_done(result, data),
        }
    }

    pub fn add_mut_data_done(&self, result: Result<(), ErrorCode>, data: SubSliceMut<'static, u8>) {
        match self {
            Self::Single(client_data, _, _) => {
                client_data.map(|c| c.add_mut_data_done(result, data));
            }
            Self::DoubleHash(client) => client.add_mut_data_done(result, data),
            Self::DoubleVerify(client) => client.add_mut_data_done(result, data),
            Self::Full(client) => client.add_mut_data_done(result, data),
        }
    }

    pub fn hash_done(&self, result: Result<(), ErrorCode>, digest: &'static mut [u8; DIGEST_LEN]) {
        match self {
            Self::Single(_, client_hash, _) => {
                client_hash.map(|c| c.hash_done(result, digest));
            }
            Self::DoubleHash(client) => client.hash_done(result, digest),
            Self::Full(client) => client.hash_done(result, digest),
            _ => (),
        }
    }

    pub fn verification_done(
        &self,
        result: Result<bool, ErrorCode>,
        compare: &'static mut [u8; DIGEST_LEN],
    ) {
        match self {
            Self::Single(_, _, client_verify) => {
                client_verify.map(|c| c.verification_done(result, compare));
            }
            Self::DoubleVerify(client) => client.verification_done(result, compare),
            Self::Full(client) => client.verification_done(result, compare),
            _ => (),
        }
    }
}

pub struct Leftover {
    buffer: Cell<Option<u32>>,
    // it is used if the FIFO is full and we cannot write the leftover we collected.
    // unfortunately, it is not possible to extend the subslice, I could have written the leftover on top of it.
    secondary_buffer: Cell<Option<u32>>,
    index: Cell<usize>,
}

impl Leftover {
    pub fn new() -> Self {
        Leftover {
            buffer: Cell::new(None),
            secondary_buffer: Cell::new(None),
            index: Cell::new(0),
        }
    }

    fn add_byte(buf: Option<u32>, byte: u8) -> Option<u32> {
        match buf {
            Some(b) => Some(b >> 8 | (byte as u32).rotate_right(8)),
            None => Some((byte as u32).rotate_right(8)),
        }
    }

    pub fn add(&self, byte: u8) {
        if !self.is_full() {
            self.buffer.update(|buf| Leftover::add_byte(buf, byte));
        } else {
            self.secondary_buffer
                .update(|buf| Leftover::add_byte(buf, byte));
        }

        self.index.update(|index| (index + 1) % 8);
    }

    pub fn to_le(&self) -> u32 {
        match self.buffer.take() {
            Some(b) => {
                let value = b >> (8 * self.bytes_left());
                self.index.update(|idx| idx.saturating_sub(4));
                if self.secondary_buffer.get().is_some() {
                    self.buffer.set(self.secondary_buffer.take());
                }
                value
            }
            None => 0,
        }
    }

    pub fn bytes_left(&self) -> usize {
        let rem = self.index.get() % 4;
        if rem == 0 {
            0
        } else {
            4 - rem
        }
    }

    pub fn is_full(&self) -> bool {
        self.index.get() >= 4 && self.buffer.get().is_some()
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.get().is_none()
    }
}

pub struct HmacKey {
    pub key: MapCell<[u8; MAX_HMAC_KEY_LEN]>,
    pub index: Cell<usize>,
    len: Cell<usize>,
}

impl HmacKey {
    pub fn new() -> Self {
        Self {
            key: MapCell::empty(),
            index: Cell::new(0),
            len: Cell::new(0),
        }
    }

    pub fn set(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
        if self.key.is_none() {
            self.key.put([0u8; MAX_HMAC_KEY_LEN]);
        }
        match self.key.map(|buf| {
            if buf.len() >= key.len() {
                buf[..key.len()].copy_from_slice(key);
                self.len.set(key.len());
                Ok(())
            } else {
                Err(ErrorCode::SIZE)
            }
        }) {
            Some(r) => r,
            None => Err(ErrorCode::FAIL),
        }
    }

    pub fn is_stored(&self) -> bool {
        self.key.is_some()
    }

    pub fn is_loaded(&self) -> bool {
        self.index.get() == self.len.get()
    }

    pub fn left_to_load(&self) -> usize {
        self.len.get().saturating_sub(self.index.get())
    }

    pub fn reset(&self) {
        if self.key.is_some() {
            self.index.take();
        }
    }

    pub fn clear(&self) {
        self.key.take();
        self.len.take();
        self.index.take();
    }
}
