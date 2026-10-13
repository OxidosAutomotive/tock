// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Test the implementation of HMAC driver by performing an HKDF operation
//! and checking it against the expected value.
//!
//! # Algorithm
//!
//! HKDF is specified in [IETF RFC 5869], section 2.
//!
//! ## Step 1: Extract (section 2.2)
//!
//! HKDF-Extract(salt, IKM) -> PRK
//!    Options:
//!       - Hash = a hash function; HashLen denotes the length of the hash function output in octets
//!
//!    Inputs:
//!       - salt = optional salt value (a non-secret random value); if not provided, it is set to a string of HashLen zeros.
//!       - IKM = input keying material
//!    Output:
//!       - PRK = a pseudorandom key (of HashLen octets)
//!
//!    The output PRK is calculated as follows:
//!    PRK = HMAC-Hash(salt, IKM)
//!
//! ## Step 2: Expand (section 2.3)
//!
//! HKDF-Expand(PRK, info, L) -> OKM
//!
//!    Options:
//!       - Hash = a hash function; HashLen denotes the length of the hash function output in octets
//!
//!    Inputs:
//!       - PRK = a pseudorandom key of at least HashLen octets (usually, the output from the extract step)
//!       - info = optional context and application specific information (can be a zero-length string)
//!       - L = length of output keying material in octets (<= 255*HashLen)
//!
//!    Output:
//!      - OKM = output keying material (of L octets)
//!
//!      The output OKM is calculated as follows:
//!
//!      N = ceil(L/HashLen)
//!      T = T(1) | T(2) | T(3) | ... | T(N)
//!      OKM = first L octets of T
//!
//!      where:
//!          - T(0) = empty string (zero length)
//!          - T(1) = HMAC-Hash(PRK, T(0) | info | 0x01)
//!          - T(2) = HMAC-Hash(PRK, T(1) | info | 0x02)
//!          - T(3) = HMAC-Hash(PRK, T(2) | info | 0x03)
//!           ...
//!
//!          (where the constant concatenated to the end of each T(n) is a
//!           single octet.)
//!
//!
//! [IETF RFC 5869]: https://www.rfc-editor.org/info/rfc5869/

use core::cell::Cell;

use capsules_core::driver_mutex::{
    DriverMutex, DriverMutexAny, DriverMutexClient, DriverMutexHandle, DriverMutexRef,
};
use capsules_core::test::capsule_test::{CapsuleTest, CapsuleTestClient};
use kernel::hil::crypto;
use kernel::hil::crypto::digest::{Algorithm, Client, HmacClient};
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::{ErrorCode, debug};

#[derive(Clone, Copy)]
enum SaltStatus {
    NotPresent,
    Present,
}

#[derive(Clone, Copy)]
enum Stage {
    Extract,
    Expand {
        iteration: usize,
        substage: Substage,
    },
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Substage {
    T,
    Info,
    Index,
}

#[derive(Clone, Copy)]
enum KeyState {
    InnerKey,
    OuterKey,
}

impl KeyState {
    pub fn update(self) -> Option<Self> {
        match self {
            KeyState::InnerKey => Some(KeyState::OuterKey),
            KeyState::OuterKey => None,
        }
    }
}

pub struct TestHkdf<H: crypto::digest::Hmac + 'static> {
    hmac_mutex: &'static DriverMutex<H>,
    hmac_handle: OptionalCell<DriverMutexHandle>,
    hmac: MapCell<DriverMutexRef<H>>,
    algorithm: Cell<Algorithm>,
    stage: Cell<Stage>,
    key_state: OptionalCell<KeyState>,
    salt_status: Cell<SaltStatus>,
    num_of_iterations: Cell<usize>,
    ikm_buffer: TakeCell<'static, [u8]>,
    salt_buffer: TakeCell<'static, [u8]>,
    prk_buffer: TakeCell<'static, [u8]>,
    info_buffer: TakeCell<'static, [u8]>,
    okm_buffer: TakeCell<'static, [u8]>,
    correct_buffer: TakeCell<'static, [u8]>,
    ikm_len: Cell<usize>,
    salt_len: Cell<usize>,
    prk_len: Cell<usize>,
    info_len: Cell<usize>,
    okm_len: Cell<usize>,
    ikm_offset: Cell<usize>,
    salt_offset: Cell<usize>,
    prk_offset: Cell<usize>,
    info_offset: Cell<usize>,
    okm_offset: Cell<usize>,
    client: OptionalCell<&'static dyn CapsuleTestClient>,
}

impl<H: crypto::digest::Hmac> TestHkdf<H> {
    pub fn new(
        hmac_mutex: &'static DriverMutex<H>,
        algorithm: Algorithm,
        ikm_buffer: &'static mut [u8],
        salt_buffer: Option<&'static mut [u8]>,
        prk_buffer: &'static mut [u8],
        info_buffer: Option<&'static mut [u8]>,
        okm_buffer: &'static mut [u8],
        correct_buffer: &'static mut [u8],
    ) -> TestHkdf<H> {
        let (salt_buffer, salt_status) = match salt_buffer {
            Some(buffer) => (TakeCell::new(buffer), Cell::new(SaltStatus::Present)),
            None => (TakeCell::empty(), Cell::new(SaltStatus::NotPresent)),
        };
        let info_buffer = match info_buffer {
            Some(buffer) => TakeCell::new(buffer),
            None => TakeCell::empty(),
        };
        TestHkdf {
            hmac_mutex,
            hmac_handle: OptionalCell::empty(),
            hmac: MapCell::empty(),
            algorithm: Cell::new(algorithm),
            stage: Cell::new(Stage::Extract),
            key_state: OptionalCell::empty(),
            num_of_iterations: Cell::new(0),
            ikm_buffer: TakeCell::new(ikm_buffer),
            salt_buffer,
            salt_status,
            prk_buffer: TakeCell::new(prk_buffer),
            info_buffer,
            okm_buffer: TakeCell::new(okm_buffer),
            correct_buffer: TakeCell::new(correct_buffer),
            ikm_len: Cell::new(0),
            salt_len: Cell::new(0),
            prk_len: Cell::new(0),
            info_len: Cell::new(0),
            okm_len: Cell::new(0),
            ikm_offset: Cell::new(0),
            salt_offset: Cell::new(0),
            prk_offset: Cell::new(0),
            info_offset: Cell::new(0),
            okm_offset: Cell::new(0),
            client: OptionalCell::empty(),
        }
    }

    fn read_at(source: &[u8], offset: usize, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        if offset >= source.len() || destination.is_empty() {
            panic!(
                "TestHKDF: Either destination is empty or offset got bigger than the actual size of source buffer, offset: {}, source size: {}",
                offset,
                source.len()
            );
        }
        let read_len = destination.len().min(source.len() - offset);
        destination[..read_len].copy_from_slice(&source[offset..offset + read_len]);
        Ok(read_len)
    }

    fn write_at(source: &[u8], offset: usize, destination: &mut [u8]) -> Result<(), ErrorCode> {
        let end = offset.checked_add(source.len()).ok_or(ErrorCode::SIZE)?;
        if end > destination.len() {
            panic!(
                "TestHKDF: Size of source buffer is larger than destination can accept, offset: {}, source size: {}",
                offset,
                source.len()
            );
        }
        destination[offset..end].copy_from_slice(source);
        Ok(())
    }

    pub fn register(&'static self) -> Result<(), ErrorCode> {
        if self.hmac_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let hmac_handle = self.hmac_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;
        self.hmac_handle.set(hmac_handle);
        Ok(())
    }

    fn reset_expand_offsets(&self) {
        self.info_offset.take();
        self.okm_offset
            .update(|offset| offset.saturating_sub(self.algorithm.get().get_digest_len()));
        self.prk_offset.take();
        self.key_state.set(KeyState::InnerKey);
    }

    fn read_ikm(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        let offset = self.ikm_offset.get();
        let read = self.ikm_buffer.map_or(Err(ErrorCode::FAIL), |source| {
            TestHkdf::<H>::read_at(source, offset, destination)
        })?;
        self.ikm_offset.set(offset + read);
        Ok(read)
    }

    fn read_salt(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        if self.key_state.is_some() {
            let offset = self.salt_offset.get();
            let read = match self.salt_status.get() {
                SaltStatus::NotPresent => {
                    let bytes_to_load = (self.salt_len.get() - offset).min(destination.len());
                    destination[..bytes_to_load]
                        .iter_mut()
                        .for_each(|byte| *byte = 0u8);
                    bytes_to_load
                }
                SaltStatus::Present => self.salt_buffer.map_or(Err(ErrorCode::FAIL), |source| {
                    TestHkdf::<H>::read_at(source, offset, destination)
                })?,
            };
            let updated_offset = offset + read;
            if updated_offset == self.salt_len.get() {
                self.salt_offset.take();
            } else {
                self.salt_offset.set(updated_offset);
            }
            self.key_state.map_or(None, |key_state| key_state.update());
            Ok(read)
        } else {
            Err(ErrorCode::ALREADY)
        }
    }

    fn read_info(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        let offset = self.info_offset.get();
        let read = self.info_buffer.map_or(Err(ErrorCode::FAIL), |source| {
            TestHkdf::<H>::read_at(source, offset, destination)
        })?;
        self.info_offset.set(offset + read);
        Ok(read)
    }

    fn read_index(&self, index: u8, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        if destination.is_empty() {
            return Err(ErrorCode::SIZE);
        }
        destination[0] = index;
        Ok(1)
    }

    fn read_okm(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        let offset = self.okm_offset.get();
        let read = self.okm_buffer.map_or(Err(ErrorCode::FAIL), |source| {
            TestHkdf::<H>::read_at(source, offset, destination)
        })?;
        self.okm_offset.set(offset + read);
        Ok(read)
    }

    fn read_prk(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        if self.key_state.is_some() {
            let offset = self.prk_offset.get();
            let read = self.prk_buffer.map_or(Err(ErrorCode::FAIL), |source| {
                TestHkdf::<H>::read_at(source, offset, destination)
            })?;
            let updated_offset = offset + read;
            if updated_offset == self.prk_len.get() {
                self.prk_offset.take();
            } else {
                self.prk_offset.set(updated_offset);
            }
            self.key_state.map_or(None, |key_state| key_state.update());
            Ok(read)
        } else {
            Err(ErrorCode::ALREADY)
        }
    }

    fn write_okm(&self, source: &[u8]) -> Result<(), ErrorCode> {
        let offset = self.okm_offset.get();
        if offset < self.okm_len.get() {
            self.okm_buffer
                .map_or(Err(ErrorCode::FAIL), |destination| {
                    TestHkdf::<H>::write_at(
                        &source[..source.len().min(self.okm_len.get() - offset)],
                        offset,
                        destination,
                    )
                })?;
            self.okm_offset
                .set(offset + source.len().min(self.okm_len.get() - offset));
        }
        Ok(())
    }

    fn write_prk(&self, source: &[u8]) -> Result<(), ErrorCode> {
        let offset = self.prk_offset.get();
        self.prk_buffer
            .map_or(Err(ErrorCode::FAIL), |destination| {
                TestHkdf::<H>::write_at(source, offset, destination)
            })?;

        self.prk_offset.set(offset + source.len());
        Ok(())
    }

    pub fn run(&self) -> Result<(), ErrorCode> {
        if self.hmac_handle.is_none() {
            return Err(ErrorCode::OFF);
        }

        self.salt_buffer.map(|buf| {
            let len = buf.len();
            if len > self.algorithm.get().get_block_size() {
                panic!("TestHKDF: Salt length is bigger than the HMAC key can be");
            }
            self.salt_len.set(buf.len());
        });

        if self.salt_len.get() == 0 {
            self.salt_len.set(self.algorithm.get().get_digest_len());
        }

        let r = self.ikm_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            self.ikm_len.set(buf.len());
            Ok(())
        });
        if r.is_err() {
            panic!("TestHKDF: Initial key material is empty");
        }

        let r = self.prk_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            let len = buf.len();
            if len != self.algorithm.get().get_digest_len() {
                panic!("TestHKDF: Pseudorandom key is bigger than it cab be");
            }
            self.prk_len.set(len);
            Ok(())
        });

        if r.is_err() {
            panic!("TestHKDF: Pseudorandom key buffer is empty");
        }
        self.info_buffer.map(|buf| {
            self.info_len.set(buf.len());
        });

        let r = self.okm_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            self.okm_len.set(buf.len());
            Ok(())
        });
        if r.is_err() {
            panic!("TestHKDF: Output key is empty");
        }

        self.salt_offset.set(0);
        self.ikm_offset.set(0);
        self.prk_offset.set(0);
        self.info_offset.set(0);
        self.okm_offset.set(0);
        self.num_of_iterations.set(
            self.okm_len
                .get()
                .div_ceil(self.algorithm.get().get_digest_len()),
        );
        self.key_state.set(KeyState::InnerKey);

        if let Err(error) = self.hmac_handle.map_or(Err(ErrorCode::OFF), |handle| {
            self.hmac_mutex.request(handle)
        }) {
            panic!(
                "TestHKDF: failed to request access to hmac peripheral, error: {:?}",
                error
            );
        }
        Ok(())
    }

    fn finish(&self, result: Result<(), ErrorCode>) {
        self.hmac.take().inspect(|hmac| {
            hmac.clear_data();
        });

        if let Err(error) = result {
            panic!("TestHKDF: verification failed, driver returned {:?}", error);
        }
        if let (Some(okm_buffer), Some(correct_buffer)) =
            (self.okm_buffer.take(), self.correct_buffer.take())
        {
            okm_buffer
                .iter()
                .zip(correct_buffer)
                .map(|(okm, correct)| *okm == *correct)
                .enumerate()
                .find(|(_, result)| !*result)
                .map(|(idx, _)| panic!("TestHKDF: Verification failed at index {}", idx));

            debug!("TestHKDF: Verification was successful!");
            self.client.map(|client| {
                client.done(Ok(()));
            });
        } else {
            panic!("TestHKDF: Veification failed, output and correct buffers are missing")
        }
    }
}

impl<H: crypto::digest::Hmac> DriverMutexClient for TestHkdf<H> {
    fn ready(&'static self, resource: DriverMutexAny) {
        self.stage.set(Stage::Extract);
        let result = match resource.downcast::<H>() {
            Ok(hmac) => {
                hmac.set_hmac_client(self);
                self.hmac.put(hmac);
                self.hmac.map_or(Err(ErrorCode::FAIL), |hmac| {
                    hmac.authenticate(
                        self.algorithm.get(),
                        self.ikm_len.get(),
                        self.salt_len.get(),
                    )
                })
            }
            Err(_) => Err(ErrorCode::INVAL),
        };

        if let Err(error) = result {
            self.finish(Err(error));
        }
    }
}

impl<H: crypto::digest::Hmac> Client for TestHkdf<H> {
    fn read_input(&self, input: &mut [u8]) -> Result<usize, ErrorCode> {
        match self.stage.get() {
            Stage::Extract => self.read_ikm(input),
            Stage::Expand {
                iteration,
                substage,
            } => {
                let mut offset = 0;
                let mut current_substage = substage;
                loop {
                    match current_substage {
                        Substage::T => {
                            let ceil = self.algorithm.get().get_digest_len()
                                - (self.okm_offset.get() % self.algorithm.get().get_digest_len());
                            let read = self.read_okm(&mut input[..ceil])?;
                            offset += read;
                            if self.okm_offset.get() % self.algorithm.get().get_digest_len() == 0 {
                                current_substage = if self.info_len.get() == 0 {
                                    Substage::Index
                                } else {
                                    Substage::Info
                                };
                            }
                            if offset == input.len() {
                                break;
                            }
                        }
                        Substage::Info => {
                            let read = self.read_info(&mut input[offset..])?;
                            offset += read;
                            if self.info_offset.get() == self.info_len.get() {
                                current_substage = Substage::Index;
                            }

                            if offset == input.len() {
                                break;
                            }
                        }
                        Substage::Index => {
                            self.read_index(iteration as u8, &mut input[offset..])?;
                            offset += 1;
                            break;
                        }
                    }
                }
                if substage != current_substage {
                    self.stage.set(Stage::Expand {
                        iteration,
                        substage: current_substage,
                    });
                }
                Ok(offset)
            }
        }
    }

    fn write_output(&self, output: &[u8]) -> Result<(), ErrorCode> {
        match self.stage.get() {
            Stage::Extract => self.write_prk(output),
            Stage::Expand { .. } => self.write_okm(output),
        }
    }

    fn hash_done(&self, result: Result<(), ErrorCode>) {
        if result.is_err() {
            self.finish(result);
        } else {
            match self.stage.get() {
                Stage::Extract => {
                    self.prk_offset.take();
                    self.stage.set(Stage::Expand {
                        iteration: 1,
                        substage: if self.info_len.get() == 0 {
                            Substage::Index
                        } else {
                            Substage::Info
                        },
                    });
                    let result = self.hmac.map_or(Err(ErrorCode::FAIL), |hmac| {
                        hmac.authenticate(
                            self.algorithm.get(),
                            self.info_len.get() + 1,
                            self.prk_len.get(),
                        )
                    });
                    if result.is_err() {
                        self.finish(result);
                    }
                }
                Stage::Expand { iteration, .. } if iteration == self.num_of_iterations.get() => {
                    self.finish(result)
                }
                Stage::Expand { iteration, .. } => {
                    self.reset_expand_offsets();
                    self.stage.set(Stage::Expand {
                        iteration: iteration + 1,
                        substage: Substage::T,
                    });
                    let result = self.hmac.map_or(Err(ErrorCode::FAIL), |hmac| {
                        hmac.authenticate(
                            self.algorithm.get(),
                            self.algorithm.get().get_digest_len() + self.info_len.get() + 1,
                            self.prk_len.get(),
                        )
                    });
                    if result.is_err() {
                        self.finish(result);
                    }
                }
            }
        }
    }
}

impl<H: crypto::digest::Hmac> HmacClient for TestHkdf<H> {
    fn read_key(&self, key: &mut [u8]) -> Result<usize, ErrorCode> {
        match self.stage.get() {
            Stage::Extract => self.read_salt(key),
            Stage::Expand { .. } => self.read_prk(key),
        }
    }
}

impl<H: crypto::digest::Hmac> CapsuleTest for TestHkdf<H> {
    fn set_client(&self, client: &'static dyn CapsuleTestClient) {
        self.client.set(client);
    }
}
