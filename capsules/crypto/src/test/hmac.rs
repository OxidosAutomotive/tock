// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Test the implementation of HMAC driver by performing an HMAC operation
//! and checking it against the expected hash value.

use core::cell::Cell;

use capsules_core::driver_mutex::{
    DriverMutex, DriverMutexAny, DriverMutexClient, DriverMutexHandle, DriverMutexRef,
};
use capsules_core::test::capsule_test::{CapsuleTest, CapsuleTestClient};

use kernel::hil::crypto::digest::{Algorithm, Client, HmacClient};
use kernel::hil::crypto::{self, digest};
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::{ErrorCode, debug};

pub struct TestHmac<H: crypto::digest::Hmac + 'static> {
    hmac_mutex: &'static DriverMutex<H>,
    hmac_handle: OptionalCell<DriverMutexHandle>,
    hmac: MapCell<DriverMutexRef<H>>,
    algorithm: Cell<Algorithm>,
    input_buffer: TakeCell<'static, [u8]>,
    output_buffer: TakeCell<'static, [u8]>,
    key_buffer: TakeCell<'static, [u8]>,
    input_len: Cell<usize>,
    output_len: Cell<usize>,
    key_len: Cell<usize>,
    input_offset: Cell<usize>,
    output_offset: Cell<usize>,
    key_offset: Cell<usize>,
    client: OptionalCell<&'static dyn CapsuleTestClient>,
}

impl<H: crypto::digest::Hmac> TestHmac<H> {
    pub fn new(
        hmac_mutex: &'static DriverMutex<H>,
        algorithm: Algorithm,
        input_buffer: &'static mut [u8],
        output_buffer: &'static mut [u8],
        key_buffer: &'static mut [u8],
    ) -> TestHmac<H> {
        TestHmac {
            hmac_mutex,
            hmac_handle: OptionalCell::empty(),
            hmac: MapCell::empty(),
            algorithm: Cell::new(algorithm),
            input_buffer: TakeCell::new(input_buffer),
            output_buffer: TakeCell::new(output_buffer),
            key_buffer: TakeCell::new(key_buffer),
            input_len: Cell::new(0),
            output_len: Cell::new(0),
            key_len: Cell::new(0),
            input_offset: Cell::new(0),
            output_offset: Cell::new(0),
            key_offset: Cell::new(0),
            client: OptionalCell::empty(),
        }
    }

    pub fn register(&'static self) -> Result<(), ErrorCode> {
        if self.hmac_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let hmac_handle = self.hmac_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;
        self.hmac_handle.set(hmac_handle);
        Ok(())
    }

    fn read_input(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        self.input_buffer.map_or(Err(ErrorCode::FAIL), |source| {
            let offset = self.input_offset.get();
            if offset >= source.len() || destination.is_empty() {
                panic!("HmacTest: Either destination is empty or offset got bigger than the actual size of input buffer, offset: {}, input size: {}", offset, source.len());
            }
            let read_len = destination.len().min(source.len() - offset);
            destination[..read_len].copy_from_slice(&source[offset..offset + read_len]);
            self.input_offset.set(offset + read_len);
            Ok(read_len)
        })
    }

    fn read_key(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        self.key_buffer.map_or(Err(ErrorCode::FAIL), |source| {
            let offset = self.key_offset.get();
            if offset >= source.len() || destination.is_empty() {
                panic!("HmacTest: Either destination is empty or offset got bigger than the actual size of key buffer, offset: {}, input size: {}", offset, source.len());
            }
            let read_len = destination.len().min(source.len() - offset);
            let updated_offset = offset + read_len;
            destination[..read_len].copy_from_slice(&source[offset..updated_offset]);
            if updated_offset == self.key_len.get() {
                self.key_offset.take();
            } else {
                self.key_offset.set(updated_offset);
            }
            Ok(read_len)
        })
    }

    fn write_output(&self, source: &[u8]) -> Result<(), ErrorCode> {
        self.output_buffer
            .map_or(Err(ErrorCode::FAIL), |destination| {
                let offset = self.output_offset.get();
                let end = offset.checked_add(source.len()).ok_or(ErrorCode::SIZE)?;
                if end > destination.len() {
                    panic!("HmacTest: offset got bigger than the actual size of output buffer, offset: {}, input size: {}", offset, destination.len());
                }
                for (idx, (dest, src)) in destination[offset..].iter().zip(source).enumerate() {
                    if *src != *dest {
                        panic!(
                            "HmacTest: Verification failed at byte 0x{:02x}, index {}",
                            dest,
                            offset + idx
                        );
                    }
                }
                self.output_offset.set(end);

                Ok(())
            })
    }

    pub fn run(&self) -> Result<(), ErrorCode> {
        if self.hmac_handle.is_none() {
            return Err(ErrorCode::OFF);
        }

        let r = self.input_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            self.input_len.set(buf.len());
            Ok(())
        });
        if r.is_err() {
            panic!("HmacTest: input buffer is missing");
        }
        let r = self.output_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            let len = buf.len();
            if len != self.algorithm.get().get_digest_len() {
                panic!("HmacTest: output buffer is incomaptible with the set mode, expected: {} bytes, got: {} bytes", self.algorithm.get().get_digest_len(), len);
            }
            self.output_len.set(len);
            Ok(())
        });
        if r.is_err() {
            panic!("HmacTest: output buffer is missing");
        }
        let r = self.key_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            self.key_len.set(buf.len());
            Ok(())
        });
        if r.is_err() {
            panic!("HmacTest: key buffer is missing");
        }
        self.input_offset.set(0);
        self.output_offset.set(0);
        self.key_offset.set(0);

        if let Err(error) = self.hmac_handle.map_or(Err(ErrorCode::OFF), |handle| {
            self.hmac_mutex.request(handle)
        }) {
            panic!(
                "HmacTest: failed to request access to hmac peripheral, error: {:?}",
                error
            );
        }
        Ok(())
    }

    fn finish(&self, result: Result<(), ErrorCode>) {
        self.hmac.take().inspect(|hmac| {
            hmac.clear_data();
        });

        if let Err(e) = result {
            panic!(
                "HmacTest: verification failed, there was an error from the driver side: {:?}",
                e
            );
        } else {
            debug!("HmacTest: Verification was successful!");
            self.client.map(|client| {
                client.done(Ok(()));
            });
        }
    }
}

impl<H: crypto::digest::Hmac> DriverMutexClient for TestHmac<H> {
    fn ready(&'static self, resource: DriverMutexAny) {
        let result = match resource.downcast::<H>() {
            Ok(hmac) => {
                hmac.set_hmac_client(self);
                self.hmac.put(hmac);
                self.hmac.map_or(Err(ErrorCode::FAIL), |hmac| {
                    hmac.authenticate(
                        self.algorithm.get(),
                        self.input_len.get(),
                        self.key_len.get(),
                    )
                })
            }
            Err(_) => Err(ErrorCode::INVAL),
        };

        if let Err(error) = result {
            panic!("HmacTest: operation didn't start, error: {:?}", error);
        }
    }
}

impl<H: crypto::digest::Hmac> Client for TestHmac<H> {
    fn read_input(&self, input: &mut [u8]) -> Result<usize, ErrorCode> {
        self.read_input(input)
    }

    fn write_output(&self, output: &[u8]) -> Result<(), ErrorCode> {
        self.write_output(output)
    }

    fn hash_done(&self, result: Result<(), ErrorCode>) {
        self.finish(result)
    }
}

impl<H: digest::Hmac> HmacClient for TestHmac<H> {
    fn read_key(&self, key: &mut [u8]) -> Result<usize, ErrorCode> {
        self.read_key(key)
    }
}

impl<H: digest::Hmac> CapsuleTest for TestHmac<H> {
    fn set_client(&self, client: &'static dyn CapsuleTestClient) {
        self.client.set(client);
    }
}
