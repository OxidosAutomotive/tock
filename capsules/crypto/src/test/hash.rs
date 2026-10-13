// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Test the implementation of Hash driver by performing a hashing operation
//! and checking it against the expected hash value.

use core::cell::Cell;

use capsules_core::driver_mutex::{
    DriverMutex, DriverMutexAny, DriverMutexClient, DriverMutexHandle, DriverMutexRef,
};
use capsules_core::test::capsule_test::{CapsuleTest, CapsuleTestClient};
use kernel::hil::crypto::digest::{Algorithm, Client};
use kernel::hil::crypto::{self, digest};
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::{ErrorCode, debug};

pub struct TestHash<H: crypto::digest::Digest + 'static> {
    hash_mutex: &'static DriverMutex<H>,
    hash_handle: OptionalCell<DriverMutexHandle>,
    hash: MapCell<DriverMutexRef<H>>,
    algorithm: Cell<Algorithm>,
    input_buffer: TakeCell<'static, [u8]>,
    output_buffer: TakeCell<'static, [u8]>,
    input_len: Cell<usize>,
    output_len: Cell<usize>,
    input_offset: Cell<usize>,
    output_offset: Cell<usize>,
    client: OptionalCell<&'static dyn CapsuleTestClient>,
}

impl<H: crypto::digest::Digest> TestHash<H> {
    pub fn new(
        hash_mutex: &'static DriverMutex<H>,
        algorithm: Algorithm,
        input_buffer: &'static mut [u8],
        output_buffer: &'static mut [u8],
    ) -> TestHash<H> {
        TestHash {
            hash_mutex,
            hash_handle: OptionalCell::empty(),
            hash: MapCell::empty(),
            algorithm: Cell::new(algorithm),
            input_buffer: TakeCell::new(input_buffer),
            output_buffer: TakeCell::new(output_buffer),
            input_len: Cell::new(0),
            output_len: Cell::new(0),
            input_offset: Cell::new(0),
            output_offset: Cell::new(0),
            client: OptionalCell::empty(),
        }
    }

    pub fn register(&'static self) -> Result<(), ErrorCode> {
        if self.hash_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let hash_handle = self.hash_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;
        self.hash_handle.set(hash_handle);
        Ok(())
    }

    fn read_input(&self, destination: &mut [u8]) -> Result<usize, ErrorCode> {
        self.input_buffer.map_or(Err(ErrorCode::FAIL), |source| {
            let offset = self.input_offset.get();
            if offset >= source.len() || destination.is_empty() {
                panic!("HashTest: Either destination is empty or offset got bigger than the actual size of input buffer, offset: {}, input size: {}", offset, source.len());
            }
            let read_len = destination.len().min(source.len() - offset);
            destination[..read_len].copy_from_slice(&source[offset..offset + read_len]);
            self.input_offset.set(offset + read_len);
            Ok(read_len)
        })
    }

    fn write_output(&self, source: &[u8]) -> Result<(), ErrorCode> {
        self.output_buffer
            .map_or(Err(ErrorCode::FAIL), |destination| {
                let offset = self.output_offset.get();
                let end = offset.checked_add(source.len()).ok_or(ErrorCode::SIZE)?;
                if end > destination.len() {
                    panic!("HashTest: offset got bigger than the actual size of output buffer, offset: {}, input size: {}", offset, destination.len());
                }
                for (idx, (dest, src)) in destination[offset..].iter().zip(source).enumerate() {
                    if *src != *dest {
                        panic!(
                            "HashTest: Verification failed at byte 0x{:02x}, index {}",
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
        if self.hash_handle.is_none() {
            return Err(ErrorCode::OFF);
        }

        let r = self.input_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            self.input_len.set(buf.len());
            Ok(())
        });
        if r.is_err() {
            panic!("HashTest: input buffer is missing");
        }
        let r = self.output_buffer.map_or(Err(ErrorCode::FAIL), |buf| {
            let len = buf.len();
            if len != self.algorithm.get().get_digest_len() {
                panic!("HashTest: output buffer is incomaptible with the set mode, expected: {} bytes, got: {} bytes", self.algorithm.get().get_digest_len(), len);
            }
            self.output_len.set(len);
            Ok(())
        });
        if r.is_err() {
            panic!("HashTest: output buffer is missing");
        }
        self.input_offset.set(0);
        self.output_offset.set(0);

        if let Err(error) = self.hash_handle.map_or(Err(ErrorCode::OFF), |handle| {
            self.hash_mutex.request(handle)
        }) {
            panic!(
                "HashTest: failed to request access to hash peripheral, error: {:?}",
                error
            );
        }
        Ok(())
    }

    fn finish(&self, result: Result<(), ErrorCode>) {
        self.hash.take().inspect(|hash| {
            hash.clear_data();
        });

        if let Err(e) = result {
            panic!(
                "HashTest: verification failed, there was an error from the driver side: {:?}",
                e
            );
        } else {
            debug!("HashTest: Verification was successful!");
            self.client.map(|client| {
                client.done(Ok(()));
            });
        }
    }
}

impl<H: crypto::digest::Digest> DriverMutexClient for TestHash<H> {
    fn ready(&'static self, resource: DriverMutexAny) {
        let result = match resource.downcast::<H>() {
            Ok(hash) => {
                hash.set_client(self);
                self.hash.put(hash);
                self.hash.map_or(Err(ErrorCode::FAIL), |hash| {
                    hash.hash(self.algorithm.get(), self.input_len.get())
                })
            }
            Err(_) => Err(ErrorCode::INVAL),
        };

        if let Err(error) = result {
            panic!("HashTest: operation didn't start, error: {:?}", error);
        }
    }
}

impl<H: crypto::digest::Digest> Client for TestHash<H> {
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

impl<H: digest::Digest> CapsuleTest for TestHash<H> {
    fn set_client(&self, client: &'static dyn CapsuleTestClient) {
        self.client.set(client);
    }
}
