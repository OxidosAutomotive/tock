// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Test the software implementation of SHA256 by performing a hash
//! and checking it against the expected hash value. It uses
//! DigestData::add_date and DigestVerify::verify through the
//! Digest trait.

use core::cell::Cell;
use core::cmp;

use capsules_core::test::capsule_test::{CapsuleTest, CapsuleTestClient};
use kernel::debug;
use kernel::hil::digest;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSlice;
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

pub struct TestSha256<'a, H: digest::Digest<'a, 32>> {
    sha: &'a H,
    data: TakeCell<'static, [u8]>,     // The data to hash
    hash: TakeCell<'static, [u8; 32]>, // The supplied hash
    position: Cell<usize>,             // Keep track of position in data
    correct: Cell<bool>,               // Whether supplied hash is correct
    client: OptionalCell<&'static dyn CapsuleTestClient>,
}

// We add data in chunks of 12 bytes to ensure that the underlying
// buffering mechanism works correctly (it can handle filling blocks
// as well as zeroing out incomplete blocks).
const CHUNK_SIZE: usize = 16;

impl<'a, H: digest::Digest<'a, 32> + digest::Sha256 + digest::Bit8Data> TestSha256<'a, H> {
    pub fn new(
        sha: &'a H,
        data: &'static mut [u8],
        hash: &'static mut [u8; 32],
        correct: bool,
    ) -> Self {
        TestSha256 {
            sha,
            data: TakeCell::new(data),
            hash: TakeCell::new(hash),
            position: Cell::new(0),
            correct: Cell::new(correct),
            client: OptionalCell::empty(),
        }
    }

    pub fn run(&'static self) {
        // We need to choose the algo and data format
        // TODO(frihetselsker): Handle errors in the capsules mroe gracefully
        let _ = self.sha.set_data_type_8_bit();
        let _ = self.sha.set_mode_sha256();
        debug!("Test_SHA: Set the mode");
        self.sha.set_client(self);
        debug!("Test_SHA: Set the client");
        let data = self.data.take().unwrap();
        let chunk_size = cmp::min(CHUNK_SIZE, data.len());
        self.position.set(chunk_size);
        let mut buffer = SubSliceMut::new(data);
        buffer.slice(0..chunk_size);
        let r = self.sha.add_mut_data(buffer);
        debug!("Test_SHA: Sent data to the buffer");
        if r.is_err() {
            panic!("Sha256Test: failed to add data: {:?}", r);
        }
    }
}

impl<'a, H: digest::Digest<'a, 32> + digest::Sha256> digest::ClientData<32> for TestSha256<'a, H> {
    fn add_data_done(&self, _result: Result<(), ErrorCode>, _data: SubSlice<'static, u8>) {
        unimplemented!()
    }

    fn add_mut_data_done(&self, result: Result<(), ErrorCode>, mut data: SubSliceMut<'static, u8>) {
        //debug!("Test_SHA: callback called");
        if data.len() != 0 {
            let r = self.sha.add_mut_data(data);
            if r.is_err() {
                panic!("Sha256Test: failed to add data: {:?}", r);
            }
        } else {
            data.reset();
            //debug!("Test_SHA: new data should be sent if we have more");
            if self.position.get() < data.len() {
                let new_position = cmp::min(data.len(), self.position.get() + CHUNK_SIZE);
                data.slice(self.position.get()..new_position);
                debug!(
                    "Sha256Test: Setting slice to {}..{}",
                    self.position.get(),
                    new_position
                );
                let r = self.sha.add_mut_data(data);
                if r.is_err() {
                    panic!("Sha256Test: failed to add data: {:?}", r);
                }
                self.position.set(new_position);
            } else {
                //debug!("Test_SHA: we do not have more data");
                data.reset();
                self.data.put(Some(data.take()));
                match result {
                    Ok(()) => {
                        //debug!("Test_SHA: start verification");
                        let v = self.sha.verify(self.hash.take().unwrap());
                        if v.is_err() {
                            panic!("Sha256Test: failed to verify: {:?}", v);
                        }
                    }
                    Err(e) => {
                        panic!("Sha256Test: adding data failed: {:?}", e);
                    }
                }
            }
        }
    }
}

impl<'a, H: digest::Digest<'a, 32> + digest::Sha256> digest::ClientVerify<32>
    for TestSha256<'a, H>
{
    fn verification_done(&self, result: Result<bool, ErrorCode>, compare: &'static mut [u8; 32]) {
        self.hash.put(Some(compare));
        debug!("Sha256Test: Verification result: {:?}", result);
        match result {
            Ok(success) => {
                if success != self.correct.get() {
                    //panic!(
                    debug!(
                        "Sha256Test: Verification should have been {}, was {}",
                        self.correct.get(),
                        success
                    );
                } else {
                    self.client.map(|client| {
                        client.done(Ok(()));
                    });
                }
            }
            Err(e) => {
                panic!("Sha256Test: Error in verification: {:?}", e);
            }
        }
    }
}

impl<'a, H: digest::Digest<'a, 32> + digest::Sha256> digest::ClientHash<32> for TestSha256<'a, H> {
    fn hash_done(&self, _result: Result<(), ErrorCode>, _digest: &'static mut [u8; 32]) {
        debug!("Test_SHA: hash_done is not used for now");
    }
}

impl<'a, H: digest::Digest<'a, 32> + digest::Sha256> CapsuleTest for TestSha256<'a, H> {
    fn set_client(&self, client: &'static dyn CapsuleTestClient) {
        self.client.set(client);
    }
}
