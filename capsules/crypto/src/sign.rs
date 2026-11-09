// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

use core::cell::Cell;

use capsules_core::driver_mutex::{
    DriverMutex, DriverMutexAny, DriverMutexClient, DriverMutexHandle, DriverMutexRef,
};
use kernel::{
    ErrorCode, ProcessId,
    errorcode::into_statuscode,
    grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount},
    hil::public_key_crypto::keys::{SetKeyBySlice, SetKeyBySliceClient},
    hil::public_key_crypto::signature::{ClientSign, SignatureSign},
    processbuffer::{ReadableProcessBuffer, WriteableProcessBuffer},
    syscall::{CommandReturn, SyscallDriver},
    utilities::cells::{MapCell, OptionalCell},
};

use crate::public_key::{GetPublicKey, GetPublicKeyClient};

/// Command numbers
mod cmd {
    pub const EXISTS: usize = 0;
    pub const SIGN: usize = 1;
    pub const SET_PRIVATE_KEY: usize = 2;
    pub const GET_PUBLIC_KEY: usize = 3;
}

/// Upcalls for operations completing.
mod upcall {
    pub const DONE: usize = 0;
    pub const COUNT: u8 = 1;
}

/// Ids for read-only allow buffers
mod ro_allow {
    /// Hash to sign
    pub const INPUT: usize = 0;
    /// Private key to load into the signer
    pub const PRIVATE_KEY: usize = 1;
    /// The number of allow buffers the kernel stores for this grant
    pub const COUNT: u8 = 2;
}

/// Ids for read-write allow buffers
mod rw_allow {
    /// Generated signature
    pub const OUTPUT: usize = 0;
    /// Public key of the loaded private key
    pub const PUBLIC_KEY: usize = 1;
    /// The number of allow buffers the kernel stores for this grant
    pub const COUNT: u8 = 2;
}

#[derive(Clone, Copy)]
enum Operation {
    Sign = 0,
    SetPrivateKey = 1,
    GetPublicKey = 2,
}

// impl Operation {
//     /// The command number that started this operation, reported to
//     /// userspace as the second upcall argument.
//     fn command_num(self) -> usize {
//         match self {
//             Operation::Sign => cmd::SIGN,
//             Operation::SetPrivateKey => cmd::SET_PRIVATE_KEY,
//             Operation::GetPublicKey => cmd::GET_PUBLIC_KEY,
//         }
//     }
// }

#[derive(Clone, Copy)]
enum State {
    Idle,
    Waiting {
        processid: ProcessId,
        operation: Operation,
    },
    Active {
        processid: ProcessId,
        operation: Operation,
    },
}

/// Syscall driver for signature signing.
///
/// `HL` is the hash length in bytes.
///
/// `SL` is the signature length in bytes.
///
/// `KL` is the private key length in bytes.
///
/// `PL` is the public key length in bytes.
///
/// The underlying signer is protected by a `DriverMutex` so multiple
/// capsules can share the same signing hardware.
pub struct Signer<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    /// Underlying signer to use for the sign operations.
    signer_mutex: &'static DriverMutex<S>,
    signer_handle: OptionalCell<DriverMutexHandle>,
    signer: MapCell<DriverMutexRef<S>>,
    /// Kernel-owned buffer containing the hash passed to the HIL.
    ///
    /// This is necessary because `SignatureSign::sign()` requires
    /// `&'static mut [u8; HL]`.
    hash: MapCell<&'static mut [u8; HL]>,

    /// Kernel-owned buffer containing the signature produced by the HIL.
    ///
    /// This is necessary because `SignatureSign::sign()` requires
    /// `&'static mut [u8; SL]`.
    signature: MapCell<&'static mut [u8; SL]>,

    /// Kernel-owned buffer used to pass a private key to the HIL.
    ///
    /// It is zeroed as soon as the HIL returns it.
    private_key: MapCell<&'static mut [u8; KL]>,

    /// Kernel-owned buffer containing the public key produced by the HIL.
    public_key: MapCell<&'static mut [u8; PL]>,

    /// Virtualized capsule that supports a single operation per app.
    apps: Grant<
        (),
        UpcallCount<{ upcall::COUNT }>,
        AllowRoCount<{ ro_allow::COUNT }>,
        AllowRwCount<{ rw_allow::COUNT }>,
    >,
    state: Cell<State>,
}

impl<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize>
    Signer<S, HL, SL, KL, PL>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    pub fn new(
        signer_mutex: &'static DriverMutex<S>,
        hash: &'static mut [u8; HL],
        signature: &'static mut [u8; SL],
        private_key: &'static mut [u8; KL],
        public_key: &'static mut [u8; PL],
        apps: Grant<
            (),
            UpcallCount<{ upcall::COUNT }>,
            AllowRoCount<{ ro_allow::COUNT }>,
            AllowRwCount<{ rw_allow::COUNT }>,
        >,
    ) -> Self {
        Signer {
            signer_mutex,
            signer_handle: OptionalCell::empty(),
            signer: MapCell::empty(),
            hash: MapCell::new(hash),
            signature: MapCell::new(signature),
            private_key: MapCell::new(private_key),
            public_key: MapCell::new(public_key),
            apps,
            state: Cell::new(State::Idle),
        }
    }

    /// Register this capsule as a client of the DriverMutex.
    pub fn register(&'static self) -> Result<(), ErrorCode> {
        if self.signer_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let handle = self.signer_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;

        self.signer_handle.set(handle);

        Ok(())
    }

    /// Get the process currently using the signer.
    fn active_processid(&self) -> Result<ProcessId, ErrorCode> {
        match self.state.get() {
            State::Active { processid, .. } => Ok(processid),

            State::Idle | State::Waiting { .. } => Err(ErrorCode::RESERVE),
        }
    }

    /// Copy a userspace read-only buffer into a kernel-owned buffer.
    ///
    /// The userspace buffer must have exactly the length of `destination`.
    fn read_from_process(&self, allow_num: usize, destination: &mut [u8]) -> Result<(), ErrorCode> {
        let processid = self.active_processid()?;

        self.apps
            .enter(processid, |_, kernel_data| {
                kernel_data
                    .get_readonly_processbuffer(allow_num)
                    .and_then(|buffer| {
                        buffer.enter(|source| {
                            if source.len() != destination.len() {
                                return Err(ErrorCode::INVAL);
                            }

                            source.copy_to_slice(destination);

                            Ok(())
                        })
                    })
                    .unwrap_or(Err(ErrorCode::RESERVE))
            })
            .unwrap_or_else(|error| Err(error.into()))
    }

    /// Copy a kernel-produced buffer into a userspace read-write buffer.
    ///
    /// The userspace buffer must have exactly the length of `source`.
    fn write_to_process(&self, allow_num: usize, source: &[u8]) -> Result<(), ErrorCode> {
        let processid = self.active_processid()?;

        self.apps
            .enter(processid, |_, kernel_data| {
                kernel_data
                    .get_readwrite_processbuffer(allow_num)
                    .and_then(|buffer| {
                        buffer.mut_enter(|destination| {
                            if destination.len() != source.len() {
                                return Err(ErrorCode::INVAL);
                            }

                            destination.copy_from_slice(source);

                            Ok(())
                        })
                    })
                    .unwrap_or(Err(ErrorCode::RESERVE))
            })
            .unwrap_or_else(|error| Err(error.into()))
    }

    /// Start an operation.
    ///
    /// This only queues the request with the DriverMutex. The actual
    /// operation begins in `DriverMutexClient::ready()`.
    fn start_operation(&self, processid: ProcessId, operation: Operation) -> Result<(), ErrorCode> {
        if !matches!(self.state.get(), State::Idle) {
            return Err(ErrorCode::BUSY);
        }
        if self.signer_handle.is_none() {
            return Err(ErrorCode::OFF);
        }

        // Validate the userspace buffers before requesting the signer.
        self.apps
            .enter(processid, |_, kernel_data| -> Result<(), ErrorCode> {
                let ro_len = |allow_num| {
                    kernel_data
                        .get_readonly_processbuffer(allow_num)
                        .map(|buffer| buffer.len())
                        .map_err(ErrorCode::from)
                };
                let rw_len = |allow_num| {
                    kernel_data
                        .get_readwrite_processbuffer(allow_num)
                        .map(|buffer| buffer.len())
                        .map_err(ErrorCode::from)
                };

                let valid = match operation {
                    Operation::Sign => {
                        ro_len(ro_allow::INPUT)? == HL && rw_len(rw_allow::OUTPUT)? == SL
                    }
                    Operation::SetPrivateKey => ro_len(ro_allow::PRIVATE_KEY)? == KL,
                    Operation::GetPublicKey => rw_len(rw_allow::PUBLIC_KEY)? == PL,
                };

                if valid { Ok(()) } else { Err(ErrorCode::INVAL) }
            })??;

        self.state.set(State::Waiting {
            processid,
            operation,
        });

        let result = self.signer_handle.map_or(Err(ErrorCode::OFF), |handle| {
            self.signer_mutex.request(handle)
        });

        if let Err(error) = result {
            self.state.set(State::Idle);
            return Err(error);
        }

        Ok(())
    }

    /// Copy the userspace hash into the kernel buffer and start signing.
    fn start_sign(&'static self, signer: &S) -> Result<(), ErrorCode> {
        signer.set_sign_client(self);

        //
        // Take the kernel-owned buffers.
        //
        // These buffers will be owned by the underlying HIL
        // until signing_done() is called.
        //
        let hash = self.hash.take().ok_or(ErrorCode::FAIL)?;

        let signature = match self.signature.take() {
            Some(signature) => signature,
            None => {
                self.hash.put(hash);
                return Err(ErrorCode::FAIL);
            }
        };

        //
        // Copy the userspace hash into the kernel buffer.
        //
        // If this fails, we must put both buffers back because
        // the HIL was never given ownership of them.
        //
        if let Err(error) = self.read_from_process(ro_allow::INPUT, hash) {
            self.hash.put(hash);
            self.signature.put(signature);
            return Err(error);
        }

        signer
            .sign(hash, signature)
            .map_err(|(error, hash, signature)| {
                self.hash.put(hash);
                self.signature.put(signature);
                error
            })
    }

    /// Copy the userspace private key into the kernel buffer and load it
    /// into the signer.
    fn start_set_private_key(&'static self, signer: &S) -> Result<(), ErrorCode> {
        signer.set_client(self);

        let key = self.private_key.take().ok_or(ErrorCode::FAIL)?;

        if let Err(error) = self.read_from_process(ro_allow::PRIVATE_KEY, key) {
            key.fill(0);
            self.private_key.put(key);
            return Err(error);
        }

        signer.set_key(key).map_err(|(error, key)| {
            key.fill(0);
            self.private_key.put(key);
            error
        })
    }

    /// Ask the signer for the public key of the loaded private key.
    fn start_get_public_key(&'static self, signer: &S) -> Result<(), ErrorCode> {
        signer.set_public_key_client(self);

        let public_key = self.public_key.take().ok_or(ErrorCode::FAIL)?;

        signer
            .get_public_key(public_key)
            .map_err(|(error, public_key)| {
                self.public_key.put(public_key);
                error
            })
    }

    /// Finish the current operation and notify userspace.
    ///
    /// The upcall carries the status as the first argument and the
    /// command number of the finished operation as the second.
    fn finish(&self, result: Result<(), ErrorCode>) {
        let state = self.state.replace(State::Idle);

        // Drop our reference to the underlying signer.
        //
        // This returns the resource to the DriverMutex.
        self.signer.take();

        let (processid, operation) = match state {
            State::Waiting {
                processid,
                operation,
            }
            | State::Active {
                processid,
                operation,
            } => (processid, operation),

            State::Idle => return,
        };

        let _ = self.apps.enter(processid, |_, kernel_data| {
            let _ = kernel_data.schedule_upcall(
                upcall::DONE,
                (into_statuscode(result), operation as usize, 0),
            );
        });
    }
}

impl<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize> DriverMutexClient
    for Signer<S, HL, SL, KL, PL>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    /// Called by DriverMutex when the underlying signer becomes
    /// available to this capsule.
    fn ready(&'static self, resource: DriverMutexAny) {
        let (processid, operation) = match self.state.get() {
            State::Waiting {
                processid,
                operation,
            } => (processid, operation),

            State::Idle | State::Active { .. } => {
                return;
            }
        };

        self.state.set(State::Active {
            processid,
            operation,
        });

        let signer = match resource.downcast::<S>() {
            Ok(signer) => signer,
            Err(_) => return self.finish(Err(ErrorCode::INVAL)),
        };

        //
        // Start the asynchronous operation.
        //
        // On success, the HIL takes ownership of the kernel buffers
        // and returns them through the matching client callback.
        //
        let result = match operation {
            Operation::Sign => self.start_sign(&signer),
            Operation::SetPrivateKey => self.start_set_private_key(&signer),
            Operation::GetPublicKey => self.start_get_public_key(&signer),
        };

        // Store our reference to the underlying signer.
        self.signer.put(signer);

        if let Err(error) = result {
            self.finish(Err(error));
        }
    }
}

impl<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize> ClientSign<HL, SL>
    for Signer<S, HL, SL, KL, PL>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    /// Called when the asynchronous signing operation completes.
    fn signing_done(
        &self,
        result: Result<(), ErrorCode>,
        hash: &'static mut [u8; HL],
        signature: &'static mut [u8; SL],
    ) {
        //
        // The HIL has returned ownership of both buffers.
        //
        // Copy the generated signature to userspace, then put them
        // back into the capsule.
        //
        let result = result.and_then(|()| self.write_to_process(rw_allow::OUTPUT, signature));

        self.hash.put(hash);
        self.signature.put(signature);

        self.finish(result);
    }
}

impl<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize> SetKeyBySliceClient<KL>
    for Signer<S, HL, SL, KL, PL>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    /// Called when the private key has been loaded into the signer.
    fn set_key_done(&self, key: &'static mut [u8; KL], result: Result<(), ErrorCode>) {
        // The signer keeps its own copy, do not leave the key behind.
        key.fill(0);
        self.private_key.put(key);

        self.finish(result);
    }
}

impl<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize> GetPublicKeyClient<PL>
    for Signer<S, HL, SL, KL, PL>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    /// Called when the signer has computed the public key.
    fn get_public_key_done(
        &self,
        result: Result<(), ErrorCode>,
        public_key: &'static mut [u8; PL],
    ) {
        let result = result.and_then(|()| self.write_to_process(rw_allow::PUBLIC_KEY, public_key));

        self.public_key.put(public_key);

        self.finish(result);
    }
}

impl<S, const HL: usize, const SL: usize, const KL: usize, const PL: usize> SyscallDriver
    for Signer<S, HL, SL, KL, PL>
where
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
{
    /// Setup and run signing operations.
    ///
    /// We expect userspace to setup the buffers needed by the operation
    /// with allow calls before issuing the command:
    ///
    /// - sign: read-only `0` (hash, `HL` bytes) and read-write `0`
    ///   (signature, `SL` bytes)
    /// - set private key: read-only `1` (private key, `KL` bytes)
    /// - get public key: read-write `1` (public key, `PL` bytes)
    ///
    /// We expect userspace not to change the value while running. If userspace
    /// changes the value we have no guarantee of what is passed to the
    /// hardware.
    ///
    /// Completion of every operation is signaled with upcall `0`, carrying
    /// the status and the command number of the finished operation.
    ///
    /// ### `command_num`
    ///
    /// - `0`: driver check
    /// - `1`: sign
    /// - `2`: set private key
    /// - `3`: get public key
    fn command(
        &self,
        command_num: usize,
        _data1: usize,
        _data2: usize,
        processid: ProcessId,
    ) -> CommandReturn {
        let operation = match command_num {
            // check if present
            cmd::EXISTS => {
                return match self.allocate_grant(processid) {
                    Ok(()) => CommandReturn::success(),
                    Err(_) => CommandReturn::failure(ErrorCode::NOMEM),
                };
            }
            cmd::SIGN => Operation::Sign,
            cmd::SET_PRIVATE_KEY => Operation::SetPrivateKey,
            cmd::GET_PUBLIC_KEY => Operation::GetPublicKey,

            // default
            _ => return CommandReturn::failure(ErrorCode::NOSUPPORT),
        };

        match self.start_operation(processid, operation) {
            Ok(()) => CommandReturn::success(),
            Err(e) => CommandReturn::failure(e),
        }
    }

    fn allocate_grant(&self, processid: ProcessId) -> Result<(), kernel::process::Error> {
        self.apps.enter(processid, |_, _| {})
    }
}
