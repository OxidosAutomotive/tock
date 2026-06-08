use kernel::hil::digest;
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;

use crate::hash::traits::HashAdaptee;
use crate::hash::{Hash, HashAdapter, HashClient};

const SHA224_DIGEST_LEN: usize = 28;

pub struct Sha224Adapter<'a> {
    hash: &'a Hash<'a>,
    client: OptionalCell<HashClient<'a, SHA224_DIGEST_LEN>>,
}

impl<'a> Sha224Adapter<'a> {
    pub fn new(hash: &'a Hash<'a>) -> Self {
        Self {
            hash,
            client: OptionalCell::empty(),
        }
    }
}

impl<'a> HashAdapter<'a, Hash<'a>> for Sha224Adapter<'a> {
    fn add_data_done(
        &self,
        result: Result<(), kernel::ErrorCode>,
        data: kernel::utilities::leasable_buffer::SubSlice<'static, u8>,
    ) {
        self.client.map(|client| {
            client.add_data_done(result, data);
        });
    }

    fn add_mut_data_done(
        &self,
        result: Result<(), kernel::ErrorCode>,
        data: kernel::utilities::leasable_buffer::SubSliceMut<'static, u8>,
    ) {
        self.client.map(|client| {
            client.add_mut_data_done(result, data);
        });
    }

    fn hash_done(&self, result: Result<(), kernel::ErrorCode>, digest: &'static mut [u8]) {
        // terrible
        self.client
            .map(|client| client.hash_done(result, digest.try_into().unwrap()));
    }

    fn verification_done(
        &self,
        result: Result<bool, kernel::ErrorCode>,
        compare: &'static mut [u8],
    ) {
        //terrible
        self.client
            .map(|client| client.verification_done(result, compare.try_into().unwrap()));
    }
}

impl<'a> digest::DigestData<'a, SHA224_DIGEST_LEN> for Sha224Adapter<'a> {
    fn set_data_client(
        &'a self,
        client: &'a dyn kernel::hil::digest::ClientData<SHA224_DIGEST_LEN>,
    ) {
        if let Some(HashClient::Single(_, hash, verify)) = self.client.get() {
            self.client
                .set(HashClient::Single(Some(client), hash, verify));
        } else {
            self.client
                .set(HashClient::Single(Some(client), None, None));
        }
    }

    fn add_data(
        &self,
        data: kernel::utilities::leasable_buffer::SubSlice<'static, u8>,
    ) -> Result<
        (),
        (
            kernel::ErrorCode,
            kernel::utilities::leasable_buffer::SubSlice<'static, u8>,
        ),
    > {
        self.hash.add_data(data)
    }

    fn add_mut_data(
        &self,
        data: kernel::utilities::leasable_buffer::SubSliceMut<'static, u8>,
    ) -> Result<
        (),
        (
            kernel::ErrorCode,
            kernel::utilities::leasable_buffer::SubSliceMut<'static, u8>,
        ),
    > {
        self.hash.add_mut_data(data)

        // self.hash
        //     .map_or(Err((ErrorCode::FAIL, data)), |hash| hash.add_mut_data(data))
    }

    fn clear_data(&self) {
        self.hash.clear_data();
    }
}

impl<'a> digest::DigestHash<'a, SHA224_DIGEST_LEN> for Sha224Adapter<'a> {
    fn set_hash_client(
        &'a self,
        client: &'a dyn kernel::hil::digest::ClientHash<SHA224_DIGEST_LEN>,
    ) {
        if let Some(HashClient::Single(data, _, verify)) = self.client.get() {
            self.client
                .set(HashClient::Single(data, Some(client), verify));
        } else {
            self.client
                .set(HashClient::Single(None, Some(client), None));
        }
    }

    fn run(
        &'a self,
        digest: &'static mut [u8; SHA224_DIGEST_LEN],
    ) -> Result<(), (kernel::ErrorCode, &'static mut [u8; SHA224_DIGEST_LEN])> {
        self.hash.run(digest).map_err(|(e, buf)| {
            //terrible
            let correct_buf: &'static mut [u8; SHA224_DIGEST_LEN] = buf.try_into().unwrap();
            (e, correct_buf)
        })

        // self.hash
        //     .map(|hash| hash.run(digest).inspect_err(|(e, buf)| {}))
    }
}

impl<'a> digest::DigestVerify<'a, SHA224_DIGEST_LEN> for Sha224Adapter<'a> {
    fn set_verify_client(
        &'a self,
        client: &'a dyn kernel::hil::digest::ClientVerify<SHA224_DIGEST_LEN>,
    ) {
        if let Some(HashClient::Single(data, hash, _)) = self.client.get() {
            self.client
                .set(HashClient::Single(data, hash, Some(client)));
        } else {
            self.client
                .set(HashClient::Single(None, None, Some(client)));
        }
    }

    fn verify(
        &'a self,
        compare: &'static mut [u8; SHA224_DIGEST_LEN],
    ) -> Result<(), (kernel::ErrorCode, &'static mut [u8; SHA224_DIGEST_LEN])> {
        self.hash.verify(compare).map_err(|(e, buf)| {
            //terrible
            let correct_buf: &'static mut [u8; SHA224_DIGEST_LEN] = buf.try_into().unwrap();
            (e, correct_buf)
        })

        // self.hash.verify(compare).map_err(|(error, buf)| {
        //     // terrible
        //     let correct_buf: &'static mut [u8; 32] = buf.try_into().unwrap();
        //     (error, correct_buf)
        // })
    }
}

impl<'a> digest::Digest<'a, SHA224_DIGEST_LEN> for Sha224Adapter<'a> {
    fn set_client(&'a self, client: &'a dyn digest::Client<SHA224_DIGEST_LEN>) {
        self.client.set(HashClient::Full(client));
    }
}

impl<'a> digest::DigestDataHash<'a, SHA224_DIGEST_LEN> for Sha224Adapter<'a> {
    fn set_client(&'a self, client: &'a dyn digest::ClientDataHash<SHA224_DIGEST_LEN>) {
        self.client.set(HashClient::DoubleHash(client));
    }
}

impl<'a> digest::DigestDataVerify<'a, SHA224_DIGEST_LEN> for Sha224Adapter<'a> {
    fn set_client(&'a self, client: &'a dyn digest::ClientDataVerify<SHA224_DIGEST_LEN>) {
        self.client.set(HashClient::DoubleVerify(client));
    }
}

impl digest::Sha256 for Sha224Adapter<'_> {
    fn set_mode_sha256(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl digest::HmacSha256 for Sha224Adapter<'_> {
    fn set_mode_hmacsha256(&self, _key: &[u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl digest::Md5 for Sha224Adapter<'_> {
    fn set_mode_md5(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}
impl digest::HmacMd5 for Sha224Adapter<'_> {
    fn set_mode_hmacmd5(&self, _key: &[u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl digest::Sha1 for Sha224Adapter<'_> {
    fn set_mode_sha1(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}
impl digest::HmacSha1 for Sha224Adapter<'_> {
    fn set_mode_hmacsha1(&self, _key: &[u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl digest::Sha224 for Sha224Adapter<'_> {
    fn set_mode_sha224(&self) -> Result<(), ErrorCode> {
        self.hash.set_mode_sha224()
    }
}
impl digest::HmacSha224 for Sha224Adapter<'_> {
    fn set_mode_hmacsha224(&self, key: &[u8]) -> Result<(), ErrorCode> {
        self.hash.set_mode_hmacsha224(key)
    }
}

impl digest::Sha384 for Sha224Adapter<'_> {
    fn set_mode_sha384(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}
impl digest::HmacSha384 for Sha224Adapter<'_> {
    fn set_mode_hmacsha384(&self, _key: &[u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl digest::Sha512 for Sha224Adapter<'_> {
    fn set_mode_sha512(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}
impl digest::HmacSha512 for Sha224Adapter<'_> {
    fn set_mode_hmacsha512(&self, _key: &[u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}
