use kernel::utilities::leasable_buffer::{SubSlice, SubSliceMut};
use kernel::ErrorCode;

pub trait HashAdapter<'a, H: HashAdaptee<'a>> {
    fn add_data_done(&self, result: Result<(), ErrorCode>, data: SubSlice<'static, u8>);
    fn add_mut_data_done(&self, result: Result<(), ErrorCode>, data: SubSliceMut<'static, u8>);
    fn hash_done(&self, result: Result<(), ErrorCode>, digest: &'static mut [u8]);
    fn verification_done(&self, result: Result<bool, ErrorCode>, compare: &'static mut [u8]);
}

pub trait HashAdaptee<'a> {
    fn set_adapter(&self, adapter: &'a dyn HashAdapter<'a, Self>);

    fn add_data(
        &self,
        data: SubSlice<'static, u8>,
    ) -> Result<(), (ErrorCode, SubSlice<'static, u8>)>;
    fn add_mut_data(
        &self,
        data: SubSliceMut<'static, u8>,
    ) -> Result<(), (ErrorCode, SubSliceMut<'static, u8>)>;
    fn clear_data(&self);
    fn run(&'a self, digest: &'static mut [u8]) -> Result<(), (ErrorCode, &'static mut [u8])>;
    fn verify(&'a self, compare: &'static mut [u8]) -> Result<(), (ErrorCode, &'static mut [u8])>;

    fn set_mode_md5(&self) -> Result<(), ErrorCode>;
    fn set_mode_sha1(&self) -> Result<(), ErrorCode>;
    fn set_mode_sha224(&self) -> Result<(), ErrorCode>;
    fn set_mode_sha256(&self) -> Result<(), ErrorCode>;
    fn set_mode_sha384(&self) -> Result<(), ErrorCode>;
    fn set_mode_sha512(&self) -> Result<(), ErrorCode>;
    fn set_mode_hmacmd5(&self, key: &[u8]) -> Result<(), ErrorCode>;
    fn set_mode_hmacsha1(&self, key: &[u8]) -> Result<(), ErrorCode>;
    fn set_mode_hmacsha224(&self, key: &[u8]) -> Result<(), ErrorCode>;
    fn set_mode_hmacsha256(&self, key: &[u8]) -> Result<(), ErrorCode>;
    fn set_mode_hmacsha384(&self, key: &[u8]) -> Result<(), ErrorCode>;
    fn set_mode_hmacsha512(&self, key: &[u8]) -> Result<(), ErrorCode>;
}
