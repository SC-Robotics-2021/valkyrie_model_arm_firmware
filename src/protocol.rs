use heapless::{
    self,
    consts::{U256, U32},
    Vec,
};
use postcard::{flavors, serialize_with_flavor};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum Status {
    // MCU is OK with this query and completed it successfully
    OK,
    // Runtime handling error.
    ERROR,
    // Object failed to decode, no state can be returned.
    DecodeError,
    // Not implemented (yet?).
    Unimplemented,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct Response {
    pub(crate) status: Status,
    pub(crate) data: Option<Vec<u8, U256>>,
}
