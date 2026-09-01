extern crate self as nuwax_codec;

#[derive(Debug)]
pub enum CodecError {
    LcmEncode(String),
    LcmDecode(String),
}

pub type CodecResult<T> = Result<T, CodecError>;

#[path = "../../services/nuwax-workspace/crates/nuwax-codec/src/lcm/canonical_measurement.rs"]
pub mod canonical_measurement;
#[path = "../../services/nuwax-workspace/crates/nuwax-codec/src/lcm/sensors.rs"]
pub mod sensors;

pub mod lcm {
    pub use crate::canonical_measurement;
    pub use crate::sensors;
}

include!(
    "../../services/nuwax-workspace/crates/nuwax-codec/examples/nws1_cross_language_golden.rs"
);
