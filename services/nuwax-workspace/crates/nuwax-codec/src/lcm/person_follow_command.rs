//! Locked codec for `nuwax_msgs.PersonFollowCommand`.

use crate::{CodecError, CodecResult};

/// Generated LCM fingerprint for `nuwax_msgs.PersonFollowCommand`.
pub const PERSON_FOLLOW_COMMAND_FINGERPRINT: [u8; 8] = 0x76ad_78cf_d903_47e8_u64.to_be_bytes();

/// The controller is waiting for target or point-cloud data.
pub const WAITING_DATA: i32 = 0;
/// The controller is actively tracking the target.
pub const TRACKING: i32 = 1;
/// The controller lost the target.
pub const LOST: i32 = 2;
/// The follow session stopped normally.
pub const STOPPED: i32 = 3;
/// The follow session failed.
pub const FAILED: i32 = 4;

/// Compatibility alias for [`WAITING_DATA`].
pub const PERSON_FOLLOW_WAITING_DATA: i32 = WAITING_DATA;
/// Compatibility alias for [`TRACKING`].
pub const PERSON_FOLLOW_TRACKING: i32 = TRACKING;
/// Compatibility alias for [`LOST`].
pub const PERSON_FOLLOW_LOST: i32 = LOST;
/// Compatibility alias for [`STOPPED`].
pub const PERSON_FOLLOW_STOPPED: i32 = STOPPED;
/// Compatibility alias for [`FAILED`].
pub const PERSON_FOLLOW_FAILED: i32 = FAILED;

/// Atomic motion command produced for one tracked-person observation.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PersonFollowCommand {
    /// Stable follow session identifier.
    pub session_id: String,
    /// Target observation sequence consumed by this command.
    pub target_seq: i64,
    /// Target image timestamp seconds.
    pub target_stamp_sec: i64,
    /// Target image timestamp nanoseconds.
    pub target_stamp_nsec: i32,
    /// Monotonic command sequence within the session.
    pub command_seq: i64,
    /// Point-cloud revision consumed by this command.
    pub cloud_revision: i64,
    /// One of the `PERSON_FOLLOW_*` command state constants.
    pub state: i32,
    /// Human-readable state or failure reason.
    pub reason: String,
    /// Forward velocity in metres per second.
    pub linear_x: f64,
    /// Lateral velocity in metres per second.
    pub linear_y: f64,
    /// Yaw rate in radians per second.
    pub angular_z: f64,
}

/// Encode a complete generated `PersonFollowCommand` message.
pub fn encode_person_follow_command(message: &PersonFollowCommand) -> CodecResult<Vec<u8>> {
    let body = encode_person_follow_command_body(message)?;
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&PERSON_FOLLOW_COMMAND_FINGERPRINT);
    encoded.extend_from_slice(&body);
    Ok(encoded)
}

/// Encode the generated body without its eight-byte fingerprint.
pub fn encode_person_follow_command_body(message: &PersonFollowCommand) -> CodecResult<Vec<u8>> {
    let mut encoded = Vec::new();
    encode_string(&message.session_id, &mut encoded)?;
    encoded.extend_from_slice(&message.target_seq.to_be_bytes());
    encoded.extend_from_slice(&message.target_stamp_sec.to_be_bytes());
    encoded.extend_from_slice(&message.target_stamp_nsec.to_be_bytes());
    encoded.extend_from_slice(&message.command_seq.to_be_bytes());
    encoded.extend_from_slice(&message.cloud_revision.to_be_bytes());
    encoded.extend_from_slice(&message.state.to_be_bytes());
    encode_string(&message.reason, &mut encoded)?;
    encoded.extend_from_slice(&message.linear_x.to_be_bytes());
    encoded.extend_from_slice(&message.linear_y.to_be_bytes());
    encoded.extend_from_slice(&message.angular_z.to_be_bytes());
    Ok(encoded)
}

/// Decode a complete generated `PersonFollowCommand` message.
pub fn decode_person_follow_command(encoded: &[u8]) -> CodecResult<PersonFollowCommand> {
    let mut cursor = Cursor::new(encoded);
    let actual = cursor.read_exact(8)?;
    if actual != PERSON_FOLLOW_COMMAND_FINGERPRINT {
        return Err(CodecError::LcmDecode(
            "fingerprint mismatch for nuwax_msgs.PersonFollowCommand".into(),
        ));
    }
    let message = decode_body(&mut cursor)?;
    cursor.finish()?;
    Ok(message)
}

/// Decode a generated body after the transport has checked its fingerprint.
pub fn decode_person_follow_command_body(encoded: &[u8]) -> CodecResult<PersonFollowCommand> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_body(&mut cursor)?;
    cursor.finish()?;
    Ok(message)
}

fn decode_body(cursor: &mut Cursor<'_>) -> CodecResult<PersonFollowCommand> {
    Ok(PersonFollowCommand {
        session_id: cursor.read_string()?,
        target_seq: cursor.read_i64()?,
        target_stamp_sec: cursor.read_i64()?,
        target_stamp_nsec: cursor.read_i32()?,
        command_seq: cursor.read_i64()?,
        cloud_revision: cursor.read_i64()?,
        state: cursor.read_i32()?,
        reason: cursor.read_string()?,
        linear_x: cursor.read_f64()?,
        linear_y: cursor.read_f64()?,
        angular_z: cursor.read_f64()?,
    })
}

fn encode_string(value: &str, output: &mut Vec<u8>) -> CodecResult<()> {
    let length = value
        .len()
        .checked_add(1)
        .and_then(|length| u32::try_from(length).ok())
        .ok_or_else(|| CodecError::LcmEncode("LCM string exceeds uint32 length".into()))?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    output.push(0);
    Ok(())
}

struct Cursor<'a> {
    input: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(input: &'a [u8]) -> Self {
        Self { input, offset: 0 }
    }

    fn read_exact(&mut self, length: usize) -> CodecResult<&'a [u8]> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(CodecError::Incomplete {
                need: usize::MAX,
                got: self.input.len(),
            })?;
        if end > self.input.len() {
            return Err(CodecError::Incomplete {
                need: end,
                got: self.input.len(),
            });
        }
        let bytes = &self.input[self.offset..end];
        self.offset = end;
        Ok(bytes)
    }

    fn read_i32(&mut self) -> CodecResult<i32> {
        let bytes = self.read_exact(4)?.try_into().expect("four bytes");
        Ok(i32::from_be_bytes(bytes))
    }

    fn read_i64(&mut self) -> CodecResult<i64> {
        let bytes = self.read_exact(8)?.try_into().expect("eight bytes");
        Ok(i64::from_be_bytes(bytes))
    }

    fn read_f64(&mut self) -> CodecResult<f64> {
        let bytes = self.read_exact(8)?.try_into().expect("eight bytes");
        Ok(f64::from_be_bytes(bytes))
    }

    fn read_string(&mut self) -> CodecResult<String> {
        let bytes = self.read_exact(4)?;
        let length = usize::try_from(u32::from_be_bytes(bytes.try_into().expect("four bytes")))
            .expect("u32 fits usize");
        if length == 0 {
            return Err(CodecError::LcmDecode(
                "LCM string length must include a trailing NUL".into(),
            ));
        }
        let bytes = self.read_exact(length)?;
        if bytes.last() != Some(&0) {
            return Err(CodecError::LcmDecode(
                "LCM string is missing its trailing NUL".into(),
            ));
        }
        Ok(String::from_utf8_lossy(&bytes[..length - 1]).into_owned())
    }

    fn finish(&self) -> CodecResult<()> {
        if self.offset == self.input.len() {
            Ok(())
        } else {
            Err(CodecError::LcmDecode(format!(
                "trailing bytes after nuwax_msgs.PersonFollowCommand: {}",
                self.input.len() - self.offset
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> PersonFollowCommand {
        PersonFollowCommand {
            session_id: "follow-7".into(),
            target_seq: 17,
            target_stamp_sec: 123,
            target_stamp_nsec: 456_000_000,
            command_seq: 9,
            cloud_revision: 23,
            state: PERSON_FOLLOW_TRACKING,
            reason: "locked".into(),
            linear_x: 0.4,
            linear_y: -0.05,
            angular_z: 0.2,
        }
    }

    #[test]
    fn complete_and_body_round_trip() {
        let message = fixture();
        let complete = encode_person_follow_command(&message).unwrap();
        assert_eq!(decode_person_follow_command(&complete).unwrap(), message);

        let body = encode_person_follow_command_body(&message).unwrap();
        assert_eq!(decode_person_follow_command_body(&body).unwrap(), message);
    }

    #[test]
    fn complete_decode_rejects_wrong_fingerprint() {
        let mut encoded = encode_person_follow_command(&fixture()).unwrap();
        encoded[0] ^= 1;
        assert!(decode_person_follow_command(&encoded).is_err());
    }

    #[test]
    fn decoders_reject_truncated_or_trailing_data() {
        let mut complete = encode_person_follow_command(&fixture()).unwrap();
        complete.pop();
        assert!(decode_person_follow_command(&complete).is_err());

        let mut body = encode_person_follow_command_body(&fixture()).unwrap();
        body.push(0);
        assert!(decode_person_follow_command_body(&body).is_err());
    }
}
