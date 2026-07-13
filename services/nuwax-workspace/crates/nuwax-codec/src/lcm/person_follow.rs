//! Locked codec for `nuwax_msgs.PersonFollowTarget`.

use crate::{CodecError, CodecResult};

/// Generated LCM fingerprint for `nuwax_msgs.PersonFollowTarget`.
pub const PERSON_FOLLOW_TARGET_FINGERPRINT: [u8; 8] = 0xe627_c308_2965_e18d_u64.to_be_bytes();

/// Target is visible and the bbox belongs to the current image timestamp.
pub const PERSON_FOLLOW_TRACKED: i32 = 0;
/// Tracker processed a frame but did not find the target.
pub const PERSON_FOLLOW_LOST: i32 = 1;
/// End the current follow session without stopping the owner process.
pub const PERSON_FOLLOW_STOP: i32 = 2;

/// Typed tracked-person update consumed by the production person-follow owner.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PersonFollowTarget {
    /// Stable follow session identifier.
    pub session_id: String,
    /// Human-readable target description.
    pub query: String,
    /// Image timestamp seconds.
    pub stamp_sec: i64,
    /// Image timestamp nanoseconds.
    pub stamp_nsec: i32,
    /// Optical camera frame used for projection.
    pub camera_frame: String,
    /// Image width in pixels.
    pub image_width: i32,
    /// Image height in pixels.
    pub image_height: i32,
    /// Pinhole focal length X.
    pub fx: f64,
    /// Pinhole focal length Y.
    pub fy: f64,
    /// Pinhole principal point X.
    pub cx: f64,
    /// Pinhole principal point Y.
    pub cy: f64,
    /// Left bbox coordinate in pixels.
    pub bbox_x1: f64,
    /// Top bbox coordinate in pixels.
    pub bbox_y1: f64,
    /// Right bbox coordinate in pixels.
    pub bbox_x2: f64,
    /// Bottom bbox coordinate in pixels.
    pub bbox_y2: f64,
    /// One of [`PERSON_FOLLOW_TRACKED`], [`PERSON_FOLLOW_LOST`], or [`PERSON_FOLLOW_STOP`].
    pub tracking_state: i32,
}

/// Encode a complete generated `PersonFollowTarget` message.
pub fn encode_person_follow_target(message: &PersonFollowTarget) -> CodecResult<Vec<u8>> {
    let body = encode_person_follow_target_body(message)?;
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&PERSON_FOLLOW_TARGET_FINGERPRINT);
    encoded.extend_from_slice(&body);
    Ok(encoded)
}

/// Encode the generated body without its eight-byte fingerprint.
pub fn encode_person_follow_target_body(message: &PersonFollowTarget) -> CodecResult<Vec<u8>> {
    let mut encoded = Vec::new();
    encode_string(&message.session_id, &mut encoded)?;
    encode_string(&message.query, &mut encoded)?;
    encoded.extend_from_slice(&message.stamp_sec.to_be_bytes());
    encoded.extend_from_slice(&message.stamp_nsec.to_be_bytes());
    encode_string(&message.camera_frame, &mut encoded)?;
    encoded.extend_from_slice(&message.image_width.to_be_bytes());
    encoded.extend_from_slice(&message.image_height.to_be_bytes());
    for value in [
        message.fx,
        message.fy,
        message.cx,
        message.cy,
        message.bbox_x1,
        message.bbox_y1,
        message.bbox_x2,
        message.bbox_y2,
    ] {
        encoded.extend_from_slice(&value.to_be_bytes());
    }
    encoded.extend_from_slice(&message.tracking_state.to_be_bytes());
    Ok(encoded)
}

/// Decode a complete generated `PersonFollowTarget` message.
pub fn decode_person_follow_target(encoded: &[u8]) -> CodecResult<PersonFollowTarget> {
    let mut cursor = Cursor::new(encoded);
    let actual = cursor.read_exact(8)?;
    if actual != PERSON_FOLLOW_TARGET_FINGERPRINT {
        return Err(CodecError::LcmDecode(
            "fingerprint mismatch for nuwax_msgs.PersonFollowTarget".into(),
        ));
    }
    let message = decode_body(&mut cursor)?;
    cursor.finish()?;
    Ok(message)
}

/// Decode a generated body after the transport has checked its fingerprint.
pub fn decode_person_follow_target_body(encoded: &[u8]) -> CodecResult<PersonFollowTarget> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_body(&mut cursor)?;
    cursor.finish()?;
    Ok(message)
}

fn decode_body(cursor: &mut Cursor<'_>) -> CodecResult<PersonFollowTarget> {
    Ok(PersonFollowTarget {
        session_id: cursor.read_string()?,
        query: cursor.read_string()?,
        stamp_sec: cursor.read_i64()?,
        stamp_nsec: cursor.read_i32()?,
        camera_frame: cursor.read_string()?,
        image_width: cursor.read_i32()?,
        image_height: cursor.read_i32()?,
        fx: cursor.read_f64()?,
        fy: cursor.read_f64()?,
        cx: cursor.read_f64()?,
        cy: cursor.read_f64()?,
        bbox_x1: cursor.read_f64()?,
        bbox_y1: cursor.read_f64()?,
        bbox_x2: cursor.read_f64()?,
        bbox_y2: cursor.read_f64()?,
        tracking_state: cursor.read_i32()?,
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
                "trailing bytes after nuwax_msgs.PersonFollowTarget: {}",
                self.input.len() - self.offset
            )))
        }
    }
}
