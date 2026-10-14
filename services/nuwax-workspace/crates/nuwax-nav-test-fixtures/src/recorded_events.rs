//! Typed recorded-event fixtures for navigation dataflow validation.

use std::io::BufRead;
use std::path::Path;

use serde::{Deserialize, Serialize};

const MAX_RECORDED_EVENTS: usize = 100_000;
const MAX_JSON_LINE_BYTES: usize = 4 * 1024 * 1024;
const MAX_FIXTURE_BYTES: usize = 64 * 1024 * 1024;
const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-6;
const GUARDED_ROUTE_RECORDING_JSONL: &str = r#"{"sequence":0,"timestamp_ns":1000000000,"channel":"/odom","payload":{"type":"odometry","data":{"position":[0.0,0.0,0.0],"orientation":[0.0,0.0,0.0,1.0],"frame_id":"map","child_frame_id":"base_link"}}}
{"sequence":1,"timestamp_ns":1010000000,"channel":"/map/occupancy","payload":{"type":"point_cloud","data":{"frame_id":"map","points":[[0.0,0.0,0.0],[1.0,0.0,0.0],[2.0,0.0,0.0]]}}}
{"sequence":2,"timestamp_ns":1020000000,"channel":"/nav/goal","payload":{"type":"goal","data":{"session_id":"session-7","goal_generation":7,"position":[2.0,0.0,0.0]}}}
{"sequence":3,"timestamp_ns":1030000000,"channel":"/nav/path","payload":{"type":"path","data":{"session_id":"session-7","goal_generation":7,"points":[[0.0,0.0,0.0],[1.0,0.0,0.0],[2.0,0.0,0.0]]}}}
{"sequence":4,"timestamp_ns":1040000000,"channel":"/cmd_vel","payload":{"type":"guard_command","data":{"session_id":"session-7","linear":[0.4,0.0],"angular_z":0.0}}}
{"sequence":5,"timestamp_ns":1050000000,"channel":"/nav/state","payload":{"type":"navigation_state","data":{"session_id":"session-7","state":"moving"}}}
{"sequence":6,"timestamp_ns":1060000000,"channel":"/nav/cancel","payload":{"type":"cancel","data":{"session_id":"session-7","goal_generation":7,"cancel_generation":1}}}
{"sequence":7,"timestamp_ns":1070000000,"channel":"/nav/state","payload":{"type":"navigation_state","data":{"session_id":"session-7","state":"cancelled"}}}
"#;

/// A point and orientation sample from recorded odometry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedOdometry {
    /// Position `(x, y, z)`.
    pub position: [f64; 3],
    /// Quaternion `(x, y, z, w)`.
    pub orientation: [f64; 4],
    /// Parent frame.
    pub frame_id: String,
    /// Child frame.
    pub child_frame_id: String,
}

/// A structured point-cloud sample.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedPointCloud {
    /// Coordinate frame.
    pub frame_id: String,
    /// XYZ points in source order.
    pub points: Vec<[f32; 3]>,
}

/// A versioned goal lease sample.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedGoal {
    /// Public navigation session.
    pub session_id: String,
    /// Monotonic goal generation.
    pub goal_generation: i64,
    /// Goal position `(x, y, z)`.
    pub position: [f64; 3],
}

/// A path correlated to a goal lease.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedPath {
    /// Public navigation session.
    pub session_id: String,
    /// Goal generation copied from the active lease.
    pub goal_generation: i64,
    /// World-frame waypoints.
    pub points: Vec<[f64; 3]>,
}

/// A cancellation correlated to a goal lease.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedCancel {
    /// Public navigation session.
    pub session_id: String,
    /// Goal generation being cancelled.
    pub goal_generation: i64,
    /// Monotonic cancellation generation.
    pub cancel_generation: u64,
}

/// Guard-routed velocity command sample.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedGuardCommand {
    /// Public navigation session.
    pub session_id: String,
    /// Linear XY command.
    pub linear: [f64; 2],
    /// Angular Z command.
    pub angular_z: f64,
}

/// Published navigation state sample.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedNavigationState {
    /// Public navigation session, empty while idle.
    pub session_id: String,
    /// Stable state label.
    pub state: String,
}

/// Strongly typed payload recorded on a navigation channel.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", content = "data", rename_all = "snake_case")]
pub enum RecordedPayload {
    /// Odometry input.
    Odometry(RecordedOdometry),
    /// Point-cloud input.
    PointCloud(RecordedPointCloud),
    /// Versioned goal input.
    Goal(RecordedGoal),
    /// Correlated path input to Guard.
    Path(RecordedPath),
    /// Versioned cancellation input.
    Cancel(RecordedCancel),
    /// Guard-approved command output.
    GuardCommand(RecordedGuardCommand),
    /// Navigation state output.
    NavigationState(RecordedNavigationState),
}

/// One recorded navigation event.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedNavEvent {
    /// Zero-based sequence in capture order.
    pub sequence: u64,
    /// Capture timestamp in nanoseconds.
    pub timestamp_ns: i64,
    /// Physical typed channel without the schema suffix.
    pub channel: String,
    /// Decoded typed payload.
    pub payload: RecordedPayload,
}

/// Validated recording loaded from JSON Lines.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RecordedNavFixture {
    events: Vec<RecordedNavEvent>,
}

/// Behavior-level evidence reconstructed from one guarded navigation session.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GuardedRouteReplay {
    /// Public session shared by every guarded-route event.
    pub session_id: String,
    /// Goal generation shared by goal, path, and cancellation.
    pub goal_generation: i64,
    /// Positive cancellation generation.
    pub cancel_generation: u64,
    /// Sequence number of the goal event.
    pub goal_sequence: u64,
    /// Sequence number of the correlated path event.
    pub path_sequence: u64,
    /// Sequence number of the moving state.
    pub moving_sequence: u64,
    /// Sequence number of the cancellation request.
    pub cancel_sequence: u64,
    /// Sequence number of the terminal cancelled state.
    pub cancelled_sequence: u64,
    /// Non-zero Guard commands emitted before movement.
    pub movement_commands: usize,
}

impl RecordedNavFixture {
    /// Ordered event list.
    #[must_use]
    pub fn events(&self) -> &[RecordedNavEvent] {
        &self.events
    }

    /// Replay offsets from the first event, in nanoseconds.
    #[must_use]
    pub fn replay_timeline(&self) -> Vec<(u64, &RecordedNavEvent)> {
        let Some(first) = self.events.first() else {
            return Vec::new();
        };
        self.events
            .iter()
            .map(|event| {
                (
                    event.timestamp_ns.saturating_sub(first.timestamp_ns) as u64,
                    event,
                )
            })
            .collect()
    }

    /// Check that goal, path, Guard command, state and cancellation share one lease.
    pub fn validate_guarded_route(&self) -> Result<(), RecordedFixtureError> {
        self.replay_guarded_route().map(|_| ())
    }

    /// Replay the guarded route as one ordered goal-to-cancellation behavior.
    pub fn replay_guarded_route(&self) -> Result<GuardedRouteReplay, RecordedFixtureError> {
        let mut session_id: Option<String> = None;
        let mut goal_generation = None;
        let mut goal_sequence = None;
        let mut path_sequence = None;
        let mut moving_sequence = None;
        let mut cancel_sequence = None;
        let mut cancelled_sequence = None;
        let mut cancel_generation = None;
        let mut movement_commands = 0usize;

        for event in &self.events {
            if cancelled_sequence.is_some()
                && !matches!(
                    event.payload,
                    RecordedPayload::Odometry(_) | RecordedPayload::PointCloud(_)
                )
            {
                return contract_error("guarded route contains events after cancelled");
            }
            match &event.payload {
                RecordedPayload::Goal(goal) => {
                    if goal_sequence.is_some() {
                        return contract_error("guarded replay contains more than one goal");
                    }
                    bind_session(&mut session_id, &goal.session_id)?;
                    goal_generation = Some(goal.goal_generation);
                    goal_sequence = Some(event.sequence);
                }
                RecordedPayload::Path(path) => {
                    if goal_sequence.is_none() || path_sequence.is_some() {
                        return contract_error("path must occur once after the goal");
                    }
                    bind_session(&mut session_id, &path.session_id)?;
                    if goal_generation != Some(path.goal_generation) {
                        return contract_error("path does not match the goal generation");
                    }
                    path_sequence = Some(event.sequence);
                }
                RecordedPayload::GuardCommand(command) => {
                    if path_sequence.is_none()
                        || moving_sequence.is_some()
                        || cancel_sequence.is_some()
                    {
                        return contract_error(
                            "Guard command must occur after path and before moving",
                        );
                    }
                    bind_session(&mut session_id, &command.session_id)?;
                    if command.linear != [0.0, 0.0] || command.angular_z != 0.0 {
                        movement_commands = movement_commands.saturating_add(1);
                    }
                }
                RecordedPayload::NavigationState(state) => {
                    if path_sequence.is_none() {
                        return contract_error("navigation state must occur after path");
                    }
                    bind_session(&mut session_id, &state.session_id)?;
                    match state.state.as_str() {
                        "moving" => {
                            if path_sequence.is_none()
                                || movement_commands == 0
                                || moving_sequence.is_some()
                                || cancel_sequence.is_some()
                            {
                                return contract_error(
                                    "moving must occur once after path and a Guard command",
                                );
                            }
                            moving_sequence = Some(event.sequence);
                        }
                        "cancelled" => {
                            if moving_sequence.is_none()
                                || cancel_sequence.is_none()
                                || cancelled_sequence.is_some()
                            {
                                return contract_error(
                                    "cancelled must occur once after moving and cancellation",
                                );
                            }
                            cancelled_sequence = Some(event.sequence);
                        }
                        _ => {}
                    }
                }
                RecordedPayload::Cancel(cancel) => {
                    if moving_sequence.is_none() || cancel_sequence.is_some() {
                        return contract_error("cancellation must occur once after moving");
                    }
                    bind_session(&mut session_id, &cancel.session_id)?;
                    if goal_generation != Some(cancel.goal_generation) {
                        return contract_error("cancellation does not match the goal generation");
                    }
                    cancel_generation = Some(cancel.cancel_generation);
                    cancel_sequence = Some(event.sequence);
                }
                RecordedPayload::Odometry(_) | RecordedPayload::PointCloud(_) => {}
            }
        }

        let replay = GuardedRouteReplay {
            session_id: session_id.ok_or_else(|| {
                RecordedFixtureError::Contract("missing guarded-route session".into())
            })?,
            goal_generation: goal_generation
                .ok_or_else(|| RecordedFixtureError::Contract("missing goal event".into()))?,
            cancel_generation: cancel_generation.ok_or_else(|| {
                RecordedFixtureError::Contract("missing cancellation event".into())
            })?,
            goal_sequence: goal_sequence
                .ok_or_else(|| RecordedFixtureError::Contract("missing goal event".into()))?,
            path_sequence: path_sequence
                .ok_or_else(|| RecordedFixtureError::Contract("missing path event".into()))?,
            moving_sequence: moving_sequence
                .ok_or_else(|| RecordedFixtureError::Contract("missing moving state".into()))?,
            cancel_sequence: cancel_sequence.ok_or_else(|| {
                RecordedFixtureError::Contract("missing cancellation event".into())
            })?,
            cancelled_sequence: cancelled_sequence
                .ok_or_else(|| RecordedFixtureError::Contract("missing cancelled state".into()))?,
            movement_commands,
        };
        if !(replay.goal_sequence < replay.path_sequence
            && replay.path_sequence < replay.moving_sequence
            && replay.moving_sequence < replay.cancel_sequence
            && replay.cancel_sequence < replay.cancelled_sequence)
        {
            return contract_error("guarded route events are not in lifecycle order");
        }
        Ok(replay)
    }
}

fn bind_session(active: &mut Option<String>, candidate: &str) -> Result<(), RecordedFixtureError> {
    validate_identifier(candidate, "session_id")?;
    match active {
        Some(expected) if expected != candidate => {
            contract_error("guarded route spans more than one session")
        }
        Some(_) => Ok(()),
        None => {
            *active = Some(candidate.to_string());
            Ok(())
        }
    }
}

fn contract_error<T>(reason: impl Into<String>) -> Result<T, RecordedFixtureError> {
    Err(RecordedFixtureError::Contract(reason.into()))
}

/// Loader or recording-contract error.
#[derive(Debug)]
pub enum RecordedFixtureError {
    /// Fixture could not be read.
    Io(std::io::Error),
    /// A JSON line was invalid.
    Json {
        /// One-based line number.
        line: usize,
        /// JSON decoder error.
        source: serde_json::Error,
    },
    /// Ordering or payload contract was invalid.
    Contract(String),
}

impl std::fmt::Display for RecordedFixtureError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "failed to read recorded fixture: {error}"),
            Self::Json { line, source } => {
                write!(
                    formatter,
                    "invalid recorded fixture JSON at line {line}: {source}"
                )
            }
            Self::Contract(reason) => {
                write!(formatter, "invalid recorded fixture contract: {reason}")
            }
        }
    }
}

impl std::error::Error for RecordedFixtureError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Json { source, .. } => Some(source),
            Self::Contract(_) => None,
        }
    }
}

impl From<std::io::Error> for RecordedFixtureError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

/// Load and validate a typed JSON Lines recording.
pub fn load_recorded_fixture(
    path: impl AsRef<Path>,
) -> Result<RecordedNavFixture, RecordedFixtureError> {
    let file = std::fs::File::open(path)?;
    load_recorded_fixture_reader(std::io::BufReader::new(file))
}

/// Load the canonical guarded-route recording without an external fixture file.
pub fn guarded_route_fixture() -> Result<RecordedNavFixture, RecordedFixtureError> {
    load_recorded_fixture_reader(GUARDED_ROUTE_RECORDING_JSONL.as_bytes())
}

/// Decode a fixture from any buffered reader.
pub fn load_recorded_fixture_reader(
    mut reader: impl BufRead,
) -> Result<RecordedNavFixture, RecordedFixtureError> {
    let mut events = Vec::new();
    let mut line = Vec::new();
    let mut line_number = 0usize;
    let mut total_bytes = 0usize;
    while read_bounded_line(&mut reader, &mut line)? {
        line_number = line_number.saturating_add(1);
        total_bytes = total_bytes
            .checked_add(line.len())
            .ok_or_else(|| RecordedFixtureError::Contract("recording size overflowed".into()))?;
        if total_bytes > MAX_FIXTURE_BYTES {
            return contract_error(format!(
                "recording exceeds {MAX_FIXTURE_BYTES} retained bytes"
            ));
        }
        if line.iter().all(u8::is_ascii_whitespace) {
            continue;
        }
        if events.len() >= MAX_RECORDED_EVENTS {
            return contract_error(format!("recording exceeds {MAX_RECORDED_EVENTS} events"));
        }
        let event = serde_json::from_slice::<RecordedNavEvent>(&line).map_err(|source| {
            RecordedFixtureError::Json {
                line: line_number,
                source,
            }
        })?;
        validate_event(&event)?;
        if event.sequence != events.len() as u64 {
            return Err(RecordedFixtureError::Contract(format!(
                "expected sequence {}, got {}",
                events.len(),
                event.sequence
            )));
        }
        if events
            .last()
            .is_some_and(|previous: &RecordedNavEvent| previous.timestamp_ns > event.timestamp_ns)
        {
            return Err(RecordedFixtureError::Contract(
                "timestamps are not monotonic".into(),
            ));
        }
        events.push(event);
    }
    if events.is_empty() {
        return Err(RecordedFixtureError::Contract("recording is empty".into()));
    }
    Ok(RecordedNavFixture { events })
}

fn read_bounded_line(
    reader: &mut impl BufRead,
    line: &mut Vec<u8>,
) -> Result<bool, std::io::Error> {
    line.clear();
    loop {
        let available = reader.fill_buf()?;
        if available.is_empty() {
            return Ok(!line.is_empty());
        }
        let newline = available.iter().position(|byte| *byte == b'\n');
        let consumed = newline.map_or(available.len(), |index| index + 1);
        if line.len().saturating_add(consumed) > MAX_JSON_LINE_BYTES {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("recorded fixture line exceeds {MAX_JSON_LINE_BYTES} bytes"),
            ));
        }
        line.extend_from_slice(&available[..consumed]);
        reader.consume(consumed);
        if newline.is_some() {
            return Ok(true);
        }
    }
}

fn validate_event(event: &RecordedNavEvent) -> Result<(), RecordedFixtureError> {
    if !event.channel.starts_with('/')
        || event
            .channel
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(RecordedFixtureError::Contract(format!(
            "channel {} is invalid",
            event.channel
        )));
    }
    let valid = match &event.payload {
        RecordedPayload::Odometry(value) => {
            event.channel == "/odom"
                && validate_frame(&value.frame_id, "odometry frame_id").is_ok()
                && validate_frame(&value.child_frame_id, "odometry child_frame_id").is_ok()
                && value.frame_id != value.child_frame_id
                && value
                    .position
                    .iter()
                    .chain(value.orientation.iter())
                    .all(|number| number.is_finite())
                && quaternion_is_normalized(&value.orientation)
        }
        RecordedPayload::PointCloud(value) => {
            event.channel == "/map/occupancy"
                && validate_frame(&value.frame_id, "point cloud frame_id").is_ok()
                && value
                    .points
                    .iter()
                    .flatten()
                    .all(|number| number.is_finite())
        }
        RecordedPayload::Goal(value) => {
            event.channel == "/nav/goal"
                && validate_identifier(&value.session_id, "goal session_id").is_ok()
                && value.goal_generation > 0
                && value.position.iter().all(|number| number.is_finite())
        }
        RecordedPayload::Path(value) => {
            event.channel == "/nav/path"
                && validate_identifier(&value.session_id, "path session_id").is_ok()
                && value.goal_generation > 0
                && !value.points.is_empty()
                && value
                    .points
                    .iter()
                    .flatten()
                    .all(|number| number.is_finite())
        }
        RecordedPayload::GuardCommand(value) => {
            event.channel == "/cmd_vel"
                && validate_identifier(&value.session_id, "command session_id").is_ok()
                && value
                    .linear
                    .iter()
                    .chain(std::iter::once(&value.angular_z))
                    .all(|number| number.is_finite())
        }
        RecordedPayload::Cancel(value) => {
            event.channel == "/nav/cancel"
                && validate_identifier(&value.session_id, "cancel session_id").is_ok()
                && value.goal_generation > 0
                && value.cancel_generation > 0
        }
        RecordedPayload::NavigationState(value) => {
            event.channel == "/nav/state"
                && validate_identifier(&value.session_id, "state session_id").is_ok()
                && validate_identifier(&value.state, "navigation state").is_ok()
        }
    };
    valid.then_some(()).ok_or_else(|| {
        RecordedFixtureError::Contract("event metadata or payload is invalid".into())
    })
}

fn validate_identifier(value: &str, field: &str) -> Result<(), RecordedFixtureError> {
    if value.is_empty()
        || value.len() > 255
        || value
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return contract_error(format!("{field} is invalid"));
    }
    Ok(())
}

fn validate_frame(value: &str, field: &str) -> Result<(), RecordedFixtureError> {
    validate_identifier(value, field)?;
    if value.starts_with('/') || value.ends_with('/') || value.contains("//") {
        return contract_error(format!("{field} is invalid"));
    }
    Ok(())
}

fn quaternion_is_normalized(quaternion: &[f64; 4]) -> bool {
    let norm_squared = quaternion.iter().map(|value| value * value).sum::<f64>();
    (norm_squared - 1.0).abs() <= QUATERNION_NORM_TOLERANCE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guarded_route_recording_loads_as_typed_events() {
        let fixture = guarded_route_fixture().expect("load recorded fixture");
        assert_eq!(fixture.events().len(), 8);
        let replay = fixture
            .replay_guarded_route()
            .expect("guarded route replay");
        assert_eq!(replay.session_id, "session-7");
        assert_eq!(replay.goal_generation, 7);
        assert_eq!(replay.cancel_generation, 1);
        assert_eq!(replay.movement_commands, 1);
        assert!(replay.goal_sequence < replay.path_sequence);
        assert!(replay.path_sequence < replay.moving_sequence);
        assert!(replay.moving_sequence < replay.cancel_sequence);
        assert!(replay.cancel_sequence < replay.cancelled_sequence);
        let timeline = fixture.replay_timeline();
        assert_eq!(timeline.first().map(|entry| entry.0), Some(0));
        assert!(timeline.windows(2).all(|pair| pair[0].0 <= pair[1].0));
    }

    #[test]
    fn mismatched_path_lease_is_rejected() {
        let raw = GUARDED_ROUTE_RECORDING_JSONL.replace(
            "\"goal_generation\":7,\"points\"",
            "\"goal_generation\":8,\"points\"",
        );
        let fixture = load_recorded_fixture_reader(raw.as_bytes()).expect("decode recording");
        assert!(fixture.validate_guarded_route().is_err());
    }

    #[test]
    fn cross_session_and_out_of_order_routes_are_rejected() {
        let mut fixture = guarded_route_fixture().unwrap();
        if let RecordedPayload::NavigationState(state) = &mut fixture.events[5].payload {
            state.session_id = "other-session".into();
        }
        assert!(fixture.replay_guarded_route().is_err());

        let mut fixture = guarded_route_fixture().unwrap();
        fixture.events.swap(2, 3);
        assert!(fixture.replay_guarded_route().is_err());
    }

    #[test]
    fn invalid_frame_and_pose_metadata_are_rejected() {
        let empty_frame =
            GUARDED_ROUTE_RECORDING_JSONL.replacen("\"frame_id\":\"map\"", "\"frame_id\":\"\"", 1);
        assert!(load_recorded_fixture_reader(empty_frame.as_bytes()).is_err());

        let invalid_quaternion = GUARDED_ROUTE_RECORDING_JSONL.replacen(
            "\"orientation\":[0.0,0.0,0.0,1.0]",
            "\"orientation\":[0.0,0.0,0.0,0.0]",
            1,
        );
        assert!(load_recorded_fixture_reader(invalid_quaternion.as_bytes()).is_err());
    }

    #[test]
    fn oversized_json_line_is_rejected_before_unbounded_allocation() {
        let input = vec![b' '; MAX_JSON_LINE_BYTES + 1];
        let error = load_recorded_fixture_reader(input.as_slice()).unwrap_err();
        assert!(matches!(error, RecordedFixtureError::Io(_)));
    }
}
