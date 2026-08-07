//! Geometry-owned backend selection and production admission contracts.

use std::env;
use std::error::Error;
use std::fmt;

pub(crate) const LIO_BACKEND_ENV: &str = "NUWAX_GEOMETRY_LIO_BACKEND";
pub(crate) const MAPPING_BACKEND_ENV: &str = "NUWAX_GEOMETRY_MAPPING_BACKEND";
pub(crate) const CORRECTION_BACKEND_ENV: &str = "NUWAX_GEOMETRY_CORRECTION_BACKEND";
pub(crate) const MAP_WRITER_ENV: &str = "NUWAX_GEOMETRY_CANONICAL_MAP_WRITER";
pub(crate) const CORRECTION_WRITER_ENV: &str = "NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER";
pub(crate) const GEOMETRY_CANONICAL_WRITER: &str = "geometry";
pub(crate) const OWNER_BACKEND_NOT_IMPLEMENTED: &str = "OWNER_BACKEND_NOT_IMPLEMENTED";

const DEFAULT_LIO_BACKEND: &str = "device";
const DEFAULT_MAPPING_BACKEND: &str = "voxel";
const DEFAULT_CORRECTION_BACKEND: &str = "none";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LioBackend {
    Device,
    FastLio2,
    PointLio,
}

impl LioBackend {
    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::Device => "device",
            Self::FastLio2 => "fastlio2",
            Self::PointLio => "pointlio",
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value {
            "device" => Some(Self::Device),
            "fastlio2" => Some(Self::FastLio2),
            "pointlio" => Some(Self::PointLio),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MappingBackend {
    Voxel,
    Raytracing,
}

impl MappingBackend {
    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::Voxel => "voxel",
            Self::Raytracing => "raytracing",
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value {
            "voxel" => Some(Self::Voxel),
            "raytracing" => Some(Self::Raytracing),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CorrectionBackend {
    None,
    LibraryPgo,
    NativePgo,
    Relocalization,
}

impl CorrectionBackend {
    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::LibraryPgo => "library_pgo",
            Self::NativePgo => "native_pgo",
            Self::Relocalization => "relocalization",
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value {
            "none" => Some(Self::None),
            "library_pgo" => Some(Self::LibraryPgo),
            "native_pgo" => Some(Self::NativePgo),
            "relocalization" => Some(Self::Relocalization),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CoreOwnerSlot {
    Lio,
    Mapping,
    Correction,
}

impl fmt::Display for CoreOwnerSlot {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::Lio => "lio",
            Self::Mapping => "mapping",
            Self::Correction => "correction",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CanonicalOutput {
    Map,
    Correction,
}

impl fmt::Display for CanonicalOutput {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::Map => "map",
            Self::Correction => "correction",
        })
    }
}

/// Unvalidated, table-testable profile selections.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct OwnerProfileInput {
    pub(crate) lio: Vec<String>,
    pub(crate) mapping: Vec<String>,
    pub(crate) correction: Vec<String>,
    pub(crate) map_writer: String,
    pub(crate) correction_writer: String,
}

impl OwnerProfileInput {
    #[cfg(test)]
    pub(crate) fn new(
        lio: impl IntoIterator<Item = impl Into<String>>,
        mapping: impl IntoIterator<Item = impl Into<String>>,
        correction: impl IntoIterator<Item = impl Into<String>>,
    ) -> Self {
        Self {
            lio: lio.into_iter().map(Into::into).collect(),
            mapping: mapping.into_iter().map(Into::into).collect(),
            correction: correction.into_iter().map(Into::into).collect(),
            map_writer: GEOMETRY_CANONICAL_WRITER.to_owned(),
            correction_writer: GEOMETRY_CANONICAL_WRITER.to_owned(),
        }
    }

    /// Loads comma-separated slot selections. Missing variables use the
    /// compatibility-safe device/voxel/none profile; explicit empty, padded,
    /// or multiple values remain invalid and are rejected during validation.
    pub(crate) fn from_env_lookup(mut lookup: impl FnMut(&str) -> Option<String>) -> Self {
        Self {
            lio: selections_from_env(lookup(LIO_BACKEND_ENV), DEFAULT_LIO_BACKEND),
            mapping: selections_from_env(lookup(MAPPING_BACKEND_ENV), DEFAULT_MAPPING_BACKEND),
            correction: selections_from_env(
                lookup(CORRECTION_BACKEND_ENV),
                DEFAULT_CORRECTION_BACKEND,
            ),
            map_writer: writer_from_env(lookup(MAP_WRITER_ENV)),
            correction_writer: writer_from_env(lookup(CORRECTION_WRITER_ENV)),
        }
    }

    pub(crate) fn from_env() -> Result<Self, ProfileEnvError> {
        let mut error = None;
        let input = Self::from_env_lookup(|key| match env::var(key) {
            Ok(value) => Some(value),
            Err(env::VarError::NotPresent) => None,
            Err(env::VarError::NotUnicode(_)) => {
                error.get_or_insert_with(|| ProfileEnvError {
                    variable: key.to_owned(),
                });
                None
            }
        });
        error.map_or(Ok(input), Err)
    }
}

fn selections_from_env(value: Option<String>, default: &str) -> Vec<String> {
    match value {
        Some(value) => value.split(',').map(str::to_owned).collect(),
        None => vec![default.to_owned()],
    }
}

fn writer_from_env(value: Option<String>) -> String {
    value
        .map(|writer| writer.trim().to_owned())
        .unwrap_or_else(|| GEOMETRY_CANONICAL_WRITER.to_owned())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ProfileEnvError {
    pub(crate) variable: String,
}

impl fmt::Display for ProfileEnvError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "profile environment variable `{}` is not valid Unicode",
            self.variable
        )
    }
}

impl Error for ProfileEnvError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ProfileValidationError {
    MissingSelection {
        slot: CoreOwnerSlot,
    },
    EmptySelection {
        slot: CoreOwnerSlot,
    },
    MultipleSelections {
        slot: CoreOwnerSlot,
        selections: Vec<String>,
    },
    UnknownSelection {
        slot: CoreOwnerSlot,
        selection: String,
    },
    CanonicalWriterConflict {
        output: CanonicalOutput,
        claimed_writer: String,
    },
}

impl fmt::Display for ProfileValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingSelection { slot } => {
                write!(formatter, "{slot} owner selection is missing")
            }
            Self::EmptySelection { slot } => {
                write!(formatter, "{slot} owner selection is empty")
            }
            Self::MultipleSelections { slot, selections } => write!(
                formatter,
                "{slot} requires exactly one owner selection, got {selections:?}"
            ),
            Self::UnknownSelection { slot, selection } => {
                write!(formatter, "unknown {slot} owner selection `{selection}`")
            }
            Self::CanonicalWriterConflict {
                output,
                claimed_writer,
            } => write!(
                formatter,
                "canonical {output} writer must be `{GEOMETRY_CANONICAL_WRITER}`, got `{claimed_writer}`"
            ),
        }
    }
}

impl Error for ProfileValidationError {}

/// A fully validated immutable S1/S3/S4 owner profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct OwnerProfile {
    lio: LioBackend,
    mapping: MappingBackend,
    correction: CorrectionBackend,
}

impl OwnerProfile {
    pub(crate) fn validate(input: OwnerProfileInput) -> Result<Self, ProfileValidationError> {
        validate_writer(CanonicalOutput::Map, &input.map_writer)?;
        validate_writer(CanonicalOutput::Correction, &input.correction_writer)?;

        let lio = parse_exactly_one(CoreOwnerSlot::Lio, input.lio, LioBackend::parse)?;
        let mapping =
            parse_exactly_one(CoreOwnerSlot::Mapping, input.mapping, MappingBackend::parse)?;
        let correction = parse_exactly_one(
            CoreOwnerSlot::Correction,
            input.correction,
            CorrectionBackend::parse,
        )?;

        Ok(Self {
            lio,
            mapping,
            correction,
        })
    }

    pub(crate) fn from_env() -> Result<Self, OwnerProfileLoadError> {
        let input = OwnerProfileInput::from_env().map_err(OwnerProfileLoadError::Environment)?;
        Self::validate(input).map_err(OwnerProfileLoadError::Validation)
    }

    pub(crate) const fn lio(&self) -> LioBackend {
        self.lio
    }

    pub(crate) const fn mapping(&self) -> MappingBackend {
        self.mapping
    }

    pub(crate) const fn correction(&self) -> CorrectionBackend {
        self.correction
    }

    /// Admits profiles whose concrete owners are currently wired into the
    /// production runtime. Checks stay in startup order so a profile with
    /// multiple unsupported selections always reports the same first slot.
    pub(crate) fn admit_production(&self) -> Result<(), OwnerBackendNotImplemented> {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct OwnerBackendNotImplemented {
    pub(crate) slot: CoreOwnerSlot,
    pub(crate) backend: &'static str,
}

impl fmt::Display for OwnerBackendNotImplemented {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{OWNER_BACKEND_NOT_IMPLEMENTED} slot={} backend={}",
            self.slot, self.backend
        )
    }
}

impl Error for OwnerBackendNotImplemented {}

fn validate_writer(
    output: CanonicalOutput,
    claimed_writer: &str,
) -> Result<(), ProfileValidationError> {
    if claimed_writer == GEOMETRY_CANONICAL_WRITER {
        Ok(())
    } else {
        Err(ProfileValidationError::CanonicalWriterConflict {
            output,
            claimed_writer: claimed_writer.to_owned(),
        })
    }
}

fn parse_exactly_one<T>(
    slot: CoreOwnerSlot,
    selections: Vec<String>,
    parse: impl FnOnce(&str) -> Option<T>,
) -> Result<T, ProfileValidationError> {
    match selections.as_slice() {
        [] => Err(ProfileValidationError::MissingSelection { slot }),
        [selection] if selection.is_empty() => Err(ProfileValidationError::EmptySelection { slot }),
        [selection] => parse(selection).ok_or_else(|| ProfileValidationError::UnknownSelection {
            slot,
            selection: selection.clone(),
        }),
        _ => Err(ProfileValidationError::MultipleSelections { slot, selections }),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum OwnerProfileLoadError {
    Environment(ProfileEnvError),
    Validation(ProfileValidationError),
}

impl fmt::Display for OwnerProfileLoadError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Environment(error) => error.fmt(formatter),
            Self::Validation(error) => error.fmt(formatter),
        }
    }
}

impl Error for OwnerProfileLoadError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Environment(error) => Some(error),
            Self::Validation(error) => Some(error),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_input() -> OwnerProfileInput {
        OwnerProfileInput::new(["device"], ["voxel"], ["none"])
    }

    #[test]
    fn every_supported_selection_combination_validates() {
        let lio = ["device", "fastlio2", "pointlio"];
        let mapping = ["voxel", "raytracing"];
        let correction = ["none", "library_pgo", "native_pgo", "relocalization"];

        let mut count = 0;
        for lio_name in lio {
            for mapping_name in mapping {
                for correction_name in correction {
                    let profile = OwnerProfile::validate(OwnerProfileInput::new(
                        [lio_name],
                        [mapping_name],
                        [correction_name],
                    ))
                    .expect("supported combination must validate");
                    assert_eq!(profile.lio().stable_name(), lio_name);
                    assert_eq!(profile.mapping().stable_name(), mapping_name);
                    assert_eq!(profile.correction().stable_name(), correction_name);
                    count += 1;
                }
            }
        }
        assert_eq!(count, 24);
    }

    #[test]
    fn production_admission_is_exhaustive_and_deterministic() {
        let lio = ["device", "fastlio2", "pointlio"];
        let mapping = ["voxel", "raytracing"];
        let correction = ["none", "library_pgo", "native_pgo", "relocalization"];

        let mut admitted = 0;
        for lio_name in lio {
            for mapping_name in mapping {
                for correction_name in correction {
                    let profile = OwnerProfile::validate(OwnerProfileInput::new(
                        [lio_name],
                        [mapping_name],
                        [correction_name],
                    ))
                    .expect("combination must be structurally valid");
                    profile
                        .admit_production()
                        .expect("every structurally valid combination must be admitted");
                    admitted += 1;
                }
            }
        }
        assert_eq!(admitted, 24);
    }

    #[test]
    fn missing_env_values_use_safe_defaults() {
        let input = OwnerProfileInput::from_env_lookup(|_| None);
        let profile = OwnerProfile::validate(input).expect("defaults must validate");
        assert_eq!(profile.lio(), LioBackend::Device);
        assert_eq!(profile.mapping(), MappingBackend::Voxel);
        assert_eq!(profile.correction(), CorrectionBackend::None);
    }

    #[test]
    fn comma_separated_env_value_is_preserved_as_a_conflict() {
        let input = OwnerProfileInput::from_env_lookup(|key| {
            (key == LIO_BACKEND_ENV).then(|| "device,fastlio2".to_owned())
        });
        assert!(matches!(
            OwnerProfile::validate(input),
            Err(ProfileValidationError::MultipleSelections {
                slot: CoreOwnerSlot::Lio,
                ..
            })
        ));
    }

    #[test]
    fn padded_slot_selections_are_rejected_without_normalization() {
        let cases = [
            (CoreOwnerSlot::Lio, " device "),
            (CoreOwnerSlot::Mapping, " voxel "),
            (CoreOwnerSlot::Correction, " none "),
        ];

        for (slot, selection) in cases {
            let mut input = valid_input();
            match slot {
                CoreOwnerSlot::Lio => input.lio = vec![selection.into()],
                CoreOwnerSlot::Mapping => input.mapping = vec![selection.into()],
                CoreOwnerSlot::Correction => input.correction = vec![selection.into()],
            }
            assert_eq!(
                OwnerProfile::validate(input),
                Err(ProfileValidationError::UnknownSelection {
                    slot,
                    selection: selection.into(),
                })
            );
        }
    }

    #[test]
    fn zero_multiple_empty_and_unknown_selections_are_rejected() {
        let cases = [
            (
                OwnerProfileInput {
                    lio: Vec::new(),
                    ..valid_input()
                },
                CoreOwnerSlot::Lio,
                "missing",
            ),
            (
                OwnerProfileInput {
                    mapping: vec!["voxel".into(), "raytracing".into()],
                    ..valid_input()
                },
                CoreOwnerSlot::Mapping,
                "multiple",
            ),
            (
                OwnerProfileInput {
                    correction: vec!["".into()],
                    ..valid_input()
                },
                CoreOwnerSlot::Correction,
                "empty",
            ),
            (
                OwnerProfileInput {
                    lio: vec!["magic".into()],
                    ..valid_input()
                },
                CoreOwnerSlot::Lio,
                "unknown",
            ),
        ];

        for (input, expected_slot, expected_kind) in cases {
            let error = OwnerProfile::validate(input).expect_err("case must be invalid");
            let (slot, kind) = match error {
                ProfileValidationError::MissingSelection { slot } => (slot, "missing"),
                ProfileValidationError::MultipleSelections { slot, .. } => (slot, "multiple"),
                ProfileValidationError::EmptySelection { slot } => (slot, "empty"),
                ProfileValidationError::UnknownSelection { slot, .. } => (slot, "unknown"),
                ProfileValidationError::CanonicalWriterConflict { .. } => {
                    panic!("unexpected writer conflict")
                }
            };
            assert_eq!(slot, expected_slot);
            assert_eq!(kind, expected_kind);
        }
    }

    #[test]
    fn non_geometry_canonical_writers_are_rejected() {
        for output in [CanonicalOutput::Map, CanonicalOutput::Correction] {
            let mut input = valid_input();
            match output {
                CanonicalOutput::Map => input.map_writer = "mapping_sidecar".into(),
                CanonicalOutput::Correction => input.correction_writer = "pgo_backend".into(),
            }
            assert_eq!(
                OwnerProfile::validate(input),
                Err(ProfileValidationError::CanonicalWriterConflict {
                    output,
                    claimed_writer: match output {
                        CanonicalOutput::Map => "mapping_sidecar".into(),
                        CanonicalOutput::Correction => "pgo_backend".into(),
                    },
                })
            );
        }
    }
}
