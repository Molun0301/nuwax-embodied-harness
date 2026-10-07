//! Differential corpus for an Inside Rectangle scan change. The oracle and all
//! shared geometry primitives must stay frozen while this corpus is used.
use super::*;
use std::panic::{catch_unwind, AssertUnwindSafe};

pub(crate) struct Case {
    pub name: String,
    pub raw: MapState,
    pub loaded: LoadedForbiddenZones,
    pub panics: bool,
}

impl Case {
    fn new(name: impl Into<String>, raw: MapState, zones: Vec<ForbiddenZone>) -> Self {
        Self {
            name: name.into(), raw, panics: false,
            loaded: LoadedForbiddenZones {
                signature: "rectangle-scan-applied".into(), generation: 23,
                status: ZoneLoadStatus::Valid, zones,
            },
        }
    }
}

fn map(width: u32, height: u32, resolution: f32, origin: (f64, f64)) -> MapState {
    MapState {
        width, height, resolution, origin_x: origin.0, origin_y: origin.1,
        frame_id: "rectangle-scan-frame".into(), revision: 137,
        timestamp_ns: 9_876_543_210_123,
        data: (0..width as usize * height as usize)
            .map(|i| [-1, 0, 42, 100][i % 4]).collect(),
    }
}

fn rectangle(a: (f64, f64), b: (f64, f64), stroke: Option<f64>) -> ForbiddenZone {
    ForbiddenZone {
        id: "rectangle".into(), shape: ZoneShape::Rectangle, mode: ZoneMode::Inside,
        frame_id: "rectangle-scan-frame".into(),
        vertices: vec![ZonePoint { x: a.0, y: a.1 }, ZonePoint { x: b.0, y: b.1 }],
        center: None, radius_m: None, stroke_width_m: stroke, enabled: true,
        metadata: serde_json::json!({"corpus": "rectangle-scan"}),
    }
}

fn small_map() -> MapState { map(7, 3, 0.25, (-0.375, 0.125)) }
fn small_rectangle() -> ForbiddenZone { rectangle((-0.125, 0.375), (0.625, 0.875), None) }

// Each loop varies a single operand/axis. No geometric Cartesian product or
// large maps: even the nonsquare mixed-shape fixture contains only 48 cells.
pub(crate) fn corpus() -> Vec<Case> {
    let mut cases = Vec::new();
    for (name, a, b) in [
        ("partial", (-0.125, 0.375), (0.625, 0.875)),
        ("reversed", (0.625, 0.875), (-0.125, 0.375)),
        ("full", (-10.0, -10.0), (10.0, 10.0)),
        ("left-disjoint", (-4.0, 0.0), (-3.0, 1.0)),
        ("right-disjoint", (3.0, 0.0), (4.0, 1.0)),
        ("below-disjoint", (-1.0, -4.0), (1.0, -3.0)),
        ("above-disjoint", (-1.0, 3.0), (1.0, 4.0)),
        ("point", (0.375, 0.5), (0.375, 0.5)),
    ] {
        cases.push(Case::new(name, small_map(), vec![rectangle(a, b, None)]));
    }
    for (width, height) in [(1, 7), (7, 1)] {
        cases.push(Case::new(format!("thin-{width}x{height}"),
            map(width, height, 0.25, (-0.0, 0.0)),
            vec![rectangle((0.0, 0.0), (0.5, 0.5), None)]));
    }
    cases.push(Case::new("overlapping-and-disjoint", small_map(), vec![
        small_rectangle(), rectangle((0.25, 0.25), (1.25, 0.625), None),
        rectangle((3.0, 3.0), (4.0, 4.0), None),
    ]));
    for (name, stroke) in [
        ("default", None), ("zero", Some(0.0)), ("sub-resolution", Some(0.0625)),
        ("equal-resolution", Some(0.25)), ("super-resolution", Some(1.25)),
        ("nan", Some(f64::NAN)), ("positive-infinity", Some(f64::INFINITY)),
        ("negative-infinity", Some(f64::NEG_INFINITY)),
        ("maximum", Some(f64::MAX)), ("negative-maximum", Some(-f64::MAX)),
    ] {
        let mut zone = small_rectangle(); zone.stroke_width_m = stroke;
        cases.push(Case::new(format!("stroke/{name}"), small_map(), vec![zone]));
    }

    // Touching inclusive comparisons and one adjacent representable coordinate
    // to either side, after both stroke expansion and GEOMETRY_EPSILON.
    for axis in 0..2 {
        for (bound, boundary) in [("lower", 2.5 + GEOMETRY_EPSILON),
                                  ("upper", 1.5 - GEOMETRY_EPSILON)] {
            for (offset, bits) in [("below", boundary.to_bits() - 1),
                                   ("exact", boundary.to_bits()),
                                   ("above", boundary.to_bits() + 1)] {
                let value = f64::from_bits(bits);
                let (lo, hi) = if bound == "lower" { (value, 3.0) } else { (0.0, value) };
                let (a, b) = if axis == 0 { ((lo, -1.0), (hi, 2.0)) }
                             else { ((-1.0, lo), (2.0, hi)) };
                cases.push(Case::new(format!("touch/{axis}/{bound}/{offset}"),
                    map(if axis == 0 { 5 } else { 1 }, if axis == 0 { 1 } else { 5 },
                        1.0, (0.0, -0.0)), vec![rectangle(a, b, None)]));
            }
        }
    }

    for axis in 0..2 {
        for (name, value) in [
            ("positive-zero", 0.0), ("negative-zero", -0.0),
            ("fractional", -0.625), ("nan", f64::from_bits(0x7ff8_0000_0000_1234)),
            ("positive-infinity", f64::INFINITY), ("negative-infinity", f64::NEG_INFINITY),
            ("maximum", f64::MAX), ("negative-maximum", -f64::MAX),
        ] {
            let mut raw = small_map(); let mut zone = small_rectangle();
            if axis == 0 {
                raw.origin_x = value;
                if value.is_finite() { zone.vertices[0].x = value; zone.vertices[1].x = value + 0.5; }
            } else {
                raw.origin_y = value;
                if value.is_finite() { zone.vertices[0].y = value; zone.vertices[1].y = value + 0.5; }
            }
            cases.push(Case::new(format!("origin/{axis}/{name}"), raw, vec![zone]));
        }
        // NaN in just one corner is consumed by f64::min/max; both NaN
        // corners, or infinity minus infinite stroke, leave NaN thresholds.
        for vertex in 0..2 {
            for (name, value) in [("nan", f64::NAN), ("positive-infinity", f64::INFINITY),
                ("negative-infinity", f64::NEG_INFINITY), ("maximum", f64::MAX),
                ("negative-maximum", -f64::MAX)] {
                let mut zone = small_rectangle();
                if axis == 0 { zone.vertices[vertex].x = value; }
                else { zone.vertices[vertex].y = value; }
                cases.push(Case::new(format!("corner/{axis}/{vertex}/{name}"), small_map(), vec![zone]));
            }
        }
        for (name, value, stroke) in [
            ("both-nan", f64::NAN, None),
            ("nan-upper", f64::NEG_INFINITY, Some(f64::INFINITY)),
            ("nan-lower", f64::INFINITY, Some(f64::INFINITY)),
        ] {
            let mut zone = small_rectangle(); zone.stroke_width_m = stroke;
            for vertex in &mut zone.vertices {
                if axis == 0 { vertex.x = value; } else { vertex.y = value; }
            }
            cases.push(Case::new(format!("threshold/{axis}/{name}"), small_map(), vec![zone]));
        }
    }
    for (name, resolution) in [("minimum-subnormal", f32::from_bits(1)),
        ("minimum-normal", f32::MIN_POSITIVE), ("maximum", f32::MAX)] {
        let r = f64::from(resolution);
        cases.push(Case::new(format!("resolution/{name}"), map(5, 3, resolution, (-0.0, 0.0)),
            vec![rectangle((r, r), (3.0 * r, 2.0 * r), None)]));
    }
    for sign in [-1.0, 1.0] {
        let edge = sign * f64::MAX;
        cases.push(Case::new(format!("overflow-expansion/{sign}"),
            map(3, 2, 0.25, (edge, edge)),
            vec![rectangle((edge * 0.75, edge * 0.75), (edge, edge), Some(f64::MAX))]));
    }
    cases.push(Case::new("overflow-expansion-both-sides", small_map(),
        vec![rectangle((-f64::MAX, -f64::MAX), (f64::MAX, f64::MAX), Some(f64::MAX))]));
    cases.push(rounding_case(false)); cases.push(rounding_case(true));

    let mut line = rectangle((0.0, 0.0), (2.0, 2.0), Some(0.125));
    line.shape = ZoneShape::Line;
    let mut circle = small_rectangle(); circle.shape = ZoneShape::Circle;
    circle.center = Some(ZonePoint { x: 6.0, y: 2.0 }); circle.radius_m = Some(0.25);
    let mut polygon = small_rectangle(); polygon.shape = ZoneShape::Polygon;
    polygon.vertices = [(4.0, 4.0), (7.0, 4.0), (7.0, 5.0), (5.0, 5.0),
        (5.0, 4.5), (4.0, 4.5)].into_iter().map(|(x, y)| ZonePoint { x, y }).collect();
    let inside = rectangle((3.25, 2.25), (4.25, 3.25), None);
    let mut outside = rectangle((1.0, -1.0), (8.0, 7.0), None); outside.mode = ZoneMode::Outside;
    let mut outside_circle = circle.clone(); outside_circle.mode = ZoneMode::Outside;
    outside_circle.center = Some(ZonePoint { x: 4.0, y: 3.0 }); outside_circle.radius_m = Some(4.0);
    cases.push(Case::new("mixed-inside-shapes", map(8, 6, 1.0, (0.0, -0.0)),
        vec![line, circle, polygon, inside.clone()]));
    cases.push(Case::new("inside-plus-outside-intersection", map(8, 6, 1.0, (0.0, -0.0)),
        vec![outside, inside, outside_circle]));

    for (name, status, enabled) in [
        ("empty-enabled", ZoneLoadStatus::Empty, true),
        ("valid-disabled", ZoneLoadStatus::Valid, false),
        ("empty-disabled", ZoneLoadStatus::Empty, false),
        ("fail-closed-enabled", ZoneLoadStatus::FailClosed("fixture".into()), true),
        ("fail-closed-disabled", ZoneLoadStatus::FailClosed("fixture".into()), false),
    ] {
        let mut case = Case::new(name, small_map(), vec![small_rectangle()]);
        case.loaded.status = status; case.loaded.zones[0].enabled = enabled; cases.push(case);
    }
    for status in [ZoneLoadStatus::Valid, ZoneLoadStatus::Empty, ZoneLoadStatus::FailClosed("fixture".into())] {
        let mut case = Case::new(format!("no-zones/{status:?}"), small_map(), vec![]);
        case.loaded.status = status; cases.push(case);
    }
    // Direct callers can bypass JSON validation. Keep both the panic and the
    // earlier disabled/fail-closed gates, including a malformed Outside shape.
    for len in 0..2 {
        for mode in [ZoneMode::Inside, ZoneMode::Outside] {
            for gate in ["enabled", "empty-enabled", "disabled", "fail-closed"] {
                let mut case = Case::new(format!("short-vertices/{len}/{mode:?}/{gate}"),
                    small_map(), vec![small_rectangle()]);
                case.loaded.zones[0].vertices.truncate(len);
                case.loaded.zones[0].mode = mode.clone();
                case.loaded.zones[0].enabled = gate != "disabled";
                case.loaded.status = match gate {
                    "empty-enabled" => ZoneLoadStatus::Empty,
                    "fail-closed" => ZoneLoadStatus::FailClosed("fixture".into()),
                    _ => ZoneLoadStatus::Valid,
                };
                case.panics = matches!(gate, "enabled" | "empty-enabled"); cases.push(case);
            }
        }
    }
    for (name, mutate) in [
        ("zero-width", (|raw: &mut MapState| raw.width = 0) as fn(&mut MapState)),
        ("zero-height", |raw: &mut MapState| raw.height = 0),
        ("short-data", |raw: &mut MapState| { raw.data.pop(); }),
        ("long-data", |raw: &mut MapState| raw.data.push(0)),
    ] {
        let mut case = Case::new(format!("invalid/{name}"), small_map(), vec![small_rectangle()]);
        mutate(&mut case.raw); cases.push(case);
    }
    for resolution in [0.0, -0.0, -1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut case = Case::new(format!("invalid/resolution/{:08x}", resolution.to_bits()),
            small_map(), vec![small_rectangle()]);
        case.raw.resolution = resolution; cases.push(case);
    }
    for (name, status, enabled) in [
        ("empty-enabled", ZoneLoadStatus::Empty, true),
        ("disabled", ZoneLoadStatus::Valid, false),
        ("fail-closed", ZoneLoadStatus::FailClosed("fixture".into()), true),
        ("malformed", ZoneLoadStatus::Valid, true),
    ] {
        let mut case = Case::new(format!("invalid/before-{name}"), small_map(), vec![small_rectangle()]);
        case.raw.data.pop(); case.loaded.status = status;
        case.loaded.zones[0].enabled = enabled; case.loaded.zones[0].vertices.clear(); cases.push(case);
    }
    cases
}

fn rounding_case(transpose: bool) -> Case {
    let origin = 9_007_199_254_740_992.0; // 2^53; the ULP is 2.
    let (raw, a, b) = if transpose {
        (map(1, 8, 0.25, (-0.0, origin)), (-1.0, origin + 2.0), (1.0, origin + 4.0))
    } else {
        (map(8, 1, 0.25, (origin, -0.0)), (origin + 2.0, -1.0), (origin + 4.0, 1.0))
    };
    Case::new(format!("rounding/{transpose}"), raw, vec![rectangle(a, b, None)])
}

pub(crate) fn assert_map_eq(name: &str, actual: &MapState, expected: &MapState) {
    assert_eq!(actual.width, expected.width, "{name}: width");
    assert_eq!(actual.height, expected.height, "{name}: height");
    assert_eq!(actual.resolution.to_bits(), expected.resolution.to_bits(), "{name}: resolution bits");
    assert_eq!(actual.origin_x.to_bits(), expected.origin_x.to_bits(), "{name}: origin_x bits");
    assert_eq!(actual.origin_y.to_bits(), expected.origin_y.to_bits(), "{name}: origin_y bits");
    assert_eq!(actual.frame_id, expected.frame_id, "{name}: frame");
    assert_eq!(actual.revision, expected.revision, "{name}: revision");
    assert_eq!(actual.timestamp_ns, expected.timestamp_ns, "{name}: timestamp");
    assert_eq!(actual.data, expected.data, "{name}: complete payload");
}

pub(crate) fn assert_same_outcome(
    name: &str, panics: bool,
    actual: impl FnOnce() -> Result<Option<MapState>, String>,
    expected: impl FnOnce() -> Result<Option<MapState>, String>,
) {
    let actual = catch_unwind(AssertUnwindSafe(actual));
    let expected = catch_unwind(AssertUnwindSafe(expected));
    assert_eq!(actual.is_err(), panics, "{name}: production panic");
    assert_eq!(expected.is_err(), panics, "{name}: frozen panic");
    match (actual, expected) {
        (Ok(Ok(Some(actual))), Ok(Ok(Some(expected)))) => assert_map_eq(name, &actual, &expected),
        (Ok(Ok(None)), Ok(Ok(None))) | (Err(_), Err(_)) => {},
        (Ok(Err(actual)), Ok(Err(expected))) => assert_eq!(actual, expected, "{name}: error"),
        _ => panic!("{name}: production/frozen result variants differ"),
    }
}

#[test]
fn rectangle_scan_borrowed_corpus_matches_frozen() {
    for case in corpus() {
        let before = case.raw.clone();
        assert_same_outcome(&case.name, case.panics,
            || compose_effective_map(&case.raw, &case.loaded).map(Some),
            || copy_trial_reference::compose_effective_map(&case.raw, &case.loaded).map(Some));
        assert_map_eq(&case.name, &case.raw, &before);
        if case.name.starts_with("invalid/") {
            assert_eq!(compose_effective_map(&case.raw, &case.loaded).err().as_deref(),
                Some("raw occupancy layout is invalid"), "{}", case.name);
        }
    }
}

#[test]
fn rectangle_scan_preserves_original_cell_max_rounding() {
    for transpose in [false, true] {
        let case = rounding_case(transpose);
        let origin = 9_007_199_254_740_992.0;
        let cell = cell_rect(&case.raw, if transpose { 0 } else { 4 }, if transpose { 4 } else { 0 });
        let (min, max) = if transpose { (cell.min_y, cell.max_y) } else { (cell.min_x, cell.max_x) };
        assert_eq!(min, origin); assert_eq!(max, origin);
        assert_eq!(origin + 5.0 * 0.25, origin + 2.0); // The tempting replacement is different.
        let expected = [-1, 0, 42, 100, -1, 100, 100, 100];
        assert_eq!(copy_trial_reference::compose_effective_map(&case.raw, &case.loaded).unwrap().data, expected);
        assert_eq!(compose_effective_map(&case.raw, &case.loaded).unwrap().data, expected);
    }
}
