use super::*;
fn free_map(width: u32, height: u32) -> MapState {
    MapState {
        width,
        height,
        resolution: 1.0,
        origin_x: 0.0,
        origin_y: 0.0,
        data: vec![0; width as usize * height as usize],
    }
}
fn loaded(zones: Vec<ForbiddenZone>) -> LoadedForbiddenZones {
    LoadedForbiddenZones { zones }
}
fn zone(shape: ZoneShape, mode: ZoneMode, vertices: Vec<ZonePoint>) -> ForbiddenZone {
    ForbiddenZone {
        id: "zone".to_string(),
        shape,
        mode,
        frame_id: "map".into(),
        vertices,
        center: None,
        radius_m: None,
        stroke_width_m: None,
        enabled: true,
        metadata: serde_json::json!({}),
    }
}
#[test]
fn diagonal_line_uses_conservative_supercover_cells() {
    let mut line = zone(
        ZoneShape::Line,
        ZoneMode::Inside,
        vec![ZonePoint { x: 0.1, y: 0.1 }, ZonePoint { x: 3.9, y: 3.9 }],
    );
    line.stroke_width_m = Some(0.2);
    let map = compose_effective_map(&free_map(4, 4), &loaded(vec![line])).unwrap();
    for index in [0_usize, 5, 10, 15] {
        assert_eq!(map.data[index], 100);
    }
}
#[test]
fn rectangle_inside_marks_interior_and_boundary() {
    let rectangle = zone(
        ZoneShape::Rectangle,
        ZoneMode::Inside,
        vec![ZonePoint { x: 2.0, y: 2.0 }, ZonePoint { x: 4.0, y: 4.0 }],
    );
    let map = compose_effective_map(&free_map(6, 6), &loaded(vec![rectangle])).unwrap();
    assert_eq!(map.data[3 + 3 * 6], 100);
    assert_eq!(map.data[0], 0);
}
#[test]
fn outside_zones_intersect_their_allowed_interiors() {
    let left = zone(
        ZoneShape::Rectangle,
        ZoneMode::Outside,
        vec![ZonePoint { x: 0.0, y: 0.0 }, ZonePoint { x: 5.0, y: 5.0 }],
    );
    let right = zone(
        ZoneShape::Rectangle,
        ZoneMode::Outside,
        vec![ZonePoint { x: 2.0, y: 0.0 }, ZonePoint { x: 7.0, y: 5.0 }],
    );
    let map = compose_effective_map(&free_map(7, 5), &loaded(vec![left, right])).unwrap();
    assert_eq!(map.data[3 + 2 * 7], 0);
    assert_eq!(map.data[0], 100);
    assert_eq!(map.data[6], 100);
}
#[test]
fn circle_outside_keeps_only_cells_fully_inside() {
    let mut circle = zone(ZoneShape::Circle, ZoneMode::Outside, Vec::new());
    circle.center = Some(ZonePoint { x: 2.5, y: 2.5 });
    circle.radius_m = Some(2.25);
    let map = compose_effective_map(&free_map(5, 5), &loaded(vec![circle])).unwrap();
    assert_eq!(map.data[2 + 2 * 5], 0);
    assert_eq!(map.data[0], 100);
}
#[test]
fn concave_polygon_inside_is_rasterized_without_filling_the_notch() {
    let polygon = zone(
        ZoneShape::Polygon,
        ZoneMode::Inside,
        vec![
            ZonePoint { x: 0.0, y: 0.0 },
            ZonePoint { x: 4.0, y: 0.0 },
            ZonePoint { x: 4.0, y: 1.0 },
            ZonePoint { x: 1.0, y: 1.0 },
            ZonePoint { x: 1.0, y: 4.0 },
            ZonePoint { x: 0.0, y: 4.0 },
        ],
    );
    let map = compose_effective_map(&free_map(5, 5), &loaded(vec![polygon])).unwrap();
    assert_eq!(map.data[0], 100);
    assert_eq!(map.data[3 + 3 * 5], 0);
}
#[test]
fn self_intersecting_polygon_is_rejected() {
    let polygon = zone(
        ZoneShape::Polygon,
        ZoneMode::Inside,
        vec![
            ZonePoint { x: 0.0, y: 0.0 },
            ZonePoint { x: 2.0, y: 2.0 },
            ZonePoint { x: 0.0, y: 2.0 },
            ZonePoint { x: 2.0, y: 0.0 },
        ],
    );
    assert!(validate_zone(&polygon).is_err());
}

#[test]
fn python_gateway_array_is_read_without_a_document_wrapper() {
    let root = std::env::temp_dir().join(format!("e7b-forbidden-{}", uuid::Uuid::new_v4()));
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).ancestors().nth(3).unwrap();
    let output = std::process::Command::new("python3")
        .current_dir(repo)
        .args(["-c", r#"
import sys
from contracts.forbidden_zones import ForbiddenZone
from services.geometry.maps import MapStore
specs = [
    dict(id='线', shape='line', vertices=[dict(x=0,y=0),dict(x=2,y=0)], stroke_width_m=0.4),
    dict(id='矩形', shape='rectangle', vertices=[dict(x=1,y=1),dict(x=2,y=2)]),
    dict(id='圆', shape='circle', center=dict(x=3,y=3), radius_m=0.5),
    dict(id='多边形', shape='polygon', vertices=[dict(x=0,y=0),dict(x=1,y=0),dict(x=0,y=1)]),
]
MapStore(sys.argv[1]).write_forbidden_zones('数组地图', [ForbiddenZone.model_validate(s).model_dump(mode='json') for s in specs])
"#])
        .arg(&root).output().unwrap();
    assert!(output.status.success(), "{}", String::from_utf8_lossy(&output.stderr));
    let path = root.join("数组地图/forbidden_zones.json");
    let zones = read(&path).unwrap();
    assert_eq!(zones.len(), 4);
    assert_eq!(zones[0].shape, ZoneShape::Line);
    assert_eq!(zones[0].stroke_width_m, Some(0.4));
    assert_eq!(zones[1].shape, ZoneShape::Rectangle);
    assert_eq!(zones[2].radius_m, Some(0.5));
    assert_eq!(zones[3].shape, ZoneShape::Polygon);
    assert!(zones.iter().all(|z| z.enabled && z.frame_id == "map"));
    assert!(compose_effective_map(&free_map(5, 5), &loaded(zones)).unwrap().data.contains(&100));
    std::fs::write(&path, r#"{"zones":[]}"#).unwrap();
    assert!(read(&path).unwrap_err().contains("禁行区格式无效"));
    std::fs::remove_dir_all(root).unwrap();
}
