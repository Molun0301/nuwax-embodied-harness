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
