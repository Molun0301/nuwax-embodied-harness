//! Compile the actual mapping re-export files without linking the full mapping crate.
//! These checks cover the shims and shared types; they are not a full mapping build.

#[path = "../../nuwax-mapping/src/occupancy/gradient.rs"]
mod mapping_gradient;
#[path = "../../nuwax-mapping/src/occupancy/types.rs"]
mod mapping_types;

use nuwax_numeric::occupancy::{gradient, types};
use serde_json::json;
use std::any::TypeId;

#[test]
fn mapping_shims_preserve_type_identity() {
    assert_eq!(TypeId::of::<mapping_types::Pose>(), TypeId::of::<types::Pose>());
    assert_eq!(
        TypeId::of::<mapping_types::MapMetaData>(),
        TypeId::of::<types::MapMetaData>()
    );
    assert_eq!(
        TypeId::of::<mapping_types::OccupancyValue>(),
        TypeId::of::<types::OccupancyValue>()
    );
    assert_eq!(
        TypeId::of::<mapping_types::OccupancyCell>(),
        TypeId::of::<types::OccupancyCell>()
    );
    assert_eq!(
        TypeId::of::<mapping_types::OccupancyGrid>(),
        TypeId::of::<types::OccupancyGrid>()
    );
    assert_eq!(
        TypeId::of::<mapping_gradient::Gradient>(),
        TypeId::of::<gradient::Gradient>()
    );
    assert_eq!(
        TypeId::of::<mapping_gradient::CostMapConfig>(),
        TypeId::of::<gradient::CostMapConfig>()
    );
    assert_eq!(
        TypeId::of::<mapping_gradient::CostMapOutput>(),
        TypeId::of::<gradient::CostMapOutput>()
    );
    assert_eq!(
        TypeId::of::<mapping_gradient::DistanceCostStrategy>(),
        TypeId::of::<gradient::DistanceCostStrategy>()
    );
}

#[test]
fn grid_serde_structure_matches_through_mapping_shim() {
    let expected = json!({
        "data": [-1, 0, 50, 65, 99, 100],
        "width": 3,
        "height": 2,
        "resolution": 0.25,
        "origin": [1.0, 2.0, 3.0],
        "origin_orientation": [0.0, 0.0, 0.0, 1.0],
        "frame_id": "map",
        "ts": 42.5
    });
    let legacy: mapping_types::OccupancyGrid = serde_json::from_value(expected.clone()).unwrap();
    let shared: types::OccupancyGrid = serde_json::from_value(expected.clone()).unwrap();
    assert_eq!(serde_json::to_value(&legacy).unwrap(), expected);
    assert_eq!(serde_json::to_value(&shared).unwrap(), expected);
    for col in 0..3 {
        assert_eq!(legacy.get(1, col), shared.get(1, col));
    }
}

#[test]
fn metadata_cell_and_enum_serde_remain_compatible() {
    let expected = json!({
        "map_load_time": {"sec": 42, "nsec": 123},
        "resolution": 0.25,
        "width": 3,
        "height": 2,
        "origin": {
            "position": [1.0, 2.0, 3.0],
            "orientation": [0.0, 0.0, 0.0, 1.0]
        }
    });
    let legacy: mapping_types::MapMetaData = serde_json::from_value(expected.clone()).unwrap();
    let shared: types::MapMetaData = serde_json::from_value(expected.clone()).unwrap();
    assert_eq!(legacy, shared);
    assert_eq!(serde_json::to_value(&legacy).unwrap(), expected);
    for raw in [-1, 0, 50, 65, 99, 100] {
        let legacy = mapping_types::OccupancyCell::from_i8(raw);
        let shared = types::OccupancyCell::from_i8(raw);
        assert_eq!(legacy, shared);
        assert_eq!(serde_json::to_value(legacy).unwrap(), json!({"value": raw, "prob": null}));
        assert_eq!(legacy.to_value(), shared.to_value());
    }
    for (value, name) in [
        (mapping_types::OccupancyValue::Unknown, "Unknown"),
        (mapping_types::OccupancyValue::Free, "Free"),
        (mapping_types::OccupancyValue::Occupied, "Occupied"),
    ] {
        assert_eq!(serde_json::to_value(value).unwrap(), json!(name));
        let shared: types::OccupancyValue = serde_json::from_value(json!(name)).unwrap();
        assert_eq!(value, shared);
    }
}

#[test]
fn gradient_functions_remain_callable_through_mapping_shim() {
    let mut grid = types::OccupancyGrid::new(5, 5, 0.5);
    grid.data.fill(0);
    grid.data[0] = -1;
    grid.data[6] = 50;
    grid.data[12] = 100;
    let config = gradient::CostMapConfig {
        resolution: 0.5,
        robot_increase: 0.5,
        max_cost: 99,
    };
    assert_eq!(
        mapping_gradient::make_costmap(&grid, &config).cells,
        gradient::make_costmap(&grid, &config).cells
    );
    for strategy in [
        gradient::DistanceCostStrategy::Gradient,
        gradient::DistanceCostStrategy::Voronoi,
    ] {
        let legacy = mapping_gradient::make_costmap_with_strategy(&grid, &config, strategy, 1.5);
        let shared = gradient::make_costmap_with_strategy(&grid, &config, strategy, 1.5);
        assert_eq!(legacy.cells, shared.cells);
        assert_eq!((legacy.width, legacy.height, legacy.resolution), (5, 5, 0.5));
    }
}

#[test]
fn existing_grid_and_prelude_types_stay_separate() {
    assert_eq!(
        TypeId::of::<nuwax_numeric::prelude::OccupancyGrid>(),
        TypeId::of::<nuwax_numeric::grid::OccupancyGrid>()
    );
    assert_eq!(
        TypeId::of::<nuwax_numeric::prelude::OccupancyValue>(),
        TypeId::of::<nuwax_numeric::grid::OccupancyValue>()
    );
    assert_ne!(
        TypeId::of::<nuwax_numeric::grid::OccupancyGrid>(),
        TypeId::of::<types::OccupancyGrid>()
    );
    assert_ne!(
        TypeId::of::<nuwax_numeric::grid::OccupancyValue>(),
        TypeId::of::<types::OccupancyValue>()
    );
}
