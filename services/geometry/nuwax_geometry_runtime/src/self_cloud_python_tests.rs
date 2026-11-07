//! Python真实输出至生产过滤器的跨语言接线检查。

use super::*;
use serde_json::Value;
use std::process::Command;


fn check_case(case: &Value) {
    let json = case["runtime_json"].as_str().unwrap();
    let data: Value = serde_json::from_str(json).unwrap();
    let collision = &data["self_collision"];
    let links: Vec<_> = collision["primitives"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| p["link_name"].as_str().unwrap())
        .collect();
    let expected_links: Vec<_> = case["fixed_links"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| p.as_str().unwrap())
        .collect();
    assert_eq!(links, expected_links);
    assert_eq!(
        collision["excluded_moving_collision_count"],
        case["excluded_moving"]
    );
    assert_eq!(collision["unsupported_collision_count"], 0);
    let parsed = RobotSelfFilter::from_config(Some(json)).unwrap();
    if expected_links.is_empty() {
        assert_eq!(case["expected_source"], "manual");
        assert!(parsed.is_none(), "手填尺寸不得回退导航外包");
        for alias in ["base_link", "go2_base_link"] {
            assert!(RobotSelfFilter::from_runtime_geometry(alias, json)
                .unwrap()
                .is_none());
        }
        return;
    }
    let parsed = parsed.unwrap();
    assert_eq!(parsed.model_base_link, "base");
    for alias in ["base_link", "go2_base_link"] {
        let filter = RobotSelfFilter::from_runtime_geometry(alias, json)
            .unwrap()
            .unwrap();
        assert_eq!(filter.model_base_link, parsed.model_base_link);
        assert_eq!(filter.primitives.len(), parsed.primitives.len());
        for (a, b) in filter.primitives.iter().zip(&parsed.primitives) {
            assert_eq!(
                a.shape_from_body, b.shape_from_body,
                "名称绑定不得改变数值变换"
            );
            match (&a.shape, &b.shape) {
                (Shape::Box { size_m: a }, Shape::Box { size_m: b }) => assert_eq!(a, b),
                (Shape::Sphere { radius_m: a }, Shape::Sphere { radius_m: b }) => assert_eq!(a, b),
                (
                    Shape::Cylinder {
                        radius_m: ar,
                        length_m: al,
                    },
                    Shape::Cylinder {
                        radius_m: br,
                        length_m: bl,
                    },
                ) => assert_eq!((ar, al), (br, bl)),
                _ => panic!("名称绑定不得改变实体类型"),
            }
        }
        let pose = Isometry3::from_parts(
            Translation3::new(2.0, -3.0, 0.7),
            UnitQuaternion::from_euler_angles(0.2, -0.3, 1.1),
        );
        let mut registered = Vec::new();
        let mut expected = Vec::new();
        for point in case["points"].as_array().unwrap() {
            let xyz = &point["xyz"];
            let body = Point3::new(
                xyz[0].as_f64().unwrap(),
                xyz[1].as_f64().unwrap(),
                xyz[2].as_f64().unwrap(),
            );
            let is_self = point["is_self"].as_bool().unwrap();
            assert_eq!(
                filter.contains_body(&body),
                is_self,
                "{} / {} / {}",
                case["name"],
                alias,
                point["label"]
            );
            let transformed = pose * body;
            registered.push(transformed);
            if !is_self {
                expected.push(transformed);
            }
        }
        filter.retain_external(&mut registered, &pose);
        assert_eq!(registered, expected, "完整位姿下仍按实体分类");
    }
}

#[test]
fn python_models_reach_production_filter() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .unwrap();
    let python = std::env::var_os("NEH_TEST_PYTHON").unwrap_or_else(|| "python3".into());
    let output = Command::new(python)
        .arg(root.join("scripts/generate_fixed_self_test_fixtures.py"))
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .output()
        .expect("启动Python生产解析器");
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let cases: Vec<Value> = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(cases.len(), 3);
    for case in cases {
        check_case(&case);
        println!("Python→生产过滤器：{} 通过",case["name"]);
    }
}
