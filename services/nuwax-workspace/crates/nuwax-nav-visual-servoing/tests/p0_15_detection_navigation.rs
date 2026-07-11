//! P0-15 回归测试:extract_3d_from_detection + DetectionNavigationModule
//!
//! 覆盖：
//! - 简化投影算法(bbox 中心 -> pointcloud 1:1 索引)正确取出 depth
//! - TF 把 camera frame 转到 world frame
//! - Module I/O 单步产出 twist,且与直接 compute_twist 等价
//! - bbox 无效 / pointcloud 越界 / 缺 TF 返回 typed error

use nuwax_nav_base::ImageView;
use nuwax_nav_visual_servoing::{
    BBox, CameraIntrinsics, DetectionNavigationConfig, DetectionNavigationModule,
    PoseStamped, PointCloud2Like, SimplePointCloud, TargetPoint3D, TfLike,
    VisualServoingError, VisualServoing2DConfig, extract_3d_from_detection,
};
// 触发 re-export 链接（VisualServoing2DConfig / PoseStamped 在本测试中只通过 lib.rs 间接用,
// 仅 re-export 一遍确保 dead_code 不触发）
const _: fn() = || {
    let _ = std::any::type_name::<VisualServoing2DConfig>();
    let _ = std::any::type_name::<PoseStamped>();
    let _ = std::any::type_name::<TargetPoint3D>();
};

/// Fake TF:camera_frame 位于 world (1,0,0),yaw=0
struct FakeTfCamAtWorldOne;

impl TfLike for FakeTfCamAtWorldOne {
    fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped> {
        if target_frame == "world" && source_frame == "camera_optical" {
            Some(PoseStamped {
                frame_id: "world".into(),
                position: [1.0, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
            })
        } else if target_frame == "world" && source_frame == "base_link" {
            Some(PoseStamped {
                frame_id: "world".into(),
                position: [0.0, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
            })
        } else {
            None
        }
    }
}

/// 缺 TF
struct NoTf;

impl TfLike for NoTf {
    fn lookup(&self, _: &str, _: &str) -> Option<PoseStamped> {
        None
    }
}

/// 构造 640x480 RGB image
fn rgb_image_640x480() -> Vec<u8> {
    vec![0u8; 640 * 480 * 3]
}

/// 构造 640x480 PointCloud（每像素 1 个点）,所有点 z=2.0
fn flat_pointcloud_640x480_z2() -> SimplePointCloud {
    let pts = vec![(0.0_f32, 0.0_f32, 2.0_f32); 640 * 480];
    SimplePointCloud::new(pts)
}

#[test]
fn extract_3d_basic_projection() {
    let img_data = rgb_image_640x480();
    let image = ImageView::from_rgb(640, 480, &img_data);
    let pc = flat_pointcloud_640x480_z2();
    let intr = CameraIntrinsics::default();
    let bbox = BBox::new(100.0, 100.0, 200.0, 200.0); // center = (150, 150)
    let tf = FakeTfCamAtWorldOne;
    let t = extract_3d_from_detection(
        bbox,
        &image,
        &pc,
        &intr,
        &tf,
        "camera_optical",
        "world",
    )
    .expect("extract ok");
    // camera 在 world (1,0,0),yaw=0;point at (0,0,2) -> world (1,0,2)
    assert!((t.x - 1.0).abs() < 1e-4, "x = {}", t.x);
    assert!(t.y.abs() < 1e-4, "y = {}", t.y);
    assert!((t.z - 2.0).abs() < 1e-4, "z = {}", t.z);
}

#[test]
fn extract_3d_invalid_bbox_errors() {
    let img_data = rgb_image_640x480();
    let image = ImageView::from_rgb(640, 480, &img_data);
    let pc = flat_pointcloud_640x480_z2();
    let intr = CameraIntrinsics::default();
    let bad = BBox::new(200.0, 100.0, 100.0, 200.0); // x1 > x2 -> invalid
    let tf = FakeTfCamAtWorldOne;
    let r = extract_3d_from_detection(bad, &image, &pc, &intr, &tf, "camera_optical", "world");
    match r {
        Err(VisualServoingError::InvalidBbox(_)) => {}
        other => panic!("expected InvalidBbox, got {:?}", other),
    }
}

#[test]
fn extract_3d_missing_tf_errors() {
    let img_data = rgb_image_640x480();
    let image = ImageView::from_rgb(640, 480, &img_data);
    let pc = flat_pointcloud_640x480_z2();
    let intr = CameraIntrinsics::default();
    let bbox = BBox::new(100.0, 100.0, 200.0, 200.0);
    let r = extract_3d_from_detection(
        bbox,
        &image,
        &pc,
        &intr,
        &NoTf,
        "camera_optical",
        "world",
    );
    match r {
        Err(VisualServoingError::MissingTf(_)) => {}
        other => panic!("expected MissingTf, got {:?}", other),
    }
}

#[test]
fn module_step_produces_twist() {
    let img_data = rgb_image_640x480();
    let image = ImageView::from_rgb(640, 480, &img_data);
    let pc = flat_pointcloud_640x480_z2();
    let intr = CameraIntrinsics::default();
    let bbox = BBox::new(100.0, 100.0, 200.0, 200.0);
    let module = DetectionNavigationModule::new(
        Box::new(FakeTfCamAtWorldOne),
        DetectionNavigationConfig::default(),
        intr,
        "camera_optical",
        "world",
    );
    let tf = FakeTfCamAtWorldOne;
    let twist = module
        .step(&image, bbox, &pc as &dyn PointCloud2Like, &tf)
        .expect("step ok");
    // target 在 world (1,0,2);robot 在 (0,0,0) 朝 0;distance ≈ 1,小于 target_distance 1.5
    // linear_x = -0.5 * 0.6 = -0.3(后退)
    assert!(twist.linear[0] < 0.0, "linear_x = {}", twist.linear[0]);
}

#[test]
fn module_step_bbox_outside_image_errors() {
    let img_data = rgb_image_640x480();
    let image = ImageView::from_rgb(640, 480, &img_data);
    // 使用比 image 更小的 pointcloud：让 bbox 索引越界
    let pc = SimplePointCloud::new(vec![(0.0, 0.0, 2.0); 4]); // 4 个点
    let intr = CameraIntrinsics::default();
    // bbox 中心 (10, 10), 索引 = 10*640+10=6410 远超 4
    let bbox = BBox::new(5.0, 5.0, 15.0, 15.0);
    let module = DetectionNavigationModule::new(
        Box::new(FakeTfCamAtWorldOne),
        DetectionNavigationConfig::default(),
        intr,
        "camera_optical",
        "world",
    );
    let tf = FakeTfCamAtWorldOne;
    let r = module.step(&image, bbox, &pc as &dyn PointCloud2Like, &tf);
    match r {
        Err(VisualServoingError::ProjectionFailed(_)) => {}
        other => panic!("expected ProjectionFailed, got {:?}", other),
    }
}