//! 回环检测可视化工具
//! **AprilTag 部分仅占位**——实装在 `nuwax-vision-fiducial` crate。
//! 本模块仅提供 trait 抽象 + 占位实现。

/// 回环标记可视化器（占位）
///
/// 实装需要：
/// 1. 引用 `nuwax-vision-fiducial::AprilTagDetector`
/// 2. 与 `nuwax-visualization-rerun` 集成输出 .rrd
///
/// 当前仅保留 trait 与占位实现，编译可过、运行可空。
pub struct LoopClosureMarkers;

impl LoopClosureMarkers {
    /// 占位：从图像路径检测 AprilTag marker（需 nuwax-vision-fiducial 集成）
    #[must_use]
    pub fn detect_markers_placeholder(image_path: &str) -> Vec<u32> {
        // TODO: 实装时调用 nuwax_vision_fiducial::AprilTagDetector::detect(image)
        let _ = image_path;
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_markers_placeholder_returns_empty() {
        let markers = LoopClosureMarkers::detect_markers_placeholder("dummy.png");
        assert!(markers.is_empty());
    }
}