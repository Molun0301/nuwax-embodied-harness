//! nuwax TF 坐标变换树
//! - 时间戳化的坐标变换
//! - 父→子链式查询
//! - 基于 `nalgebra` 4x4 变换矩阵

#![deny(unsafe_code)]
#![warn(missing_docs)]

use nalgebra::{Isometry3, Matrix4, Translation3, UnitQuaternion};
use parking_lot::{Condvar, Mutex};
use std::collections::{BTreeSet, HashMap, HashSet};
use std::time::{Duration, Instant};
use thiserror::Error;

const DEFAULT_BUFFER_SIZE_SECS: f64 = 10.0;

/// Real-LCM `/tf` publisher/subscriber adapter.
#[cfg(feature = "link-liblcm")]
pub mod lcm;

/// TF 错误
#[derive(Debug, Error)]
pub enum TfError {
    /// 坐标系不存在
    #[error("frame '{0}' not found")]
    FrameNotFound(String),
    /// 链式变换不连通
    #[error("no transform path between '{from}' and '{to}'")]
    Disconnected {
        /// 来源坐标系
        from: String,
        /// 目标坐标系
        to: String,
    },
    /// 指定时间附近没有落在容差内的变换
    #[error(
        "no transform for frame '{child}' near {requested_secs:.9}s within {tolerance_secs:.9}s; closest stamp is {closest_stamp_secs}.{closest_stamp_nanos:09}s"
    )]
    OutsideTolerance {
        /// 查询的 child frame
        child: String,
        /// 查询时间（epoch 秒）
        requested_secs: f64,
        /// 最大允许时间差（秒）
        tolerance_secs: f64,
        /// 实际最接近的 transform 时间戳
        closest_stamp_secs: i64,
        /// 实际最接近的 transform 纳秒部分
        closest_stamp_nanos: u32,
    },
    /// 时间查询参数非法
    #[error("invalid transform time query: time={time_secs}, tolerance={tolerance_secs}")]
    InvalidTimeQuery {
        /// 查询时间（epoch 秒）
        time_secs: f64,
        /// 时间容差（秒）
        tolerance_secs: f64,
    },
    /// 等待 transform 链补齐超时
    #[error("timed out waiting for transform path between '{from}' and '{to}'")]
    WaitTimeout {
        /// 来源坐标系
        from: String,
        /// 目标坐标系
        to: String,
    },
}

/// TF 结果
pub type TfResult<T> = Result<T, TfError>;

/// 单条变换（父→子，时间戳化）
#[derive(Debug, Clone)]
pub struct TransformStamped {
    /// 父坐标系
    pub parent: String,
    /// 子坐标系
    pub child: String,
    /// 时间戳（epoch 秒）
    pub stamp_secs: i64,
    /// 时间戳纳秒部分
    pub stamp_nanos: u32,
    /// 平移 + 旋转（SE3）
    pub transform: Isometry3<f64>,
}

impl TransformStamped {
    /// 构造默认值
    #[must_use]
    pub fn identity(parent: impl Into<String>, child: impl Into<String>) -> Self {
        Self {
            parent: parent.into(),
            child: child.into(),
            stamp_secs: 0,
            stamp_nanos: 0,
            transform: Isometry3::identity(),
        }
    }
}

/// TF tree（线程安全，parking_lot RwLock）
pub struct TfTree {
    /// 按时间升序保存的变换历史（key = child frame）
    transforms: Mutex<HashMap<String, Vec<TransformStamped>>>,
    buffer_size_secs: f64,
    updated: Condvar,
}

impl Default for TfTree {
    fn default() -> Self {
        Self::new()
    }
}

impl TfTree {
    /// 构造空 TF tree
    #[must_use]
    pub fn new() -> Self {
        Self::with_buffer_size(DEFAULT_BUFFER_SIZE_SECS)
    }

    /// 构造带指定历史窗口的 TF tree。
    ///
    /// 非有限或负数配置回退到原版默认的 10 秒窗口。
    #[must_use]
    pub fn with_buffer_size(buffer_size_secs: f64) -> Self {
        Self {
            transforms: Mutex::new(HashMap::new()),
            buffer_size_secs: if buffer_size_secs.is_finite() && buffer_size_secs >= 0.0 {
                buffer_size_secs
            } else {
                DEFAULT_BUFFER_SIZE_SECS
            },
            updated: Condvar::new(),
        }
    }

    /// 注册一条父→子变换并保留历史窗口。
    pub fn set(&self, t: TransformStamped) {
        let mut g = self.transforms.lock();
        let history = g.entry(t.child.clone()).or_default();
        match history.binary_search_by_key(&(t.stamp_secs, t.stamp_nanos), |item| {
            (item.stamp_secs, item.stamp_nanos)
        }) {
            Ok(index) => history[index] = t,
            Err(index) => history.insert(index, t),
        }

        let newest_stamp = history.last().map_or(0.0, timestamp_secs);
        let oldest_retained = newest_stamp - self.buffer_size_secs;
        let first_retained = history.partition_point(|item| timestamp_secs(item) < oldest_retained);
        if first_retained > 0 {
            history.drain(..first_retained);
        }
        drop(g);
        self.updated.notify_all();
    }

    /// 查询某个子 frame 的直接父变换
    ///
    /// # Errors
    /// 当 frame 不存在时返回 `TfError::FrameNotFound`
    pub fn lookup(&self, child: &str) -> TfResult<TransformStamped> {
        let g = self.transforms.lock();
        lookup_in(&g, child, None)
    }

    /// 查询指定时间附近最接近的一条直接父变换。
    ///
    /// 等距时选择较晚的时间戳，与原版 `InMemoryStore.find_closest` 一致。
    ///
    /// # Errors
    /// frame 不存在、参数非法或最接近帧超出容差时返回 typed error。
    pub fn lookup_at(
        &self,
        child: &str,
        time_secs: f64,
        tolerance_secs: f64,
    ) -> TfResult<TransformStamped> {
        validate_time_query(time_secs, tolerance_secs)?;
        let g = self.transforms.lock();
        lookup_in(&g, child, Some((time_secs, tolerance_secs)))
    }

    /// 使用采集时刻前后的位姿插值；不把最新位姿当成过去或未来的观测位姿。
    ///
    /// # Errors
    /// 没有包围采集时刻的历史，或两帧父坐标不同，返回现有坐标查询错误。
    pub fn lookup_interpolated_at(
        &self,
        child: &str,
        time_secs: f64,
    ) -> TfResult<TransformStamped> {
        validate_time_query(time_secs, 0.0)?;
        let histories = self.transforms.lock();
        let history = histories.get(child).filter(|h| !h.is_empty())
            .ok_or_else(|| TfError::FrameNotFound(child.into()))?;
        let next = history.partition_point(|entry| timestamp_secs(entry) < time_secs);
        if history.get(next).is_some_and(|entry| timestamp_secs(entry) == time_secs) {
            return Ok(history[next].clone());
        }
        let (Some(previous), Some(after)) = (next.checked_sub(1), history.get(next)) else {
            return lookup_in(&histories, child, Some((time_secs, 0.0)));
        };
        let before = &history[previous];
        if before.parent != after.parent {
            return Err(TfError::Disconnected { from: before.parent.clone(), to: after.parent.clone() });
        }
        let ratio = (time_secs - timestamp_secs(before))
            / (timestamp_secs(after) - timestamp_secs(before));
        let position = before.transform.translation.vector.lerp(&after.transform.translation.vector, ratio);
        let rotation = before.transform.rotation.slerp(&after.transform.rotation, ratio);
        let stamp_ns = (time_secs * 1e9).round() as i64;
        Ok(TransformStamped {
            parent: before.parent.clone(), child: child.into(),
            stamp_secs: stamp_ns.div_euclid(1_000_000_000),
            stamp_nanos: stamp_ns.rem_euclid(1_000_000_000) as u32,
            transform: Isometry3::from_parts(Translation3::from(position), rotation),
        })
    }

    /// 返回某个 child frame 当前保留的历史帧数。
    #[must_use]
    pub fn history_len(&self, child: &str) -> usize {
        self.transforms.lock().get(child).map_or(0, Vec::len)
    }

    /// 返回已知的 parent/child frame 集合。
    #[must_use]
    pub fn frames(&self) -> BTreeSet<String> {
        let g = self.transforms.lock();
        let mut frames = BTreeSet::new();
        for (child, history) in g.iter() {
            frames.insert(child.clone());
            if let Some(latest) = history.last() {
                frames.insert(latest.parent.clone());
            }
        }
        frames
    }

    /// 链式查询：从 `from` 到 `to` 的总变换（向上回溯到共同祖先再向下）
    ///
    /// 语义：返回变换 `T`，使得 `T · p_in_from == p_in_to`，即
    /// 把 `from` 坐标系下的点变换到 `to` 坐标系。
    ///
    /// 算法：
    /// 1. 沿 `from` 向上回溯到 root，记录每一级 `(frame, child→parent)`。
    /// 2. 沿 `to` 向上回溯到 root，记录每一级 `(frame, child→parent)`。
    /// 3. 找最近共同祖先 `a`。
    /// 4. `from → a` 沿向上链直接累乘每条 child→parent 变换。
    /// 5. `a → to` 反向遍历向上链，并对每条 child→parent 变换取 inverse。
    /// 6. 总变换 = `(a→to) * (from→a)`（矩阵作用顺序从右到左）。
    ///
    /// # Errors
    /// 当链路不可达时返回 `TfError::Disconnected`
    pub fn chain(&self, from: &str, to: &str) -> TfResult<Isometry3<f64>> {
        let g = self.transforms.lock();
        chain_in(&g, from, to, None)
    }

    /// 在指定时间点查询整条 transform 链。
    ///
    /// 每条边都使用相同的 `time_secs` 和 `tolerance_secs` 选择历史帧。
    ///
    /// # Errors
    /// 参数非法、任一边超出时间容差或链路不可达时返回 typed error。
    pub fn chain_at(
        &self,
        from: &str,
        to: &str,
        time_secs: f64,
        tolerance_secs: f64,
    ) -> TfResult<Isometry3<f64>> {
        validate_time_query(time_secs, tolerance_secs)?;
        let g = self.transforms.lock();
        chain_in(&g, from, to, Some((time_secs, tolerance_secs)))
    }

    /// 等待指定时间的 transform 链在 `forward_tolerance` 内补齐。
    ///
    /// `set` 会唤醒所有 waiter；查询与等待共享同一互斥区，避免错过通知。
    ///
    /// # Errors
    /// 参数非法，或到期时链仍不可用，返回 typed error。
    pub fn wait_chain_at(
        &self,
        from: &str,
        to: &str,
        time_secs: f64,
        tolerance_secs: f64,
        forward_tolerance: Duration,
    ) -> TfResult<Isometry3<f64>> {
        validate_time_query(time_secs, tolerance_secs)?;
        let deadline = Instant::now() + forward_tolerance;
        let mut g = self.transforms.lock();
        loop {
            if let Ok(transform) = chain_in(&g, from, to, Some((time_secs, tolerance_secs))) {
                return Ok(transform);
            }
            let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                return Err(TfError::WaitTimeout {
                    from: from.to_string(),
                    to: to.to_string(),
                });
            };
            if remaining.is_zero() {
                return Err(TfError::WaitTimeout {
                    from: from.to_string(),
                    to: to.to_string(),
                });
            }
            self.updated.wait_for(&mut g, remaining);
        }
    }
}

fn validate_time_query(time_secs: f64, tolerance_secs: f64) -> TfResult<()> {
    if time_secs.is_finite() && tolerance_secs.is_finite() && tolerance_secs >= 0.0 {
        Ok(())
    } else {
        Err(TfError::InvalidTimeQuery {
            time_secs,
            tolerance_secs,
        })
    }
}

fn timestamp_secs(transform: &TransformStamped) -> f64 {
    transform.stamp_secs as f64 + f64::from(transform.stamp_nanos) * 1e-9
}

fn lookup_in(
    transforms: &HashMap<String, Vec<TransformStamped>>,
    child: &str,
    query: Option<(f64, f64)>,
) -> TfResult<TransformStamped> {
    let history = transforms
        .get(child)
        .filter(|history| !history.is_empty())
        .ok_or_else(|| TfError::FrameNotFound(child.to_string()))?;
    let Some((time_secs, tolerance_secs)) = query else {
        return history
            .last()
            .cloned()
            .ok_or_else(|| TfError::FrameNotFound(child.to_string()));
    };

    let next_index = history.partition_point(|item| timestamp_secs(item) < time_secs);
    let closest = match (next_index.checked_sub(1), history.get(next_index)) {
        (Some(previous_index), Some(next)) => {
            let previous = &history[previous_index];
            let previous_delta = (timestamp_secs(previous) - time_secs).abs();
            let next_delta = (timestamp_secs(next) - time_secs).abs();
            if previous_delta < next_delta {
                previous
            } else {
                next
            }
        }
        (Some(previous_index), None) => &history[previous_index],
        (None, Some(next)) => next,
        (None, None) => return Err(TfError::FrameNotFound(child.to_string())),
    };
    if (timestamp_secs(closest) - time_secs).abs() > tolerance_secs {
        return Err(TfError::OutsideTolerance {
            child: child.to_string(),
            requested_secs: time_secs,
            tolerance_secs,
            closest_stamp_secs: closest.stamp_secs,
            closest_stamp_nanos: closest.stamp_nanos,
        });
    }
    Ok(closest.clone())
}

fn chain_in(
    transforms: &HashMap<String, Vec<TransformStamped>>,
    from: &str,
    to: &str,
    query: Option<(f64, f64)>,
) -> TfResult<Isometry3<f64>> {
    if from == to {
        return Ok(Isometry3::identity());
    }

    let up_from = upward_chain(transforms, from, query)?;
    let up_to = upward_chain(transforms, to, query)?;
    let to_frames: HashSet<&String> = up_to.iter().map(|(frame, _)| frame).collect();
    let ancestor_idx = up_from
        .iter()
        .rposition(|(frame, _)| to_frames.contains(frame))
        .ok_or_else(|| TfError::Disconnected {
            from: from.to_string(),
            to: to.to_string(),
        })?;
    let ancestor = &up_from[ancestor_idx].0;
    let from_to_ancestor = up_from
        .iter()
        .take(ancestor_idx)
        .fold(Isometry3::identity(), |acc, (_, transform)| transform * acc);
    let to_ancestor_idx = up_to
        .iter()
        .position(|(frame, _)| frame == ancestor)
        .ok_or_else(|| TfError::Disconnected {
            from: from.to_string(),
            to: to.to_string(),
        })?;
    let ancestor_to_to = (0..to_ancestor_idx)
        .rev()
        .fold(Isometry3::identity(), |acc, index| {
            up_to[index].1.inverse() * acc
        });
    Ok(ancestor_to_to * from_to_ancestor)
}

fn upward_chain(
    transforms: &HashMap<String, Vec<TransformStamped>>,
    start: &str,
    query: Option<(f64, f64)>,
) -> TfResult<Vec<(String, Isometry3<f64>)>> {
    let mut chain = Vec::new();
    let mut cursor = start.to_string();
    let mut visited = HashSet::new();
    loop {
        if !visited.insert(cursor.clone()) {
            return Err(TfError::Disconnected {
                from: start.to_string(),
                to: cursor,
            });
        }
        match lookup_in(transforms, &cursor, query) {
            Ok(transform) => {
                let parent = transform.parent.clone();
                chain.push((cursor.clone(), transform.transform));
                if parent == cursor {
                    break;
                }
                cursor = parent;
            }
            Err(TfError::FrameNotFound(_)) => {
                chain.push((cursor, Isometry3::identity()));
                break;
            }
            Err(error) => return Err(error),
        }
    }
    Ok(chain)
}

/// 便捷构造：从平移 + 四元数构造 TransformStamped
///
/// `quaternion` 采用 XYZW 顺序（与 geometry_msgs/Pose、`dimos.Quaternion(x, y, z, w)`、
/// scipy `R.from_quat` 一致），即 `quaternion = [x, y, z, w]`。
/// 注：nalgebra 内部使用 WXYZ 顺序，本函数在传入时按 WXYZ 重排。
#[must_use]
pub fn make_transform(
    parent: impl Into<String>,
    child: impl Into<String>,
    translation: [f64; 3],
    quaternion: [f64; 4], // [x, y, z, w] (XYZW，与 dimos/Python/geometry_msgs 一致)
) -> TransformStamped {
    let trans = Translation3::new(translation[0], translation[1], translation[2]);
    // XYZW 输入 → nalgebra 内部 WXYZ：交换位置 3 (w) 到第一位
    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        quaternion[3], // w
        quaternion[0], // x
        quaternion[1], // y
        quaternion[2], // z
    ));
    TransformStamped {
        parent: parent.into(),
        child: child.into(),
        stamp_secs: 0,
        stamp_nanos: 0,
        transform: Isometry3::from_parts(trans, q),
    }
}

/// 便捷读取：从 transform 提取 4x4 矩阵
#[must_use]
pub fn to_matrix4(t: &Isometry3<f64>) -> Matrix4<f64> {
    t.to_matrix()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn set_and_lookup() {
        let tree = TfTree::new();
        tree.set(TransformStamped::identity("world", "base_link"));
        let t = tree.lookup("base_link").unwrap();
        assert_eq!(t.parent, "world");
        assert_eq!(t.child, "base_link");
    }

    #[test]
    fn chain_through_tree() {
        let tree = TfTree::new();
        tree.set(TransformStamped::identity("world", "base_link"));
        tree.set(TransformStamped::identity("base_link", "lidar"));

        let t = tree.chain("world", "lidar").unwrap();
        assert_relative_eq!(t.translation.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(t.translation.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(t.translation.z, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn chain_self_is_identity() {
        let tree = TfTree::new();
        let t = tree.chain("world", "world").unwrap();
        assert_relative_eq!(t.translation.x, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn chain_disconnected() {
        let tree = TfTree::new();
        tree.set(TransformStamped::identity("world", "base_link"));
        // 没有 camera 变换
        let err = tree.chain("base_link", "camera").unwrap_err();
        assert!(matches!(err, TfError::Disconnected { .. }));
    }

    #[test]
    fn make_transform_extract_matrix() {
        let t = make_transform("world", "base_link", [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
        let m = to_matrix4(&t.transform);
        assert_relative_eq!(m[(0, 3)], 1.0, epsilon = 1e-9);
        assert_relative_eq!(m[(1, 3)], 2.0, epsilon = 1e-9);
        assert_relative_eq!(m[(2, 3)], 3.0, epsilon = 1e-9);
    }

    /// 回归 B5:验证 `make_transform` 采用 XYZW 顺序（与 dimos/Python `Quaternion(x,y,z,w)` 一致）。
    /// 绕 Z 轴 90° 的真值四元数为 `xyzw = [0, 0, √2/2, √2/2]`。
    /// 把该四元数喂给本函数后，把 `(1, 0, 0)` 旋转应当得到 `(0, 1, 0)`。
    #[test]
    fn make_transform_quaternion_is_xyzw_z_axis_90deg() {
        let s = 0.5_f64.sqrt();
        let t = make_transform("world", "base_link", [0.0, 0.0, 0.0], [0.0, 0.0, s, s]);
        let m = to_matrix4(&t.transform);
        // 旋转矩阵部分（线性列）：把 (1, 0, 0) 映射到 (0, 1, 0)
        assert_relative_eq!(m[(0, 0)], 0.0, epsilon = 1e-9);
        assert_relative_eq!(m[(1, 0)], 1.0, epsilon = 1e-9);
        assert_relative_eq!(m[(2, 0)], 0.0, epsilon = 1e-9);
        // 把 (0, 1, 0) 映射到 (-1, 0, 0)
        assert_relative_eq!(m[(0, 1)], -1.0, epsilon = 1e-9);
        assert_relative_eq!(m[(1, 1)], 0.0, epsilon = 1e-9);
        assert_relative_eq!(m[(2, 1)], 0.0, epsilon = 1e-9);
        // 把 (0, 0, 1) 映射到 (0, 0, 1)
        assert_relative_eq!(m[(0, 2)], 0.0, epsilon = 1e-9);
        assert_relative_eq!(m[(1, 2)], 0.0, epsilon = 1e-9);
        assert_relative_eq!(m[(2, 2)], 1.0, epsilon = 1e-9);
    }

    /// 回归 B5:用同一个字节组 `[a, b, c, d]` 喂入两种语义解释（XYZW vs WXYZ）
    /// 时，应该得到明显不同的旋转矩阵。本测试用来防止有人把 `make_transform`
    /// 的入参顺序改回 WXYZ 后导致静默错位。
    #[test]
    fn make_transform_quaternion_order_semantics() {
        // 用一个非平凡、未归一化的四元数对（nalgebra 的 UnitQuaternion 会自动归一化）。
        let q = [0.1_f64, 0.2, 0.3, 0.4];
        // 修复后路径：XYZW 解释 → nalgebra 收到 (w=0.4, x=0.1, y=0.2, z=0.3)
        let t_xyzw = make_transform("world", "base_link", [0.0, 0.0, 0.0], q);
        let m_xyzw = to_matrix4(&t_xyzw.transform);
        // 修复前路径：WXYZ 解释（同一组字节被读作 wxyz）→ nalgebra 收到 (w=0.1, x=0.2, y=0.3, z=0.4)
        let q_wxyz_unit = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            q[0], q[1], q[2], q[3],
        ));
        let m_wxyz =
            nalgebra::Isometry3::from_parts(nalgebra::Translation3::identity(), q_wxyz_unit)
                .to_matrix();
        // Manhattan 距离判定两种解释产生显著不同的旋转矩阵
        let mut diff = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                diff += (m_wxyz[(i, j)] - m_xyzw[(i, j)]).abs();
            }
        }
        assert!(
            diff > 0.1,
            "WXYZ 与 XYZW 解释应当产生显著不同的旋转矩阵；diff={diff}，疑似顺序混淆"
        );
    }

    /// 对齐 dimos `test_tf_ros_example`：旋转后的父子平移不交换。
    #[test]
    fn chain_composes_rotated_translation_in_path_order() {
        let tree = TfTree::new();
        let half_angle = core::f64::consts::FRAC_PI_6 / 2.0;
        tree.set(make_transform(
            "base_link",
            "arm",
            [1.0, -1.0, 0.0],
            [0.0, 0.0, half_angle.sin(), half_angle.cos()],
        ));
        tree.set(make_transform(
            "arm",
            "end_effector",
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ));

        // TransformStamped stores child -> parent. This is the same physical pose returned by
        // dimos `get("base_link", "end_effector")`, expressed as a point transform end -> base.
        let end_to_base = tree.chain("end_effector", "base_link").unwrap();
        assert_relative_eq!(end_to_base.translation.x, 1.366_025_403_784, epsilon = 1e-9);
        assert_relative_eq!(end_to_base.translation.y, 0.366_025_403_784, epsilon = 1e-9);
        assert_relative_eq!(end_to_base.translation.z, 0.0, epsilon = 1e-9);

        let base_to_end = tree.chain("base_link", "end_effector").unwrap();
        let roundtrip = end_to_base * base_to_end;
        assert_relative_eq!(roundtrip.translation.vector.norm(), 0.0, epsilon = 1e-9);
        assert_relative_eq!(roundtrip.rotation.angle(), 0.0, epsilon = 1e-9);
    }
    #[test]
    fn interpolated_capture_pose_preserves_translation_and_rotation() {
        let tree = TfTree::new();
        let mut before = TransformStamped::identity("odom", "body");
        before.stamp_secs = 10;
        let mut after = before.clone();
        after.stamp_secs = 12;
        after.transform = Isometry3::from_parts(Translation3::new(2.0, 0.0, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI / 2.0));
        tree.set(before); tree.set(after);
        let actual = tree.lookup_interpolated_at("body", 11.0).unwrap();
        assert_relative_eq!(actual.transform.translation.x, 1.0, epsilon=1e-12);
        assert_relative_eq!(actual.transform.rotation.angle(), std::f64::consts::PI / 4.0, epsilon=1e-12);
        assert!(tree.lookup_interpolated_at("body", 9.9).is_err());
        assert!(tree.lookup_interpolated_at("body", 12.1).is_err());
    }

}
