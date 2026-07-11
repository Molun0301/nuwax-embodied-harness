//! nuwax TF 坐标变换树
//! - 时间戳化的坐标变换
//! - 父→子链式查询
//! - 基于 `nalgebra` 4x4 变换矩阵

#![deny(unsafe_code)]
#![warn(missing_docs)]

use nalgebra::{Isometry3, Matrix4, Translation3, UnitQuaternion};
use parking_lot::RwLock;
use std::collections::{HashMap, HashSet};
use thiserror::Error;

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
            transform: Isometry3::identity(),
        }
    }
}

/// TF tree（线程安全，parking_lot RwLock）
pub struct TfTree {
    /// 变换存储（key = child frame）
    transforms: RwLock<HashMap<String, TransformStamped>>,
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
        Self {
            transforms: RwLock::new(HashMap::new()),
        }
    }

    /// 注册一条父→子变换（覆盖）
    pub fn set(&self, t: TransformStamped) {
        let mut g = self.transforms.write();
        g.insert(t.child.clone(), t);
    }

    /// 查询某个子 frame 的直接父变换
    ///
    /// # Errors
    /// 当 frame 不存在时返回 `TfError::FrameNotFound`
    pub fn lookup(&self, child: &str) -> TfResult<TransformStamped> {
        let g = self.transforms.read();
        g.get(child)
            .cloned()
            .ok_or_else(|| TfError::FrameNotFound(child.to_string()))
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
    /// 4. `from → a` 用向上链的反向（即对每条 child→parent 取 inverse 累乘）。
    /// 5. `a → to` 用向下链（即对每条 child→parent 取 inverse 累乘），方向相反。
    /// 6. 总变换 = `(from→a) * (a→to)`。
    ///
    /// # Errors
    /// 当链路不可达时返回 `TfError::Disconnected`
    pub fn chain(&self, from: &str, to: &str) -> TfResult<Isometry3<f64>> {
        if from == to {
            return Ok(Isometry3::identity());
        }

        // 沿 from 向上走到 root，记录每一级 (frame, child→parent transform)
        // up_from[0] = (from, T: from → parent(from))
        // up_from[i] = (frame_i, T: frame_i → parent(frame_i))
        // 最后一级是 root（root 没有父变换，用 identity 兜底）。
        let mut up_from: Vec<(String, Isometry3<f64>)> = Vec::new();
        let mut cursor = from.to_string();
        loop {
            match self.lookup(&cursor) {
                Ok(t) => {
                    let parent = t.parent.clone();
                    up_from.push((cursor.clone(), t.transform));
                    if parent == cursor {
                        break;
                    }
                    cursor = parent;
                }
                Err(TfError::FrameNotFound(_)) => {
                    up_from.push((cursor.clone(), Isometry3::identity()));
                    break;
                }
                Err(e) => return Err(e),
            }
        }

        // 沿 to 向上走到 root，同样记录每一级 (frame, child→parent)。
        let mut up_to: Vec<(String, Isometry3<f64>)> = Vec::new();
        cursor = to.to_string();
        loop {
            match self.lookup(&cursor) {
                Ok(t) => {
                    let parent = t.parent.clone();
                    up_to.push((cursor.clone(), t.transform));
                    if parent == cursor {
                        break;
                    }
                    cursor = parent;
                }
                Err(TfError::FrameNotFound(_)) => {
                    up_to.push((cursor.clone(), Isometry3::identity()));
                    break;
                }
                Err(e) => return Err(e),
            }
        }

        // 找最近共同祖先：up_from 中最深的（rposition）也出现在 up_to 集合里。
        let to_frames: HashSet<&String> = up_to.iter().map(|(f, _)| f).collect();
        let ancestor_idx = match up_from.iter().rposition(|(f, _)| to_frames.contains(f)) {
            Some(i) => i,
            None => {
                return Err(TfError::Disconnected {
                    from: from.to_string(),
                    to: to.to_string(),
                });
            }
        };
        let ancestor = up_from[ancestor_idx].0.clone();

        // 累乘 from → ancestor：
        // `Isometry3::from_parts(trans, q)` 是 body→parent 的方向（即把 child-frame 的
        // 点映到 parent-frame），其 translation 是 body 在 parent 中的位置。
        // 因此树上存的 tr 直接就是把 child-frame 点映到 parent-frame 的变换。
        // 沿 up_from 向上走时，每一级 tr 正是我们要的方向（child→parent），直接累乘即可。
        let from_to_ancestor = up_from
            .iter()
            .take(ancestor_idx)
            .fold(Isometry3::identity(), |acc, (_, tr)| tr * acc);

        // 累乘 ancestor → to：
        // 现在我们要从 ancestor（parent）下到 to（child），方向与存储的 tr 相反，
        // 因此每级取 inverse。
        // up_to[0] = (to, tr: to→parent(to))，
        // up_to[k] = (frame_k, tr: frame_k→parent(frame_k))。
        // up_to[to_ancestor_idx] = (ancestor, identity)；to 自身在 up_to[0]。
        // 路径 ancestor → to 依次经过 ancestor 的 child → ... → to 的父 → to 自身。
        // 在 up_to 中按从 to 到 ancestor 的顺序：up_to[0]=to, ..., up_to[to_ancestor_idx]=ancestor。
        // 反向遍历：up_to[to_ancestor_idx - 1]（ancestor 的 child）→ ... → up_to[0]（to 本身），
        // 每级 tr 是 child→parent 方向（向上），取 inverse 才是 ancestor→child 方向（向下）。
        let to_ancestor_idx = match up_to.iter().position(|(f, _)| f == &ancestor) {
            Some(i) => i,
            None => {
                return Err(TfError::Disconnected {
                    from: from.to_string(),
                    to: to.to_string(),
                });
            }
        };
        let ancestor_to_to = (0..to_ancestor_idx)
            .rev()
            .fold(Isometry3::identity(), |acc, k| {
                let tr = up_to[k].1;
                tr.inverse() * acc
            });

        // 总变换：from → ancestor → to
        Ok(from_to_ancestor * ancestor_to_to)
    }
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
        let m_wxyz = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            q_wxyz_unit,
        )
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
}
