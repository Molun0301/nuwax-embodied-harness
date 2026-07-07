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
    /// # Errors
    /// 当链路不可达时返回 `TfError::Disconnected`
    pub fn chain(&self, from: &str, to: &str) -> TfResult<Isometry3<f64>> {
        if from == to {
            return Ok(Isometry3::identity());
        }

        // 沿 from 向上走到 root，记录每一级 (frame, child→parent transform)
        // up_from[0] = (from, transform from → parent(from))
        // up_from[i] = (frame_i, transform frame_i → parent(frame_i))
        // 最后一级是 root（root 没有父变换，用 identity 兜底）
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

        // 沿 to 向上走到 root，同样记录每一级
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

        // 找最近共同祖先：在 up_from 中找到最后一个（最深的）frame 出现在 up_to 集合里。
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

        // 累乘 from → ancestor：每一级 tr 是 child→parent，从 from 开始逐级向上。
        // 但 ancestor 这一级本身不需要应用（ancestor 是目标，不是中间节点）。
        // 所以只取 up_from[..ancestor_idx]。
        let mut t = Isometry3::identity();
        for (_, tr) in up_from.iter().take(ancestor_idx) {
            t = *tr * t;
        }

        // 累乘 ancestor → to：从 ancestor 向下到 to，需要找到 to_frames 中 ancestor 的位置。
        let to_ancestor_idx = match up_to.iter().position(|(f, _)| f == &ancestor) {
            Some(i) => i,
            None => {
                return Err(TfError::Disconnected {
                    from: from.to_string(),
                    to: to.to_string(),
                });
            }
        };
        // up_to[0] = (to, tr_to→parent(to))
        // up_to[to_ancestor_idx] = (ancestor, identity)
        // 向下走：从 ancestor 到 to 的方向是 up_to[to_ancestor_idx - 1], ..., up_to[0]
        // 每级 tr 是 child→parent，但我们向下走要的是 parent→child 的方向……不对。
        // 重新思考：
        //   up_to[k] = (frame_k, tr_k: child=frame_k, parent=parent(frame_k))
        //   所以 frame_k = to, parent(to), grandparent(to), ..., ancestor
        // 向下走 ancestor → to：先到 parent(ancestor)，再到 parent(parent(ancestor))，... 直到 to
        // 即 ancestor 的孩子，ancestor 的孙子，... 直到 to
        // 在 up_to 里：从 up_to[to_ancestor_idx - 1] 开始（这是 ancestor 的孩子），
        // 反向遍历到 up_to[0]（这是 to 本身）。但 up_to[0] = (to, tr: to→parent(to))。
        // 应用 tr：把 point_in_to 映射到 point_in_parent(to)。我们要的是把
        // point_in_parent(to) 映射到 point_in_to，即 inverse。
        // 所以向下走的累乘顺序：从上到下，每个 transform 取 inverse（除了最后一帧是 to 本身……）。
        //
        // 让我们换个思路：up_to[k].tr 是 "frame_k → parent(frame_k)" 的变换。
        // 要从 ancestor 走到 to，等价于依次穿过 ancestor 的 child, grandchild, ..., to。
        // 在 up_to 里这些 child 按 from-child-to-root 的顺序是 up_to[to_ancestor_idx-1], ..., up_to[1]（to 的父）。
        // 但 up_to[0] 是 to 本身，没有"to 的父"这条信息作为 child?不，
        // up_to[0] = (to, tr_to_parent) 表示 "to 帧里的点 映射到 parent(to) 帧"。
        // 所以如果 ancestor 走到 to，要从 ancestor 出发，先 apply 一次 child_ancestor → ancestor 的 inverse，
        // 再 apply 一次 grandchild → child_ancestor 的 inverse，... 直到 to 的父 → to 的 inverse。
        //
        // 等等更简单：在 up_to 中，up_to[k].tr 是 child→parent。等价于
        // `transform that maps a point in frame_k frame to its parent frame`。
        // 我们要 compose 一个变换 T，使得：给定点 p 在 to 帧的坐标，T·p 给出 p 在 ancestor 帧的坐标。
        // p 在 ancestor = T · p 在 to
        //   = (T_{child→parent} 的 inverse) · p 在 child     // 第一次：把 to 的点映到 parent(to)
        //   = (T_{grandchild→child} 的 inverse) · ...        // 第二次：把 parent(to) 的点映到 grandparent
        //   ...
        //   = (T_{ancestor→child(ancestor)} 的 inverse) · p 在 child(ancestor)
        //
        // 嗯，那 up_to 里的每一帧 tr_k = (frame_k → parent(frame_k))。
        // 也就是说 frame_k 的子帧的"父变换"是把子帧点映到 frame_k 的点（即 inverse）。
        //
        // 简化思考：up_to[0]=(to, T_to_parent). T_to_parent 把 to-点映到 parent(to)-点。
        //          up_to[1]=(parent(to), T_parent_to_grandparent). ...
        //          up_to[k]=(frame_k, T_frame_k_to_parent).
        // 要 compose 出一个 T_total：把 to-点映到 ancestor-点。
        //   p_in_parent(to) = T_to_parent · p_in_to
        //   p_in_grandparent = T_parent_to_grandparent · p_in_parent(to)
        //   ...
        //   p_in_ancestor = T_child(ancestor)_to_ancestor · ... · p_in_to
        //
        // 所以从 up_to[0] 开始，到 up_to[to_ancestor_idx - 1] 结束：
        //   T_total = T_child(ancestor)_to_ancestor · ... · T_parent(to)_to_grandparent · T_to_parent
        // 注意：up_to[to_ancestor_idx - 1].tr 是 child(ancestor) → ancestor 的变换，
        //       up_to[to_ancestor_idx].tr 是 ancestor → parent(ancestor) 的变换（identity，因为 ancestor 是 root）
        //
        // 因此 down-chain 累乘顺序是 up_to[0], up_to[1], ..., up_to[to_ancestor_idx - 1]
        // 每次累乘 `t = t * tr_k`（左乘），因为 tr_k 把当前点映到上一级。
        // 实际上更精确：设 T_total_so_far = T_k · ... · T_0（已经能把 to-点映到 frame_k-点）。
        // 现在乘上 T_{k+1}：新 T_total = T_{k+1} · T_k · ... · T_0。代码里 t = tr * t。

        for (_, tr) in up_to.iter().take(to_ancestor_idx) {
            t = *tr * t;
        }

        Ok(t)
    }
}

/// 便捷构造：从平移 + 四元数构造 TransformStamped
#[must_use]
pub fn make_transform(
    parent: impl Into<String>,
    child: impl Into<String>,
    translation: [f64; 3],
    quaternion: [f64; 4], // [w, x, y, z]
) -> TransformStamped {
    let trans = Translation3::new(translation[0], translation[1], translation[2]);
    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3],
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
        let t = make_transform("world", "base_link", [1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
        let m = to_matrix4(&t.transform);
        assert_relative_eq!(m[(0, 3)], 1.0, epsilon = 1e-9);
        assert_relative_eq!(m[(1, 3)], 2.0, epsilon = 1e-9);
        assert_relative_eq!(m[(2, 3)], 3.0, epsilon = 1e-9);
    }
}
