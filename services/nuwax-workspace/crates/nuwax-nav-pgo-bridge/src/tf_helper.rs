//! PGO TF 适配模块（独立可测试）
//!
//! 订阅 native `pgo_tf` topic（消息类型与 native 一致，由调用方从 LCM/transport
//! 反序列化得到），转换为 [`TransformStamped`] 并写入 [`TfTree`]。
//!
//! 该模块**不**依赖 `NativeRunner`，**不**藏在 runner 内。可以独立 `cargo test`
//! 验证几何转换与 TF tree 更新行为。

use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use serde::{Deserialize, Serialize};

/// 与 native `pgo_tf` topic 消息体对应的本地结构。
///
/// 当前实现采用 native Odometry 体语义（position + orientation + stamp），
/// 与 Python `pgo.py` 中对 `pgo_tf` 订阅消息的字段提取保持一致。
/// 若 native 实际输出 `TransformStamped`，只需替换此 struct 而不影响 [`PgoBridge`] API。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PgoTfMessage {
    /// 时间戳（epoch 秒），与 nuwax-tf `TransformStamped::stamp_secs` 对齐
    pub stamp_secs: i64,
    /// 平移 (x, y, z)
    pub translation: [f64; 3],
    /// 四元数 (x, y, z, w) —— **注意顺序：xyzw 不是 wxyz**
    pub orientation_xyzw: [f64; 4],
}

/// 把 [`PgoTfMessage`] 转换为 `nuwax_tf::TransformStamped`。
///
/// `parent` 通常是 `world_frame`（map），`child` 是 `local_frame`（odom）。
///
/// 与 Python `pgo.py::_publish_tf` 等价。
#[must_use]
pub fn pgo_tf_to_transform_stamped(
    msg: &PgoTfMessage,
    parent: &str,
    child: &str,
) -> nuwax_tf::TransformStamped {
    let [x, y, z] = msg.translation;
    let trans = Translation3::new(x, y, z);
    let [qx, qy, qz, qw] = msg.orientation_xyzw;
    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(qw, qx, qy, qz));
    let transform = Isometry3::from_parts(trans, q);
    nuwax_tf::TransformStamped {
        parent: parent.to_string(),
        child: child.to_string(),
        stamp_secs: msg.stamp_secs,
        transform,
    }
}

/// identity seed TF（与 Python `start()` 中 `_publish_tf(translation=(0,0,0), rotation=(0,0,0,1))` 等价）。
#[must_use]
pub fn identity_pgo_tf(parent: &str, child: &str, stamp_secs: i64) -> nuwax_tf::TransformStamped {
    let msg = PgoTfMessage {
        stamp_secs,
        translation: [0.0, 0.0, 0.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };
    pgo_tf_to_transform_stamped(&msg, parent, child)
}

/// 把 [`PgoTfMessage`] 写入 [`TfTree`]。
///
/// 这是订阅侧的标准入口；测试可以直接调用验证 TF 更新。
pub fn apply_pgo_tf_to_tree(
    tree: &nuwax_tf::TfTree,
    msg: &PgoTfMessage,
    parent: &str,
    child: &str,
) {
    let t = pgo_tf_to_transform_stamped(msg, parent, child);
    tree.set(t);
}

/// 与 native Odometry 消息体对应的本地结构（订阅侧常见）。
///
/// `pgo_tf` 实际是 Odometry 类型消息；该 helper 便于已有 transport layer 直接转换。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OdometryLike {
    /// 时间戳（epoch 秒）
    pub stamp_secs: i64,
    /// 平移 (x, y, z)
    pub position: [f64; 3],
    /// 四元数 (x, y, z, w)
    pub orientation_xyzw: [f64; 4],
}

impl From<&OdometryLike> for PgoTfMessage {
    fn from(o: &OdometryLike) -> Self {
        Self {
            stamp_secs: o.stamp_secs,
            translation: o.position,
            orientation_xyzw: o.orientation_xyzw,
        }
    }
}

/// 把 `OdometryLike`（订阅侧消息）转换为 [`PgoTfMessage`]，便于统一 TF 更新路径。
///
/// 这是 [`odometry_to_transform_stamped`] 与 [`apply_pgo_tf_to_tree`] 的串接。
#[must_use]
pub fn odometry_to_transform_stamped(
    odom: &OdometryLike,
    parent: &str,
    child: &str,
) -> nuwax_tf::TransformStamped {
    let msg: PgoTfMessage = odom.into();
    pgo_tf_to_transform_stamped(&msg, parent, child)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn identity_pgo_tf_zero_translation_unit_quaternion() {
        let t = identity_pgo_tf("map", "odom", 123);
        assert_eq!(t.parent, "map");
        assert_eq!(t.child, "odom");
        assert_eq!(t.stamp_secs, 123);
        assert_relative_eq!(t.transform.translation.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(t.transform.translation.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(t.transform.translation.z, 0.0, epsilon = 1e-9);
        let q = t.transform.rotation;
        assert_relative_eq!(q.w, 1.0, epsilon = 1e-9);
        assert_relative_eq!(q.i, 0.0, epsilon = 1e-9);
        assert_relative_eq!(q.j, 0.0, epsilon = 1e-9);
        assert_relative_eq!(q.k, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn pgo_tf_to_transform_stamped_translates_and_rotates() {
        let msg = PgoTfMessage {
            stamp_secs: 100,
            translation: [1.0, 2.0, 3.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0], // 单位四元数 xyzw
        };
        let t = pgo_tf_to_transform_stamped(&msg, "map", "odom");
        assert_eq!(t.parent, "map");
        assert_eq!(t.child, "odom");
        assert_eq!(t.stamp_secs, 100);
        assert_relative_eq!(t.transform.translation.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(t.transform.translation.y, 2.0, epsilon = 1e-9);
        assert_relative_eq!(t.transform.translation.z, 3.0, epsilon = 1e-9);
        let q = t.transform.rotation;
        assert_relative_eq!(q.w, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn odometry_to_transform_stamped_passthrough() {
        let odom = OdometryLike {
            stamp_secs: 200,
            position: [4.0, 5.0, 6.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        };
        let t = odometry_to_transform_stamped(&odom, "map", "odom");
        assert_eq!(t.stamp_secs, 200);
        assert_relative_eq!(t.transform.translation.x, 4.0, epsilon = 1e-9);
        assert_relative_eq!(t.transform.translation.y, 5.0, epsilon = 1e-9);
        assert_relative_eq!(t.transform.translation.z, 6.0, epsilon = 1e-9);
    }

    #[test]
    fn apply_pgo_tf_to_tree_updates_lookup() {
        let tree = nuwax_tf::TfTree::new();
        // seed identity
        apply_pgo_tf_to_tree(
            &tree,
            &PgoTfMessage {
                stamp_secs: 0,
                translation: [0.0, 0.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            "map",
            "odom",
        );
        let before = tree.lookup("odom").unwrap();
        assert_relative_eq!(before.transform.translation.x, 0.0, epsilon = 1e-9);

        // 后续 PGO 修正
        apply_pgo_tf_to_tree(
            &tree,
            &PgoTfMessage {
                stamp_secs: 1,
                translation: [0.5, 0.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            "map",
            "odom",
        );
        let after = tree.lookup("odom").unwrap();
        assert_eq!(after.stamp_secs, 1);
        assert_relative_eq!(after.transform.translation.x, 0.5, epsilon = 1e-9);
    }

    #[test]
    fn apply_pgo_tf_seed_then_chain_visible() {
        let tree = nuwax_tf::TfTree::new();
        // seed identity map -> odom
        apply_pgo_tf_to_tree(
            &tree,
            &PgoTfMessage {
                stamp_secs: 0,
                translation: [0.0, 0.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            "map",
            "odom",
        );
        // 同时设置 odom -> base_link（外部来源；这里直接构造）。
        // `make_transform` 采用 XYZW 顺序，与 `orientation_xyzw` 字段约定一致；
        // 故此处使用 xyzw 单位四元数 `[0, 0, 0, 1]`。
        let base_link =
            nuwax_tf::make_transform("odom", "base_link", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set(base_link);

        // 链语义(nalgebra Isometry / ROS tf2 约定):
        //   `chain("a", "b")` 返回 `T_a→b`(把 a-frame 点映到 b-frame)。
        //   `T_a→b.translation` = "a origin 在 b 中的坐标"。
        // 测试场景:base_link origin 在 map 中是 (1, 0, 0),所以
        //   map origin 在 base_link 中是 (-1, 0, 0),即 chain.translation = (-1, 0, 0)。
        // (旧版期望 +1 是 ROS tf 风格 `T_target→source` 反着用,在 rework22 P0-05
        //  算法语义下被修正。)
        let chain = tree.chain("map", "base_link").unwrap();
        assert_relative_eq!(chain.translation.x, -1.0, epsilon = 1e-9);
        assert_relative_eq!(chain.translation.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(chain.translation.z, 0.0, epsilon = 1e-9);
    }
}
