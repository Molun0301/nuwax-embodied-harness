//! 启动入口已物化的固定碰撞几何；字段直接对应现有 self_collision JSON。
use serde::{Deserialize, Serialize};

/// 与机体基准刚性连接的模型 collision，保留未支持及活动关节统计。
/// 该数据不是导航凸外包，也不推断未提供的躯干或实时腿部状态。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FixedCollisionGeometry {
    pub model_base_link: String,
    pub primitives: Vec<FixedCollisionPrimitive>,
    pub excluded_moving_collision_count: u64,
    pub unsupported_collision_count: u64,
}

/// 形状局部坐标到机体模型基准坐标的平移和行优先旋转。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FixedCollisionPrimitive {
    pub link_name: String,
    pub translation_m: [f64; 3],
    pub rotation_row_major: [f64; 9],
    #[serde(flatten)]
    pub shape: FixedCollisionShape,
}

/// 现有模型解析器支持的原始实体；圆柱轴沿形状局部 Z。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum FixedCollisionShape {
    Box { size_m: [f64; 3] },
    Sphere { radius_m: f64 },
    Cylinder { radius_m: f64, length_m: f64 },
}

impl FixedCollisionGeometry {
    /// 只校验几何；缺少实体、基准名称和排除统计不构成运动准入条件。
    pub fn validate(&self) -> Result<(), String> {
        for primitive in &self.primitives {
            primitive.validate()?;
        }
        Ok(())
    }
}

impl FixedCollisionPrimitive {
    pub fn validate(&self) -> Result<(), String> {
        if !self.translation_m.into_iter().chain(self.rotation_row_major).all(f64::is_finite) {
            return Err("固定碰撞几何的平移和旋转必须有限".into());
        }
        // 与实体相交核使用相同的列向量及舍入尺度，避免配置通过而查询再拒绝。
        let tolerance = 128.0 * f64::EPSILON;
        let axes: [[f64; 3]; 3] = std::array::from_fn(|column|
            std::array::from_fn(|row| self.rotation_row_major[row * 3 + column]));
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        for (i, a) in axes.iter().enumerate() {
            for (j, b) in axes.iter().enumerate() {
                let residual = dot(*a, *b) - if i == j { 1.0 } else { 0.0 };
                if !residual.is_finite() || residual.abs() > tolerance {
                    return Err("固定碰撞几何需要正交旋转矩阵".into());
                }
            }
        }
        let [a,b,c] = axes;
        let determinant = dot([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],c);
        if (determinant - 1.0).abs() > tolerance {
            return Err("固定碰撞几何需要右手旋转矩阵".into());
        }
        let positive = |value: f64| value.is_finite() && value > 0.0;
        let valid = match self.shape {
            FixedCollisionShape::Box { size_m } => size_m.into_iter().all(positive),
            FixedCollisionShape::Sphere { radius_m } => positive(radius_m),
            FixedCollisionShape::Cylinder { radius_m, length_m } => positive(radius_m) && positive(length_m),
        };
        if !valid { return Err("固定碰撞形状尺寸必须有限且大于零".into()); }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_collision_json_preserves_shapes_transforms_and_metadata() {
        let value = serde_json::json!({"model_base_link":"trunk","excluded_moving_collision_count":12,
            "unsupported_collision_count":2,"primitives":[
            {"link_name":"trunk","shape":"box","translation_m":[0.1,0.2,0.3],
                "rotation_row_major":[0,-1,0,1,0,0,0,0,1],"size_m":[0.6,0.3,0.1]},
            {"link_name":"fixed_sensor","shape":"sphere","translation_m":[0,0,0.2],
                "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.04},
            {"link_name":"fixed_cylinder","shape":"cylinder","translation_m":[0.2,0,0],
                "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.03,"length_m":0.2}]});
        let geometry: FixedCollisionGeometry = serde_json::from_value(value.clone()).unwrap();
        geometry.validate().unwrap();
        assert_eq!(geometry.model_base_link,"trunk");
        assert_eq!(geometry.excluded_moving_collision_count,12);
        assert_eq!(geometry.unsupported_collision_count,2);
        assert_eq!(geometry.primitives[0].rotation_row_major,[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]);
        // 按结构反序列化比较，避免 JSON 整数/浮点字面量的表示差异。
        let roundtrip: FixedCollisionGeometry = serde_json::from_value(serde_json::to_value(&geometry).unwrap()).unwrap();
        assert_eq!(roundtrip,geometry);
        assert_eq!(geometry.primitives[1].shape,FixedCollisionShape::Sphere {radius_m:0.04});
        assert_eq!(geometry.primitives[2].shape,FixedCollisionShape::Cylinder {radius_m:0.03,length_m:0.2});
    }

    #[test]
    fn fixed_collision_validation_rejects_invalid_geometry_without_completeness_gate() {
        let mut geometry = FixedCollisionGeometry {model_base_link:String::new(),primitives:vec![],
            excluded_moving_collision_count:12,unsupported_collision_count:3};
        geometry.validate().unwrap();
        let primitive = FixedCollisionPrimitive {link_name:"trunk".into(),translation_m:[0.0;3],
            rotation_row_major:[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],
            shape:FixedCollisionShape::Box {size_m:[0.6,0.3,0.1]}};
        geometry.primitives.push(primitive.clone());
        geometry.validate().unwrap();
        for shape in [FixedCollisionShape::Box {size_m:[0.6,0.0,0.1]},
            FixedCollisionShape::Sphere {radius_m:f64::NAN},
            FixedCollisionShape::Cylinder {radius_m:0.03,length_m:-0.1}] {
            geometry.primitives[0].shape=shape;assert!(geometry.validate().is_err());
        }
        geometry.primitives[0]=primitive.clone();
        geometry.primitives[0].rotation_row_major[0]=-1.0;
        assert!(geometry.validate().is_err(),"反射不是刚体旋转");
        geometry.primitives[0]=primitive.clone();
        geometry.primitives[0].rotation_row_major[1]=0.1;
        assert!(geometry.validate().is_err(),"错切不是刚体旋转");
        geometry.primitives[0]=primitive;
        geometry.primitives[0].translation_m[0]=f64::INFINITY;
        assert!(geometry.validate().is_err());
    }
}
