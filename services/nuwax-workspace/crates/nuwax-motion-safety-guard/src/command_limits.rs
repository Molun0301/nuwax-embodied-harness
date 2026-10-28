//! 把设备命令帧限制转换为局部搜索可使用的保守速度盒。
use crate::wire::MotionCommandLimits;

impl MotionCommandLimits {
    /// 跨进程边界校验；缺失声明由调用方保留为未知。
    pub fn validate(&self) -> Result<(), String> {
        let nonnegative = [self.max_linear_x_mps, self.max_linear_y_mps, self.max_angular_z_rps];
        if nonnegative.into_iter().chain(self.max_planar_speed_mps)
            .any(|v| !v.is_finite() || v < 0.)
            || !self.input_origin_in_command_x_m.is_finite()
            || !self.input_origin_in_command_y_m.is_finite()
            || self.source.trim().is_empty() || self.input_frame_id.trim().is_empty()
            || self.limit_frame_id.trim().is_empty() {
            return Err("设备命令限制缺少来源、坐标或包含非法数值".into());
        }
        let r = &self.command_from_input_rotation;
        if r.len() != 4 || r.iter().any(|v| !v.is_finite())
            || (r[0]*r[0]+r[2]*r[2]-1.).abs() > 1e-6
            || (r[1]*r[1]+r[3]*r[3]-1.).abs() > 1e-6
            || (r[0]*r[1]+r[2]*r[3]).abs() > 1e-6
            || (r[0]*r[3]-r[1]*r[2]-1.).abs() > 1e-6 {
            return Err("设备命令变换必须为右手平面旋转".into());
        }
        Ok(())
    }

    /// 输出前进、倒退、横向与偏航上限；整个速度盒经变换后都满足设备限制。
    /// 这是搜索约束，不对已经生成的命令作静默截断。
    pub fn search_bounds(&self, forward: f64, reverse: f64, lateral: f64, yaw: f64) -> [f64; 4] {
        let r = &self.command_from_input_rotation;
        let rx = self.input_origin_in_command_x_m.abs();
        let ry = self.input_origin_in_command_y_m.abs();
        // 留出浮点运算尾差；不能借执行器容差提升物理上限。
        let margin = 1. - 32. * f64::EPSILON;
        let lx = self.max_linear_x_mps * margin;
        let ly = self.max_linear_y_mps * margin;
        let mut w = yaw.min(self.max_angular_z_rps * margin);
        if ry > 0. { w = w.min(lx / ry); }
        if rx > 0. { w = w.min(ly / rx); }
        if let Some(planar) = self.max_planar_speed_mps {
            if rx.hypot(ry) > 0. { w = w.min(planar * margin / rx.hypot(ry)); }
        }
        let available = [(lx - ry*w).max(0.), (ly - rx*w).max(0.)];
        // 先保留各输入轴的可用区间，再为旋转混合的两行共同收缩。
        let projected = |column: usize| (0..2).filter(|row| r[row*2+column] != 0.)
            .map(|row| available[row] / r[row*2+column].abs()).fold(f64::INFINITY, f64::min);
        let x = forward.max(reverse).min(projected(0));
        let y = lateral.min(projected(1));
        let mut scale = 1_f64;
        for row in 0..2 {
            let demand = r[row*2].abs()*x + r[row*2+1].abs()*y;
            if demand > 0. { scale = scale.min(available[row] / demand); }
        }
        if let Some(planar) = self.max_planar_speed_mps {
            // 对实际矩阵使用列范数，避免近似正交系数导致边界超限。
            let demand = r[0].hypot(r[2])*x + r[1].hypot(r[3])*y;
            if demand > 0. { scale = scale.min((planar*margin - rx.hypot(ry)*w).max(0.) / demand); }
        }
        [forward.min(x)*scale, reverse.min(x)*scale, y*scale, w]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transformed_box_vertices_respect_anisotropic_device_axes_and_planar_limit() {
        for angle in [0_f64, 0.3, std::f64::consts::FRAC_PI_2] {
            for planar in [None, Some(0.45)] {
                let (s,c)=angle.sin_cos();
                let limits=MotionCommandLimits {max_linear_x_mps:1.,max_linear_y_mps:0.2,
                    max_angular_z_rps:1.,max_planar_speed_mps:planar,source:"设备夹具".into(),
                    input_frame_id:"imu".into(),limit_frame_id:"base_link".into(),
                    command_from_input_rotation:vec![c,-s,s,c],input_origin_in_command_x_m:-0.0256,
                    input_origin_in_command_y_m:0.014};
                limits.validate().unwrap();
                let [f,b,l,w]=limits.search_bounds(0.5,0.3,0.5,0.8);
                assert!(f>0. && b>0. && l>0. && w>0.);
                for vx in [-b,f] { for vy in [-l,l] { for wz in [-w,w] {
                    let x=c*vx-s*vy+0.014*wz;
                    let y=s*vx+c*vy+0.0256*wz;
                    assert!(x.abs()<=1. && y.abs()<=0.2 && wz.abs()<=1.);
                    if let Some(max)=planar {assert!(x.hypot(y)<=max);}
                } } }
                if angle==0. && planar.is_none() {assert!(f>0.49 && l<0.18 && l>0.179);}
            }
        }
    }
}
