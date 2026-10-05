// Qualification-only fixture generator. Preserve quaternion source bits and
// evaluate both existing acceptance expressions; no new tolerance or policy.
use nalgebra::Quaternion;

fn main() {
    let mut cases: Vec<(String, [f64; 4])> = Vec::new();
    let seeds = [
        ("scalar", [0.0, 0.0, 0.0, 1.0]),
        ("four-equal", [0.5, 0.5, 0.5, 0.5]),
        ("four-distinct", [0.18257418583505536, 0.3651483716701107,
                          0.5477225575051661, 0.7302967433402214]),
    ];
    for side in [-1_i32, 1] {
        let center = (1.0_f64 + f64::from(side) * 1.0e-6).sqrt().to_bits();
        for offset in -4_i64..=4 {
            let scale = f64::from_bits((center as i64 + offset) as u64);
            for (seed_name, seed) in seeds {
                cases.push((format!("{seed_name}:side={side}:ulp={offset}"), seed.map(|v| v * scale)));
            }
        }
    }
    cases.extend([
        ("zero".into(), [0.0, 0.0, 0.0, 0.0]),
        ("negative-zero".into(), [-0.0, -0.0, -0.0, -0.0]),
        ("identity".into(), [0.0, 0.0, 0.0, 1.0]),
        ("negative-identity".into(), [-0.0, 0.0, -0.0, -1.0]),
        ("nan-payload".into(), [0.0, f64::from_bits(0x7ff8_0000_0000_0001), 0.0, 1.0]),
        ("positive-infinity".into(), [0.0, 0.0, 0.0, f64::INFINITY]),
        ("negative-infinity".into(), [f64::NEG_INFINITY, 0.0, 0.0, 1.0]),
        ("finite-overflow".into(), [1.0e308, 0.0, 0.0, 1.0]),
    ]);
    println!("[");
    for (index, (label, q)) in cases.iter().enumerate() {
        let quaternion = Quaternion::new(q[3], q[0], q[1], q[2]);
        let norm = quaternion.norm();
        let retained_accepts = norm.is_finite() && (norm * norm - 1.0).abs() <= 1.0e-6;
        let norm_squared = q.iter().map(|v| v * v).sum::<f64>();
        let candidate_accepts = q.iter().all(|v| v.is_finite())
            && (norm_squared - 1.0).abs() <= 1.0e-6;
        println!("{{\"label\":\"{}\",\"rotation_xyzw_bits\":{:?},\"retained_accepts\":{},\"candidate_accepts\":{}}}{}",
            label, q.map(f64::to_bits), retained_accepts, candidate_accepts,
            if index + 1 == cases.len() { "" } else { "," });
    }
    println!("]");
}
