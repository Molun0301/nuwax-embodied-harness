// Qualification utility only. Uses the exact retained native expression and
// emits source-owned raw IEEE bits; it does not normalize either quaternion.
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

fn iso(t: [f64; 3], q: [f64; 4]) -> Isometry3<f64> {
    Isometry3::from_parts(Translation3::new(t[0], t[1], t[2]),
        UnitQuaternion::new_unchecked(Quaternion::new(q[3], q[0], q[1], q[2])))
}
fn pose(value: &Isometry3<f64>) -> String {
    let q = value.rotation.quaternion();
    let t = value.translation.vector;
    format!("{{\"translation_m_bits\":{:?},\"rotation_xyzw_bits\":{:?}}}",
        [t[0].to_bits(), t[1].to_bits(), t[2].to_bits()],
        [q.i.to_bits(), q.j.to_bits(), q.k.to_bits(), q.w.to_bits()])
}
fn main() {
    let vectors = [
        (iso([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
         iso([-0.0, 0.0, -0.0], [-0.0, 0.0, -0.0, 1.0])),
        (iso([1.25, -2.75, 3.125], [0.5, -0.5, 0.5, 0.5]),
         iso([-4.5, 6.25, -8.125], [0.0, 0.0, 0.6, 0.8])),
        (iso([12500.123456789, -0.000031, 0.75], [0.6, 0.0, 0.0, 0.8]),
         iso([12500.123456790, 0.000043, -2.5], [0.0, 0.8, 0.0, 0.6])),
        (iso([-99999.875, 98765.4321, -0.000000000000001],
             [0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214]),
         iso([0.000000000000003, -54321.12345, 10000.98765], [-0.5, 0.5, -0.5, 0.5])),
        (iso([f64::from_bits(1), -f64::from_bits(1), -0.0], [0.0, -0.0, 0.0, 1.0000001]),
         iso([-f64::from_bits(1), f64::from_bits(1), 0.0], [0.0, 0.0, -0.0, 0.9999999])),
    ];
    println!("[");
    for (index, (base, estimate)) in vectors.iter().enumerate() {
        let expected = base.inverse() * estimate;
        println!("{{\"base\":{},\"estimate\":{},\"expected\":{}}}{}",
            pose(base), pose(estimate), pose(&expected), if index + 1 == vectors.len() { "" } else { "," });
    }
    println!("]");
}
