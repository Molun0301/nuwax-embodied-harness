//! Shared immutable source points. Capturing moves the existing Vec allocation;
//! cloning frame metadata never duplicates its point payload.
use nalgebra::Point3;
use std::sync::Arc;

#[derive(Clone, Debug)]
pub(crate) struct CorrectionPoints(Arc<CorrectionPointData>);

#[derive(Debug)]
struct CorrectionPointData {
    points: Vec<Point3<f64>>,
}

impl CorrectionPoints {

}

impl From<Vec<Point3<f64>>> for CorrectionPoints {
    fn from(points: Vec<Point3<f64>>) -> Self {
        Self(Arc::new(CorrectionPointData { points }))
    }
}

impl FromIterator<Point3<f64>> for CorrectionPoints {
    fn from_iter<T: IntoIterator<Item = Point3<f64>>>(iter: T) -> Self {
        Self::from(iter.into_iter().collect::<Vec<_>>())
    }
}

impl std::ops::Deref for CorrectionPoints {
    type Target = [Point3<f64>];
    fn deref(&self) -> &Self::Target { self.0.points.as_slice() }
}

impl<'a> IntoIterator for &'a CorrectionPoints {
    type Item = &'a Point3<f64>;
    type IntoIter = std::slice::Iter<'a, Point3<f64>>;
    fn into_iter(self) -> Self::IntoIter { self.iter() }
}

impl PartialEq for CorrectionPoints {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0) || self.len() == other.len()
            && self.iter().zip(other.iter()).all(|(a, b)| {
                a.x.to_bits() == b.x.to_bits() && a.y.to_bits() == b.y.to_bits()
                    && a.z.to_bits() == b.z.to_bits()
            })
    }
}
impl Eq for CorrectionPoints {}

#[cfg(test)]
impl CorrectionPoints {
    // Fixtures deliberately construct a new conflicting owner. Production has
    // no mutable accessor and cannot alter an already admitted frame in place.
    pub(crate) fn edited_for_test(&self, edit: impl FnOnce(&mut Vec<Point3<f64>>)) -> Self {
        let mut points = self.0.points.clone();
        edit(&mut points);
        points.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_and_frame_clones_preserve_original_allocation_until_last_owner() {
        let points = vec![Point3::new(1.0, 2.0, 3.0); 100];
        let allocation = points.as_ptr();
        let owner = CorrectionPoints::from(points);
        assert_eq!(owner.as_ptr(), allocation);
        let weak = Arc::downgrade(&owner.0);
        let clones = (0..100).map(|_| owner.clone()).collect::<Vec<_>>();
        assert!(clones.iter().all(|copy| Arc::ptr_eq(&owner.0, &copy.0)));
        drop(owner);
        assert!(weak.upgrade().is_some());
        drop(clones);
        assert!(weak.upgrade().is_none());
    }



    #[test]
    fn independent_replay_compares_exact_bits_order_and_duplicate_count() {
        let points = vec![Point3::new(0.0, 1.0, 2.0), Point3::new(3.0, 4.0, 5.0)];
        let owner = CorrectionPoints::from(points.clone());
        assert_eq!(owner, CorrectionPoints::from(points));
        assert_ne!(owner, owner.edited_for_test(|p| p[0].x = -0.0));
        assert_ne!(owner, owner.edited_for_test(|p| p.swap(0, 1)));
        assert_ne!(owner, owner.edited_for_test(|p| p.push(p[0])));
        assert_eq!(owner[0].x.to_bits(), 0.0_f64.to_bits());
    }
}
