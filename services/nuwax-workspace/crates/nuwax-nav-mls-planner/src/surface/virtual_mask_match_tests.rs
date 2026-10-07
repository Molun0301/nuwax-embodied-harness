use super::*;
use std::cell::Cell;

#[test]
fn mask_match_checks_every_cell_without_allocating_a_second_mask() {
    let values: Vec<_> = (0..35).map(|i| i % 4 == 1 || i == 34).collect();
    let mask = VirtualObstacleMask::new([0.0, -1.0], 0.05, 7, 5, &values).unwrap();
    let visits = Cell::new(0);
    assert!(mask.matches_cells([0.0, -1.0], 0.05, 7, 5, values.iter().map(|&v| {
        visits.set(visits.get() + 1); v
    })));
    assert_eq!(visits.get(), values.len());
    for changed in 0..values.len() {
        visits.set(0);
        assert!(!mask.matches_cells([0.0, -1.0], 0.05, 7, 5,
            values.iter().enumerate().map(|(i, &v)| {
                visits.set(visits.get() + 1); v ^ (i == changed)
            })));
        assert_eq!(visits.get(), changed + 1);
    }
    let mut moved = values.clone();
    moved.swap(0, 1);
    assert_eq!(moved.iter().filter(|&&v| v).count(), values.iter().filter(|&&v| v).count());
    assert!(!mask.matches_cells([0.0, -1.0], 0.05, 7, 5, moved.into_iter()));
}

#[test]
fn mask_match_validates_retained_layout_before_reading_cells() {
    let values = [false; 6];
    let mask = VirtualObstacleMask::new([0.0, -1.0], 0.05, 3, 2, &values).unwrap();
    for (origin, resolution, width, height) in [
        ([-0.0, -1.0], 0.05, 3, 2),
        ([0.0, -0.0], 0.05, 3, 2),
        ([0.0, -1.0], f64::from_bits(0.05_f64.to_bits() + 1), 3, 2),
        ([0.0, -1.0], 0.05, 2, 3),
        ([f64::MAX, -1.0], 0.05, 3, 2),
        ([0.0, -1.0], f64::NAN, 3, 2),
        ([0.0, -1.0], 0.05, MAX_VIRTUAL_MASK_DIMENSION + 1, 1),
        ([0.0, -1.0], 0.05, 0, 0),
    ] {
        assert!(!mask.matches_cells(origin, resolution, width, height,
            values.iter().map(|_| panic!("different retained layout must not inspect payload"))));
    }
    assert!(!mask.matches_cells([0.0, -1.0], 0.05, 3, 2, values[..5].iter().copied()));
    assert!(!mask.matches_cells([0.0, -1.0], 0.05, 3, 2, [false; 7].into_iter()));
}

#[test]
fn mask_match_rejects_incorrect_exact_iterator_prefix_or_trailer() {
    struct IncorrectLength(std::vec::IntoIter<bool>);
    impl Iterator for IncorrectLength {
        type Item = bool;
        fn next(&mut self) -> Option<bool> { self.0.next() }
        fn size_hint(&self) -> (usize, Option<usize>) { (6, Some(6)) }
    }
    impl ExactSizeIterator for IncorrectLength {}
    let mask = VirtualObstacleMask::new([0.0, 0.0], 1.0, 3, 2, &[false; 6]).unwrap();
    for count in [5, 7] {
        assert!(!mask.matches_cells([0.0, 0.0], 1.0, 3, 2,
            IncorrectLength(vec![false; count].into_iter())));
    }
}
