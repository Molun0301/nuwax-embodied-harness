
use super::*;

#[test]
fn coarsening_keeps_signed_boundaries_and_converges_to_one_root() {
    let mut directory = BlockDirectory::new(true, 0.05);
    let samples = [
        ((i32::MIN, -1, 0), 0),
        ((-1, 0, 1), 1),
        ((0, -1, -1), 2),
        ((i32::MAX, 1, -1), 3),
    ];
    for (raw, page) in samples {
        directory.insert(raw, page);
    }
    for level in 0..=32 {
        assert_eq!(directory.level(), level);
        for (raw, page) in samples {
            assert!(
                directory.get(directory.key(raw)).unwrap().contains(&page),
                "层级{level}不得漏掉符号边界关联"
            );
        }
        let low = directory.key((-1, -1, -1));
        let high = directory.key((0, 0, 0));
        assert!((0..3).all(|axis| low[axis] <= high[axis]));
        if level < 32 {
            directory.coarsen();
        }
    }
    assert_eq!(directory.get([0; 3]).unwrap().len(), 4);
    assert_eq!(directory.entries(), 4);
}
