use crate::Cell;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

/// Shared cell storage keeps the existing borrowed and mutable iteration API.
/// A fork clones persistent roots; editing clones only the affected tree leaves.
#[derive(Clone, Debug, Default)]
pub struct Cells {
    values: im::Vector<Cell>,
    externally_mutable: bool,
    // Bind private indexes to this storage even if the public cells field is replaced.
    // Clones share identity; a completed edit gets a fresh identity, without hashing.
    basis: Arc<()>,
}
impl Cells {
    pub(crate) fn indices_current(&self) -> bool {
        !self.externally_mutable
    }
    pub(crate) fn seal_indices(&mut self) {
        if self.externally_mutable {
            self.basis = Arc::new(());
        }
        self.externally_mutable = false;
    }
    pub(crate) fn basis(&self) -> &Arc<()> {
        &self.basis
    }
}
impl Deref for Cells {
    type Target = im::Vector<Cell>;
    fn deref(&self) -> &Self::Target {
        &self.values
    }
}
impl DerefMut for Cells {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.externally_mutable = true;
        &mut self.values
    }
}
impl IntoIterator for Cells {
    type Item = Cell;
    type IntoIter = im::vector::ConsumingIter<Cell>;
    fn into_iter(self) -> Self::IntoIter {
        self.values.into_iter()
    }
}
impl<'a> IntoIterator for &'a Cells {
    type Item = &'a Cell;
    type IntoIter = im::vector::Iter<'a, Cell>;
    fn into_iter(self) -> Self::IntoIter {
        self.values.iter()
    }
}
impl<'a> IntoIterator for &'a mut Cells {
    type Item = &'a mut Cell;
    type IntoIter = im::vector::IterMut<'a, Cell>;
    fn into_iter(self) -> Self::IntoIter {
        self.externally_mutable = true;
        self.values.iter_mut()
    }
}

#[cfg(test)]
mod tests {
    use crate::{ColumnUpdate, Tomogram};

    #[test]
    fn direct_cell_or_resolution_mutation_rejects_stale_incremental_indices_without_panic() {
        let map = Tomogram::from_columns(
            0.125,
            [([0, 0], vec![[-0.125, 0.0]])],
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        .unwrap();
        let expected = "层析单元或分辨率已直接修改，需重新构图后再作增量更新";
        for mode in 0..4 {
            let mut changed = map.clone();
            match mode {
                0 => changed.cells[0].cost = 123.0,
                1 => {
                    for cell in &mut changed.cells {
                        cell.cost = 123.0;
                    }
                }
                2 => {
                    for cell in changed.cells.iter_mut() {
                        cell.cost = 123.0;
                    }
                }
                _ => changed.resolution = 0.25,
            }
            let result = changed.with_column_updates([ColumnUpdate {
                column: [0, 0],
                bands: Some(vec![[0.0, 0.125]]),
            }]);
            assert_eq!(result.unwrap_err(), expected);
        }
        assert!(
            map.with_column_updates([]).is_ok(),
            "retained roots are unchanged"
        );
    }

    #[test]
    fn replacing_public_cells_cannot_reuse_another_storage_index_basis() {
        let make = |xy| {
            Tomogram::from_columns(0.125, [(xy, vec![[-0.125, 0.0]])], 0.5, 0.0, 0.0, 0.0, 0.0)
                .unwrap()
        };
        let map = make([0, 0]);
        let foreign = make([9, 9]);
        let (updated, _) = map
            .with_column_updates([ColumnUpdate {
                column: [0, 0],
                bands: Some(vec![[0.0, 0.125]]),
            }])
            .unwrap();
        assert_eq!(map.cells.len(), foreign.cells.len());
        for replacement in [super::Cells::default(), foreign.cells, updated.cells] {
            let mut changed = map.clone();
            changed.cells = replacement;
            let result = changed.with_column_updates([ColumnUpdate {
                column: [0, 0],
                bands: None,
            }]);
            assert_eq!(
                result.unwrap_err(),
                "层析单元或分辨率已直接修改，需重新构图后再作增量更新"
            );
        }
        let mut retained = map.clone();
        retained.cells = map.cells.clone();
        assert!(
            retained.with_column_updates([]).is_ok(),
            "the actual matching basis remains valid"
        );
    }
}
