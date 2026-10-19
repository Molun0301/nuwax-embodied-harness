//! Logical FIFO seed order is independent of source iteration and dense storage IDs.
use crate::Cell;
#[cfg(feature = "mls-adapter")]
use nuwax_nav_mls_planner::surface::GroundKey;
use std::cmp::Ordering;

/// Existing column geometry and source-local record identity, without new hashes.
/// Raw bands are disjoint within a unique XY column, so their heights are distinct.
/// Measured records at the same height remain distinct through their GroundKey.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SeedKey {
    column: [i32; 2],
    height: f64,
    #[cfg(feature = "mls-adapter")]
    source: Option<GroundKey>,
}
impl SeedKey {
    pub(crate) fn for_cell(cell: &Cell) -> Self {
        Self {
            column: cell.column,
            height: cell.ground,
            #[cfg(feature = "mls-adapter")]
            source: cell.source_ground,
        }
    }
}
impl PartialEq for SeedKey {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for SeedKey {}
impl PartialOrd for SeedKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for SeedKey {
    fn cmp(&self, other: &Self) -> Ordering {
        let order = self
            .column
            .cmp(&other.column)
            .then_with(|| self.height.total_cmp(&other.height));
        #[cfg(feature = "mls-adapter")]
        {
            order.then_with(|| self.source.cmp(&other.source))
        }
        #[cfg(not(feature = "mls-adapter"))]
        {
            order
        }
    }
}

/// Persistent ordered index; swaps update only values, never logical seed positions.
#[derive(Clone, Debug, Default)]
pub(crate) struct SeedOrder(im::OrdMap<SeedKey, usize>);
impl SeedOrder {
    pub(crate) fn ids(&self) -> impl Iterator<Item = usize> + '_ {
        self.0.iter().map(|(_, &id)| id)
    }
    pub(crate) fn insert(&mut self, cell: &Cell, id: usize) -> Result<(), String> {
        if self.0.insert(SeedKey::for_cell(cell), id).is_some() {
            return Err("层析输入含重复地表种子键".into());
        }
        Ok(())
    }
    pub(crate) fn remove(&mut self, cell: &Cell, id: usize) -> Result<(), String> {
        if self.0.remove(&SeedKey::for_cell(cell)) != Some(id) {
            return Err("层析地表种子索引失配".into());
        }
        Ok(())
    }
    pub(crate) fn move_id(&mut self, cell: &Cell, old: usize, new: usize) -> Result<(), String> {
        let value = self
            .0
            .get_mut(&SeedKey::for_cell(cell))
            .filter(|value| **value == old)
            .ok_or("层析地表种子索引失配")?;
        *value = new;
        Ok(())
    }
}
