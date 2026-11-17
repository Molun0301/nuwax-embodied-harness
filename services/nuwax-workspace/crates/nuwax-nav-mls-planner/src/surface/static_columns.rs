//! 静态几何的变化列记录：表面图每次更新时登记本次变化的列，全局层析缓存据此只处理这些列。

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use super::{GroundEvidence, SurfaceSnapshot};

/// 保留的最近变化步数；缓存身份早于这些步时，由层析缓存按整图比对。
const STEPS: usize = 64;
static NEXT_ID: AtomicU64 = AtomicU64::new(1);

/// 一次静态几何变化：变化前的身份与变化的层析列（含删除）。
#[derive(Debug)]
struct Step {
    from: u64,
    columns: Vec<[i32; 2]>,
}

/// 层析列的来源：是否取自完整精细源、列边长、地面语义；三者不变时变化列才能逐步累计。
#[derive(Debug, PartialEq)]
struct Source {
    fine: bool,
    size: u32,
    ground: GroundEvidence,
}

/// 静态几何身份与最近若干步的变化列；几何不变时沿用同一记录。
#[derive(Debug)]
pub struct StaticColumnLog {
    id: u64,
    source: Source,
    steps: Vec<Arc<Step>>,
}

impl StaticColumnLog {
    /// 本记录的静态几何身份。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 从身份 `id` 到本记录之间变化过的层析列；记录里查不到该身份时返回 `None`。
    pub fn columns_since(&self, id: u64) -> Option<BTreeSet<[i32; 2]>> {
        let mut columns = BTreeSet::new();
        if self.id == id {
            return Some(columns);
        }
        for step in self.steps.iter().rev() {
            columns.extend(&step.columns);
            if step.from == id {
                return Some(columns);
            }
        }
        None
    }
}

impl SurfaceSnapshot {
    /// 表面图登记的静态变化列记录；没有登记时为 `None`。
    pub fn static_columns(&self) -> Option<&StaticColumnLog> {
        self.static_columns.as_deref()
    }

    /// 表面图更新时登记相对 `previous` 的变化列：`changed` 为（精细源变化列，规划变化列），
    /// 按本快照的层析来源取其一。两者都为空表示静态几何未变，沿用上一份记录；
    /// 给不出变化列、或层析来源与上一份不同时，记录重新开始。
    pub fn with_static_columns(
        &self,
        previous: Option<&SurfaceSnapshot>,
        changed: Option<(Vec<[i32; 2]>, Vec<[i32; 2]>)>,
    ) -> Self {
        let (resolution, _) = self.navigation_source_columns(None);
        let source = Source {
            fine: self.navigation_uses_fine_source(),
            size: (resolution as f32).to_bits(),
            ground: self.ground_evidence(),
        };
        let old = previous.and_then(|snapshot| snapshot.static_columns.as_ref())
            .filter(|old| old.source == source);
        let log = match (old, changed) {
            (Some(old), Some((fine, coarse))) if fine.is_empty() && coarse.is_empty() => Arc::clone(old),
            (Some(old), Some((fine, coarse))) => {
                let keep = old.steps.len().min(STEPS - 1);
                let mut steps = old.steps[old.steps.len() - keep..].to_vec();
                steps.push(Arc::new(Step { from: old.id, columns: if source.fine { fine } else { coarse } }));
                Arc::new(StaticColumnLog { id: NEXT_ID.fetch_add(1, Ordering::Relaxed), source, steps })
            }
            _ => Arc::new(StaticColumnLog { id: NEXT_ID.fetch_add(1, Ordering::Relaxed), source, steps: Vec::new() }),
        };
        let mut result = self.clone();
        result.static_columns = Some(log);
        result
    }
}
