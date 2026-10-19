//! Borrowed source-native ground geometry. Statistics stay in immutable blocks.
use super::*;
use nuwax_map_chunks::{GroundRecordRef, GroundRecords, HeightfieldFragment, Key};
use std::collections::BTreeMap;

/// Local to the containing immutable source. Acquisition identity is a separate axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum GroundKey {
    LegacyPlane(Key),
    HeightfieldV1(Key),
}
impl GroundKey {
    pub fn anchor(self) -> Key {
        match self {
            Self::LegacyPlane(key) | Self::HeightfieldV1(key) => key,
        }
    }
}

#[derive(Clone, Debug)]
struct Fragment {
    block: Key,
    local: u16,
    records: GroundRecords,
}
impl Fragment {
    fn value(&self) -> &HeightfieldFragment {
        match self.records.get(self.local) {
            Some(GroundRecordRef::HeightfieldV1(value)) => value,
            _ => unreachable!("immutable native descriptor names its original heightfield record"),
        }
    }
}

/// A lightweight projection of admitted fragments, never a second member distribution.
#[derive(Clone, Debug)]
pub struct ObservedHeightfield {
    anchor: Key,
    height: f64,
    bounds: [f64; 2],
    fragments: Vec<Fragment>,
    // Missing local direction does not erase the observed column height.
    direction: Option<[f64; 3]>,
}
impl ObservedHeightfield {
    fn geometry_eq(&self, other: &Self) -> bool {
        self.anchor == other.anchor
            && self.height == other.height
            && self.bounds == other.bounds
            && self.direction == other.direction
            && self.members().eq(other.members())
    }
    fn members(&self) -> GroundMembers<'_> {
        GroundMembers(MemberIter::Heightfield {
            fragments: self.fragments.iter(),
            current: None,
        })
    }
    fn contains(&self, key: Key) -> bool {
        if key[..2] != self.anchor[..2] {
            return false;
        }
        let block = nuwax_map_chunks::block_key(key);
        let first = self.fragments.partition_point(|fragment| fragment.block < block);
        let local = nuwax_map_chunks::local_key(key);
        self.fragments[first..]
            .iter()
            .take_while(|fragment| fragment.block == block)
            .any(|fragment| fragment.value().members.binary_search_by_key(&local, |member| member.0).is_ok())
    }
}

/// One source selection, borrowed for a query. No member vector is copied.
#[derive(Clone, Copy, Debug)]
pub enum SourceGroundRecord<'a> {
    LegacyPlane(&'a ObservedGroundLayer),
    HeightfieldV1(&'a ObservedHeightfield),
}
impl<'a> SourceGroundRecord<'a> {
    pub fn key(self) -> GroundKey {
        match self {
            Self::LegacyPlane(record) => GroundKey::LegacyPlane(record.anchor),
            Self::HeightfieldV1(record) => GroundKey::HeightfieldV1(record.anchor),
        }
    }
    /// Height of this already-selected source column. For Heightfield, callers
    /// must select again at a different support XY; this is not plane extrapolation.
    pub fn height_at(self, xy: [f64; 2]) -> Option<f64> {
        match self {
            Self::LegacyPlane(record) => record.height_at(xy),
            Self::HeightfieldV1(record) => Some(record.height),
        }
    }
    pub fn direction(self) -> Option<[f64; 3]> {
        match self {
            Self::LegacyPlane(record) => Some(record.normal),
            Self::HeightfieldV1(record) => record.direction,
        }
    }
    pub fn occupied_bounds(self, quantum: f64) -> [f64; 2] {
        match self {
            Self::LegacyPlane(record) => [
                f64::from(record.members[0]) * quantum,
                (f64::from(*record.members.last().expect("admitted nonempty members")) + 1.0)
                    * quantum,
            ],
            Self::HeightfieldV1(record) => record.bounds,
        }
    }
    pub fn members(self) -> GroundMembers<'a> {
        match self {
            Self::LegacyPlane(record) => GroundMembers(MemberIter::Legacy(record.members.iter())),
            Self::HeightfieldV1(record) => record.members(),
        }
    }
    pub fn contains_member(self, key: Key) -> bool {
        match self {
            Self::LegacyPlane(record) => {
                key[..2] == record.anchor[..2] && record.members.binary_search(&key[2]).is_ok()
            }
            Self::HeightfieldV1(record) => record.contains(key),
        }
    }
    pub fn is_heightfield(self) -> bool {
        matches!(self, Self::HeightfieldV1(_))
    }
    pub fn scatter(self) -> Option<f64> {
        match self {
            Self::HeightfieldV1(_) => None,
            Self::LegacyPlane(record) => {
                let n = record.normal;
                Some(
                    record
                        .statistics
                        .moments()
                        .sample_scatter()
                        .map_or(0.0, |s| {
                            (n[0] * n[0] * s[0]
                                + 2.0 * n[0] * n[1] * s[1]
                                + 2.0 * n[0] * n[2] * s[2]
                                + n[1] * n[1] * s[3]
                                + 2.0 * n[1] * n[2] * s[4]
                                + n[2] * n[2] * s[5])
                                .max(0.0)
                                .sqrt()
                        }),
                )
            }
        }
    }
    /// The continuous-plane special case applies only to the same legacy record.
    pub fn same_legacy_plane(self, other: Self) -> bool {
        matches!((self, other), (Self::LegacyPlane(a), Self::LegacyPlane(b)) if std::ptr::eq(a, b))
    }
}

/// Allocation-free iteration over global native z members.
pub struct GroundMembers<'a>(MemberIter<'a>);
enum MemberIter<'a> {
    Legacy(std::slice::Iter<'a, i32>),
    Heightfield {
        fragments: std::slice::Iter<'a, Fragment>,
        current: Option<(i32, u32)>,
    },
}
impl Iterator for GroundMembers<'_> {
    type Item = i32;
    fn next(&mut self) -> Option<i32> {
        match &mut self.0 {
            MemberIter::Legacy(members) => members.next().copied(),
            MemberIter::Heightfield { fragments, current } => loop {
                if let Some((block_z, members)) = current {
                    if *members != 0 {
                        let z = members.trailing_zeros();
                        *members &= *members - 1;
                        return Some(*block_z * 32 + z as i32);
                    }
                }
                let fragment = fragments.next()?;
                let block = fragment.block;
                // Admission permits disjoint fragments of one surface within
                // the same block. A block has only 32 native z positions: merge
                // their borrowed members into one stack mask, not a copied Vec.
                let mut mask = 0_u32;
                for (local, _) in &fragment.value().members { mask |= 1 << (local / 1024); }
                while fragments.as_slice().first().is_some_and(|next| next.block == block) {
                    let next = fragments.next().unwrap();
                    for (local, _) in &next.value().members { mask |= 1 << (local / 1024); }
                }
                *current = Some((block[2], mask));
            },
        }
    }
}

#[derive(Clone, Debug, Default)]
pub(super) struct NativeGroundIndex {
    columns: im::HashMap<[i32; 2], Arc<Vec<ObservedHeightfield>>, ahash::RandomState>,
}
impl NativeGroundIndex {
    pub(super) fn records(&self, xy: [i32; 2]) -> impl Iterator<Item = SourceGroundRecord<'_>> {
        self.columns
            .get(&xy)
            .into_iter()
            .flat_map(|records| records.iter())
            .map(SourceGroundRecord::HeightfieldV1)
    }
    pub(super) fn is_empty(&self) -> bool {
        self.columns.is_empty()
    }

    /// A block-only update must not retain a claim for a removed occupied member.
    /// Complete-root readers subsequently install the new admitted column group.
    pub(super) fn retire_missing_members(
        &mut self,
        changed: &[[i32; 2]],
        present: impl Fn(Key) -> bool,
    ) -> bool {
        if self.columns.is_empty() {
            return false;
        }
        let mut retired = false;
        for &xy in changed {
            let Some(records) = self.columns.get(&xy) else {
                continue;
            };
            let valid =
                |record: &ObservedHeightfield| record.members().all(|z| present([xy[0], xy[1], z]));
            if records.iter().all(valid) {
                continue;
            }
            let retained: Vec<_> = records.iter().filter(|r| valid(r)).cloned().collect();
            if retained.is_empty() {
                self.columns.remove(&xy);
            } else {
                self.columns.insert(xy, Arc::new(retained));
            }
            retired = true;
        }
        retired
    }

    /// All vertical native blocks for one changed block-XY, already admitted by
    /// the original complete-root boundary. Query code never reassembles them.
    pub(super) fn replace_block_xy(
        &mut self,
        block_xy: [i32; 2],
        blocks: &[(Key, GroundRecords)],
    ) -> Vec<[i32; 2]> {
        let mut groups: BTreeMap<Key, ObservedHeightfield> = BTreeMap::new();
        for (block, records) in blocks {
            for (local, record) in records.iter() {
                let GroundRecordRef::HeightfieldV1(record) = record else {
                    continue;
                };
                let group = groups
                    .entry(record.anchor)
                    .or_insert_with(|| ObservedHeightfield {
                        anchor: record.anchor,
                        height: record.height_m,
                        bounds: record.occupied_bounds,
                        fragments: Vec::new(),
                        direction: None,
                    });
                group.fragments.push(Fragment {
                    block: *block,
                    local,
                    records: records.clone(),
                });
            }
        }
        let mut columns: BTreeMap<[i32; 2], Vec<ObservedHeightfield>> = BTreeMap::new();
        for (anchor, mut record) in groups {
            record
                .fragments
                .sort_unstable_by_key(|fragment| fragment.block);
            columns
                .entry([anchor[0], anchor[1]])
                .or_default()
                .push(record);
        }
        let mut changed = Vec::new();
        for x in 0..32 {
            for y in 0..32 {
                let xy = [block_xy[0] * 32 + x, block_xy[1] * 32 + y];
                let next = columns.remove(&xy).unwrap_or_default();
                let old = self.columns.get(&xy);
                let equal = old.map_or(next.is_empty(), |old| {
                    old.len() == next.len() && old.iter().zip(&next).all(|(a, b)| a.geometry_eq(b))
                });
                if !equal {
                    changed.push(xy);
                }
                if next.is_empty() {
                    self.columns.remove(&xy);
                } else {
                    self.columns.insert(xy, Arc::new(next));
                }
            }
        }
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
    use nuwax_map_chunks::{
        ColumnGroundRecords, Domain, HeightfieldSurface, Manifest, Receiver, RegionReader,
        Repository, Transfer, Writer,
    };
    const Q: f32 = 0.05;
    const BUDGET: usize = 4 * 1024 * 1024;

    fn writer() -> Writer {
        Writer::new(Repository::temporary().unwrap(), Q, BUDGET).unwrap()
    }
    fn column(xy: [i32; 2], zs: &[i32]) -> ColumnGroundRecords {
        ColumnGroundRecords {
            legacy: vec![],
            heightfields: vec![HeightfieldSurface {
                column: HeightfieldColumn::from_counts(
                    xy,
                    Q,
                    zs.iter().map(|&z| (z, [1, 0, 0, 0, 0, 0, 0, 0])).collect(),
                )
                .unwrap()
                .unwrap(),
                samples: vec![],
            }],
        }
    }
    fn install(w: &mut Writer, xy: [i32; 2], value: &ColumnGroundRecords) {
        w.observe(
            value
                .heightfields
                .iter()
                .flat_map(|s| s.column.members().map(|(z, _)| [xy[0], xy[1], z])),
            20,
        )
        .unwrap();
        w.replace_ground_column(xy, value).unwrap();
    }
    fn root(w: &mut Writer, revision: u64) -> Arc<Manifest> {
        w.snapshot(
            Domain {
                session: "native-ground-api".into(),
                generation: 1,
                frame_generation: 0,
                producer_generation: 1,
            },
            revision,
            20,
        )
        .unwrap()
    }
    // Real writer and receiver admission supply the trusted immutable records.
    // This adapter is a fixture, not the production FineMapReader activation.
    fn admitted(w: &Writer, root: &Manifest) -> (SurfaceObservations, Vec<(Key, GroundRecords)>) {
        let mut receiver = Receiver::new(Repository::temporary().unwrap());
        receiver
            .receive(
                &Transfer {
                    root: root.encode().unwrap(),
                    included: BTreeMap::new(),
                },
                &root.domain,
                |_, r| {
                    w.repository()
                        .read_object(r.object, nuwax_map_chunks::MAX_BLOCK_BYTES)
                },
            )
            .unwrap();
        let mut source = SurfaceObservations::new(Q, 32).unwrap();
        let mut parts = Vec::new();
        for reference in &root.blocks {
            let (block, records) = receiver.ground_parts(reference).unwrap();
            source
                .replace_block(
                    reference.key,
                    block
                        .iter()
                        .map(|(&local, e)| {
                            (
                                nuwax_map_chunks::global_key(reference.key, local).unwrap(),
                                SurfaceObservation {
                                    observed_ns: e.observed_ns,
                                    hits: e.hits,
                                },
                            )
                        })
                        .collect(),
                )
                .unwrap();
            let legacy = records
                .iter()
                .filter_map(|(k, r)| match r {
                    GroundRecordRef::LegacyPlane(r) => Some((k, r.clone())),
                    _ => None,
                })
                .collect();
            source
                .replace_ground_layers(reference.key, Arc::new(legacy))
                .unwrap();
            parts.push((reference.key, records));
        }
        let groups: std::collections::BTreeSet<_> =
            parts.iter().map(|(k, _)| [k[0], k[1]]).collect();
        for xy in groups {
            let group = parts
                .iter()
                .filter(|(k, _)| k[..2] == xy)
                .cloned()
                .collect::<Vec<_>>();
            source.replace_heightfield_columns(xy, &group);
        }
        source.mark_complete_geometry_source();
        (source, parts)
    }

    #[test]
    fn native_heightfield_borrows_cross_block_members_and_keeps_missing_direction() {
        let mut w = writer();
        let xy = [-1, 3];
        let value = column(xy, &[-33, -1, 0, 32, 65]);
        install(&mut w, xy, &value);
        let neighbour_xy = [0, 3];
        let neighbour = column(neighbour_xy, &[5]);
        install(&mut w, neighbour_xy, &neighbour);
        let root = root(&mut w, 1);
        let (source, parts) = admitted(&w, &root);
        let record = source.ground_records(xy).next().unwrap();
        assert_eq!(record.key(), GroundKey::HeightfieldV1([-1, 3, -33]));
        assert_eq!(
            record.members().collect::<Vec<_>>(),
            vec![-33, -1, 0, 32, 65]
        );
        assert_eq!(
            record.height_at([-0.025, 0.175]),
            Some(value.heightfields[0].column.height())
        );
        assert_eq!(
            record.height_at([-0.049, 0.151]),
            record.height_at([-0.001, 0.199])
        );
        assert_eq!(
            record.occupied_bounds(f64::from(Q)),
            value.heightfields[0].column.occupied_bounds()
        );
        // The support location selects its own source column. A center record
        // is not a plane that may be extrapolated across a column boundary.
        let neighbour_record = source.ground_records(neighbour_xy).next().unwrap();
        assert_eq!(
            neighbour_record.height_at([0.025, 0.175]),
            Some(neighbour.heightfields[0].column.height())
        );
        assert_ne!(
            neighbour_record.height_at([0.025, 0.175]),
            record.height_at([-0.025, 0.175])
        );
        assert_ne!(neighbour_record.key(), record.key());
        assert_eq!(record.direction(), None);
        assert_eq!(record.scatter(), None);
        for z in [-33, -1, 0, 32, 65] {
            assert!(record.contains_member([xy[0], xy[1], z]));
        }
        assert!(!record.contains_member([-1, 3, 1]));
        assert!(!record.contains_member([0, 3, 0]));
        let SourceGroundRecord::HeightfieldV1(native) = record else {
            panic!("native");
        };
        for fragment in &native.fragments {
            let original = parts.iter().find(|(k, _)| *k == fragment.block).unwrap();
            let (GroundRecords::Versioned(a), GroundRecords::Versioned(b)) =
                (&fragment.records, &original.1)
            else {
                panic!("versioned");
            };
            assert!(
                Arc::ptr_eq(a, b),
                "projection must borrow the original admitted payload"
            );
        }
        let fork = source.fork_for_planning();
        let SourceGroundRecord::HeightfieldV1(shared) = fork.ground_record(record.key()).unwrap()
        else {
            panic!("native");
        };
        assert!(std::ptr::eq(native, shared));
        drop(parts);
        drop(w);
        assert_eq!(
            source
                .ground_record(record.key())
                .unwrap()
                .members()
                .count(),
            5
        );
    }

    #[test]
    fn native_heightfield_metadata_refresh_preserves_geometry_and_tracks_members() {
        let mut w = writer();
        let xy = [0, 0];
        let make =
            |a, b, with_middle| ColumnGroundRecords {
                legacy: vec![],
                heightfields: vec![HeightfieldSurface {
                // Public wire boundary constructs this admitted zero-mass fixture;
                // the private map-chunks assembler is not exposed for tests.
                column: nuwax_map_chunks::heightfield_observations::decode_heightfield_column(
                    &serde_json::to_vec(&serde_json::json!({
                        "representation": "heightfield_v1", "xy": xy, "voxel_size_m": Q,
                        "members": ([(0,[a,b,0.,0.,0.,0.,0.,0.]),(32,[0.;8]),(64,[0.;8])]
                            .into_iter().filter(|(z,_)|*z!=32 || with_middle).collect::<Vec<_>>())
                    })).unwrap()).unwrap(),
                samples: vec![],
            }],
            };
        install(&mut w, xy, &make(0.75, 0.25, true));
        let first = root(&mut w, 1);
        let (mut source, _) = admitted(&w, &first);
        let old = source.clone();
        w.replace_ground_column(xy, &make(0.625, 0.375, true))
            .unwrap();
        let second = root(&mut w, 2);
        let (_, parts) = admitted(&w, &second);
        assert!(source
            .replace_heightfield_columns([0, 0], &parts)
            .is_empty());
        source.mark_complete_geometry_source();
        assert!(source.same_geometry(&old));
        assert!(source.same_planning_query_view(&old));
        w.replace_ground_column(xy, &make(0.625, 0.375, false))
            .unwrap();
        let third = root(&mut w, 3);
        let (_, parts) = admitted(&w, &third);
        assert_eq!(source.replace_heightfield_columns([0, 0], &parts), vec![xy]);
        source.mark_complete_geometry_source();
        assert!(!source.same_geometry(&old));
        assert!(!source.same_planning_query_view(&old));
        assert!(
            source.get([0, 0, 32]).is_some(),
            "withdrawn claim retains original occupancy"
        );
        assert!(!source
            .ground_record(GroundKey::HeightfieldV1([0, 0, 0]))
            .unwrap()
            .contains_member([0, 0, 32]));
        assert!(old
            .ground_record(GroundKey::HeightfieldV1([0, 0, 0]))
            .unwrap()
            .contains_member([0, 0, 32]));
    }

    #[test]
    fn native_heightfield_anchor_deletion_and_empty_update_keep_pinned_source() {
        let mut w = writer();
        let xy = [0, 0];
        install(&mut w, xy, &column(xy, &[-33, 0, 33]));
        let first = root(&mut w, 1);
        let (mut source, _) = admitted(&w, &first);
        let old = source.clone();
        w.remove([[0, 0, -33]]).unwrap();
        let second = root(&mut w, 2);
        let (_, parts) = admitted(&w, &second);
        assert_eq!(source.replace_heightfield_columns([0, 0], &parts), vec![xy]);
        assert!(source
            .ground_record(GroundKey::HeightfieldV1([0, 0, -33]))
            .is_none());
        assert_eq!(
            source
                .ground_record(GroundKey::HeightfieldV1([0, 0, 0]))
                .unwrap()
                .members()
                .collect::<Vec<_>>(),
            vec![0, 33]
        );
        assert_eq!(
            old.ground_record(GroundKey::HeightfieldV1([0, 0, -33]))
                .unwrap()
                .members()
                .count(),
            3
        );
        assert_eq!(source.replace_heightfield_columns([0, 0], &[]), vec![xy]);
        assert_eq!(source.ground_records(xy).count(), 0);
        assert!(source.replace_heightfield_columns([0, 0], &[]).is_empty());
    }

    #[test]
    fn native_legacy_borrowing_keeps_plane_identity_and_exact_members() {
        let mut w = writer();
        w.observe([[0, 0, 0]], 20).unwrap();
        let layer = nuwax_map_chunks::LayerObservation {
            members: vec![0],
            normal: [0., 0., 1.],
            plane_origin: [0.025, 0.025, 0.02],
            samples: vec![nuwax_map_chunks::LayerSample {
                source: Default::default(),
                point: [0.025, 0.025, 0.02],
                source_sequence: 1,
                source_timestamp_ns: 20,
                point_index: 0,
            }],
        };
        w.replace_layer_observations([0, 0, 0], [(0, layer)].into())
            .unwrap();
        let root = root(&mut w, 1);
        let (source, _) = admitted(&w, &root);
        let borrowed = source.ground_records([0, 0]).next().unwrap();
        let original = source.ground_layers([0, 0]).next().unwrap();
        let SourceGroundRecord::LegacyPlane(record) = borrowed else {
            panic!("legacy");
        };
        assert!(std::ptr::eq(record, original));
        assert_eq!(borrowed.key(), GroundKey::LegacyPlane([0, 0, 0]));
        assert_eq!(
            borrowed.height_at([0.025, 0.025]),
            original.height_at([0.025, 0.025])
        );
        assert_eq!(borrowed.direction(), Some(original.normal));
        assert!(borrowed.same_legacy_plane(borrowed));
        assert_eq!(borrowed.members().collect::<Vec<_>>(), vec![0]);
        assert!(borrowed.contains_member([0, 0, 0]));
        assert!(!borrowed.contains_member([0, 0, 1]));
        let mut reader = RegionReader::new(w.repository().clone(), BUDGET);
        assert!(reader
            .ground_records(&root.blocks[0])
            .unwrap()
            .legacy()
            .is_ok());
    }
    #[test]
    fn native_block_only_member_removal_retires_stale_claim() {
        let mut writer = writer();
        let xy = [0, 0];
        install(&mut writer, xy, &column(xy, &[-33, 0, 33]));
        let root = root(&mut writer, 1);
        let (mut source, _) = admitted(&writer, &root);
        let old = source.clone();
        assert_eq!(
            source
                .replace_block_with_geometry_changes([0, 0, -2], BTreeMap::new())
                .unwrap(),
            vec![xy]
        );
        assert!(!source.is_complete_geometry_source());
        assert!(source.ground_records(xy).next().is_none());
        assert!(source.get([0, 0, 0]).is_some());
        assert_eq!(old.ground_records(xy).next().unwrap().members().count(), 3);
    }
}
