//! Versioned ground records in the original immutable blocks. No classifier.
use crate::*;
use crate::heightfield_observations::{HeightfieldColumn, HEIGHT_SUB_BINS};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HeightfieldFragment {
    /// Minimum real global member of this surface in this snapshot.
    pub anchor: Key,
    pub height_m: f64,
    pub occupied_bounds: [f64; 2],
    /// Local member plus its share of the whole-column distribution.
    pub members: Vec<(u16, [f64; HEIGHT_SUB_BINS])>,
    /// Bounded original diagnostics; never the distribution or contact extent.
    #[serde(with = "sample_encoding")]
    pub samples: Vec<LayerSample>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "representation", content = "data", rename_all = "snake_case", deny_unknown_fields)]
pub enum GroundRecord {
    LegacyPlane(LayerObservation),
    #[serde(rename = "heightfield_v1_compact", alias = "heightfield_v1")]
    HeightfieldV1(HeightfieldFragment),
    /// Classified XY with no ground claim. Occupancy remains solid evidence.
    ClassifiedEmptyColumn {},
}
/// One original decode-cache value. Legacy readers retain their original Arc.
#[derive(Clone, Debug, PartialEq)]
pub enum GroundRecords {
    Legacy(Arc<LayerObservations>),
    Versioned(Arc<BTreeMap<u16, GroundRecord>>),
}
#[derive(Clone, Copy, PartialEq)]
pub enum GroundRecordRef<'a> {
    LegacyPlane(&'a LayerObservation),
    HeightfieldV1(&'a HeightfieldFragment),
    ClassifiedEmptyColumn,
}
pub enum GroundRecordIter<'a> {
    Legacy(std::collections::btree_map::Iter<'a, u16, LayerObservation>),
    Versioned(std::collections::btree_map::Iter<'a, u16, GroundRecord>),
}
impl<'a> Iterator for GroundRecordIter<'a> {
    type Item = (u16, GroundRecordRef<'a>);
    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Legacy(iter) => iter.next().map(|(&k, v)| (k, GroundRecordRef::LegacyPlane(v))),
            Self::Versioned(iter) => iter.next().map(|(&k, v)| (k, match v {
                GroundRecord::LegacyPlane(v) => GroundRecordRef::LegacyPlane(v),
                GroundRecord::HeightfieldV1(v) => GroundRecordRef::HeightfieldV1(v),
                GroundRecord::ClassifiedEmptyColumn {} => GroundRecordRef::ClassifiedEmptyColumn,
            })),
        }
    }
}
impl GroundRecords {
    pub(crate) fn empty() -> Self { Self::Legacy(Arc::new(BTreeMap::new())) }
    pub fn legacy(&self) -> Result<Arc<LayerObservations>, String> {
        match self {
            Self::Legacy(records) => Ok(records.clone()),
            Self::Versioned(_) => Err("versioned ground requires the typed ground-record consumer".into()),
        }
    }
    pub(crate) fn into_legacy(self) -> Result<LayerObservations, String> {
        match self {
            Self::Legacy(v) => Ok(Arc::try_unwrap(v).unwrap_or_else(|v| v.as_ref().clone())),
            Self::Versioned(_) => Err("versioned ground requires the typed ground-record consumer".into()),
        }
    }
    pub fn iter(&self) -> GroundRecordIter<'_> {
        match self { Self::Legacy(v) => GroundRecordIter::Legacy(v.iter()), Self::Versioned(v) => GroundRecordIter::Versioned(v.iter()) }
    }
    /// Borrow one admitted block-local record without copying its members or bins.
    pub fn get(&self, key: u16) -> Option<GroundRecordRef<'_>> {
        match self {
            Self::Legacy(v) => v.get(&key).map(GroundRecordRef::LegacyPlane),
            Self::Versioned(v) => v.get(&key).map(|r| match r {
                GroundRecord::LegacyPlane(v) => GroundRecordRef::LegacyPlane(v),
                GroundRecord::HeightfieldV1(v) => GroundRecordRef::HeightfieldV1(v),
                GroundRecord::ClassifiedEmptyColumn {} => GroundRecordRef::ClassifiedEmptyColumn,
            }),
        }
    }
    pub(crate) fn versioned_mut(&mut self) -> &mut BTreeMap<u16, GroundRecord> {
        if matches!(self, Self::Legacy(_)) { *self = Self::Versioned(Arc::new(self.owned_versioned())); }
        match self { Self::Versioned(v) => Arc::make_mut(v), Self::Legacy(_) => unreachable!() }
    }
    pub(crate) fn compact(self, block: &Block) -> Self {
        match self {
            Self::Versioned(v) if v.values().all(|r| match r {
                GroundRecord::LegacyPlane(layer) => layer.latest_timestamp_ns() <= layer.members.iter().map(|k| block[k].observed_ns).max().unwrap_or(0),
                GroundRecord::HeightfieldV1(_) | GroundRecord::ClassifiedEmptyColumn {} => false,
            }) => {
                let records = Arc::try_unwrap(v).unwrap_or_else(|v| v.as_ref().clone());
                Self::Legacy(Arc::new(records.into_iter().filter_map(|(k,r)| match r {
                    GroundRecord::LegacyPlane(v) => Some((k,v)), GroundRecord::HeightfieldV1(_) | GroundRecord::ClassifiedEmptyColumn {} => None,
                }).collect()))
            }
            other => other,
        }
    }
    pub fn is_empty(&self) -> bool { self.iter().next().is_none() }
    pub fn has_heightfield(&self) -> bool { self.iter().any(|(_, r)| matches!(r, GroundRecordRef::HeightfieldV1(_))) }
    pub(crate) fn owned_versioned(&self) -> BTreeMap<u16, GroundRecord> {
        match self {
            Self::Legacy(v) => v.iter().map(|(&k, v)| (k, GroundRecord::LegacyPlane(v.clone()))).collect(),
            Self::Versioned(v) => v.as_ref().clone(),
        }
    }
    pub(crate) fn legacy_mut(&mut self) -> Result<&mut LayerObservations, String> {
        match self {
            Self::Legacy(records) => Ok(Arc::make_mut(records)),
            Self::Versioned(_) => Err("versioned ground requires whole-column mutation".into()),
        }
    }
    pub(crate) fn bytes(&self) -> usize {
        self.iter().map(|(_, r)| match r {
            GroundRecordRef::LegacyPlane(v) => terrain_observations::layer_bytes(v),
            GroundRecordRef::ClassifiedEmptyColumn => 128,
            GroundRecordRef::HeightfieldV1(v) => {
                let mut sources = BTreeSet::new();
                128 + v.members.len() * (8 * 8 + 2) + v.samples.len() * 64
                    + v.samples.iter().filter(|s| sources.insert(Arc::as_ptr(&s.source))).map(|s| s.source.resident_bytes()).sum::<usize>()
            }
        }).sum()
    }
    pub(crate) fn geometry_eq(&self, other: &Self) -> bool {
        let mut a = self.iter(); let mut b = other.iter();
        loop { match (a.next(), b.next()) {
            (None, None) => return true,
            (Some((ka, va)), Some((kb, vb))) if ka == kb => {
                let same = match (va, vb) {
                    (GroundRecordRef::LegacyPlane(a), GroundRecordRef::LegacyPlane(b)) => a.geometry_eq(b),
                    (GroundRecordRef::HeightfieldV1(a), GroundRecordRef::HeightfieldV1(b)) =>
                        a.anchor == b.anchor && a.height_m == b.height_m && a.occupied_bounds == b.occupied_bounds
                            && a.members.iter().map(|p| p.0).eq(b.members.iter().map(|p| p.0)),
                    (GroundRecordRef::ClassifiedEmptyColumn, GroundRecordRef::ClassifiedEmptyColumn) => true,
                    _ => false,
                };
                if !same { return false; }
            }
            _ => return false,
        }}
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct HeightfieldSurface {
    pub column: HeightfieldColumn,
    pub samples: Vec<LayerSample>,
}
#[derive(Clone, Debug, PartialEq)]
pub struct ColumnGroundRecords {
    /// Mutually exclusive with actual ground records. No height or permission.
    pub classified_empty: bool,
    pub legacy: Vec<(Key, LayerObservation)>,
    pub heightfields: Vec<HeightfieldSurface>,
}

/// 来源描述按值共享，逐样本仍保留原始坐标、序号、纳秒和点下标。
/// 旧BL4对象数组继续可读；新记录标签使旧读器明确拒绝紧凑表示。
mod sample_encoding {
    use super::*;
    use serde::de::{Error, MapAccess, SeqAccess, Visitor};

    type NumericSample = (u32, [f64; 3], u64, u64, u32);

    #[derive(Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Compact {
        sources: Vec<Arc<LayerSource>>,
        values: Vec<NumericSample>,
    }

    pub fn serialize<S: serde::Serializer>(
        samples: &[LayerSample],
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        if samples.len() > MAX_LAYER_SAMPLES {
            return Err(serde::ser::Error::custom(
                "heightfield sample count exceeds budget",
            ));
        }
        let sources: Vec<_> = samples
            .iter()
            .map(|s| s.source.clone())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        let values = samples
            .iter()
            .map(|sample| {
                (
                    sources
                        .binary_search(&sample.source)
                        .expect("sample source was collected") as u32,
                    sample.point,
                    sample.source_sequence,
                    sample.source_timestamp_ns,
                    sample.point_index,
                )
            })
            .collect();
        Compact { sources, values }.serialize(serializer)
    }

    pub fn deserialize<'de, D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Vec<LayerSample>, D::Error> {
        struct Samples;
        impl<'de> Visitor<'de> for Samples {
            type Value = Vec<LayerSample>;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("heightfield samples or a compact source table")
            }

            fn visit_seq<A: SeqAccess<'de>>(self, mut input: A) -> Result<Self::Value, A::Error> {
                let mut samples = Vec::new();
                let mut sources = BTreeSet::new();
                while let Some(mut sample) = input.next_element::<LayerSample>()? {
                    if samples.len() == MAX_LAYER_SAMPLES {
                        return Err(A::Error::custom("heightfield sample count exceeds budget"));
                    }
                    // 旧数据的来源值相同也只拥有一个不可变描述，不改变任一样本身份。
                    if let Some(source) = sources.get(&sample.source) {
                        sample.source = Arc::clone(source);
                    } else {
                        sources.insert(sample.source.clone());
                    }
                    samples.push(sample);
                }
                Ok(samples)
            }

            fn visit_map<A: MapAccess<'de>>(self, input: A) -> Result<Self::Value, A::Error> {
                let compact =
                    Compact::deserialize(serde::de::value::MapAccessDeserializer::new(input))?;
                if compact.values.len() > MAX_LAYER_SAMPLES
                    || compact.sources.len() > MAX_LAYER_SAMPLES
                    || compact.sources.windows(2).any(|v| v[0] >= v[1])
                {
                    return Err(A::Error::custom("invalid compact heightfield source table"));
                }
                for source in &compact.sources {
                    source.validate().map_err(A::Error::custom)?;
                }
                let mut used = vec![false; compact.sources.len()];
                let mut samples = Vec::with_capacity(compact.values.len());
                for (index, point, source_sequence, source_timestamp_ns, point_index) in
                    compact.values
                {
                    let source = compact
                        .sources
                        .get(index as usize)
                        .ok_or_else(|| A::Error::custom("heightfield source index out of range"))?;
                    used[index as usize] = true;
                    samples.push(LayerSample {
                        source: source.clone(),
                        point,
                        source_sequence,
                        source_timestamp_ns,
                        point_index,
                    });
                }
                if used.iter().any(|value| !value) {
                    return Err(A::Error::custom("unreferenced heightfield source"));
                }
                Ok(samples)
            }
        }
        deserializer.deserialize_any(Samples)
    }
}

pub(crate) fn validate_records(records: &GroundRecords, block: &Block) -> Result<(), String> {
    if let GroundRecords::Legacy(v) = records { return terrain_observations::validate_layers(v, block); }
    if records.iter().count() > MAX_LAYERS_PER_BLOCK { return Err("ground record count exceeds block budget".into()); }
    let mut latest = None;
    let mut claimed = BTreeSet::new();
    let mut empty_columns = BTreeSet::new();
    let mut ground_columns = BTreeSet::new();
    for (key, record) in records.iter() {
        match record {
            GroundRecordRef::LegacyPlane(v) => {
                ground_columns.insert(key % 1024);
                terrain_observations::validate_versioned_legacy(&BTreeMap::from([(key, v.clone())]), block, *latest.get_or_insert_with(|| block.values().map(|v| v.observed_ns).max().unwrap_or(0)))?;
                if v.members.iter().any(|m| !claimed.insert(*m)) { return Err("overlapping ground record members".into()); }
            }
            GroundRecordRef::HeightfieldV1(v) => {
                ground_columns.insert(key % 1024);
                if v.members.is_empty() || v.members.len() > MAX_LAYER_MEMBERS || v.members[0].0 != key
                    || v.members.windows(2).any(|w| w[0].0 >= w[1].0)
                    || !v.height_m.is_finite() || !v.occupied_bounds.iter().all(|n| n.is_finite())
                    || v.occupied_bounds[0] >= v.occupied_bounds[1]
                    || v.members.iter().any(|(m, mass)| *m >= 32768 || *m % 1024 != key % 1024
                        || !block.contains_key(m) || !claimed.insert(*m)
                        || mass.iter().any(|n| !n.is_finite() || *n < 0.0)) {
                    return Err("invalid heightfield fragment members or geometry".into());
                }
                terrain_observations::validate_diagnostic_samples(&v.samples)?;
                if v.samples.iter().map(|s| s.source_timestamp_ns).max().unwrap_or(0)
                    > v.members.iter().map(|(m, _)| block[m].observed_ns).max().unwrap_or(0) {
                    return Err("heightfield sample exceeds occupancy evidence".into());
                }
            }
            GroundRecordRef::ClassifiedEmptyColumn => {
                if key >= 1024 || !(0..32_u16).any(|z| block.contains_key(&(key + z*1024))) {
                    return Err("classified empty column lacks local occupancy".into());
                }
                empty_columns.insert(key);
            }
        }
    }
    if !empty_columns.is_disjoint(&ground_columns) {
        return Err("classified empty column conflicts with ground records".into());
    }
    Ok(())
}

/// Only the new BL4 variant can retain a diagnostic detached from ground
/// membership. Original members already prove the old contract. Any newer
/// diagnostic must be backed by its own actual occupied voxel in this block.
pub(crate) fn validate_legacy_sample_occupancy(records: &GroundRecords, block: &Block, key: Key, q: f32) -> Result<(), String> {
    let GroundRecords::Versioned(records) = records else { return Ok(()); };
    let reciprocal = 1.0 / f64::from(q);
    for record in records.values() {
        let GroundRecord::LegacyPlane(layer) = record else { continue; };
        let member_time = layer.members.iter().map(|k| block[k].observed_ns).max().unwrap_or(0);
        for sample in layer.samples.iter().filter(|s| s.source_timestamp_ns > member_time) {
            let cells = sample.point.map(|v| (v * reciprocal).floor());
            if cells.iter().any(|v| !v.is_finite() || *v < f64::from(i32::MIN) || *v > f64::from(i32::MAX)) {
                return Err("legacy diagnostic lacks actual occupancy evidence".into());
            }
            let actual = cells.map(|v| v as i32);
            if block_key(actual) != key || !block.get(&local_key(actual)).is_some_and(|v| sample.source_timestamp_ns <= v.observed_ns) {
                return Err("legacy diagnostic lacks actual occupancy evidence".into());
            }
        }
    }
    Ok(())
}

/// Complete declared column, assembled once at the source boundary. A removed
/// zero-mass member is a real membership change, not an implicit missing packet.
pub(crate) fn assemble_column(
    xy: [i32; 2], q: f32, blocks: &[(Key, GroundRecords)],
) -> Result<ColumnGroundRecords, String> {
    let mut legacy = Vec::new();
    let mut classified_empty = false;
    let mut groups: BTreeMap<Key, Vec<(Key, &HeightfieldFragment)>> = BTreeMap::new();
    let local_xy = local_key([xy[0], xy[1], 0]) % 1024;
    for (block, records) in blocks {
        if block[..2] != [xy[0].div_euclid(32), xy[1].div_euclid(32)] { continue; }
        for z in 0..32_u16 {
            let key = local_xy + 1024 * z;
            match records.get(key) {
                Some(GroundRecordRef::LegacyPlane(v)) => legacy.push((*block, v.clone())),
                Some(GroundRecordRef::HeightfieldV1(v)) => {
                    if v.anchor[..2] != xy { return Err("heightfield anchor changes XY".into()); }
                    groups.entry(v.anchor).or_default().push((*block, v));
                }
                Some(GroundRecordRef::ClassifiedEmptyColumn) => {
                    if classified_empty { return Err("duplicate classified empty column".into()); }
                    classified_empty = true;
                }
                None => {},
            }
        }
    }
    if classified_empty && (!legacy.is_empty() || !groups.is_empty()) {
        return Err("classified empty column conflicts with ground records".into());
    }
    let mut heightfields = Vec::new();
    for (anchor, fragments) in groups {
        let first = fragments[0].1;
        let mut mass = BTreeMap::new(); let mut samples = Vec::new();
        for (block, fragment) in fragments {
            if fragment.height_m != first.height_m || fragment.occupied_bounds != first.occupied_bounds {
                return Err("heightfield fragments disagree on geometry".into());
            }
            for (local, bins) in &fragment.members {
                let key = global_key(block, *local)?;
                if key[..2] != xy || mass.insert(key[2], *bins).is_some() { return Err("overlapping heightfield fragments".into()); }
            }
            samples.extend(fragment.samples.iter().cloned());
        }
        let column = HeightfieldColumn::from_normalized_members(xy, q, mass)?;
        if [xy[0], xy[1], column.members().next().unwrap().0] != anchor
            || column.height() != first.height_m || column.occupied_bounds() != first.occupied_bounds {
            return Err("heightfield declared geometry differs from complete members".into());
        }
        // Diagnostics are bounded per fragment; complete distributions are not
        // reconstructed from them. Retain only the original bounded return set.
        samples.truncate(MAX_LAYER_SAMPLES);
        heightfields.push(HeightfieldSurface { column, samples });
    }
    Ok(ColumnGroundRecords { classified_empty, legacy, heightfields })
}

pub(crate) fn split_column(xy: [i32; 2], q: f32, column: &ColumnGroundRecords) -> Result<BTreeMap<Key, BTreeMap<u16, GroundRecord>>, String> {
    if column.classified_empty && (!column.legacy.is_empty() || !column.heightfields.is_empty()) {
        return Err("classified empty column conflicts with ground records".into());
    }
    // The Writer attaches an empty declaration to one actual occupied shard.
    let mut out: BTreeMap<Key, BTreeMap<u16, GroundRecord>> = BTreeMap::new();
    for (block, layer) in &column.legacy {
        let key = global_key(*block, *layer.members.first().ok_or("empty legacy column")?)?;
        if key[..2] != xy { return Err("legacy column changes XY".into()); }
        if out.entry(*block).or_default().insert(layer.members[0], GroundRecord::LegacyPlane(layer.clone())).is_some() {
            return Err("duplicate ground anchor".into());
        }
    }
    for surface in &column.heightfields {
        if surface.column.xy() != xy || surface.column.voxel_size_m() != q { return Err("heightfield column changes XY/resolution".into()); }
        terrain_observations::validate_diagnostic_samples(&surface.samples)?;
        let anchor = [xy[0], xy[1], surface.column.members().next().unwrap().0];
        let height_m = surface.column.height();
        let occupied_bounds = surface.column.occupied_bounds();
        let mut fragments: BTreeMap<Key, HeightfieldFragment> = BTreeMap::new();
        for (z, bins) in surface.column.members() {
            let key = [xy[0], xy[1], z]; let block = block_key(key);
            fragments.entry(block).or_insert_with(|| HeightfieldFragment {
                anchor, height_m, occupied_bounds,
                members: Vec::new(), samples: Vec::new(),
            }).members.push((local_key(key), *bins));
        }
        for sample in &surface.samples {
            let key = sample.point.map(|v| (v * (1.0 / f64::from(q))).floor() as i32);
            if key[..2] == xy {
                if let Some(fragment) = fragments.get_mut(&block_key(key)) {
                    if fragment.members.iter().any(|p| p.0 == local_key(key)) { fragment.samples.push(sample.clone()); }
                }
            }
        }
        for (block, fragment) in fragments {
            let key = fragment.members[0].0;
            if out.entry(block).or_default().insert(key, GroundRecord::HeightfieldV1(fragment)).is_some() { return Err("duplicate ground anchor".into()); }
        }
    }
    Ok(out)
}

/// Root-first validation only. Incremental roots pass just changed/deleted XY;
/// legacy roots never enter this traversal. Query consumers use admitted values.
pub(crate) fn validate_root_columns(
    root: &Manifest, reader: &mut RegionReader, columns: &BTreeSet<[i32; 2]>,
) -> Result<(), String> {
    let mut by_block_xy: BTreeMap<[i32; 2], Vec<[i32; 2]>> = BTreeMap::new();
    for xy in columns { by_block_xy.entry([xy[0].div_euclid(32), xy[1].div_euclid(32)]).or_default().push(*xy); }
    for (bxy, columns) in by_block_xy {
        let low = [bxy[0], bxy[1], i32::MIN]; let high = [bxy[0], bxy[1], i32::MAX];
        let start = root.blocks.partition_point(|r| r.key < low);
        let mut records = Vec::new();
        for r in root.blocks[start..].iter().take_while(|r| r.key <= high) {
            records.push((r.key, reader.ground_records(r)?));
        }
        for xy in columns { assemble_column(xy, root.voxel_size_m, &records)?; }
    }
    Ok(())
}
pub(crate) fn heightfield_columns(block: Key, records: &GroundRecords, out: &mut BTreeSet<[i32; 2]>) -> Result<(), String> {
    // In root5 a changed Legacy record may conflict with an unchanged empty
    // marker in another z shard. Collect all records in only the touched block.
    for (key, _) in records.iter() {
        let global = global_key(block, key)?; out.insert([global[0], global[1]]);
    }
    Ok(())
}
