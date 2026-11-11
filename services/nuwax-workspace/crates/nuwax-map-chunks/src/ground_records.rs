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
    #[serde(rename = "heightfield_v1_compact")]
    HeightfieldV1(Arc<HeightfieldFragment>),
    /// Classified XY with no ground claim. Occupancy remains solid evidence.
    ClassifiedEmptyColumn {},
}
/// One original decode-cache value. Legacy readers retain their original Arc.
#[derive(Clone, Debug, PartialEq)]
pub enum GroundRecords {
    Versioned(Arc<BTreeMap<u16, GroundRecord>>),
}
#[derive(Clone, Copy, PartialEq)]
pub enum GroundRecordRef<'a> {
    HeightfieldV1(&'a Arc<HeightfieldFragment>),
    ClassifiedEmptyColumn,
}
impl GroundRecordRef<'_> {
    /// 地面查询的原始几何；接收统计和诊断样本不改变支撑成员。
    pub fn geometry_eq(self, other: Self) -> bool {
        match (self, other) {
            (Self::HeightfieldV1(a), Self::HeightfieldV1(b)) => a.anchor == b.anchor
                && a.height_m == b.height_m && a.occupied_bounds == b.occupied_bounds
                && a.members.iter().map(|p| p.0).eq(b.members.iter().map(|p| p.0)),
            (Self::ClassifiedEmptyColumn, Self::ClassifiedEmptyColumn) => true,
            _ => false,
        }
    }
}
impl Default for GroundRecords {
    fn default() -> Self { Self::empty() }
}

pub enum GroundRecordIter<'a> {
    Versioned(std::collections::btree_map::Iter<'a, u16, GroundRecord>),
}
impl<'a> Iterator for GroundRecordIter<'a> {
    type Item = (u16, GroundRecordRef<'a>);
    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Versioned(iter) => iter.next().map(|(&k, v)| (k, match v {
                GroundRecord::HeightfieldV1(v) => GroundRecordRef::HeightfieldV1(v),
                GroundRecord::ClassifiedEmptyColumn {} => GroundRecordRef::ClassifiedEmptyColumn,
            })),
        }
    }
}
impl GroundRecords {
    pub(crate) fn empty() -> Self { Self::Versioned(Arc::new(BTreeMap::new())) }
    pub fn iter(&self) -> GroundRecordIter<'_> {
        match self { Self::Versioned(v) => GroundRecordIter::Versioned(v.iter()) }
    }
    pub fn get(&self, key: u16) -> Option<GroundRecordRef<'_>> {
        match self { Self::Versioned(v) => v.get(&key).map(GroundRecord::as_ref) }
    }
    pub(crate) fn versioned_mut(&mut self) -> &mut BTreeMap<u16, GroundRecord> {
        match self { Self::Versioned(v) => Arc::make_mut(v) }
    }
    pub fn is_empty(&self) -> bool { self.iter().next().is_none() }
    pub fn has_heightfield(&self) -> bool { self.iter().any(|(_, r)| matches!(r, GroundRecordRef::HeightfieldV1(_))) }
    pub(crate) fn bytes(&self) -> usize { self.iter().map(|(_, r)| record_bytes(r)).sum() }
    pub(crate) fn geometry_eq(&self, other: &Self) -> bool {
        let mut a = self.iter(); let mut b = other.iter();
        loop { match (a.next(), b.next()) {
            (None, None) => return true,
            (Some((ka, va)), Some((kb, vb))) if ka == kb => {
                let same = va.geometry_eq(vb);
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
    pub heightfields: Vec<HeightfieldSurface>,
}

/// 来源描述按值共享，逐样本仍保留原始坐标、序号、纳秒和点下标。
/// 旧BL4对象数组继续可读；新记录标签使旧读器明确拒绝紧凑表示。
mod sample_encoding {
    use super::*;
    use serde::de::{Error, MapAccess, Visitor};

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
                formatter.write_str("compact heightfield source table")
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
        deserializer.deserialize_map(Samples)
    }
}

/// 单条记录的常驻字节；整块字节是各条之和。
pub(crate) fn record_bytes(record: GroundRecordRef<'_>) -> usize {
    match record {
        GroundRecordRef::ClassifiedEmptyColumn => 128,
        GroundRecordRef::HeightfieldV1(v) => {
            // 来源对象按指针去重：只计每个来源第一次出现的样本，与集合去重结果相同。
            128 + v.members.len() * (8 * 8 + 2) + v.samples.len() * 64
                + v.samples.iter().enumerate()
                    .filter(|(i, s)| !v.samples[..*i].iter().any(|earlier| Arc::ptr_eq(&earlier.source, &s.source)))
                    .map(|(_, s)| s.source.resident_bytes()).sum::<usize>()
        }
    }
}
impl GroundRecord {
    pub(crate) fn as_ref(&self) -> GroundRecordRef<'_> {
        match self {
            GroundRecord::HeightfieldV1(v) => GroundRecordRef::HeightfieldV1(v),
            GroundRecord::ClassifiedEmptyColumn {} => GroundRecordRef::ClassifiedEmptyColumn,
        }
    }
}
/// Complete declared column, assembled once at the source boundary. A removed
/// zero-mass member is a real membership change, not an implicit missing packet.
pub(crate) fn assemble_column(
    xy: [i32; 2], q: f32, blocks: &[(Key, GroundRecords)],
) -> Result<ColumnGroundRecords, String> {
    let local_xy = local_key([xy[0], xy[1], 0]) % 1024;
    let block_xy = [xy[0].div_euclid(32), xy[1].div_euclid(32)];
    assemble_column_records(xy, q, blocks.iter().filter(|(block, _)| block[..2] == block_xy)
        .flat_map(|(block, records)| (0..32_u16)
            .filter_map(move |z| records.get(local_xy + 1024 * z).map(|record| (*block, record)))))
}
/// 由同一列的全部原始记录组装完整列。记录须按块顺序、块内高度升序给出，
/// 与逐块逐层查找的顺序相同，因此结果和首个错误也相同。
pub(crate) fn assemble_column_records<'a>(
    xy: [i32; 2], q: f32, records: impl IntoIterator<Item = (Key, GroundRecordRef<'a>)>,
) -> Result<ColumnGroundRecords, String> {
    let mut classified_empty = false;
    let mut groups: BTreeMap<Key, Vec<(Key, &HeightfieldFragment)>> = BTreeMap::new();
    for (block, record) in records {
        match record {
            GroundRecordRef::HeightfieldV1(v) => {
                if v.anchor[..2] != xy { return Err("heightfield anchor changes XY".into()); }
                groups.entry(v.anchor).or_default().push((block, v));
            }
            GroundRecordRef::ClassifiedEmptyColumn => {
                if classified_empty { return Err("duplicate classified empty column".into()); }
                classified_empty = true;
            }
        }
    }
    if classified_empty && !groups.is_empty() {
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
    Ok(ColumnGroundRecords { classified_empty, heightfields })
}

pub(crate) fn split_column(xy: [i32; 2], q: f32, column: &ColumnGroundRecords) -> Result<BTreeMap<Key, BTreeMap<u16, GroundRecord>>, String> {
    if column.classified_empty && !column.heightfields.is_empty() {
        return Err("classified empty column conflicts with ground records".into());
    }
    // The Writer attaches an empty declaration to one actual occupied shard.
    let mut out: BTreeMap<Key, BTreeMap<u16, GroundRecord>> = BTreeMap::new();

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
            if out.entry(block).or_default().insert(key, GroundRecord::HeightfieldV1(Arc::new(fragment))).is_some() { return Err("duplicate ground anchor".into()); }
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
