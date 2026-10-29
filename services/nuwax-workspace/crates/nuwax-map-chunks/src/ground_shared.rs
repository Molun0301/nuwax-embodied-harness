//! BL6按整块共享来源，数值固定宽度；容量由成员、样本和既有来源边界推导。
use crate::*;
use std::collections::{HashMap, HashSet};

pub(crate) fn source_bytes(source: &LayerSource) -> usize {
    match source {
        LayerSource::Legacy => 1,
        LayerSource::Lidar { sensor_frame_id, .. } => 17 + sensor_frame_id.len(),
        LayerSource::Auxiliary { sensor_id, clock_id, .. } => 25 + sensor_id.len() + clock_id.len(),
    }
}

fn samples(record: GroundRecordRef<'_>) -> &[LayerSample] {
    match record {
        GroundRecordRef::LegacyPlane(v) => &v.samples,
        GroundRecordRef::HeightfieldV1(v) => &v.samples,
        GroundRecordRef::ClassifiedEmptyColumn => &[],
    }
}

struct Sources<'a> {
    values: Vec<&'a Arc<LayerSource>>,
    indices: HashMap<*const LayerSource, u32>,
    bytes: usize,
}
impl<'a> Sources<'a> {
    fn collect(records: &'a GroundRecords) -> Self {
        let mut pointers = HashSet::new();
        let mut originals = Vec::new();
        let mut values = BTreeSet::new();
        for sample in records.iter().flat_map(|(_, record)| samples(record)) {
            if pointers.insert(Arc::as_ptr(&sample.source)) {
                originals.push(&sample.source);
                values.insert(&sample.source);
            }
        }
        let values: Vec<_> = values.into_iter().collect();
        let bytes = values.iter().map(|s| source_bytes(s)).sum();
        let indices = originals.into_iter().map(|source| (Arc::as_ptr(source),
            values.binary_search(&source).expect("已收集来源") as u32)).collect();
        Self { values, indices, bytes }
    }
}

/// 根未声明的旧诊断来源仍有原整块4MiB兼容额度，不推造来源水位。
pub(crate) fn validate_unregistered_sources(records: &GroundRecords, declared: &BTreeSet<&LayerSource>) -> Result<(), String> {
    let sources = Sources::collect(records);
    let unregistered: usize = sources.values.iter().filter(|s| !declared.contains(s.as_ref()))
        .map(|s| source_bytes(s)).sum();
    if unregistered > MAX_V1_TO_V5_BLOCK_BYTES {
        return Err("BL6 unregistered source metadata exceeds legacy block allowance".into());
    }
    Ok(())
}

pub(crate) fn declared_sources(root: &Manifest) -> Result<BTreeSet<&LayerSource>, String> {
    let sources: BTreeSet<_> = root.source_watermarks.iter().map(|s| s.source.as_ref()).collect();
    if sources.iter().map(|s| source_bytes(s)).sum::<usize>() > MAX_MANIFEST_BYTES {
        return Err("declared source metadata exceeds root budget".into());
    }
    Ok(sources)
}

pub(crate) fn admit(info: Intrinsic, records: &GroundRecords, root: &Manifest) -> Result<(), String> {
    if info.source_budget_required { validate_unregistered_sources(records, &declared_sources(root)?)?; }
    Ok(())
}

struct Output(Vec<u8>);
impl Output {
    fn put(&mut self, bytes: &[u8]) -> Result<(), String> {
        if bytes.len() > MAX_BLOCK_BYTES - self.0.len() { return Err("BL6 exceeds proven byte bound".into()); }
        self.0.extend_from_slice(bytes); Ok(())
    }
    fn floats(&mut self, values: &[f64]) -> Result<(), String> {
        for value in values { self.put(&value.to_le_bytes())?; } Ok(())
    }
    fn string(&mut self, value: &str) -> Result<(), String> {
        self.put(&(value.len() as u32).to_le_bytes())?; self.put(value.as_bytes())
    }
    fn source(&mut self, source: &LayerSource) -> Result<(), String> {
        match source {
            LayerSource::Legacy => self.put(&[0])?,
            LayerSource::Lidar { backend, runtime_generation, sensor_frame_id } => {
                self.put(&[1])?; self.put(&backend.to_le_bytes())?;
                self.put(&runtime_generation.to_le_bytes())?; self.string(sensor_frame_id)?;
            }
            LayerSource::Auxiliary { sensor_id, clock_id, runtime_generation, session_generation } => {
                self.put(&[2])?; self.put(&runtime_generation.to_le_bytes())?;
                self.put(&session_generation.to_le_bytes())?; self.string(sensor_id)?; self.string(clock_id)?;
            }
        } Ok(())
    }
    fn samples(&mut self, values: &[LayerSample], sources: &Sources<'_>) -> Result<(), String> {
        self.put(&[values.len() as u8])?;
        for sample in values {
            self.put(&sources.indices[&Arc::as_ptr(&sample.source)].to_le_bytes())?;
            self.floats(&sample.point)?; self.put(&sample.source_sequence.to_le_bytes())?;
            self.put(&sample.source_timestamp_ns.to_le_bytes())?; self.put(&sample.point_index.to_le_bytes())?;
        } Ok(())
    }
}

pub(crate) fn encode(block: &Block, records: &GroundRecords) -> Result<Vec<u8>, String> {
    let sources = Sources::collect(records);
    if sources.bytes > MAX_BLOCK_SOURCE_BYTES { return Err("BL6 source metadata exceeds root and legacy allowance".into()); }
    let mode = match records {
        GroundRecords::Versioned(_) => 2,
        GroundRecords::Legacy(_) => u8::from(sources.values.iter().any(|s| !s.is_legacy())),
    };
    let mut out = Output(encode_block(block)?); let start = out.0.len();
    out.put(&[mode])?; out.put(&(sources.bytes as u32).to_le_bytes())?;
    out.put(&(sources.values.len() as u32).to_le_bytes())?;
    for source in &sources.values { out.source(source)?; }
    out.put(&(records.iter().count() as u32).to_le_bytes())?;
    for (key, record) in records.iter() {
        out.put(&key.to_le_bytes())?;
        match record {
            GroundRecordRef::LegacyPlane(layer) => {
                out.put(&[0, layer.members.len() as u8])?;
                for member in &layer.members { out.put(&member.to_le_bytes())?; }
                out.floats(&layer.plane_origin)?; out.floats(&layer.normal)?;
                out.samples(&layer.samples, &sources)?;
            }
            GroundRecordRef::HeightfieldV1(field) => {
                out.put(&[1])?;
                for coord in field.anchor { out.put(&coord.to_le_bytes())?; }
                out.floats(&[field.height_m])?; out.floats(&field.occupied_bounds)?;
                out.put(&[field.members.len() as u8])?;
                for (member, bins) in &field.members { out.put(&member.to_le_bytes())?; out.floats(bins)?; }
                out.samples(&field.samples, &sources)?;
            }
            GroundRecordRef::ClassifiedEmptyColumn => out.put(&[2])?,
        }
    }
    let size = (out.0.len() - start) as u32;
    out.0[..8].copy_from_slice(b"NUWAXBL6"); out.0[12..16].copy_from_slice(&size.to_le_bytes()); Ok(out.0)
}

struct Input<'a>(&'a [u8]);
impl<'a> Input<'a> {
    fn take(&mut self, count: usize) -> Result<&'a [u8], String> {
        let bytes = self.0.get(..count).ok_or("truncated BL6 ground record")?;
        self.0 = &self.0[count..]; Ok(bytes)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], String> { Ok(self.take(N)?.try_into().unwrap()) }
    fn byte(&mut self) -> Result<u8, String> { Ok(self.array::<1>()?[0]) }
    fn count(&mut self, maximum: usize) -> Result<usize, String> {
        let value = usize::from(self.byte()?);
        if value > maximum { return Err("BL6 collection exceeds structural bound".into()); } Ok(value)
    }
    fn floats<const N: usize>(&mut self) -> Result<[f64; N], String> {
        let mut values = [0.; N]; for value in &mut values { *value = f64::from_le_bytes(self.array()?); } Ok(values)
    }
    fn string(&mut self) -> Result<String, String> {
        let size = u32::from_le_bytes(self.array()?) as usize;
        String::from_utf8(self.take(size)?.to_vec()).map_err(|e| e.to_string())
    }
    fn source(&mut self) -> Result<Arc<LayerSource>, String> {
        let source = match self.byte()? {
            0 => LayerSource::Legacy,
            1 => LayerSource::Lidar { backend: i32::from_le_bytes(self.array()?),
                runtime_generation: i64::from_le_bytes(self.array()?), sensor_frame_id: self.string()? },
            2 => {
                let runtime_generation = u64::from_le_bytes(self.array()?);
                let session_generation = u64::from_le_bytes(self.array()?);
                LayerSource::Auxiliary { runtime_generation, session_generation, sensor_id: self.string()?, clock_id: self.string()? }
            }
            _ => return Err("unknown BL6 source tag".into()),
        };
        source.validate()?; Ok(Arc::new(source))
    }
    fn samples(&mut self, sources: &[Arc<LayerSource>], used: &mut [bool]) -> Result<Vec<LayerSample>, String> {
        let mut samples = Vec::new();
        for _ in 0..self.count(MAX_LAYER_SAMPLES)? {
            let index = u32::from_le_bytes(self.array()?) as usize;
            let source = sources.get(index).ok_or("invalid BL6 sample source")?.clone(); used[index] = true;
            samples.push(LayerSample { source, point: self.floats()?,
                source_sequence: u64::from_le_bytes(self.array()?), source_timestamp_ns: u64::from_le_bytes(self.array()?),
                point_index: u32::from_le_bytes(self.array()?) });
        } Ok(samples)
    }
}

pub(crate) fn decode(bytes: &[u8]) -> Result<GroundRecords, String> {
    let mut input = Input(bytes); let mode = input.byte()?;
    if mode > 2 { return Err("unknown BL6 ground mode".into()); }
    let size = u32::from_le_bytes(input.array()?) as usize;
    let count = u32::from_le_bytes(input.array()?) as usize;
    if size > MAX_BLOCK_SOURCE_BYTES || count > MAX_LAYERS_PER_BLOCK * MAX_LAYER_SAMPLES {
        return Err("BL6 sources exceed structural bound".into());
    }
    let mut body = Input(input.take(size)?); let mut sources: Vec<Arc<LayerSource>> = Vec::new();
    for _ in 0..count {
        let source = body.source()?;
        if sources.last().is_some_and(|old| old >= &source) { return Err("unsorted or duplicate BL6 sources".into()); }
        sources.push(source);
    }
    if !body.0.is_empty() { return Err("BL6 source length mismatch".into()); }
    if mode < 2 && (mode == 1) != sources.iter().any(|s| !s.is_legacy()) { return Err("BL6 source mode mismatch".into()); }
    let mut used = vec![false; count];
    let count = u32::from_le_bytes(input.array()?) as usize;
    if count == 0 || count > MAX_LAYERS_PER_BLOCK { return Err("invalid BL6 record count".into()); }
    let mut records = BTreeMap::new();
    for _ in 0..count {
        let key = u16::from_le_bytes(input.array()?);
        if records.last_key_value().is_some_and(|(&old, _)| old >= key) { return Err("invalid ground record anchors".into()); }
        let record = match input.byte()? {
            0 => {
                let mut members = Vec::new();
                for _ in 0..input.count(MAX_LAYER_MEMBERS)? { members.push(u16::from_le_bytes(input.array()?)); }
                GroundRecord::LegacyPlane(LayerObservation { members, plane_origin: input.floats()?, normal: input.floats()?,
                    samples: input.samples(&sources, &mut used)? })
            }
            1 if mode == 2 => {
                let mut anchor = [0; 3]; for coord in &mut anchor { *coord = i32::from_le_bytes(input.array()?); }
                let height_m = input.floats::<1>()?[0]; let occupied_bounds = input.floats()?;
                let mut members = Vec::new();
                for _ in 0..input.count(MAX_LAYER_MEMBERS)? { members.push((u16::from_le_bytes(input.array()?), input.floats()?)); }
                GroundRecord::HeightfieldV1(HeightfieldFragment { anchor, height_m, occupied_bounds, members,
                    samples: input.samples(&sources, &mut used)? })
            }
            2 if mode == 2 => GroundRecord::ClassifiedEmptyColumn {},
            _ => return Err("unknown BL6 ground record tag or mode".into()),
        };
        records.insert(key, record);
    }
    if !input.0.is_empty() || used.contains(&false) { return Err("trailing BL6 bytes or unused source".into()); }
    if mode == 2 { Ok(GroundRecords::Versioned(Arc::new(records))) } else {
        Ok(GroundRecords::Legacy(Arc::new(records.into_iter().map(|(key, value)| {
            let GroundRecord::LegacyPlane(value) = value else { unreachable!() }; (key, value)
        }).collect())))
    }
}
