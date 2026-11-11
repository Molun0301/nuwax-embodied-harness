//! BL5：数值按原位宽小端保存；每条记录共享来源表，不改动地面语义。
use crate::*;

#[cfg(test)]
struct Output(Vec<u8>);
#[cfg(test)]
impl Output {
    fn put(&mut self, bytes: &[u8]) -> Result<(), String> {
        if bytes.len() > MAX_V1_TO_V5_BLOCK_BYTES - self.0.len() {
            return Err("ground block exceeds byte budget".into());
        }
        self.0.extend_from_slice(bytes);
        Ok(())
    }
    fn floats(&mut self, values: &[f64]) -> Result<(), String> {
        for value in values { self.put(&value.to_le_bytes())?; }
        Ok(())
    }
    fn samples(&mut self, samples: &[LayerSample]) -> Result<(), String> {
        let sources: Vec<_> = samples.iter().map(|s| &s.source).collect::<BTreeSet<_>>().into_iter().collect();
        self.put(&[sources.len() as u8])?;
        for source in &sources {
            // 来源描述沿用严格JSON结构；只有每条记录的来源表使用文本。
            let bytes = serde_json::to_vec(source).map_err(|e| e.to_string())?;
            let size = u32::try_from(bytes.len()).map_err(|_| "ground block exceeds byte budget")?;
            self.put(&size.to_le_bytes())?;
            self.put(&bytes)?;
        }
        self.put(&[samples.len() as u8])?;
        for sample in samples {
            self.put(&[sources.binary_search(&&sample.source).unwrap() as u8])?;
            self.floats(&sample.point)?;
            self.put(&sample.source_sequence.to_le_bytes())?;
            self.put(&sample.source_timestamp_ns.to_le_bytes())?;
            self.put(&sample.point_index.to_le_bytes())?;
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) fn encode(block: &Block, records: &BTreeMap<u16, GroundRecord>) -> Result<Vec<u8>, String> {
    let mut out = Output(encode_block(block)?);
    let start = out.0.len();
    out.put(&(records.len() as u32).to_le_bytes())?;
    for (&key, record) in records {
        out.put(&key.to_le_bytes())?;
        match record {

            GroundRecord::HeightfieldV1(field) => {
                out.put(&[1])?;
                for coord in field.anchor { out.put(&coord.to_le_bytes())?; }
                out.floats(&[field.height_m])?;
                out.floats(&field.occupied_bounds)?;
                out.put(&[field.members.len() as u8])?;
                for (member, bins) in &field.members {
                    out.put(&member.to_le_bytes())?;
                    out.floats(bins)?;
                }
                out.samples(&field.samples)?;
            }
            GroundRecord::ClassifiedEmptyColumn {} => out.put(&[2])?,
        }
    }
    let size = (out.0.len() - start) as u32;
    out.0[..8].copy_from_slice(b"NUWAXBL5");
    out.0[12..16].copy_from_slice(&size.to_le_bytes());
    Ok(out.0)
}

struct Input<'a>(&'a [u8]);
impl<'a> Input<'a> {
    fn take(&mut self, count: usize) -> Result<&'a [u8], String> {
        let bytes = self.0.get(..count).ok_or("truncated BL5 ground record")?;
        self.0 = &self.0[count..];
        Ok(bytes)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], String> {
        Ok(self.take(N)?.try_into().unwrap())
    }
    fn byte(&mut self) -> Result<u8, String> { Ok(self.array::<1>()?[0]) }
    fn count(&mut self, maximum: usize) -> Result<usize, String> {
        let value = usize::from(self.byte()?);
        if value > maximum { return Err("BL5 ground collection exceeds budget".into()); }
        Ok(value)
    }
    fn floats<const N: usize>(&mut self) -> Result<[f64; N], String> {
        let mut values = [0.; N];
        for value in &mut values { *value = f64::from_le_bytes(self.array()?); }
        Ok(values)
    }
    fn samples(&mut self) -> Result<Vec<LayerSample>, String> {
        let count = self.count(MAX_LAYER_SAMPLES)?;
        let mut sources: Vec<Arc<LayerSource>> = Vec::new();
        for _ in 0..count {
            let size = u32::from_le_bytes(self.array()?) as usize;
            let source: Arc<LayerSource> = serde_json::from_slice(self.take(size)?).map_err(|e| e.to_string())?;
            source.validate()?;
            if sources.last().is_some_and(|old| old >= &source) {
                return Err("unsorted or duplicate BL5 sources".into());
            }
            sources.push(source);
        }
        let count = self.count(MAX_LAYER_SAMPLES)?;
        let mut samples = Vec::new();
        let mut used = vec![false; sources.len()];
        for _ in 0..count {
            let index = usize::from(self.byte()?);
            let source = sources.get(index).ok_or("invalid BL5 sample source")?.clone();
            used[index] = true;
            samples.push(LayerSample {
                source,
                point: self.floats()?,
                source_sequence: u64::from_le_bytes(self.array()?),
                source_timestamp_ns: u64::from_le_bytes(self.array()?),
                point_index: u32::from_le_bytes(self.array()?),
            });
        }
        if used.contains(&false) { return Err("unused BL5 sample source".into()); }
        Ok(samples)
    }
}

pub(crate) fn decode(bytes: &[u8]) -> Result<GroundRecords, String> {
    let mut input = Input(bytes);
    let count = u32::from_le_bytes(input.array()?) as usize;
    if count == 0 || count > MAX_LAYERS_PER_BLOCK { return Err("invalid BL5 ground record count".into()); }
    let mut records = BTreeMap::new();
    for _ in 0..count {
        let key = u16::from_le_bytes(input.array()?);
        if records.last_key_value().is_some_and(|(&old, _)| old >= key) {
            return Err("invalid ground record anchors".into());
        }
        let record = match input.byte()? {

            1 => {
                let mut anchor = [0; 3];
                for coord in &mut anchor { *coord = i32::from_le_bytes(input.array()?); }
                let height_m = input.floats::<1>()?[0];
                let occupied_bounds = input.floats()?;
                let count = input.count(MAX_LAYER_MEMBERS)?;
                let mut members = Vec::new();
                for _ in 0..count { members.push((u16::from_le_bytes(input.array()?), input.floats()?)); }
                GroundRecord::HeightfieldV1(Arc::new(HeightfieldFragment {
                    anchor, height_m, occupied_bounds, members, samples: input.samples()?,
                }))
            }
            2 => GroundRecord::ClassifiedEmptyColumn {},
            _ => return Err("unknown BL5 ground record tag".into()),
        };
        records.insert(key, record);
    }
    if !input.0.is_empty() { return Err("trailing BL5 ground bytes".into()); }
    Ok(GroundRecords::Versioned(Arc::new(records)))
}
