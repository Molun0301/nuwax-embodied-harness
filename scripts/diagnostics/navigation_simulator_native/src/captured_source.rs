//! Diagnostic artifact admission. Ground is read only from the actual publication.
use super::*;
use nuwax_codec::lcm::schema;
use nuwax_map_chunks::{Receiver, Repository, MAX_BLOCK_BYTES};
use nuwax_nav_stack::surface_map::SurfaceMapCache;
use std::collections::BTreeSet;
use std::path::Path;

pub struct CapturedSource {
    pub map: Arc<SurfaceSnapshot>,
    pub oracle: Value,
    pub admission: Value,
}
fn column(map: &SurfaceSnapshot, xy: [i32; 2]) -> Result<Value, String> {
    let source = map
        .observations()
        .ok_or("captured source has no observations")?;
    let q = f64::from(source.voxel_size_m());
    let fields: Vec<_> = source
        .ground_records(xy)
        .map(|record| {
            json!({"height": record.height_at(xy.map(|v| (f64::from(v)+0.5)*q)),
            "bounds": record.occupied_bounds(q), "members": record.members().collect::<Vec<_>>()})
        })
        .collect();
    Ok(
        json!({"xy": xy, "classified_empty": source.classified_empty_ground(xy), "heightfields": fields}),
    )
}
fn xy(value: &Value) -> Result<[i32; 2], String> {
    serde_json::from_value(value.clone()).map_err(|e| e.to_string())
}
impl CapturedSource {
    pub fn load(path: &Path, config: &SurfaceNavigationConfig) -> Result<Self, String> {
        let oracle: Value =
            serde_json::from_slice(&fs::read(path.join("source.json")).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        if oracle["format"] != "neh-capture-simulator-v1" {
            return Err("unsupported capture artifact".into());
        }
        // Wire encoding includes the original codec fingerprint. No format guessing.
        let message = schema::decode_navigation_map_3d(
            &fs::read(path.join("publication.lcm")).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        schema::validate_navigation_map_3d(&message).map_err(|e| e.to_string())?;
        if message.cleared || message.source_voxel_size.to_bits() != 0.05_f32.to_bits() {
            return Err("capture simulator requires a non-cleared original 0.05m source".into());
        }
        let transfer = message
            .fine
            .as_ref()
            .ok_or("capture publication has no fine source")?;
        if fs::read(path.join("root.json")).map_err(|e| e.to_string())? != transfer.root {
            return Err("artifact root differs from publication".into());
        }
        let objects = path.join("objects");
        if !objects.is_dir() {
            return Err("captured object repository is missing".into());
        }
        let source = Repository::open(objects).map_err(|e| e.to_string())?;
        // Receiver may collect its objects. Never give it the preserved source store.
        let target = Repository::temporary().map_err(|e| e.to_string())?;
        let target_path = target.path().to_path_buf();
        let mut receiver = Receiver::with_cache_budget(target, 32 * 1024 * 1024);
        let mut fetched = BTreeSet::new();
        let mut fetch_calls = 0;
        let root = receiver.receive(
            transfer,
            &schema::navigation_map_3d_domain(&message),
            |_, reference| {
                fetch_calls += 1;
                fetched.insert(reference.object);
                source.read_object(reference.object, MAX_BLOCK_BYTES)
            },
        )?;
        let expected: BTreeSet<_> = root
            .blocks
            .iter()
            .filter(|r| !transfer.included.contains_key(&r.object))
            .map(|r| r.object)
            .collect();
        if fetched != expected || fetch_calls != expected.len() {
            return Err("missing-object fetch mismatch".into());
        }
        let mut cache = SurfaceMapCache::configured_with_admitted_fine_source(config, receiver);
        let map = cache.build(&message).map_err(|e| e.to_string())?;
        let again = cache.build(&message).map_err(|e| e.to_string())?;
        if !Arc::ptr_eq(&map, &again) {
            return Err("same publication lost the original Reader Arc".into());
        }
        let observations = map
            .observations()
            .ok_or("Reader did not install observations")?;
        if !observations.is_complete_geometry_source() {
            return Err("Reader geometry is incomplete".into());
        }
        let mut expected_columns = BTreeMap::<[i32; 2], Vec<i32>>::new();
        for item in oracle["occupancy"]
            .as_array()
            .ok_or("occupancy oracle missing")?
        {
            let key: [i32; 3] =
                serde_json::from_value(item["key"].clone()).map_err(|e| e.to_string())?;
            if observations.get(key).is_none() {
                return Err(format!("Reader lost occupancy {key:?}"));
            }
            expected_columns
                .entry([key[0], key[1]])
                .or_default()
                .push(key[2]);
        }
        for (key, mut levels) in expected_columns {
            levels.sort_unstable();
            if observations.column_levels(key[0], key[1]) != Some(levels.as_slice()) {
                return Err(format!("Reader occupancy column mismatch {key:?}"));
            }
        }
        let admission = json!({"blocks": root.blocks.len(), "inline_objects":transfer.included.len(),
            "inline_bytes":transfer.included.values().map(Vec::len).sum::<usize>(),
            "fetch_calls":fetch_calls,"fetched_objects":fetched.len(),"root_version":root.version,
            "same_publication_arc":true,"receiver_store":target_path});
        drop(cache);
        let mut result = Self {
            map,
            oracle,
            admission,
        };
        result.admission["receiver_store_removed"] = json!(!target_path.exists());
        result.verify(&result.map)?;
        Ok(result)
    }
    /// The oracle is compared after real admission; it never creates map geometry.
    pub fn verify(&self, map: &SurfaceSnapshot) -> Result<(), String> {
        for expected in self.oracle["columns"]
            .as_array()
            .ok_or("column oracle missing")?
        {
            let actual = column(map, xy(&expected["xy"])?)?;
            if actual != *expected {
                return Err(format!(
                    "Writer/Reader column mismatch: {actual} != {expected}"
                ));
            }
        }
        Ok(())
    }
    pub fn inspect(&self, map: &Arc<SurfaceSnapshot>, body: Option<Body>) -> Result<Value, String> {
        self.verify(map)?;
        let observations = map.observations().ok_or("Reader observations missing")?;
        let original = self.map.observations().unwrap();
        let mut seams = Vec::new();
        for key in self.oracle["query_columns"]
            .as_array()
            .ok_or("query columns missing")?
        {
            let key = xy(key)?;
            let mut value = column(map, key)?;
            value["occupied_levels"] = json!(observations.column_levels(key[0], key[1]));
            seams.push(value);
        }
        let pose = body.map(|pose| {
            let query = map.body_query();
            match query.start_ground_state(pose) {
                Ok(ground) => {
                    let mut witness = None;
                    let result = query.validate_motion_from_ground(pose,pose,&ground,&[],&mut witness).map(|_|());
                    json!({"pose":pose,"ground_height":ground.height(),"state":format!("{:?}",ground.query_state()),
                        "ground_key":format!("{:?}",ground.ground_key()),"result":format!("{result:?}"),"witness":format!("{witness:?}")})
                }
                Err(error) => json!({"pose":pose,"start_error":format!("{error:?}")}),
            }
        });
        Ok(
            json!({"case":self.oracle["case"],"input":self.oracle["input"],"admission":self.admission,
            "writer_reader_columns_verified":self.oracle["columns"].as_array().unwrap().len(),
            "ground_indices":self.oracle["ground_indices"].as_array().unwrap().len(),
            "queries":seams,"pose":pose,"same_static_geometry":self.map.same_geometry(map),
            "same_observations_arc":Arc::ptr_eq(original,observations),
            "same_ground_evidence":self.map.ground_evidence()==map.ground_evidence(),
            "same_initial_snapshot_arc":Arc::ptr_eq(&self.map,map)}),
        )
    }
}
