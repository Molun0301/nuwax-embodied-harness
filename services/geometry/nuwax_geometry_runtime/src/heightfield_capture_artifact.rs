// Explicit, ignored artifact exporters for the four frozen C raw point sources.
// Only the existing admitted FineCloud producer constructs ground records.
use super::*;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{fs, path::PathBuf};

fn export(case: &str) {
    let input_file = std::env::var("NEH_CAPTURE_SIM_INPUTS").expect("fixed input manifest required");
    let inputs: Value = serde_json::from_slice(&fs::read(input_file).unwrap()).unwrap();
    let input = &inputs[case];
    let bytes = fs::read(input["cloud"].as_str().unwrap()).unwrap();
    assert_eq!(format!("{:x}", Sha256::digest(&bytes)), input["sha256"].as_str().unwrap());
    assert_eq!(bytes.len() % 16, 0);
    assert_eq!(bytes.len() / 16, input["points"].as_u64().unwrap() as usize);
    let points: Vec<_> = bytes.chunks_exact(16).map(|b| Point3::new(
        f64::from(f32::from_le_bytes(b[0..4].try_into().unwrap())),
        f64::from(f32::from_le_bytes(b[4..8].try_into().unwrap())),
        f64::from(f32::from_le_bytes(b[8..12].try_into().unwrap())),
    )).collect();
    let stamp = 1_000_000_000_u64;
    let indices = (0..points.len()).map(|i| u32::try_from(i).unwrap()).collect();
    // This is a fixed auxiliary test identity, not a claim about the original sensor.
    let source = AcceptedMappingCloud::auxiliary(RegisteredObservation::for_test_source(
        stamp, 1, LayerSource::Auxiliary {
            sensor_id: format!("capture-simulator-{case}"), clock_id: "fixture".into(),
            runtime_generation: 1, session_generation: 1,
        }, points, indices, Isometry3::identity()));
    let mut fine = FineCloud::fresh().unwrap();
    ingest(&mut fine, &source, 0);
    assert_occupancy(&mut fine, source.map_points().iter().copied());
    let ground_indices = fine.latest_ground_indices.to_vec();
    assert!(ground_indices.windows(2).all(|v| v[0] < v[1]));
    assert!(ground_indices.iter().all(|&i| (i as usize) < source.map_points().len()));
    let columns = records(&mut fine);
    let mut sample_count = 0;
    let oracle: Vec<_> = columns.iter().map(|(xy, column)| {
        assert!(column.legacy.is_empty(), "fresh capture must not fabricate legacy planes");
        let fields: Vec<_> = column.heightfields.iter().map(|surface| {
            for sample in &surface.samples {
                sample_count += 1;
                assert_eq!(sample.source_timestamp_ns, stamp);
                assert_eq!(sample.source_sequence, 1);
                assert_eq!(sample.source.as_ref(), source.source().layer_source().as_ref());
                assert_eq!(sample.point, <[f64; 3]>::from(source.map_points()[sample.point_index as usize].coords));
            }
            json!({"height": surface.column.height(), "bounds": surface.column.occupied_bounds(),
                "members": surface.column.members().map(|(z,_)| z).collect::<Vec<_>>()})
        }).collect();
        json!({"xy": xy, "classified_empty": column.classified_empty, "heightfields": fields})
    }).collect();
    let occupancy: Vec<_> = super::super::owner_test_observations::snapshot(&mut fine).0
        .into_iter().map(|(key, value)| json!({"key": key, "value": format!("{value:?}")})).collect();
    let sealed = fine.seal(Domain {session: format!("capture-simulator-{case}"), generation: 1,
        frame_generation: 1, producer_generation: 1}, 1, stamp).unwrap();
    let publication = message(&sealed);
    schema::validate_navigation_map_3d(&publication).unwrap();
    let wire = schema::encode_navigation_map_3d(&publication).unwrap();
    assert_eq!(schema::decode_navigation_map_3d(&wire).unwrap(), publication);
    let out = PathBuf::from(std::env::var("NEH_CAPTURE_SIM_OUTPUT").expect("artifact destination required")).join(case);
    assert!(!out.exists(), "do not overwrite an earlier artifact");
    fs::create_dir_all(&out).unwrap();
    let destination = Repository::open(out.join("objects")).unwrap();
    let mut object_bytes = 0;
    for reference in &sealed.manifest().blocks {
        let bytes = sealed.repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
        object_bytes += bytes.len();
        destination.put_object(reference.object, &bytes).unwrap();
    }
    let transfer = publication.fine.as_ref().unwrap();
    fs::write(out.join("root.json"), &transfer.root).unwrap();
    fs::write(out.join("publication.lcm"), wire).unwrap();
    let report = json!({"format": "neh-capture-simulator-v1", "case": case, "input": input,
        "source_kind": "fixed_auxiliary_test_identity", "stamp": stamp, "sequence": 1,
        "input_points": source.map_points().len(), "ground_indices": ground_indices,
        "diagnostic_samples": sample_count, "columns": oracle, "occupancy": occupancy,
        "root_version": sealed.manifest().version, "blocks": sealed.manifest().blocks.len(),
        "object_bytes": object_bytes, "inline_objects": transfer.included.len(),
        "inline_bytes": transfer.included.values().map(Vec::len).sum::<usize>(),
        "query_columns": input["query_columns"]});
    fs::write(out.join("source.json"), serde_json::to_vec_pretty(&report).unwrap()).unwrap();
    eprintln!("CAPTURE_SIM_ARTIFACT case={case} input={} ground={} columns={} samples={} blocks={} bytes={} path={}",
        source.map_points().len(), report["ground_indices"].as_array().unwrap().len(), columns.len(),
        sample_count, sealed.manifest().blocks.len(), object_bytes, out.display());
}

#[test] #[ignore = "requires frozen external C input artifacts and a fresh output directory"]
fn capture_sim_flat_noise_artifact() { export("flat_noise"); }
#[test] #[ignore = "requires frozen external C input artifacts and a fresh output directory"]
fn capture_sim_missing_patch_artifact() { export("missing_patch"); }
#[test] #[ignore = "requires frozen external C input artifacts and a fresh output directory"]
fn capture_sim_gentle_slope_artifact() { export("gentle_slope"); }
#[test] #[ignore = "requires frozen external C input artifacts and a fresh output directory"]
fn capture_sim_bump_10_artifact() { export("bump_10"); }
