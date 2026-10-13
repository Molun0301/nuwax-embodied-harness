use super::*;

struct NoHash(bool);
impl NoHash {
    fn enter() -> Self {
        Self(crate::map_checksum::FORBID_HASH.with(|flag| flag.replace(true)))
    }
}
impl Drop for NoHash {
    fn drop(&mut self) {
        crate::map_checksum::FORBID_HASH.with(|flag| flag.set(self.0));
    }
}

struct Fixture(PathBuf);
impl Fixture {
    fn new(bytes: &[u8]) -> Self {
        let result = Self(
            std::env::temp_dir().join(format!("nuwax-zone-observation-{}", uuid::Uuid::new_v4())),
        );
        std::fs::write(&result.0, bytes).unwrap();
        result
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

fn document(signature: &str, generation: u64) -> Vec<u8> {
    format!(r#"{{"schema_version":1,"maps":{{"map-a":{{"map_name":"map-a","generation":{generation},"signature":"{signature}","zones":[]}}}}}}"#).into_bytes()
}

#[test]
fn explicit_alias_and_legacy_observations_never_hash() {
    let _no_hash = NoHash::enter();
    let expected = "ABCDEF0123456789".repeat(4);
    let bytes = document(&format!("  {expected}  "), 7);
    assert_eq!(
        parse_for_map(&bytes, "map-a").signature,
        expected.to_ascii_lowercase()
    );
    assert_eq!(parse_for_map(&bytes, "map-a").generation, 7);
    for signature in ["", "   "] {
        let bytes = document(signature, 7);
        let first = parse_for_map(&bytes, "map-a");
        let second = parse_for_map(&bytes, "map-a");
        assert_eq!(first.status, ZoneLoadStatus::Empty);
        assert!(first.signature.starts_with("observed:map-a:"));
        assert_ne!(
            first.signature, second.signature,
            "uncached reads are distinct observations"
        );
    }
}

#[test]
fn error_observations_are_cached_fail_closed_and_recover_without_hashing() {
    let _no_hash = NoHash::enter();
    let wrong_map =
        br#"{"schema_version":1,"maps":{"map-a":{"map_name":"map-b","generation":7,"zones":[]}}}"#
            .to_vec();
    for bytes in [
        b"{broken".to_vec(),
        br#"{"schema_version":9}"#.to_vec(),
        wrong_map,
        document("bad-alias", 7),
        document("", 0),
    ] {
        let fixture = Fixture::new(&bytes);
        let mut cache = ForbiddenZoneCache::default();
        let first = cache.load(&fixture.0, "map-a");
        assert!(matches!(first.status, ZoneLoadStatus::FailClosed(_)));
        assert_eq!(cache.load(&fixture.0, "map-a").signature, first.signature);
        assert_ne!(load_for_map(&fixture.0, "map-a").signature, first.signature);
        std::fs::write(&fixture.0, document("", 7)).unwrap();
        let recovered = cache.load(&fixture.0, "map-a");
        assert_eq!(recovered.status, ZoneLoadStatus::Empty);
        assert_ne!(recovered.signature, first.signature);
        assert_eq!(
            cache.load(&fixture.0, "map-a").signature,
            recovered.signature
        );
    }
}

#[test]
fn legacy_cache_hits_clones_and_source_changes_use_observation_identity() {
    let _no_hash = NoHash::enter();
    let bytes = document("", 7);
    let fixture = Fixture::new(&bytes);
    let mut cache = ForbiddenZoneCache::default();
    let first = cache.load(&fixture.0, " map-a ");
    let mut cloned = cache.clone();
    assert_eq!(cloned.load(&fixture.0, "map-a").signature, first.signature);
    assert!(cache.same_source_version(&cloned));
    let modified = std::fs::metadata(&fixture.0).unwrap().modified().unwrap();
    let replacement = Fixture::new(&bytes);
    std::fs::File::open(&replacement.0)
        .unwrap()
        .set_times(std::fs::FileTimes::new().set_modified(modified))
        .unwrap();
    std::fs::rename(&replacement.0, &fixture.0).unwrap();
    let replaced = cache.load(&fixture.0, "map-a");
    assert_eq!(replaced.generation, first.generation);
    assert_ne!(
        replaced.signature, first.signature,
        "even identical replacement bytes are a new observation"
    );
    assert!(!cache.same_source_version(&cloned));
    assert_eq!(
        cache.load(&fixture.0, "map-a").signature,
        replaced.signature
    );
    // Restore mtime after an in-place edit: ctime must still invalidate the cache.
    let old_stamp = file_stamp(&std::fs::metadata(&fixture.0).unwrap());
    for _ in 0..100 {
        std::fs::write(&fixture.0, &bytes).unwrap();
        std::fs::File::open(&fixture.0)
            .unwrap()
            .set_times(std::fs::FileTimes::new().set_modified(modified))
            .unwrap();
        if file_stamp(&std::fs::metadata(&fixture.0).unwrap()) != old_stamp {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    assert_ne!(
        file_stamp(&std::fs::metadata(&fixture.0).unwrap()),
        old_stamp,
        "fixture requires an observable in-place edit"
    );
    let edited = cache.load(&fixture.0, "map-a");
    assert_eq!(edited.generation, 7);
    assert_ne!(edited.signature, replaced.signature);
    assert_eq!(cache.load(&fixture.0, "map-a").signature, edited.signature);
    assert_eq!(cache.load(&fixture.0, "map-b").signature, "absent:map-b");
    assert_ne!(
        cache.load(&fixture.0, "map-a").signature,
        edited.signature,
        "map binding cache miss is a new observation"
    );
    std::fs::remove_file(&fixture.0).unwrap();
    assert_eq!(cache.load(&fixture.0, "map-a").signature, "missing:map-a");
    assert!(cache.entry.is_none());
}

#[test]
fn torn_read_observation_is_not_cached() {
    let _no_hash = NoHash::enter();
    let fixture = Fixture::new(&document("", 7));
    let mut cache = ForbiddenZoneCache::default();
    let torn = cache.load_inner_with_reader(&fixture.0, "map-a", |file, bytes| {
        let count = file.read_to_end(bytes)?;
        // A deterministic concurrent write between the before/after metadata reads.
        std::fs::write(&fixture.0, document(" ", 7))?;
        Ok(count)
    });
    assert!(cache.entry.is_none());
    let stable = cache.load(&fixture.0, "map-a");
    assert_ne!(stable.signature, torn.signature);
    assert_eq!(cache.load(&fixture.0, "map-a").signature, stable.signature);
}

#[test]
fn missing_absent_empty_binding_and_io_sentinels_are_unchanged() {
    let _no_hash = NoHash::enter();
    let fixture = Fixture::new(br#"{"schema_version":1,"maps":{}}"#);
    let mut cache = ForbiddenZoneCache::default();
    assert_eq!(cache.load(&fixture.0, "map-a").signature, "absent:map-a");
    assert_eq!(
        cache.load(&fixture.0, "").signature,
        "invalid:empty-map-binding"
    );
    std::fs::remove_file(&fixture.0).unwrap();
    assert_eq!(cache.load(&fixture.0, "map-a").signature, "missing:map-a");
    let io = cache.load(&std::env::temp_dir(), "map-a");
    assert!(matches!(io.status, ZoneLoadStatus::FailClosed(_)));
    assert!(io.signature.starts_with("io-error:map-a:"));
    assert_eq!(
        cache.load(&std::env::temp_dir(), "map-a").signature,
        io.signature
    );
    assert!(cache.entry.is_none());
}
