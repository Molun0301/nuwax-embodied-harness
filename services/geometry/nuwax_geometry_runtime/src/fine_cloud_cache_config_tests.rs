use super::*;

#[test]
fn fine_cloud_cache_config_parses_bounds_defaults_and_rejects_others() {
    let parse = |mode: Option<&str>, max: Option<&str>| parse_fine_cloud_cache(mode, max);
    // 未设置与空值按缺省：自动模式、上限 256 MiB。
    for (mode, max) in [
        (None, None),
        (Some(""), None),
        (None, Some("")),
        (Some(" "), Some("\t")),
        (Some("auto"), None),
    ] {
        assert_eq!(parse(mode, max), Ok(FineCloudCache::Auto { max_mib: 256 }));
    }
    assert_eq!(
        parse(Some("16"), None),
        Ok(FineCloudCache::Fixed { mib: 16 })
    );
    assert_eq!(
        parse(Some("4096"), None),
        Ok(FineCloudCache::Fixed { mib: 4096 })
    );
    assert_eq!(
        parse(Some(" 064 "), None),
        Ok(FineCloudCache::Fixed { mib: 64 })
    );
    assert_eq!(
        parse(Some("auto"), Some("32")),
        Ok(FineCloudCache::Auto { max_mib: 32 })
    );
    assert_eq!(
        parse(None, Some("4096")),
        Ok(FineCloudCache::Auto { max_mib: 4096 })
    );
    // 固定模式忽略自动上限。
    assert_eq!(
        parse(Some("64"), Some("not-a-number")),
        Ok(FineCloudCache::Fixed { mib: 64 })
    );
    for mode in [
        "15",
        "4097",
        "0",
        "-32",
        "+64",
        "abc",
        "AUTO",
        "1.5",
        "99999999999999999999999",
    ] {
        let error = parse(Some(mode), None).unwrap_err();
        assert!(
            error.contains(FINE_CLOUD_CACHE_MIB_VAR) && error.contains("16～4096"),
            "{mode}: {error}"
        );
    }
    for max in ["31", "4097", "auto", "x"] {
        let error = parse(Some("auto"), Some(max)).unwrap_err();
        assert!(
            error.contains(FINE_CLOUD_CACHE_AUTO_MAX_VAR) && error.contains("32～4096"),
            "{max}: {error}"
        );
    }
}
