use super::*;
use nuwax_config::sections::{CacheBudget,Geometry};
#[test]
fn fine_cloud_cache_config_parses_bounds_defaults_and_rejects_others() {
    let g: Geometry = nuwax_config::test_support::section("geometry");
    assert_eq!(
        parse_fine_cloud_cache(&g.fine_cloud_cache_mib, g.fine_cloud_cache_auto_max_mib),
        Ok(FineCloudCache::Auto { max_mib: 256 })
    );
    for mib in [16, 64, 4096] {
        assert_eq!(
            parse_fine_cloud_cache(&CacheBudget::Mib(mib), 0),
            Ok(FineCloudCache::Fixed { mib: mib as usize })
        );
    }
    for max in [32, 4096] {
        assert_eq!(
            parse_fine_cloud_cache(&CacheBudget::Mode("auto".into()), max),
            Ok(FineCloudCache::Auto {
                max_mib: max as usize
            })
        );
    }
    for mib in [15, 4097, 0, -32] {
        assert!(parse_fine_cloud_cache(&CacheBudget::Mib(mib), 256).is_err());
    }
    for mode in ["", "abc", "AUTO", "1.5", "64"] {
        assert!(parse_fine_cloud_cache(&CacheBudget::Mode(mode.into()), 256).is_err());
    }
    for max in [31, 4097] {
        assert!(parse_fine_cloud_cache(&CacheBudget::Mode("auto".into()), max).is_err());
    }
}
