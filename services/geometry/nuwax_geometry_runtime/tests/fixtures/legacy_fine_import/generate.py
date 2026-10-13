"""Generate native fixtures through the real, guarded legacy-to-assigned importer.

Run with project Python dependencies from any working directory. A fresh run
allocates new assigned IDs, so request bytes may change; archive bytes do not.
No legacy publisher or content-hash function is used.
"""
import sys
from pathlib import Path


def main():
    destination = Path(__file__).resolve().parent
    root = Path(__file__).resolve().parents[6]
    sys.path[:0] = [str(root), str(root / 'tests')]
    from test_legacy_cloud_import import LegacyCloudImportTests, fine_archive
    from services.geometry import geometry_runtime_provider as provider

    case = LegacyCloudImportTests()
    case.setUp()
    try:
        # Literal source for lower-level restore tests, not an assigned AR1 import.
        (destination / 'ar1_source.fine.archive').write_bytes(fine_archive(1))
    finally:
        case.doCleanups()

    for version, backend, dense in ((2, 'voxel', False), (3, 'raytracing', True)):
        case = LegacyCloudImportTests()
        case.setUp()  # Hash sentinels precede every fixture and import operation.
        try:
            case.prepare(backend, dense)
            case.add_fine_archive(fine_archive(version))
            result = case.import_current()
            request = provider._assigned_activation_request_proto(result.stored_checkpoint,
                asset_identity=result.bundle.asset_identity, session_id='mapload:fine', map_name='map',
                activation_request_id='activate:fine', expected_runtime_generation=17)
            prefix = 'ar%d_%s' % (version, backend)
            values = {'request.pb': request.SerializeToString(),
                      'fine.archive': bytes(result.stored_checkpoint.checkpoint.fine_archive)}
            if dense:
                values['dense.archive'] = bytes(result.stored_checkpoint.checkpoint.dense_archive)
            for suffix, payload in values.items():
                target = destination / (prefix + '.' + suffix)
                target.write_bytes(payload)
                print('%s: %d bytes' % (target.name, len(payload)))
        finally:
            case.doCleanups()


if __name__ == '__main__':
    main()
