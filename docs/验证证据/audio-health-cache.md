# Audio health snapshots

`GatewayRuntime.get_health_summary()` reads `AudioService.get_cached_volume_state()`.
It never refreshes volume from a robot. An explicit `get_volume` or `set_volume`
operation still calls the available device capability and publishes its returned
volume into the cache. With no device setter, an explicit software-volume change
is itself the completed result.

The existing `volume`, `backend` and device-result fields remain available. Cache
metadata distinguishes what is known:

- `volume: null`, `observed_at: null`, `freshness: unknown`: no valid result yet
- `freshness: unverified_since_observation`: a real completed result exists;
  `observed_at` records that result's UTC time, not a freshness guarantee
- `freshness: stale_after_error`: a later operation failed; the last confirmed
  volume and timestamp remain intact, with `last_error`/`robot_volume_error`
- `refresh_in_progress: true`: an explicit device operation is active; cached
  health reads still return immediately without waiting for that operation

A failed set or an invalid/missing device readback never records the requested
volume as a successful observation. Successful later reads clear old errors.
Explicit device operations serialize separately from the cache lock, preventing
an older concurrent read from overwriting a later set. Returned snapshots own
their nested device metadata.

This change does not add audio capture/playback or a background polling loop.
The existing volume-change event's requested-volume semantics are unchanged.

Offline checks:

```sh
PYTHONDONTWRITEBYTECODE=1 python scripts/check_audio_health_cache.py
```

The fixture gates fake device calls, invokes the actual health-summary method,
checks concurrent explicit operations and cache ownership, and calls the public
get/set-volume handlers. It does not contact a device or external service.
