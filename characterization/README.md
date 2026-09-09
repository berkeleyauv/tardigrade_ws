# Characterization records

Create one dated directory per campaign, for example
`2026-10-14_pool_coastdown/`, containing a Markdown report and a JSON manifest.
Raw rosbags belong in team object storage and are referenced by URI and SHA-256.

Required manifest fields:

```json
{
  "date": "YYYY-MM-DD",
  "vehicle_revision": "...",
  "configuration_before_sha256": "...",
  "source_bags": [{"uri": "...", "sha256": "..."}],
  "water": {"temperature_c": 0.0, "density_kg_m3": 1000.0},
  "battery_voltage_v": 0.0,
  "method": "coast-down / thrust stand / stationary Allan variance / ...",
  "results": [{"parameter": "...", "value": 0.0, "units": "...", "uncertainty": 0.0}],
  "configuration_after_sha256": "...",
  "reviewer": "..."
}
```

Never silently replace an estimated coefficient. Preserve its old value and
provenance in the report, update `tardigrade_description/config/vehicle.json`,
regenerate both consumer files, and attach fit/residual plots.
