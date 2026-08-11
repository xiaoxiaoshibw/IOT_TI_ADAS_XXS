# ADAS fixed-scenario schema v1

This document freezes the JSON contract shared by the CARLA and SIL scenario
loaders.  Runtime support is introduced in later phases; the v1 files and the
offline validator are the authority for scenario data now.

## Files and versioning

- `scenarios/catalog.json` lists every selectable scenario and points to one
  scenario file below `scenarios/`.
- Every file has integer `schema_version: 1`.  Readers must reject unknown
  versions instead of guessing.
- `id` is a stable, repository-wide identifier.  Existing IDs are retained.
- Paths in the catalog are repository-relative and may not escape the
  `scenarios/` directory.

Catalog entries contain `id`, `display_name`, `description`, `file`,
`default_town`, `supported_backends`, `default_seed`,
`expected_actor_count`, `acceptance_profile`, and `operator_tunable`.
`operator_tunable` is empty in Phase 0, so operators may select a scenario but
may not edit its values.

## Scenario fields and units

All numbers use SI units and simulation time:

- `duration_s`: seconds; zero means no time limit.
- `ego.spawn_index`: CARLA spawn-point index retained for legacy scenarios.
- `actors[].initial_station_m`: signed longitudinal distance from the ego
  spawn reference, in metres; positive is forward.
- `actors[].initial_lateral_m`: metres left of the reference lane centre;
  negative is right.
- `actors[].initial_speed_mps`: metres per second.
- `actors[].accel_limit_mps2`: positive acceleration-magnitude limit.
- `actors[].speed_profile`: `[time_s, target_speed_mps]` pairs in strictly
  increasing time order.  The first entry must be at `0.0` and must agree with
  `initial_speed_mps`.
- `hard_brake_window_s`: optional `[start_s, end_s]`, end-exclusive at runtime.
- Pedestrian `trigger_ego_gap_m` and `crossing_speed_mps` are metres and metres
  per second respectively.

Actor `classification` is one of `vehicle`, `pedestrian`, `cyclist`, or
`unknown`.  `lane` is semantic: `ego`, `left`, `right`, or `crossing`.
`legacy_role` is present only on the actor adapting an old `lead` or
`pedestrian` field and is used to prove backward equivalence.

## Determinism and lifecycle

`seed` is an unsigned integer.  `seed: 0` means zero perturbation: no random
blueprint choice, spawn jitter, timing jitter, or traffic-manager traffic.
Non-zero seeds may only select variants explicitly declared by a future schema
revision or by catalog fields marked `operator_tunable`.

Actor IDs are positive integers, unique within a scenario, stable from spawn
through disappearance, and never reused during one run.  Publishers emit
objects in ascending actor-ID order.  A failed spawn or count mismatch is a
startup failure; partial scenarios must not run.

Initial actor reference points must be separated by at least 2 m in the
station/lateral plane.  Backend loaders remain responsible for checking actual
bounding-box overlap before starting.

## Backend mapping

`supported_backends` may contain only `carla` and `sil`, and must match the
catalog entry.

- CARLA maps `town` to the CARLA map (v1 uses `Town04`), `lane` to a waypoint
  relative to the ego waypoint, station to distance along that waypoint chain,
  and lateral offset to the waypoint right vector.  The existing bridge main
  thread remains the sole `carla.Client` and `world.tick()` owner.
- SIL maps `lane` to the corresponding deterministic track lane and station to
  arc length on its piecewise-circular track.  CARLA lane IDs and SIL track
  lane IDs are deliberately not assumed to be physically identical.

Both backends implement the same ID, classification, speed-profile, spawn, and
disappearance semantics.  Backend geometry may differ without changing those
semantics.

## Compatibility and current limits

The nine IDs in the existing Python `SCENARIOS` dictionary (eight finite demo
scenarios plus `free`) remain valid.  `tools/validate_scenario.py` reconstructs
their old dictionaries from v1 files and compares every field.

Schema v1 supports fixed-lane actors only.  Actor lane changes and runtime
scenario hot reload are unsupported.  Ego planning may choose a lane, but
scripted actors may not teleport or change lanes.  There are no new ROS
messages or topic/QoS changes in this contract.

The GUI's current `ADAS_GUI_MODE=sil` owns `run_sil_fallback.sh`; it does not
own or represent the local-three-machine orchestrator.  A later GUI phase must
introduce an explicit backend and a single process owner rather than infer the
backend from a scenario name.
