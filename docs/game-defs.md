# Engine and transmission defs (SCS modding wiki)

Sources: https://modding.scssoft.com/wiki/Documentation/Engine/Units/accessory_engine_data ,
https://modding.scssoft.com/wiki/Documentation/Engine/Units/accessory_transmission_data

## accessory_engine_data (what the automatic uses)

- `torque` (N·m, max) + `torque_curve` array of (rpm, 0..1 factor), linear, clamped. Default curve:
  (300,0) (440,0.5) (1000,1) (1300,1) (1900,0.77) (2400,0.4) (2600,0).
- `secondary_torque` / `secondary_torque_gear_ratio` / `secondary_torque_curve` (multi-torque, below a ratio).
- `rpm_idle` 550, `rpm_limit` 2500, `rpm_limit_neutral` 2500, hysteresis fields.
- Automatic shift points, all float2 (down, up):
  - `rpm_range_low_gear` default (700, 1500)
  - `rpm_range_high_gear` default (1000, 1350), "around peak torque"
  - `rpm_range_power` default (1400, 1900): more power requested (hills, adaptive, power mode). Since 1.37.
  - `rpm_range_engine_brake` default (1500, 2500); `engine_brake_downshift` (bool) forces a downshift into it.
  - `rpm_range_power_boost` obsolete since 1.37.
- `engine_brake` strength, `engine_brake_positions` 3, `auxiliary_brakes` (engine brake, forced downshift,
  retarder) per position.
- `resistance_torque` (N·m at 2000 rpm, default volume×20), `flywheel_inertia` (1.61), `volume` (l).
- `info[]` = UI text only (hp, torque, rpm), not simulated. Power is not stored for diesels: compute from curve.

## accessory_transmission_data

- `ratios_forward[]`, `ratios_reverse[]`, `differential_ratio`, `shift_time` (s, automatic, default 1.0),
  `transmission_class` heavy|standard (affects shifting), `stall_torque_ratio` + `torque_converter_lock_gear`
  (TC boxes), `retarder` levels + `auxiliary_brakes`, `crawls`.

## Link to the hooked code (hypothesis, verify with the probe)

The window function (1.60) reads float2s from the engine object at `0x23c` (returned as is when the inner flag
`0x1440` is set: likely engine-brake mode → `rpm_range_engine_brake`) and `0x244`, and floats at `0x24c`/`0x250`.
These look like the `rpm_range_*` pairs. 1.61 has them 8 bytes later.

## Truck used during the first probe run

Volvo FH6 Aero Sleeper 4x2, 420 hp (309 kW), 2400 N·m at 900-1230 rpm, I-Shift SPO2812 Dual Clutch
(a DCT mod), cargo 17 t (used car batteries, Parma → Monaco).
