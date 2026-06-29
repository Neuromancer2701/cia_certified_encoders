# CiA 406 encoder templates for ros2_canopen + ros2_control

Ready-to-adapt configuration bundles for bringing **read-only CANopen absolute
encoders** (CiA 406 device profile) into the `ros2_control` ecosystem via
`ros2_canopen`, with a small chainable controller that converts raw device
counts into SI units and **re-exports them as a state interface** so downstream
controllers can consume engineering units directly.

> Target distro: ROS 2 **Lyrical Luth** (rolling-derived). Some
> `ChainableControllerInterface` export signatures have churned across distros —
> see the comment block in `cia406_position_controller.cpp` and check the header
> for your distro before building.

---

## Why CiA 406 is a good subgroup to template

Every CiA 406 encoder shares the same object dictionary, so one template covers
all of them with per-device differences reduced to the EDS, the PDO layout, and
a resolution constant:

| Object   | Meaning                                   | Used for |
|----------|-------------------------------------------|----------|
| `0x6000` | Operating parameters (code sequence, etc.)| SDO startup config |
| `0x6001` | Measuring units per revolution (resolution)| counts/unit scaling |
| `0x6002` | Total measuring range in measuring units  | range / wrap handling |
| `0x6003` | Preset value                              | optional homing/offset |
| `0x6004` | Position value (32-bit)                   | **the feedback you map to a TPDO** |
| `0x6008` | High-resolution position value            | high-res alternative to 0x6004 |

## The key constraint: the proxy interface is a generic mailbox

`ros2_canopen` ships **no** CiA 406 driver. Read-only encoders therefore use the
generic `ProxyDriver` + `canopen_ros2_control/CanopenSystem`, which exposes
**generic, multiplexed** interfaces per joint — not a named `position`:

- state:   `rpdo/index`, `rpdo/subindex`, `rpdo/type`, `rpdo/data`, `nmt/state`
- command: `tpdo/index`, `tpdo/subindex`, `tpdo/type`, `tpdo/data`, `tpdo/owns`, `nmt/*`

`rpdo/data` holds whatever object arrived most recently, identified by
`rpdo/index`. So the templates **map only 0x6004 to the encoder's TPDO**, keeping
`rpdo/data` unambiguous, and the conversion controller still demuxes on
`rpdo/index` defensively.

> Perspective flip: the encoder *transmits* its position, so it is configured
> under the node's `tpdo:` in `bus.yaml`, but it *arrives* on the master's
> `rpdo/data` **state** interface.

## Two paths

**Path 1 — config-only (this repo).** Proxy + single position TPDO +
`Cia406PositionController` (or a `chained_filter_controller` gain if you prefer
zero code). Works well for position feedback. If you also need speed (0x6030),
the single `rpdo/data` slot multiplexes and you must demux by index.

**Path 2 — the real open-source gap.** Write a proper `Cia406Driver` +
`Cia406System` mirroring the `Cia402Driver` pattern, exposing named
`position`/`velocity` interfaces with built-in scaling (the encoder analogue of
`scale_pos_from_dev`). This is the genuinely missing upstream piece and the
higher-value long-term contribution. The Path-1 controller here is a stopgap
that doubles as a reference for the scaling math.

## Files

```
config/bus.yaml                          # proxy bus config, single 0x6004 TPDO
config/ros2_controllers.yaml             # controller_manager wiring
description/encoder.ros2_control.xacro   # CanopenSystem + generic interfaces
cia406_position_controller/              # chainable counts->SI controller (skeleton)
```

## EDS licensing — do not blindly commit vendor EDS

Several vendors gate EDS downloads behind a click-through license (e.g.
Wachendorff requires agreeing to their GTC + software clause). That can preclude
redistributing the EDS in a public repo. Prefer **linking** to each vendor's
download page and/or fetching the EDS at build time, rather than committing the
blobs. Track per-device EDS source + license in `config/devices.md` (add your own).

## Confirmed device list (CiA 406, EDS available online)

Wachendorff WDGA · EPC (encoder.com) · Kübler Sendix 58X8 · POSITAL/FRABA IXARC ·
Sensata POSI+ (PHM5) · Lika · Hohner · TR-Electronic · Pepperl+Fuchs ENA58IL ·
Baumer MAGRES EAM580.  (SICK AFS/AFM60, Hengstler AC58 = easy substitutes.)
