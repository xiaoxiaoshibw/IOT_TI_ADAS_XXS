# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository overview

`bowen_ADAS` is a three-machine HIL (Hardware-in-the-Loop) ADAS stack with the following physical topology:

```
PC (CARLA sim + ROS2 Jazzy bridge + Qt6 GUI)
        ⇄ DDS / ROS2 ⇄  Jetson Orin Nano (SoC stack, ROS2 Humble)
                                  │  CAN (SocketCAN can1 @ 500 kbps, PEAK PCAN-USB)
                                  ▼
                 TI LAUNCHXL-F280025C MCU (1 kHz safety arbiter + actuator driver)
                                  │
                       servo / turn signal / buzzer

                ┌── adas_iot / MQTT ──┐
                │   ↓                  │
                │ Dashboard / WeChat / Webhook / Email
                └──────────────────────┘
```

`ROS_DOMAIN_ID=43` must match between PC and Orin. Jetson uses an external PEAK PCAN-USB on `can1` (on-board `mttcan` `can0` is not pinned out).

## Top-level layout

| Directory | Toolchain | Role |
|-----------|-----------|------|
| `adas_soc/` | colcon, **Jazzy** on PC, **Humble** on Jetson | C++17 ROS2 stack — perception → behavior → trajectory → tracker → command gate → AEB → redundancy → safety |
| `adas_mcu/` | CCS 21.0 + CGT 25.11.1 LTS + C2000Ware 26.01; host tests via GCC | TMS320F280025C bare-metal C firmware — CAN v3 protocol, safety state machine, 1 kHz actuator loop |
| `adas_bridge_pc/` | colcon + ROS2 Jazzy | CARLA↔SoC Python bridge (`adas_carla_bridge`) + Qt6 safety monitor GUI (`adas_gui`) |
| `adas_iot/` | Python | MQTT bridge, Flask/SocketIO dashboard, WeChat mini-program, webhook/email alerts |
| `adas_tools/` | Python / Shell / MATLAB | HIL preflight gate, hil_logger, MATLAB plotting |
| `scripts/` | Bash | Top-level helpers (notably `run_sil_fallback.sh`) |
| `adas_bridge_pc/carla_ros2_bridge/Orin同步/` | — | Mirror of source packages deployed to Orin's `~/adas/adas_soc_ws/` |

## Critical cross-codebase contracts

Changing one side without the other breaks the closed loop. Read these files before touching the topics they govern:

1. **`adas_msgs` is duplicated and must stay byte-identical**
   - `adas_soc/src/common/adas_msgs/` (Jetson/PC SIL)
   - `adas_bridge_pc/carla_ros2_bridge/ws/src/adas_msgs/` (PC bridge)
2. **CAN protocol v3 — single authoritative definition**
   - `adas_mcu/include/adas_can_protocol.h` (line format, scaling, CRC-8 polynomial 0x31)
   - Reference encoders: `adas_mcu/tools/orin_can_reference.py` / `.cpp`
   - Byte-level conformance test: `adas_mcu/tests/test_protocol_reference.py`
   - Sole owner on Jetson: `adas_soc/src/vehicle/adas_can_gateway`
3. **`/adas/*` topic contracts** — publishers' QoS and types live in `adas_bridge_pc`; the SoC stack consumes them.
4. **Tunable parameter single sources**
   - MCU: `adas_mcu/include/adas_config.h` only
   - SoC: `adas_soc/src/launch/adas_launch/config/*.yaml`
5. **`hil_preflight_before_mcu.sh` exit codes** (0=PASS, 1=ROS, 2=CAN if, 3=CAN frames missing, 4=CAN not cont, 5=CAN err growing, 6=timeout, 7=missing tool) are a hard contract for systemd/starters — do not change them.

## SoC stack (`adas_soc/`)

20 ROS2 nodes organized by role. Each package has the canonical `<pkg>_node.cpp` (Lifecycle node shell) + `*_core.{hpp,cpp}` (pure algorithm, no rclcpp) so algorithms are unit-testable offline.

Data flow (single stack):

```
vehicle_interface ←─ CAN (gateway) ─ external Orin CAN link
       │ publishes actuation (last-mile actuator command)
       ↓
command_gate ←─ trajectory_follower / aeb / mqtt_bridge (each may request control)
       ↓ gate output
       ↓
vehicle_interface (executes to CAN)
```

Perception → planning → control → vehicle (top-down calls, bottom-up data):

- **perception**: `adas_lidar_perception`, `adas_object_tracker`
- **planning**: `adas_global_planner` (route), `adas_behavior_planner`, `adas_trajectory_planner`
- **control**: `adas_trajectory_follower` (LQR/pure-pursuit lateral + PID longitudinal), `adas_aeb`, `adas_command_gate`, `adas_controller_base` (shared base class)
- **vehicle**: `adas_vehicle_interface` (the unique actuator owner, talks to `adas_can_gateway`), `adas_can_gateway` (sole CAN owner on Jetson)
- **system**: `adas_safety_monitor` (lifecycle + diagnostic aggregation), `adas_redundancy` (dual-stack arbiter), `adas_mqtt_bridge`, `adas_dtc_recorder`, `adas_resource_monitor`
- **simulation**: `adas_sim_vehicle` (kinematic world for SIL)
- **common**: `adas_common` (cross-cutting utilities), `adas_msgs` (interface definitions)

### Deterministic activation chain

Single stack: `vehicle_interface → command_gate → safety_monitor → aeb → trajectory_follower → trajectory_planner → behavior_planner → object_tracker` (see `adas_soc/src/launch/adas_launch/launch/sil_launch_common.py`). Each node is a `LifecycleNode`; the next node is only `configure`d after the current one reaches `active`. **Readiness is judged by lifecycle state, never by `ros2 node list`** (cross-distro unreliable).

### Redundant stacks (`sil_redundant.launch.py`)

Two isolated stacks at `/primary` and `/backup`, each with the same 7 nodes (gate/aeb/safety_monitor/follower/planner/behavior/tracker). Per-stack topics are remapped via `STACK_TOPICS` (`/diagnostics` **must** be isolated — the backup stack's safety monitor would otherwise see the primary's faults and trigger MRM oscillation). `adas_redundancy::redundancy_arbiter_node` subscribes to both gates' outputs at 100 Hz and publishes the global `/adas/control/gate/control_cmd` for `vehicle_interface`.

Activation order: `vehicle_interface → arbiter → primary[7] → backup[7]`.

## MCU firmware (`adas_mcu/`)

TMS320F280025C bare-metal C, C28x @ 100 MHz, 1 kHz control loop. ISR (`cpuTimer0ISR`) only ticks + sets a flag; the full pipeline runs in the main loop. Watchdog is fed at the end of `run_tick` — so a stalled control loop triggers reset (TWDT semantics).

Pipeline: `CAN RX (0x100–0x103 primary, 0x110–0x113 backup) → CRC + range → per-source link monitor → primary/backup arbitration → safety state machine → actuator mapping → CAN TX (0x201/0x202/0x203)`.

Module split (`adas_mcu/src/`):
- `can_comm.c` — CAN driver + protocol decode/encode, CRC-8 (poly 0x31)
- `safety.c` — safety state machine, freshness gating, FTTI budgets
- `control.c` — final control mapping (steer/brake/drive_dir)
- `actuators.c` — servo / turn signal / buzzer PWM drivers
- `hil_session.c` — v3 session protocol (ANNOUNCE → READY → ARMED → ACTIVE, self-driven)
- `asr_pro.c` — voice command parser
- `hmi.c` + `oled2.c` + `dgus_screen.c` — local UI
- `self_test.c` — power-on self-test

Header `adas_can_protocol.h` is the only line-format authority — all byte/scaling/endianness/CRC details live there. C28x `char` is 16-bit; CAN data is carried as `uint16_t[8]` with only the low 8 bits valid.

`adas_can_protocol_v3_generated.h` is the generated v3 frame definitions; do not hand-edit.

Build:
- **Windows (production image)**: `tools\build_ti.ps1 -Configuration all -ImageSet production` (CCS 21.0)
- **Linux (cl2000 + dslite.sh)**: `adas_mcu/build.sh`

## Bridge & GUI (`adas_bridge_pc/`)

PC-side ROS2 Jazzy workspace at `adas_bridge_pc/carla_ros2_bridge/ws/`. Packages:

- `adas_carla_bridge` — Python rclpy node (`bridge_node.py`) that drives CARLA and republishes its world as `/adas/*` topics; `can_protocol.py` is a second copy of the v3 line format (must match `adas_mcu/include/adas_can_protocol.h`)
- `adas_gui` — Qt6 safety monitor (process_manager, launch_panel, fault_inject_panel, safety_panel, map_view, log_drawer, realtime_plot, telemetry_freshness, secure_settings, ros_bridge)
- `adas_sil_gui` — SIL-mode GUI variant (data source = `adas_sim_vehicle`, no CARLA/Orin/MCU/CAN needed)
- `adas_map` — lane graph for navigation
- `adas_msgs` — duplicate of the SoC-side package (must stay in sync)

Top-level scripts in `adas_bridge_pc/`:
- `build.sh` — colcon build the workspace
- `start_carla.sh` — launch CARLA simulator
- `start_gui.sh` — launch Qt6 GUI (sourced workspace + `ros2 run adas_gui adas_gui`)
- `start_bridge.sh` — launch the bridge node
- `start_pc_stack.sh` — orchestrate CARLA + GUI + bridge, gate on `check_hil_ready.py`, then start watchdog/monitor
- `start_pc_stack_clean.sh` — clean variant for tests

`scripts/check_hil_ready.py` is the post-launch HIL health gate (CARLA RPC, ROS topic freshness, lifecycle state). **Never `SIGKILL` the bridge process** — it can freeze CARLA. Use `SIGTERM` and the watchdog's two-stage (TERM → KILL) exit.

## SIL fallback (`scripts/run_sil_fallback.sh`)

The race-day fallback that runs without CARLA, Orin, F280025C, or CAN hardware. Internally launches `adas_launch/<scenario>.launch.py` from `adas_soc`, using `adas_sim_vehicle` as the world.

Common usage:
```bash
./scripts/run_sil_fallback.sh --build --host-tests --check       # first-time validation
./scripts/run_sil_fallback.sh --scenario acc|aeb|overtake|redundant|lqr
./scripts/run_sil_fallback.sh                                    # keep running, Ctrl-C to stop
```

- Default `ROS_DOMAIN_ID=145` (isolated from real HIL/GUI on `43`); other tools connecting to SIL must use the same ID.
- Logs → `logs/sil/<timestamp>_<scenario>.log`.
- `--check` exits after sampling required topics once and confirming `/adas/vehicle/actuation_cmd` heartbeats.
- `--host-tests` first runs the MCU GCC host regression (`adas_mcu/tests/run_host_tests.sh`).
- GUI SIL mode: `ADAS_GUI_MODE=sil ROS_DOMAIN_ID=145 ./adas_bridge_pc/start_gui.sh` reuses `run_sil_fallback.sh` as the back-end and never tries to launch CARLA / Orin / MCU.

## IoT stack (`adas_iot/`)

- `mqtt_bridge.py` — Orin-side bridge that subscribes to `/adas/*` and publishes to MQTT (deployed on Orin as `adas-mqtt.service`)
- `mqtt_bridge_node.py` — rclpy variant
- `dashboard/app.py` — Flask + SocketIO web dashboard
- `notifier.py` — webhook + email alerts
- `wechat_store.py` — WeChat mini-program backend
- `miniapp/` — WeChat mini-program (WXML/WXSS/JS)
- `config.yaml` — broker / endpoints / notifier config

## HIL tooling (`adas_tools/`)

- `harness/hil_preflight_before_mcu.sh` — operation-side gate before powering/resetting the MCU. Strict exit codes 0..7; never outputs PASS when conditions aren't met; never uses fixed sleeps.
- `harness/run_town04_navigation.sh`, `harness/wait_for_mcu_safe.sh` — scenario launchers
- `hil_logger/hil_logger.py` — log collection; `analyze_navigation.py` for offline analysis
- `performance_matlab/` — MATLAB plotting
- `harness_tests/` — Python unit tests for the harness scripts

## Adding a new SoC node (the canonical pattern)

Every package under `adas_soc/src/<role>/<pkg>/` follows the same two-layer split so algorithms can be unit-tested without `rclcpp`:

1. **`<pkg>_core.{hpp,cpp}`** — pure algorithm class, no rclcpp dependency, takes inputs by value/reference, returns outputs. This is what the gtest in `test/` exercises.
2. **`<pkg>_node.cpp`** — `LifecycleNode` shell that owns the core, declares params, sets up publishers/subscribers/timers, and forwards messages into the core. It registers `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` callbacks.
3. **`launch/<pkg>.launch.py`** (or added to an existing scenario in `adas_launch/launch/`) — declares the executable and parameters, and the orchestrator (`sil_launch_common.py`) calls `configure`/`activate` in deterministic order.

If the new node belongs to the activation chain, register it in `sil_launch_common.py` in the correct position; readiness is checked via lifecycle state, not `ros2 node list`.

The package's `CMakeLists.txt` must add the test executable via `ament_add_gtest(... <test_xxx>.cpp)` and `ament_target_dependencies(... $<pkg>_core)` so the core is testable without a ROS2 runtime.

## Common commands

### PC SIL (no hardware)
```bash
cd /home/xxs/bowen_ADAS
./scripts/run_sil_fallback.sh --build --host-tests --check
./scripts/run_sil_fallback.sh --scenario aeb
./scripts/run_sil_fallback.sh --scenario redundant
```

### Running a single test (faster iteration)

The default `colcon test` runs the entire workspace. To iterate on one package or one gtest binary:

```bash
# SoC: build + run only one package's gtests
cd adas_soc
source /opt/ros/jazzy/setup.bash
colcon build --packages-select adas_trajectory_follower
colcon test --packages-select adas_trajectory_follower --event-handlers console_direct+
colcon test-result --verbose                       # shows per-test pass/fail

# Even narrower: run a single gtest binary directly after build
./build/adas_trajectory_follower/test_pure_pursuit --gtest_filter='*Arc*'

# GUI: build + run only the Qt6 GUI tests
cd ../adas_bridge_pc/carla_ros2_bridge/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select adas_gui
colcon test --packages-select adas_gui --event-handlers console_direct+

# MCU: single host-test module (run_host_tests.sh runs the whole suite)
cd ../../adas_mcu
bash tests/run_host_tests.sh                      # full GCC regression
python3 tests/test_protocol_reference.py          # CAN v3 byte-level conformance
```

### Orin deploy / sync

`adas_bridge_pc/carla_ros2_bridge/Orin同步/src/` is the source-of-truth mirror that is copied to Orin's `~/adas/adas_soc_ws/src/`. After editing anything under `adas_soc/src/`, also update the same package under `Orin同步/src/` (they must stay byte-identical at the source level — Jetson rebuilds via `adas_soc/deploy/install_on_jetson.sh`, which never copies the PC build tree).

### SoC stack build & test
```bash
cd adas_soc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
ros2 launch adas_launch sil.launch.py
ros2 launch adas_launch sil_aeb.launch.py
ros2 launch adas_launch sil_redundant.launch.py
```

### Bridge + GUI build & run
```bash
cd adas_bridge_pc
./build.sh
./start_pc_stack.sh                  # CARLA + GUI + bridge, then gate on check_hil_ready.py
./start_gui.sh                       # GUI only
```

### MCU host tests (no hardware)
```bash
cd adas_mcu
bash tests/run_host_tests.sh         # full GCC host regression: safety, crc, control, self_test,
                                     # hil_session, asr_parser, can_comm, oled2, dgus_screen,
                                     # bus-off guard, main.c syntax, protocol reference
python3 tests/test_protocol_reference.py
```

### MCU target build
```bash
# Windows (CCS 21.0)
.\tools\build_ti.ps1 -Configuration all -ImageSet production
# Linux (cl2000)
./build.sh
```

### IoT
```bash
cd adas_iot
pip install -r requirements.txt
python3 mqtt_bridge.py
python3 dashboard/app.py
bash test_dashboard.sh
```

### HIL preflight (before MCU power-up)
```bash
bash adas_tools/harness/hil_preflight_before_mcu.sh
# exit codes 0=PASS, 1=ROS, 2=CAN if, 3=CAN frames missing,
# 4=CAN not contiguous, 5=CAN err growing, 6=timeout, 7=missing tool
```

### Deploy to Jetson
```bash
cd adas_soc
./deploy/install_on_jetson.sh        # rebuild there — never copy PC build tree
```

## Build artifacts & runtime layout

- `adas_soc/build/`, `adas_soc/install/`, `adas_soc/log/` — colcon output. `install/setup.bash` is the entry point.
- `adas_bridge_pc/carla_ros2_bridge/ws/build/`, `install/`, `log/` — same shape.
- `logs/` at repo root — `logs/sil/` (SIL fallback) and `logs/hil_run_<timestamp>/` (HIL stack).
- `adas_mcu/adas_mcu_flash.out` and `.hex` — production image output of `build.sh`.

## Safety invariants to preserve

- **Fail-closed**: stale/invalid input → lock brakes; recovery requires 3 consecutive valid frames.
- **MCU v3 session gate**: self-driven authorization; on power-up auto-progresses `ANNOUNCE → READY → ARMED → ACTIVE`.
- **FTTI budgets** in `adas_config.h` are compile-time checked via `#error` — changing timeouts must satisfy all six budget constraints or build fails.
- **bus-off recovery cannot be disabled** — `run_host_tests.sh` includes a negative compile guard that must keep failing.
- **No silent reconfiguration**: a single `adas_msgs` field rename breaks PC↔Orin↔MCU; the duplicate `adas_msgs` in `adas_bridge_pc` is a contract, not a convenience copy.
- **CARLA is shared**: probe with `adas_bridge_pc/scripts/carla_readiness.py` before/after starting, never `SIGKILL` the bridge, and treat an existing port-2000 listener as an external instance to reuse (don't double-start).