# bowen_ADAS — 三机 HIL 硬件在环 ADAS 系统

本仓库汇总了原 `ADAS_ORIN_TI` 项目中的全部核心代码(已剔除构建产物、二进制、文档、缓存),并按模块重新组织为 6 个一级目录,便于检索与二次开发。

> 完整项目背景、协议契约、跨代码库约定请参见 [`PROTOCOL.md`](./PROTOCOL.md) 与原 `ADAS_ORIN_TI/CLAUDE.md`;本仓库聚焦代码,不再保留历史 markdown 笔记。

---

## 1. 仓库结构

| 一级目录 | 原路径 | 内容 | 工具链 |
|----------|--------|------|--------|
| **`adas_soc/`** | `ADAS0.0.2/SoC/` | C++17 多节点 ROS2 ADAS 栈:感知→行为→轨迹→跟踪→命令门控→AEB→冗余→安全监控 | colcon;PC SIL 用 **Jazzy**,Jetson HIL 用 **Humble** |
| **`adas_mcu/`** | `ADAS0.0.2/MCU/` | TMS320F280025C 裸机 C 固件:CAN v3 协议、安全状态机、1 kHz 执行器驱动 | CCS + C2000Ware(Windows);主机测试用 GCC |
| **`adas_bridge_pc/`** | `adas_pc/` | CARLA↔SoC 桥接(Python rclpy)+ Qt6 安全监控 GUI | colcon + ROS2 Jazzy |
| **`adas_iot/`** | `adas_iot/` | MQTT 桥接、Flask/SocketIO Dashboard、微信小程序、Webhook/邮件告警 | Python(Orin 部署为 `adas-mqtt.service`) |
| **`can_bench/`** | `can_benchmark/` | CAN 往返延迟基准:ESP32 应答器 / F280025C / PC | 各平台独立 |
| **`adas_tools/`** | `tools/` | hil_logger、HIL 前置检查脚本、MATLAB 绘图 | Python / Shell / MATLAB |

> 完整节点/包清单共 **20 个 ROS2 节点**(soc 端)+ **4 个桥接包**(bridge 端),详见 [`adas_soc/src/`](./adas_soc/src) 与 [`adas_bridge_pc/carla_ros2_bridge/ws/src/`](./adas_bridge_pc/carla_ros2_bridge/ws/src)。

---

## 2. 快速安装

### 2.0 比赛日硬件故障兜底：PC SIL

`scripts/run_sil_fallback.sh` 是独立的 PC SIL 入口。它只使用 `adas_soc` 内置的
运动学仿真车辆和 ROS2 节点，不需要 CARLA、Orin Nano、F280025C、CAN 适配器或实际传感器。
首次使用可直接执行：

```bash
cd /home/xxs/bowen_ADAS
./scripts/run_sil_fallback.sh --build --host-tests --check
```

验收通过后，比赛现场持续运行基准 SIL：

```bash
./scripts/run_sil_fallback.sh
```

按 `Ctrl-C` 停止。可用 `--scenario acc|aeb|overtake|redundant|lqr` 切换内置场景，
日志默认写入 `logs/sil/`。脚本默认使用 `ROS_DOMAIN_ID=145`，与真实 HIL/GUI 隔离；
其它要接入 SIL 的工具必须显式使用同一 Domain。

原 Qt GUI 的 SIL 适配保留原有界面、地图/导航、日志、安全态势和故障注入交互，
只替换数据源与启动后端。启动方式：

```bash
cd /home/xxs/bowen_ADAS
ADAS_GUI_MODE=sil ROS_DOMAIN_ID=145 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ./adas_bridge_pc/start_gui.sh
```

GUI 的“一键启动完整系统”在该模式下启动或复用 `run_sil_fallback.sh`，不会拉起
CARLA，也不会尝试连接 Orin、F280025C 或 CAN 设备。

### 2.1 顶层依赖(系统级)

```bash
# Ubuntu 24.04 (PC) 与 22.04 (Jetson) 通用
sudo apt update && sudo apt install -y \
  build-essential cmake git python3-pip python3-venv \
  libeigen3-dev libpcl-dev libboost-all-dev \
  can-utils net-tools iproute2

# ROS2 — PC 端 Jazzy
sudo apt install ros-jazzy-desktop ros-jazzy-rclpy ros-jazzy-std-msgs \
  ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-nav-msgs \
  ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-cyclonedds ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-launch ros-jazzy-launch-ros

# ROS2 — Jetson 端 Humble
sudo apt install ros-humble-desktop ros-humble-rclpy ros-humble-std-msgs \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
  ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-launch ros-humble-launch-ros
```

### 2.2 克隆仓库

```bash
git clone https://github.com/xiaoxiaoshibw/IOT_TI_ADAS_XXS.git bowen_ADAS
cd bowen_ADAS
```

### 2.3 模块构建(按需)

```bash
# ── adas_soc:SoC 栈 ─────────────────────────────────────
cd adas_soc
source /opt/ros/jazzy/setup.bash          # PC SIL 用 Jazzy
colcon build --symlink-install
colcon test && colcon test-result --verbose
ros2 launch adas_launch sil.launch.py     # 全栈 SIL 闭环

# ── adas_mcu:MCU 固件(主机测试,无需硬件) ─────────────
cd ../adas_mcu
bash tests/run_host_tests.sh              # 完整主机回归套件

# ── adas_mcu:目标板烧录(需要 LAUNCHXL-F280025C) ──────
# 详见 .claude/skills/flash-f280025c(CCS 21.0 / CGT 25.11.1 LTS / C2000Ware 26.01)
# Windows:  .\tools\build_ti.ps1 -Configuration all -ImageSet production
# Linux  :  ./build.sh                     # cl2000 + dslite.sh

# ── adas_bridge_pc:PC 桥接栈 ───────────────────────────
cd ../adas_bridge_pc
./build.sh                                # 内部走 carla_ros2_bridge/ws/ 下 colcon
./start_pc_stack.sh                       # CARLA + GUI + 桥接 一键启动

# ── adas_iot:IoT 端(MQTT + Dashboard) ─────────────────
cd ../adas_iot
pip install -r requirements.txt
python3 mqtt_bridge.py                    # MQTT 桥接节点
python3 dashboard/app.py                  # Flask 仪表板

# ── can_bench:CAN 延迟基准 ────────────────────────────
cd ../can_bench/pc
python3 bench.py                          # PC 端基准
# 嵌入式端:esp32_responder/ 与 f280025c_bench/ 见各自 README

# ── adas_tools:HIL 工具集 ─────────────────────────────
cd ../adas_tools
bash harness/hil_preflight_before_mcu.sh  # HIL 前置检查
python3 hil_logger/hil_logger.py          # 日志采集
```

---

## 3. 三机协同架构

```
PC (CARLA 仿真 + ROS2 Jazzy 桥接)  ⇄  DDS/ROS2  ⇄  Jetson Orin Nano (SoC 栈, ROS2 Humble)
                                                          │ CAN (SocketCAN can1 @ 500k, PEAK PCAN-USB)
                                                          ▼
                                         TI LAUNCHXL-F280025C MCU (实时安全裁决器)
                                                          │
                                              舵机 / 转向灯 / 蜂鸣器

                                       ┌── adas_iot/MQTT ──┐
                                       │   ↓               │
                                       │ Dashboard / 微信小程序 / Webhook / 邮件
                                       └───────────────────┘
```

> `ROS_DOMAIN_ID=43` 在 PC 与 Orin 上必须保持一致。
> Jetson 板载 `mttcan`(can0)引脚未引出至 40-pin 排针,**必须**使用 PEAK PCAN-USB 外接适配器(`can1` @ 500k)。

---

## 4. 跨代码库契约(修改前必读)

下列接口跨越多个子目录,改一边不改动另一边会破坏闭环:

1. **`adas_msgs`** — ROS2 消息包在两处必须完全一致:
   - [`adas_soc/src/common/adas_msgs/`](./adas_soc/src/common/adas_msgs)
   - [`adas_bridge_pc/carla_ros2_bridge/ws/src/adas_msgs/`](./adas_bridge_pc/carla_ros2_bridge/ws/src/adas_msgs)
2. **CAN 协议 v3** — 唯一权威定义: [`adas_mcu/include/adas_can_protocol.h`](./adas_mcu/include/adas_can_protocol.h)
   - 参考编码器: `adas_mcu/tools/orin_can_reference.py` / `.cpp`
   - 字节级一致性测试: `adas_mcu/tests/test_protocol_reference.py`
   - Jetson 侧 CAN 唯一所有者: [`adas_soc/src/vehicle/adas_can_gateway`](./adas_soc/src/vehicle/adas_can_gateway)
3. **`/adas/*` 主题契约** — 发布者 QoS 与类型定义在 `adas_bridge_pc` 侧;SoC 栈消费这些话题。
4. **`adas_bridge_pc/carla_ros2_bridge/Orin同步/`** — 部署到 Orin 的 `~/adas/adas_soc_ws/` 的源码。

---

## 5. 安全约定(关键)

- **故障闭合(fail-closed)**:过期/无效输入 → 锁止制动;恢复需连续 3 帧有效验证
- **MCU v3 会话门**:自驱授权 — 上电后自动推进 `ANNOUNCE → READY → ARMED → ACTIVE`
- **可调参数唯一来源**:
  - MCU:仅 [`adas_mcu/include/adas_config.h`](./adas_mcu/include/adas_config.h)
  - SoC:`adas_soc/src/launch/adas_launch/config/*.yaml`
- **节点激活顺序**(SoC 生命周期):`vehicle_interface → command_gate → safety_monitor → aeb → follower → planner → behavior → tracker`
- **就绪判定**:以节点达到 `active` 为准,**不要**用 `ros2 node list`(跨发行版不可靠)

---

## 6. 环境差异警告

- SoC 栈 PC 端用 **Jazzy** 验证,Jetson 端用 **Humble** 部署 — **不要**把 PC 的 build 树拷到 Jetson,必须在 Orin 上重新 `colcon build`(`adas_soc/deploy/install_on_jetson.sh`)
- MCU 的 C28x 架构 `char` 是 16 位,CAN 数据用 `uint16_t[8]` 承载,每元素仅低 8 位有效
- 跨发行版节点名发现不可靠 — 用话题心跳判断 HIL 健康,见 `adas_bridge_pc/scripts/check_hil_ready.py`
- Jetson HIL CAN 链路:**PEAK PCAN-USB** → SocketCAN `can1` @ 500k(2026-07-19 复验)
- PC CARLA **共享且容易撞车** — 启动/停止前先 `adas_bridge_pc/scripts/carla_readiness.py` 探活;桥接进程**不要**用 `SIGKILL`(会冻死 CARLA),用 `SIGTERM`

---

## 7. 文件统计

```
总文件:615 个,4.7 MB
├─ py  147  (Python 桥接 / 测试 / 工具)
├─ cpp 108  (SoC 节点)
├─ hpp  71  (SoC 头文件)
├─ msg  66  (ROS2 消息定义)
├─ sh   41  (Shell 脚本)
├─ yaml 39  (配置)
├─ xml  30  (ROS2 / CycloneDDS / CCS / systemd)
├─ txt  29  (协议/参考)
├─ c    25  (MCU 固件)
├─ h    18  (MCU 头文件)
├─ json  9  (小程序 / 配置)
├─ service  5  (systemd 单元)
├─ js    4  (微信小程序)
├─ cfg   4
├─ wxss  3
├─ srv   3
└─ 其他:cmd / ccxml / timer / wxml / m / dockerignore / gitignore / clang-format / html / example
```

---

## 8. 致谢

- TI C2000Ware 26.01 / CGT 25.11.1 LTS / CCS 21.0
- ROS2 Jazzy(PC)+ ROS2 Humble(Jetson)
- CARLA Simulator
- PEAK-System PCAN-USB 适配器

---

## 9. 许可证

本仓库为内部研究/竞赛代码,具体许可请参考原 `ADAS_ORIN_TI/CLAUDE.md` 与各模块说明;如未明确,默认为项目内使用。
