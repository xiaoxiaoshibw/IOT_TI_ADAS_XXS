# Phase 4 物理会话 — 现场审核 Checklist

> **目的**: 在把 `physical_session.env` 提交给 GUI / runner 之前，由现场负责人对照本表
> 逐条勾选。它不是自动化测试，而是**让单次现场跑可以追溯到一次明确的人工审核**。
> 闭环基线见 [hil_physical_session_command_contract.md](hil_physical_session_command_contract.md) 第 1 节。

## 0. 元信息（必填）

| 字段 | 值 |
|---|---|
| 审核日期 | YYYY-MM-DD |
| 审核人 | （姓名 + 签字 / 电子签字 ID） |
| 台架编号 | （哪一台 Jetson + 哪一台 MCU + 哪一台 PC） |
| 当前 run_id | （本次会话的 canonical UUID v4） |
| `physical_session.env` 文件名 |  |

## 1. 全局公共检查（适用于 13 条非 MANUAL_ONLY 命令）

每一条命令都需要通过 §1.1～§1.8。

### 1.1 argv[0] 可解析

- [ ] `argv[0]` 为绝对路径或在 PATH 上（`which <argv[0]>` 在执行账号下能找到）
- [ ] 如走 `ssh`/`sudo`/封装脚本，已知其版本与参数方言（不假设新版本默认行为）

### 1.2 退出码可信（C7）

- [ ] 健康路径退出 0
- [ ] 异常路径非 0，且能区分"工具不存在 / 命令超时 / 抓取到异常现象"等语义
- [ ] 没有"看着差不多就 exit 0"的吞错路径

### 1.3 超时与 SIGTERM（C2 / C3）

- [ ] 命令在 runner 设定的 `duration+15s` 超时内可自然退出
- [ ] 命令响应 SIGTERM 时不会留下孤儿进程（自身 + 直接 fork 的子进程）
- [ ] 命令**不**调用 `setsid` 创建第二层 process group（runner 用 `killpg` 触达不到）

### 1.4 不读 stdin / 不用 TTY（C4）

- [ ] 不期望任何交互输入
- [ ] 不向 `isatty` 分支走不同逻辑（如 SSH 已强制 `BatchMode=yes`、`sudo -n` 无 TTY）

### 1.5 不污染 runner 之外的文件（C5）

- [ ] 写文件仅限 `${SESSION_DIR}/`（evidence 目录）
- [ ] 不写 `~/.bash_history`、`/etc/...`、`/var/log/syslog` 等外部位置

### 1.6 不携带凭据（C6）

- [ ] argv 里不出现明文密码、token、密钥
- [ ] 凭据走 SSH agent / NOPASSWD sudo / 预部署凭证文件

### 1.7 作用域受限（C8）

- [ ] 命令的副作用被限定在合同 §3 这一步列出的范围内
- [ ] 没有 `rm -rf /`、`kill -9`、防火墙规则改动、车体电动作等隐式副作用

### 1.8 stdout / stderr 可审计

- [ ] 关键字段（run_id、axis、状态名、计数）以可 grep 的键值形式输出
- [ ] 不在 stdout/stderr 回显凭据、长 payload、二进制数据

## 2. 逐条 step 加项检查

下表覆盖 §3 合同里命令特别的越界红线。**所有打 ✗ 项必须在启动前修复**，否则不要勾选本节。
对 MANUAL_ONLY step（4.3.3 / 4.3.4）跳过本节。

| # | step_id | 加项 | ✓/✗ |
|---|---|---|---|
| 2.1 | `pre_session_checklist` (4.0) | 仅读取本机 `git status`、`flash_meta.json`、`<SESSION_DIR>`；不调外网；不写 `/etc` | ☐ |
| 2.2 | `orin_service_readiness` (4.1.2) | SSH 强制 `-o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new`；stdout 期望 `active` | ☐ |
| 2.3 | `mcu_session_handshake` (4.1.3) | 只读 `can1`；退出码与状态名一致；artifact 路径为 `<SESSION_DIR>/can1_session.log` | ☐ |
| 2.4 | `actuator_drive_check` (4.2.1) | `--axis drive`；不动 trajectory planner；超时内自然退出 | ☐ |
| 2.5 | `actuator_steer_check` (4.2.2) | `--axis steer`；不动 trajectory planner；超时内自然退出 | ☐ |
| 2.6 | `actuator_brake_check` (4.2.3) | `--axis brake`；不动 trajectory planner；超时内自然退出 | ☐ |
| 2.7 | `feedback_freshness_check` (4.2.4) | 仅读 DDS/CAN；不动控制环；输出三行键值 | ☐ |
| 2.8 | `fault_mcu_power_off` (4.3.1) | 电源命令作用域仅 `mcu-main`，**禁止** `all` / `jetson` / 辅助电源 | ☐ |
| 2.9 | `fault_can_disconnect` (4.3.2) | 仅 down `can1`；**禁止**动 `can0` / `eth0` / `wlan0`；**禁止** `ip link delete` | ☐ |
| 2.10 | `service_restart` (4.3.5) | `sudo -n` 已配 NOPASSWD；**禁止**触发 `carla-bridge.service` 链式重启 | ☐ |
| 2.11 | `long_run_stability` (4.4.1) | 自带内部退出机制，不依赖 runner 强 SIGTERM；输出 `latches=0 run_id_drift=0` | ☐ |
| 2.12 | `service_stop_start_round_trip` (4.4.2) | stop+start 之间 sleep ≥ 5 s；只为 `adas-hil.service` 一轮 | ☐ |
| 2.13 | `pc_carla_restart_round_trip` (4.4.3) | 走 `start_pc_stack_clean.sh` 自身两阶段退出；**禁止**命令内显式 `kill -9` | ☐ |

## 3. dry-run 一次闭环

- [ ] `bash adas_bridge_pc/tools/hil/physical_session_checklist.sh --dry-run --run-id <UUID>` 退出 0
- [ ] 15 对 `step_start` / `step_end` 全部出现
- [ ] 所有 artifact 首行均为 `DRY_RUN`；`fault_bad_crc_firmware` / `fault_sequence_corruption` 为 `SKIP` 并写有 `MANUAL_ONLY` 标记
- [ ] `state.json` 中 13 步 PASS、2 步 SKIP，无 FAIL/crash

## 4. 真实台架会签

dry-run 通过后，**只在现场**做一次真实 `Run all`，并要求两位操作员见证：

| 角色 | 姓名 | 签字 / 电子 ID | 时间 |
|---|---|---|---|
| 主操作员 | | | |
| 监督人（按物理急停） | | | |

## 5. 异常 / FAIL 时追溯路径

- 当前 `RUNNING` 步骤自动被 startup 恢复为 `FAIL(reason=crashed)`：见 `state.json` → `crashed=True`
- 物理急停已按 → GUI `E-stop` 之后才点：确认 `runner.log` 末尾为 `ABORTED <ts>`
- 命令超时：artifact 文件末尾为 `TIMEOUT`
- evidence 目录**不会被清理**，可以从 `logs/hil_run_<ts>/physical_session/` 整目录复制归档

## 6. 关闭 G12 物理部分的硬条件（4.2 三步）

- [ ] 4.2.1 / 4.2.2 / 4.2.3 全部为 `PASS`
- [ ] `causal_timing.csv` 至少包含 `drive`、`steer`、`brake` 三个轴行
- [ ] 三行均含 `<cmd_ts>,<fb_ts>,<carla_ts>,PASS`
- [ ] 三行的 `<fb_ts>-<cmd_ts>` 在 §3 合同给定的窗口内
