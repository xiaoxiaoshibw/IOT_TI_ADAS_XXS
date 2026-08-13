# Phase 4 物理步骤命令合同

> **状态**: 待现场审核。本文件只描述每一步的 `PHYSICAL_<STEP_ID>_COMMAND` 应当具备的
> **形状**，不写任何主机名 / IP / 电源控制器地址 / 线缆编号 / 凭据。
> 与 `hil_physical_session.md`、`hil_phase4_plan.md` 同源；执行入口仍是
> `adas_bridge_pc/tools/hil/physical_session_checklist.sh`。

## 1. Runner 对命令的处理（这是不可变合同）

`steps.py::StepContext.run_configured` 在执行一个 step 的命令时，严格按下面 6 条处理：

1. **读取**: `os.environ["PHYSICAL_" + STEP_ID.upper() + "_COMMAND"]`。
2. **拆分**: `shlex.split(raw)`。**不再做任何 shell 展开、不读 `~/`、不解析变量。**
   即字符串字面是什么样，argv 就是什么样。
3. **存在性**: `argv` 为空 → 返回 `FAIL(exit_code=7)`，理由 `missing PHYSICAL_<STEP_ID>_COMMAND`。
4. **可执行**: `shutil.which(argv[0]) is None` → 返回 `FAIL(exit_code=7)`，理由 `tool missing: <argv[0]>`。
   argv[0] 必须是 PATH 上能查到的可执行，或绝对路径。
5. **执行**: `subprocess.Popen(argv, stdout=PIPE, stderr=PIPE, env=os.environ, start_new_session=True)`。
   - 超时 = `parse(duration) + 15s`。超时后调用 `terminate_active_process()`，**只发 SIGTERM**，
     永远不会从 runner 内部升 SIGKILL（这是为了不误杀命令可能拉起的辅助进程）。
   - `stdout` 与 `stderr` 都被读取，合并后写入对应 artifact 文件。
6. **判定**: 进程退出码为 0 → `PASS`；非 0 → `FAIL(exit_code=returncode)`。

由此得出**对每条命令的硬约束**：

| # | 约束 | 触发原因 |
|---|---|---|
| C1 | argv[0] 在 PATH 上或为绝对路径 | 缺 C1 → 步骤 exit 7 |
| C2 | 命令必须在 `duration+15s` 内自然退出；否则 SIGTERM 后最多 5s 内必须退出 | 缺 C2 → 步骤 FAIL，超时 |
| C3 | 命令必须主动响应 SIGTERM（同 process group）；不能屏蔽或忽略 | 缺 C3 → 卡死只能由现场物理断电 |
| C4 | 不允许从 stdin 读（runner 不送 stdin） | 缺 C4 → 命令 EOF 即退出 |
| C5 | 不允许向 evidence 目录外的路径写敏感文件 | 缺 C5 → 污染外部证据 |
| C6 | `argv` 不携带密码、私钥、`--sudo` 之类需要 TTY 的参数；命令需要的敏感信息从环境变量（runner 转发 `os.environ`）或预部署的可读文件读 | 缺 C6 → 凭据落 argv 被 `ps`/日志暴露 |
| C7 | 命令执行后退出码必须可信地反映"执行结果"。**禁止**"看起来成功就 exit 0，异常时也 exit 0" | 缺 C7 → FAIL 路径被静默吃掉 |
| C8 | 命令的作用对象必须是本台架（jetson / MCU / CAN / PC / CARLA）其中**一个**已授权可执行域；任何"以防万一 rm -rf /" / "kill -9 systemd" / 改防火墙 这种隐式副作用都禁止 | 缺 C8 → 越界 |

**自动派生**:

- `start_new_session=True` → 命令获得自己的 process group。runner 的 SIGTERM 通过 `os.killpg(pid, SIGTERM)` 发给整组。
  命令如果 fork 子进程又忘了卸下 group，子进程会一起被 TERM。**禁止命令内部再用 `setsid` 创建第二层 group**（否则 runner 无法触达）。
- `env=os.environ` → 命令能读到 `PATH`、`HOME`、`ROS_DOMAIN_ID` 等。需要 run_id / session_dir 时由 bench 在
  `physical_session.env` 里预先 export，命令从 env 读。

## 2. 不读 `PHYSICAL_<STEP_ID>_COMMAND` 的 step

以下两步是 `MANUAL_ONLY`，**runner 不调用命令**——只写 `<step_id>.MANUAL_ONLY` 标记，
`dry-run` 时为 `SKIP`，正式 `RUN_PHYSICAL_CONFIRM=1` 后为 `PASS`。

| step_id | runner 行为 | 占位策略 |
|---|---|---|
| `fault_bad_crc_firmware` (4.3.3) | 写标记 → SKIP / PASS；不调用命令 | env 文件可保持 unset |
| `fault_sequence_corruption` (4.3.4) | 同上 | env 文件可保持 unset |

即 `physical_session.env` 只需要给 **13 条**命令填占位。

## 3. 每一步的字段定义

每条命令应当满足：

- **角色**: 这一步在台架上要证明什么
- **argv[0]**: 可执行文件 / 脚本 / 工具的固定名或路径模式
- **位置参数或必填开关**: 命令接收哪些 runner 不能帮它填的固定输入
- **可选 runtime 输入**: 命令想要拿到的 run-id / session-dir / 时长 / 轴等
- **副作用范围**: 命令允许触碰的硬件 / 进程 / 文件
- **退出码契约**: 0 / 特定非 0 / 必须非 0 的语义
- **stdout / stderr 期望形态**: 后续审计需要从中 grep 的字段
- **artifact 路径**: 该 step 写入的 evidence 文件名（runner 已固定，操作员不必改）

下方表格里 `<...>` 为**字面占位符**，必须在 bench 填写时替换为真实值。
所有"前置"列与 `hil_phase4_plan.md` §1 一致。

### 4.0 D-1  `pre_session_checklist`

| 字段 | 期望 |
|---|---|
| 角色 | 重跑 `hil_preflight_before_mcu.sh` 之外的本地检查（flash_meta、run_id、CAN topology 三件套需要本地读） |
| argv[0] | `<PREFLIGHT_HELPER>` （路径模式，下同；bench 决定走哪个二进制 / 脚本） |
| runtime 输入 | `--session-dir "<SESSION_DIR>" --run-id "<RUN_ID>"` |
| 副作用范围 | 仅本机：读取 `git status`、MCU `flash_meta.json`、当前 `logs/hil_run_<ts>/` |
| 退出码 | 0=PASS；非 0=FAIL |
| stdout/stderr 期望 | 末尾不少于一行形如 `preflight=PASS run_id=<uuid> topology=can1`（grep 锚点） |
| artifact | `preflight.log`（runner 写）、`flash_meta.json`、`run_id.txt`、`can_topology.txt`（runner 写） |
| C6 备忘 | 不读 SSH 凭据；不调外网 |

### 4.1.2  `orin_service_readiness`

| 字段 | 期望 |
|---|---|
| 角色 | 确认 jetson 端 `adas-hil.service` 处于 active，且与 GUI 的 run_id 一致 |
| argv[0] | `ssh` 或封装 SSH 的同等工具；本机零依赖可执行 |
| runtime 输入 | `<USER>@<JETSON_HOST> systemctl is-active adas-hil.service` |
| 副作用范围 | 仅 SSH 会话（BatchMode=yes，不分配 TTY） |
| 退出码 | SSH 退出 0 且 `systemctl is-active` 文本为 `active` → PASS；否则 FAIL |
| stdout/stderr 期望 | 仅 `active\n` |
| 强制 SSH 选项 | `-o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new`，避免 runner 卡 SSH 提示符 |
| artifact | `systemd.txt`、`run_id.txt`（runner 写） |

### 4.1.3  `mcu_session_handshake`

| 字段 | 期望 |
|---|---|
| 角色 | 录制 MCU v3 session 上 ANNOUNCE → READY → ARMED → ACTIVE 的 0x101~0x104 状态迁移 |
| argv[0] | `<BENCH_CAPTURE_BIN>`（bench 自定，可能是 `candump` + `systemd-run` 包装） |
| runtime 输入 | `--run-id "<RUN_ID>" --session-dir "<SESSION_DIR>" --timeout 30` |
| 副作用范围 | 只读 `can1`；写 `<SESSION_DIR>/can1_session.log` |
| 退出码 | 0=PASS；非 0=FAIL |
| stdout/stderr 期望 | 至少包含四个状态名（顺序无关）以及一个时间戳锚点 |
| artifact | `can1_session.log`（runner 写） |

### 4.2.1 / 4.2.2 / 4.2.3  `actuator_{drive,steer,brake}_check`

> 三步结构同形，按轴替换：`--axis drive|steer|brake`。

| 字段 | 期望 |
|---|---|
| 角色 | 在 MCU closed loop 下对单一执行器发命令并捕获因果（command→feedback→CARLA telemetry） |
| argv[0] | `<ACTUATOR_PROBE_BIN>` |
| runtime 输入 | `--axis <drive\|steer\|brake> --run-id "<RUN_ID>" --session-dir "<SESSION_DIR>" --timeout 75` |
| 副作用范围 | 只对 MCU 发控制字；通过 `carla_measure`（或 bench 等价物）采样 telemetry；不修改 trajectory planner 输出 |
| 退出码 | 0=捕获完整因果链；1=任何一帧缺失；2=超时但部分帧已采 |
| stdout/stderr 期望 | 至少一行 `<axis>,<cmd_ts>,<fb_ts>,<carla_ts>,PASS\|FAIL`（与 `causal_timing.csv` 表头一致） |
| artifact | `causal_timing.csv`（runner 追加） |
| C2/C3 备忘 | 必须响应 SIGTERM；超时由 runner 触发 SIGTERM 后进程须在 5s 内退出 |

### 4.2.4  `feedback_freshness_check`

| 字段 | 期望 |
|---|---|
| 角色 | 校验 `/adas/mcu/actuation_feedback` 在窗口内 ≥ 100 Hz、`max_age_ms < 10`、无 sequence gap |
| argv[0] | `<FEEDBACK_PROBE_BIN>`（大概率是 `topic_hz -w <window>` 的包装） |
| runtime 输入 | `--topic "<FEEDBACK_TOPIC>" --window 5s` |
| 副作用范围 | 只读 DDS / `candump`；不动控制环 |
| 退出码 | 0=PASS；非 0=FAIL |
| stdout/stderr 期望 | 含 `rate_hz=`、`max_age_ms=`、`sequence_gaps=` 三行字段名 |
| artifact | `feedback_hz.txt`（runner 写） |

### 4.3.1  `fault_mcu_power_off`

| 字段 | 期望 |
|---|---|
| 角色 | 通过 bench 电源控制器切断 MCU 供电（≤ 1 s 失电），记录 `/adas/control/...` 收到 MRM brake 的延迟 |
| argv[0] | `<POWER_CTRL_BIN>` 或 `ssh ... <POWER_CTRL_SHELL>` |
| runtime 输入 | `--action power-off --run-id "<RUN_ID>" --session-dir "<SESSION_DIR>" --duration-ms 200` |
| 副作用范围 | **仅** `ssh jetson power-ctrl cut mcu main`（或 bench 等价）；不触碰车体电、不触发 ABS / 转向 |
| 退出码 | 0=电源已确认切断且 MRM 已被捕获；1=电源切断失败；2=MRM 未触发（FAIL 双重原因） |
| stdout/stderr 期望 | 包含 `power_off_ts=`、`mrm_brake_ts=`、`delta_ms=` |
| artifact | `fault_mcu_power_off.log`（runner 写） + 条件追加 `causal_brake_trace.csv` |
| C8 红线 | 严禁 `power-ctrl cut all`、`power-ctrl cut jetson`、任何波及 jetson 或辅助电源的命令 |

### 4.3.2  `fault_can_disconnect`

| 字段 | 期望 |
|---|---|
| 角色 | `ip link set can1 down`（仅 `can1`，不动 `can0`），随后捕获 0x201 brake 命令 |
| argv[0] | `ssh` 调远端 `ip` 命令，或 bench 等价封装 |
| runtime 输入 | `<USER>@<JETSON_HOST> sudo -n ip link set can1 down` |
| 副作用范围 | **只** down `can1`；其它网口、车体网、wifi 不动 |
| 退出码 | 0=down 完成且 MRM 已捕获；1=链路未 down；2=MRM 未触发 |
| stdout/stderr 期望 | 包含 `link_down_ts=`、`mrm_brake_ts=`、`delta_ms=` |
| artifact | `fault_can_disconnect.log`（runner 写） + 条件追加 `causal_brake_trace.csv` |
| C8 红线 | 严禁 `ip link set can0 down`、严禁 `ip link delete can1`、严禁动 eth0 / wlan0 |

### 4.3.3 / 4.3.4  `fault_bad_crc_firmware` / `fault_sequence_corruption`

> MANUAL_ONLY：runner 不调用命令，bench 无需 export。但 §3 起首字段顺序保留，
> 便于现场随时把"操作员口头指令"也用同一结构登记。

| 字段 | 期望 |
|---|---|
| 角色 | 由现场监督人重刷 CRC-bad 固件 / 拔出 gripper 制造 sequence 错位 |
| argv[0] | —（runner 不读） |
| runtime 输入 | — |
| 副作用范围 | 仅由人执行；bench 不命令化 |
| 退出码 | 监督人在 GUI 点 `Confirm` 视为 `PASS`；点 `Skip` 视为 `SKIP` |
| artifact | `<step_id>.MANUAL_ONLY`（runner 写标记） |

### 4.3.5  `service_restart`

| 字段 | 期望 |
|---|---|
| 角色 | `systemctl restart adas-hil.service`，确认新 PID、run_id 不漂移 |
| argv[0] | `ssh` 或封装 SSH |
| runtime 输入 | `<USER>@<JETSON_HOST> sudo -n systemctl restart adas-hil.service` |
| 副作用范围 | **仅** `adas-hil.service`；不让它把 `carla-bridge.service` 一起重启 |
| 退出码 | 0=PASS；非 0=FAIL |
| stdout/stderr 期望 | 含新 PID、active 状态字 |
| artifact | `service_restart.log`、`run_id.txt`（runner 写） |
| C8 红线 | `sudo -n` 即 `non-interactive sudo`，必须有 NOPASSWD；禁止要 TTY 的 `sudo` |

### 4.4.1  `long_run_stability`

| 字段 | 期望 |
|---|---|
| 角色 | 30 分钟无 latch、`/adas/vehicle/actuation_cmd` 心跳连续、`run_id` 无漂移 |
| argv[0] | `hil_logger` 或等价长时间采样工具 |
| runtime 输入 | `--duration 30m --topic "/adas/vehicle/actuation_cmd" --rate-min 100` |
| 副作用范围 | 只读 ROS / CAN；不动 actuator |
| 退出码 | 0=PASS；非 0=FAIL（含 `latch` 或 `gap` 触发） |
| stdout/stderr 期望 | `duration_minutes=`、`latches=`、`run_id_drift=` |
| artifact | `long_run_summary.txt`（runner 写） |
| C2 备忘 | runner 把超时设到 30m+15s=1815s，命令必须自己控制退出时机 |

### 4.4.2  `service_stop_start_round_trip`

| 字段 | 期望 |
|---|---|
| 角色 | `systemctl stop adas-hil.service` → 等待 inactive → `systemctl start` → 确认 active，run_id 在整轮中不变 |
| argv[0] | `ssh` 调远端 `systemctl` |
| runtime 输入 | `<USER>@<JETSON_HOST> sudo -n bash -c 'systemctl stop adas-hil.service; sleep 5; systemctl start adas-hil.service; systemctl is-active adas-hil.service'` |
| 副作用范围 | **仅** `adas-hil.service` |
| 退出码 | 0=PASS；非 0=FAIL |
| artifact | `service_round_trip.log`（runner 写） |

### 4.4.3  `pc_carla_restart_round_trip`

| 字段 | 期望 |
|---|---|
| 角色 | 在 PC 端 `start_pc_stack_clean.sh` 软重启一遍，runner **不**触发 bridge watchdog 自动 kill，等待 30s 自然恢复 |
| argv[0] | 本地可执行（`bash` 直接调起即可） |
| runtime 输入 | `<SCRIPT_PATH> --clean-restart --run-id "<RUN_ID>" --wait 30` |
| 副作用范围 | 仅 PC 端 CARLA + bridge 进程栈；不触碰 jetson 端 |
| 退出码 | 0=新栈 READY；非 0=FAIL |
| artifact | `pc_carla_round_trip.log`（runner 写） |
| C8 红线 | 命令内禁止显式 `kill -9`、`kill -SIGKILL`；必须走 `start_pc_stack_clean.sh` 自带的两阶段退出 |

## 4. 占位符约定（bench 替换规则）

| 占位符 | 含义 | 谁来替换 |
|---|---|---|
| `<RUN_ID>` | canonical UUID v4（runner 已经持有，bench export 时通过 `envsubst` 注入命令） | bench |
| `<SESSION_DIR>` | `logs/hil_run_<ts>/physical_session`（evidence 根） | bench |
| `<USER>` | jetson 上的非 root 服务账号 | bench |
| `<JETSON_HOST>` | jetson 主机名 / IP（不写在此文件中；运行时由 SSH config 解析） | bench |
| `<PREFLIGHT_HELPER>` / `<BENCH_CAPTURE_BIN>` / `<ACTUATOR_PROBE_BIN>` / `<FEEDBACK_PROBE_BIN>` / `<POWER_CTRL_BIN>` | bench 自定的工具路径 | bench |
| `<FEEDBACK_TOPIC>` | 反馈 topic 实际名（典型 `/adas/mcu/actuation_feedback`；冗余模式下 `/primary/adas/mcu/actuation_feedback`，由 bench 决定看哪个栈） | bench |
| `<SCRIPT_PATH>` | `start_pc_stack_clean.sh` 的相对路径 | bench |
| `<DURATION_*>` | 命令内自带的局部超时（不含 runner 加的 +15s） | bench |

替换方式推荐：

```bash
# 1. 在 session 顶层放 physical_session.env.template
# 2. 现场填好所有占位后：
envsubst < physical_session.env.template > physical_session.env
set -a; source physical_session.env; set +a
# 3. 之后物理 session 启动器内部 export RUN_ID / SESSION_DIR，再 export 这 13 条命令
#    （命令里"$RUN_ID" / "$SESSION_DIR"已被 envsubst 预替换为字面值）
```

**禁止**用 `${VAR}` 写在 `physical_session.env.template` 中后假定 runner 会展开——runner 用的是 `shlex.split`，不会再做 shell 展开。

## 5. 不在合同里、由 review checklist 处理的事

- 命令所需的工具是否在 bench PATH 上（`shutil.which` 会提前暴露）
- 命令所属账号的 NOPASSWD sudo 范围
- 命令的 process group / signal mask 是否合规
- artifact 内容里是否包含凭据、密钥、长 payload

详见 [hil_physical_session_on_site_review.md](hil_physical_session_on_site_review.md)。
