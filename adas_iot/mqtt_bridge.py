"""
ADAS MQTT IoT 桥接器
====================
功能：
1. 模拟模式下：主动生成 ADAS 车辆状态数据，定时发布到 MQTT
2. ROS2 模式下：订阅 /adas/* 话题，桥接到 MQTT
3. Dashboard 可通过 MQTT WebSocket 实时查看

竞赛场景：
- 端侧（Orin/MCU）→ MQTT Broker → 云 Dashboard
- 评委可在任意浏览器打开 Dashboard 看到实时车辆状态
"""

import json
import logging
import random
import math
import time
import threading
from dataclasses import dataclass, field, asdict
from typing import Optional

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

logger = logging.getLogger("adas_mqtt")


# ============================================================================
# 数据模型
# ============================================================================

@dataclass
class VehicleState:
    """车辆状态（对应 CAN 0x201 + 0x202 融合）"""
    timestamp: float = 0.0
    speed_mps: float = 0.0          # 车速 m/s
    speed_kmh: float = 0.0          # 车速 km/h
    steering_deg: float = 0.0       # 转向角 deg（左正）
    throttle_pct: float = 0.0       # 油门 %
    brake_pct: float = 0.0          # 制动 %
    target_accel_ms2: float = 0.0   # 目标加速度 m/s²
    lateral_accel_ms2: float = 0.0  # 横向加速度
    yaw_rate: float = 0.0           # 横摆角速度 rad/s

    def to_dict(self):
        return asdict(self)


@dataclass
class SafetyStatus:
    """安全状态（对应 MCU 状态机）"""
    timestamp: float = 0.0
    hil_state: str = "INIT"          # MCU 会话状态
    active_source: str = "none"      # 当前控制源
    safety_level: str = "ACTIVE"     # 安全等级
    fault_level: int = 0             # 故障等级 0-3
    fault_code: int = 0              # 故障码
    loop_load_pct: float = 0.0       # CPU 负载
    can_status: str = "OK"           # CAN 总线状态
    primary_timeout_cnt: int = 0     # 主源超时计数
    backup_timeout_cnt: int = 0      # 备源超时计数
    crc_error_cnt: int = 0           # CRC 错误计数

    def to_dict(self):
        return asdict(self)


@dataclass
class AlertEvent:
    """告警事件"""
    timestamp: float = 0.0
    event_type: str = ""             # aeb / mrm / fault / takeover / warning
    severity: str = "info"           # info / warning / critical
    message: str = ""
    source: str = ""

    def to_dict(self):
        return asdict(self)


# ============================================================================
# ADAS 车辆模拟器（无硬件时的数据源）
# ============================================================================

class AdasVehicleSimulator:
    """模拟 ADAS 车辆行为，生成逼真的状态数据"""

    def __init__(self, config: dict):
        self.cfg = config.get("simulation", {}).get("vehicle", {})
        self._speed = self.cfg.get("initial_speed", 15.0)
        self._steering = 0.0
        self._brake = 0.0
        self._throttle = 25.0
        self._target_accel = 0.0
        self._lane_offset = 0.0
        self._yaw_rate = 0.0
        self._lateral_accel = 0.0

        # 驾驶场景循环
        self._scenario_time = 0.0
        self._scenario_phase = 0  # 0:cruise 1:turn 2:acc 3:brake

        # 安全状态模拟
        self._hil_state = "ACTIVE"
        self._safety_level = "ACTIVE"
        self._fault_level = 0
        self._fault_inject_time = 0
        self._random_faults = self.cfg.get("random_injection", False)
        self._inject_interval = config.get("simulation", {}).get("inject_interval_s", 30)

        # CAN 诊断
        self._can_seq = 0
        self._primary_timeout = 0
        self._backup_timeout = 0
        self._crc_errors = 0

    def step(self, dt: float = 0.1) -> tuple[VehicleState, SafetyStatus, Optional[AlertEvent]]:
        """前进 dt 秒，返回 (state, safety, alert)"""
        self._scenario_time += dt
        alert = None

        # ---- 驾驶场景循环（~60s 一个完整周期） ----
        cycle_time = self._scenario_time % 60.0

        if cycle_time < 20.0:
            # Phase 0: 匀速巡航
            self._speed = 15.0
            self._steering *= 0.95  # 回正
            self._brake = 0.0
            self._throttle = 20.0
            self._target_accel = 0.0
            self._yaw_rate *= 0.9

        elif cycle_time < 25.0:
            # Phase 1: 弯道（左转）
            t = (cycle_time - 20.0) / 5.0
            self._speed = 12.0 - t * 2.0
            self._steering = -15.0 * math.sin(t * math.pi)
            self._brake = 5.0
            self._throttle = 18.0
            self._target_accel = -0.5
            self._yaw_rate = -0.08 * math.sin(t * math.pi)
            self._lateral_accel = 1.2 * math.sin(t * math.pi)

        elif cycle_time < 35.0:
            # Phase 2: 跟车巡航
            self._speed = 12.0
            self._steering *= 0.95
            self._brake = 0.0
            self._throttle = 22.0
            self._target_accel = 0.0

        elif cycle_time < 40.0:
            # Phase 3: 前车急刹 → AEB
            t = (cycle_time - 35.0) / 5.0
            decel = 6.0 * t
            self._speed = max(2.0, 12.0 - 6.0 * t)
            self._steering *= 0.98
            self._brake = min(100, 30.0 + decel * 10)
            self._throttle = 0.0
            self._target_accel = -decel

            if t > 0.3 and abs(t - 0.3) < dt * 2:
                alert = AlertEvent(
                    event_type="aeb",
                    severity="critical",
                    message=f"AEB 触发！前车急刹 {decel:.1f} m/s²",
                    source="adas_aeb"
                )

        elif cycle_time < 45.0:
            # Phase 4: 停车
            self._speed = max(0, self._speed - 4.0 * dt)
            self._brake = 60.0
            self._throttle = 0.0
            if self._speed < 0.5 and self._brake > 50:
                self._hil_state = "ACTIVE"

        elif cycle_time < 50.0:
            # Phase 5: 起步
            self._speed = min(10.0, self._speed + 3.0 * dt)
            self._brake = max(0, self._brake - 5.0)
            self._throttle = 30.0
            self._hil_state = "ACTIVE"

        else:
            # Phase 6: 加速
            self._speed = min(30.0, self._speed + 2.0 * dt)
            self._brake = 0.0
            self._throttle = min(50, self._throttle + 1.0)
            self._hil_state = "ACTIVE"

        # ---- 故障注入（演示用） ----
        if self._random_faults and self._fault_level == 0:
            if self._scenario_time - self._fault_inject_time > self._inject_interval:
                self._fault_level = 2
                self._safety_level = "DEGRADED"
                self._fault_inject_time = self._scenario_time
                self._hil_state = "RECOV"
                alert = AlertEvent(
                    event_type="fault",
                    severity="warning",
                    message="主源通信超时！切换至备用源",
                    source="adas_safety_monitor"
                )

        # 故障恢复
        if self._fault_level > 0 and self._scenario_time - self._fault_inject_time > 5.0:
            self._fault_level = 0
            self._safety_level = "ACTIVE"
            self._hil_state = "ACTIVE"

        # ---- CAN 诊断模拟 ----
        self._can_seq = (self._can_seq + 1) % 256

        # 打包状态
        state = VehicleState(
            timestamp=time.time(),
            speed_mps=round(self._speed, 2),
            speed_kmh=round(self._speed * 3.6, 1),
            steering_deg=round(self._steering, 1),
            throttle_pct=round(self._throttle, 1),
            brake_pct=round(self._brake, 1),
            target_accel_ms2=round(self._target_accel, 3),
            lateral_accel_ms2=round(self._lateral_accel, 3),
            yaw_rate=round(self._yaw_rate, 4),
        )

        safety = SafetyStatus(
            timestamp=time.time(),
            hil_state=self._hil_state,
            active_source="primary" if self._fault_level == 0 else "backup",
            safety_level=self._safety_level,
            fault_level=self._fault_level,
            fault_code=0x12 if self._fault_level > 0 else 0,
            loop_load_pct=round(random.uniform(4.0, 12.0), 1),
            can_status="OK" if self._fault_level < 2 else "WARN",
            primary_timeout_cnt=self._primary_timeout,
            backup_timeout_cnt=self._backup_timeout,
            crc_error_cnt=self._crc_errors,
        )

        return state, safety, alert


# ============================================================================
# MQTT 客户端封装
# ============================================================================

class AdasMqttClient:
    """MQTT 客户端，管理连接和发布"""

    def __init__(self, config: dict):
        self.cfg = config["mqtt"]
        self._connected = False
        self._client = None

        if mqtt is None:
            logger.warning("paho-mqtt not installed — MQTT disabled")
            return

        client_id = f"{self.cfg['client_id_prefix']}{random.randint(1000,9999)}"
        self._client = mqtt.Client(
            client_id=client_id,
            protocol=mqtt.MQTTv311
        )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            logger.info(f"MQTT connected to {self.cfg['broker']}:{self.cfg['port']}")
        else:
            logger.error(f"MQTT connection failed: rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        if rc != 0:
            logger.warning(f"MQTT unexpected disconnect (rc={rc}), will reconnect")

    def connect(self):
        if self._client is None:
            return False
        try:
            self._client.connect_async(self.cfg["broker"], self.cfg["port"])
            self._client.loop_start()
            return True
        except Exception as e:
            logger.error(f"MQTT connect error: {e}")
            return False

    def disconnect(self):
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()

    @property
    def connected(self) -> bool:
        return self._connected

    def publish(self, topic_suffix: str, data: dict):
        """发布数据到 adas/v1/<suffix>"""
        if self._client is None or not self._connected:
            return False
        topic = f"{self.cfg['topic_prefix']}/{topic_suffix}"
        payload = json.dumps(data, ensure_ascii=False)
        try:
            info = self._client.publish(topic, payload, qos=self.cfg.get("qos", 1))
            return info.is_published() or info.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception:
            return False


# ============================================================================
# 主桥接器
# ============================================================================

class AdasIoTBridge:
    """ADAS IoT 桥接器主类"""

    def __init__(self, config_path: str = "config.yaml"):
        self._load_config(config_path)
        self._mqtt = AdasMqttClient(self._config)
        self._simulator = AdasVehicleSimulator(self._config) if self._config.get("simulation", {}).get("enabled") else None
        self._running = False
        self._thread = None

    def _load_config(self, path: str):
        import yaml
        try:
            with open(path) as f:
                self._config = yaml.safe_load(f)
        except FileNotFoundError:
            logger.warning(f"Config {path} not found, using defaults")
            self._config = {"mqtt": {"broker": "broker.emqx.io", "port": 1883,
                                     "topic_prefix": "adas/v1"},
                            "simulation": {"enabled": True}}

    def start(self):
        """启动桥接器"""
        self._mqtt.connect()
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("AdasIoTBridge started")
        return self

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        self._mqtt.disconnect()
        logger.info("AdasIoTBridge stopped")

    def _run_loop(self):
        """主循环：按配置频率发布各话题"""
        rates = self._config.get("publish_rate", {})
        last_pub = {}

        while self._running:
            now = time.time()
            dt = 0.1

            if self._simulator:
                state, safety, alert = self._simulator.step(dt)

                # 按频率发布
                self._publish_if_due("vehicle_state", state.to_dict(),
                                     rates.get("vehicle_state", 10), last_pub, now)
                self._publish_if_due("can_feedback", safety.to_dict(),
                                     rates.get("can_feedback", 10), last_pub, now)
                self._publish_if_due("safety_status", safety.to_dict(),
                                     rates.get("safety_status", 5), last_pub, now)
                self._publish_if_due("diagnostics", {
                    "timestamp": now,
                    "loop_load_pct": safety.loop_load_pct,
                    "can_status": safety.can_status,
                    "crc_errors": safety.crc_error_cnt,
                    "primary_timeouts": safety.primary_timeout_cnt,
                    "backup_timeouts": safety.backup_timeout_cnt,
                    "hil_state": safety.hil_state,
                }, rates.get("diagnostics", 2), last_pub, now)

                if alert:
                    self._publish_if_due("alerts", alert.to_dict(),
                                         rates.get("alerts", 1), last_pub, now)

            time.sleep(dt)

    def _publish_if_due(self, topic: str, data: dict, rate_hz: int,
                        last_pub: dict, now: float):
        """按频率控制发布"""
        min_interval = 1.0 / max(rate_hz, 0.1)
        last = last_pub.get(topic, 0)
        if now - last >= min_interval:
            self._mqtt.publish(topic, data)
            last_pub[topic] = now

    def publish_raw(self, topic_suffix: str, data: dict):
        """供 ROS2 节点调用的外部发布接口"""
        self._mqtt.publish(topic_suffix, data)


# ============================================================================
# 命令行入口
# ============================================================================

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(name)s %(levelname)s: %(message)s"
    )

    import argparse
    parser = argparse.ArgumentParser(description="ADAS IoT MQTT Bridge")
    parser.add_argument("--config", default="config.yaml", help="配置文件路径")
    parser.add_argument("--standalone", action="store_true", default=True,
                        help="独立模拟模式（无ROS2）")
    args = parser.parse_args()

    bridge = AdasIoTBridge(args.config)
    bridge.start()

    print("=" * 60)
    print("  ADAS IoT MQTT Bridge 已启动")
    print(f"  MQTT Broker: {bridge._config['mqtt']['broker']}:{bridge._config['mqtt']['port']}")
    print(f"  Topic前缀:   {bridge._config['mqtt']['topic_prefix']}")
    print("  模拟数据:    已启用 (60s 驾驶场景循环)")
    print("=" * 60)
    print("  按 Ctrl+C 停止\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n正在停止...")
        bridge.stop()
        print("已停止")
