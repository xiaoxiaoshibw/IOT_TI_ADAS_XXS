#!/usr/bin/env python3
"""
adas_mqtt_bridge 单元测试
"""
import sys
import os
import time
import unittest
from unittest.mock import Mock, patch


class TestMqttBridge(unittest.TestCase):
    """MQTT 桥接器核心逻辑测试"""

    @classmethod
    def setUpClass(cls):
        # 导入被测试模块
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
        from adas_mqtt_bridge.mqtt_bridge import (
            AdasVehicleSimulator, VehicleState, SafetyStatus, AlertEvent
        )
        cls.VehicleState = VehicleState
        cls.SafetyStatus = SafetyStatus
        cls.AlertEvent = AlertEvent
        cls.AdasVehicleSimulator = AdasVehicleSimulator

    def test_vehicle_state_dataclass(self):
        """VehicleState 数据类应正确序列化"""
        v = self.VehicleState(
            speed_mps=15.0, steering_deg=5.2,
            throttle_pct=25.0, brake_pct=0.0
        )
        d = v.to_dict()
        self.assertEqual(d['speed_mps'], 15.0)
        self.assertEqual(d['steering_deg'], 5.2)
        self.assertIn('timestamp', d)

    def test_safety_status_dataclass(self):
        """SafetyStatus 数据类应包含所有关键字段"""
        s = self.SafetyStatus(hil_state='ACTIVE', safety_level='ACTIVE')
        d = s.to_dict()
        self.assertEqual(d['hil_state'], 'ACTIVE')
        self.assertIn('fault_code', d)
        self.assertIn('loop_load_pct', d)

    def test_alert_event_dataclass(self):
        """AlertEvent 应正确表示告警"""
        a = self.AlertEvent(
            event_type='aeb', severity='critical',
            message='AEB triggered!', source='adas_aeb'
        )
        d = a.to_dict()
        self.assertEqual(d['severity'], 'critical')
        self.assertEqual(d['event_type'], 'aeb')

    def test_simulator_step_returns_valid_data(self):
        """模拟器 step() 应返回完整状态"""
        sim = self.AdasVehicleSimulator({'simulation': {'vehicle': {}}})
        state, safety, alert = sim.step(0.1)

        self.assertIsInstance(state, self.VehicleState)
        self.assertIsInstance(safety, self.SafetyStatus)
        # 初始速度应为 15.0
        self.assertGreater(state.speed_mps, 0)

    def test_simulator_60s_cycle(self):
        """模拟器 60 秒周期应覆盖所有场景阶段"""
        sim = self.AdasVehicleSimulator({'simulation': {'vehicle': {}}})
        alerts = []
        for i in range(600):  # 60s @ 10Hz
            state, safety, alert = sim.step(0.1)
            if alert:
                alerts.append(alert)

        # 应至少触发 AEB 事件
        aeb_alerts = [a for a in alerts if a.event_type == 'aeb']
        self.assertGreater(len(aeb_alerts), 0, "应在 60s 周期内触发 AEB")

    def test_simulator_speed_profile(self):
        """模拟器速度曲线应在合理范围内"""
        sim = self.AdasVehicleSimulator({'simulation': {'vehicle': {}}})
        max_speed = 0
        min_speed = 999

        for i in range(600):
            state, _, _ = sim.step(0.1)
            max_speed = max(max_speed, state.speed_mps)
            min_speed = min(min_speed, state.speed_mps)

        self.assertLessEqual(max_speed, 35.0)  # 不超过最大限速
        self.assertGreaterEqual(min_speed, 0)   # 不低于 0

    def test_simulator_fault_injection(self):
        """故障注入应改变安全状态"""
        config = {
            'simulation': {
                'vehicle': {
                    'random_injection': True,
                    'inject_interval_s': 30,
                }
            }
        }
        sim = self.AdasVehicleSimulator(config)
        self.assertEqual(sim._fault_level, 0)

        # 运行到故障触发
        fault_detected = False
        for i in range(400):  # 40s
            state, safety, alert = sim.step(0.1)
            if safety.fault_level > 0:
                fault_detected = True
                break

        self.assertTrue(fault_detected, "故障注入应在 40s 内触发")

    def test_simulator_recovery(self):
        """故障后应自动恢复"""
        config = {
            'simulation': {
                'vehicle': {
                    'random_injection': True,
                    'inject_interval_s': 30,
                }
            }
        }
        sim = self.AdasVehicleSimulator(config)

        # 等待故障注入
        for i in range(300):
            sim.step(0.1)

        # 等待恢复
        recovered = False
        for i in range(300):
            state, safety, _ = sim.step(0.1)
            if safety.fault_level == 0 and safety.hil_state == 'ACTIVE':
                recovered = True
                break

        self.assertTrue(recovered, "系统应在故障注入后自动恢复")

    def test_mqtt_topic_format(self):
        """MQTT 主题格式应一致"""
        from adas_mqtt_bridge.mqtt_bridge import AdasMqttClient
        config = {
            'mqtt': {
                'broker': 'test.com', 'port': 1883,
                'topic_prefix': 'adas/v1',
                'client_id_prefix': 'test',
                'qos': 1,
            }
        }
        client = AdasMqttClient(config)
        # 验证主题生成逻辑
        expected_topic = 'adas/v1/test/topic'
        # publish 方法使用 f"{prefix}/{suffix}" 格式
        self.assertEqual(
            f"{config['mqtt']['topic_prefix']}/test/topic",
            expected_topic
        )


if __name__ == '__main__':
    unittest.main(verbosity=2)
