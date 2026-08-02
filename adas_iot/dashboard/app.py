#!/usr/bin/env python3
"""
ADAS IoT Dashboard — Flask + SocketIO 实时监控看板
=================================================
从 MQTT 接收 ADAS 车辆数据，通过 WebSocket 推送到浏览器。
支持无硬件的模拟模式，也支持连接真实 MQTT Broker。

Usage:
  python3 app.py              # 启动服务器（默认 http://localhost:5000）
  python3 app.py --sim-only   # 内置模拟器（无需外部 MQTT）
  python3 app.py --broker broker.emqx.io
"""

import argparse
import json
import os
import sys
import time
import threading
from datetime import datetime
from pathlib import Path

# Flask + WebSocket
from flask import Flask, render_template, send_from_directory
from flask_socketio import SocketIO, emit

# 将上级目录加入路径，以便导入 mqtt_bridge
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

app = Flask(__name__)
app.config['SECRET_KEY'] = 'adas-iot-dashboard-2026'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 全局状态缓存
state = {
    'vehicle': {
        'speed': 0.0,           # m/s
        'steering': 0.0,        # deg
        'brake': 0.0,           # 0-1
        'throttle': 0.0,        # 0-1
        'source': 'none',
        'timestamp': 0,
    },
    'can': {
        'frame_period_ms': 10.0,
        'crc_errors': 0,
        'seq_errors': 0,
        'timeout_count': 0,
        'bus_status': 'ACTIVE',
        'loop_load_pct': 6.2,
    },
    'safety': {
        'state': 'INIT',
        'fault_level': 0,
        'active_source': 'none',
        'aeb_active': False,
        'mrm_active': False,
    },
    'system': {
        'uptime_s': 0,
        'cpu_temp': 45.0,
        'ros2_nodes': 0,
        'mqtt_connected': False,
    },
    'alerts': [],
    'history': {
        'speed': [],
        'brake': [],
        'timestamps': [],
    },
}

MAX_HISTORY = 300  # 30s @ 10Hz
MAX_ALERTS = 50


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/state')
def get_state():
    """REST API 获取当前状态快照"""
    return json.dumps(state)


@socketio.on('connect')
def handle_connect():
    print(f'[Dashboard] Client connected')
    emit('state_update', state)


@socketio.on('request_state')
def handle_request_state():
    emit('state_update', state)


def update_and_broadcast():
    """广播最新状态到所有客户端"""
    socketio.emit('state_update', state)


def add_alert(level, source, message):
    """添加告警事件"""
    alert = {
        'time': datetime.now().strftime('%H:%M:%S.%f')[:-3],
        'level': level,
        'source': source,
        'message': message,
    }
    state['alerts'].insert(0, alert)
    if len(state['alerts']) > MAX_ALERTS:
        state['alerts'].pop()
    socketio.emit('alert', alert)


def update_history():
    """更新历史数据"""
    history = state['history']
    history['speed'].append(state['vehicle']['speed'])
    history['brake'].append(state['vehicle']['brake'])
    history['timestamps'].append(time.time())
    if len(history['speed']) > MAX_HISTORY:
        history['speed'].pop(0)
        history['brake'].pop(0)
        history['timestamps'].pop(0)


# ============ 模拟数据生成器 ============

class SimDataGenerator:
    """在没有真实 MQTT 数据时生成模拟车辆数据"""
    
    def __init__(self):
        self._running = False
        self._thread = None
        self._speed = 0.0
        self._steering = 0.0
        self._brake = 0.0
        self._throttle = 0.0
        self._safety_state_idx = 0
        self._safety_states = ['INIT', 'STANDBY', 'ACTIVE', 'ACTIVE', 'ACTIVE',
                               'DEGRADED', 'ACTIVE', 'ACTIVE']
        self._tick = 0
        self._fault_timer = 0
        
    def start(self, mqtt_bridge=None):
        self._running = True
        self._mqtt = mqtt_bridge
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print('[Sim] Simulator started')
        
    def stop(self):
        self._running = False
        
    def _run(self):
        """模拟一个完整的驾驶循环"""
        import math
        
        # 阶段定义：加速 → 巡航 → 转弯 → 跟车 → AEB → 恢复
        phase = 'accel'
        phase_time = 0
        
        while self._running:
            self._tick += 1
            dt = 0.1  # 10Hz
            phase_time += dt
            
            # 阶段切换
            if phase == 'accel' and self._speed >= 15.0:
                phase = 'cruise'
                phase_time = 0
                add_alert('info', 'sim', '巡航模式 — 速度 15.0 m/s')
            elif phase == 'cruise' and phase_time > 8:
                phase = 'turn'
                phase_time = 0
                add_alert('info', 'sim', '进入弯道 — 转向中')
            elif phase == 'turn' and phase_time > 6:
                phase = 'follow'
                phase_time = 0
                add_alert('info', 'sim', '跟车模式 — 前车减速')
            elif phase == 'follow' and phase_time > 5:
                phase = 'aeb_approach'
                phase_time = 0
                add_alert('warning', 'sim', 'AEB 触发 — 前车急刹！')
            elif phase == 'aeb_approach' and self._speed < 1.0:
                phase = 'aeb_stop'
                phase_time = 0
                add_alert('critical', 'sim', '🛑 AEB 紧急制动 — 车辆停止')
            elif phase == 'aeb_stop' and phase_time > 3:
                phase = 'recover'
                phase_time = 0
                add_alert('info', 'sim', '恢复行驶 — 重新起步')
            elif phase == 'recover' and self._speed > 8:
                phase = 'cruise'
                phase_time = 0
                add_alert('info', 'sim', '巡航模式')

            # 各阶段物理模拟
            if phase == 'accel':
                self._speed += 2.0 * dt
                self._steering *= 0.9
                self._throttle = min(0.4 + self._speed / 50.0, 0.8)
                self._brake *= 0.8
                state['safety']['state'] = 'ACTIVE'
                
            elif phase == 'cruise':
                self._speed += (15.0 - self._speed) * 0.1 * dt * 10
                self._steering *= 0.95
                self._throttle = 0.25
                self._brake *= 0.9
                state['safety']['state'] = 'ACTIVE'
                
            elif phase == 'turn':
                self._speed = max(10.0, self._speed - 1.0 * dt)
                self._steering = 12.0 * math.sin(phase_time * 1.5)
                self._throttle = 0.2
                self._brake = 0.05
                state['safety']['state'] = 'ACTIVE'
                
            elif phase == 'follow':
                self._speed = max(8.0, self._speed - 3.0 * dt)
                self._steering *= 0.9
                self._throttle = max(0.05, self._throttle - 0.05)
                self._brake = min(0.3, self._brake + 0.05)
                state['safety']['state'] = 'ACTIVE'
                
            elif phase == 'aeb_approach':
                self._speed = max(0.0, self._speed - 8.0 * dt)  # 8 m/s² 急刹
                self._steering *= 0.5
                self._throttle = 0.0
                self._brake = min(1.0, self._brake + 0.3)
                state['safety']['state'] = 'EMERGENCY_BRAKE'
                state['safety']['aeb_active'] = True
                
            elif phase == 'aeb_stop':
                self._speed = 0.0
                self._steering = 0.0
                self._throttle = 0.0
                self._brake = 1.0
                state['safety']['state'] = 'EMERGENCY_BRAKE'
                state['safety']['aeb_active'] = True
                
            elif phase == 'recover':
                self._speed += 1.5 * dt
                self._steering *= 0.9
                self._throttle = min(0.3, self._throttle + 0.02)
                self._brake = max(0.0, self._brake - 0.1)
                state['safety']['state'] = 'ACTIVE'
                state['safety']['aeb_active'] = False
                state['safety']['mrm_active'] = False

            # 随机故障注入
            self._fault_timer += dt
            if self._fault_timer > 25 and self._running:
                self._fault_timer = 0
                import random
                fault_type = random.choice(['crc_error', 'timeout', 'seq_stall'])
                if fault_type == 'crc_error':
                    state['can']['crc_errors'] += 1
                    add_alert('warning', 'CAN', f'CRC 校验错误 #{state["can"]["crc_errors"]}')
                elif fault_type == 'timeout':
                    state['can']['timeout_count'] += 1
                    state['safety']['state'] = 'DEGRADED'
                    add_alert('warning', 'CAN', f'通信超时 #{state["can"]["timeout_count"]} — 降级模式')
                elif fault_type == 'seq_stall':
                    state['can']['seq_errors'] += 1
                    add_alert('critical', 'CAN', f'Seq 停滞检测 #{state["can"]["seq_errors"]} — 有帧无控')
                    state['safety']['state'] = 'FAILSAFE'

            # 安全状态恢复
            if state['safety']['state'] in ('DEGRADED', 'FAILSAFE') and \
               state['safety']['aeb_active'] == False and \
               phase not in ('aeb_approach', 'aeb_stop'):
                # 3 秒后自动恢复
                if hasattr(self, '_recovery_timer'):
                    self._recovery_timer += dt
                    if self._recovery_timer > 3.0:
                        state['safety']['state'] = 'ACTIVE'
                        add_alert('info', 'system', '安全状态恢复 — ACTIVE')
                        self._recovery_timer = 0
                else:
                    self._recovery_timer = dt

            # 更新状态
            state['vehicle']['speed'] = round(self._speed, 2)
            state['vehicle']['steering'] = round(self._steering, 1)
            state['vehicle']['brake'] = round(self._brake, 3)
            state['vehicle']['throttle'] = round(self._throttle, 3)
            state['vehicle']['source'] = 'primary' if state['safety']['state'] == 'ACTIVE' else 'safety'
            state['vehicle']['timestamp'] = time.time()
            
            state['system']['uptime_s'] = round(self._tick * dt, 1)
            state['system']['mqtt_connected'] = True
            state['system']['cpu_temp'] = round(42.0 + 5.0 * math.sin(self._tick * 0.01), 1)
            state['system']['ros2_nodes'] = 13

            state['safety']['active_source'] = state['vehicle']['source']

            # CAN 诊断模拟
            if state['safety']['state'] == 'ACTIVE':
                state['can']['frame_period_ms'] = round(9.999 + 0.2 * math.sin(self._tick * 0.05), 3)
            elif state['safety']['state'] == 'EMERGENCY_BRAKE':
                state['can']['frame_period_ms'] = round(10.5 + random.random(), 3) if hasattr(self, '_r') else 10.5
            state['can']['loop_load_pct'] = round(6.2 + 2.0 * math.sin(self._tick * 0.02), 1)

            # 历史数据
            update_history()
            update_and_broadcast()

            # 通过 MQTT 发布（如果 bridge 可用）
            if self._mqtt:
                self._mqtt.publish_vehicle_state(
                    speed=state['vehicle']['speed'],
                    steering=state['vehicle']['steering'],
                    brake=state['vehicle']['brake'],
                    throttle=state['vehicle']['throttle'],
                    source=state['vehicle']['source'],
                    seq=self._tick,
                )

            time.sleep(dt)


# ============ MQTT 监听器 ============

class MqttListener:
    """监听 MQTT Broker 的数据并更新 Dashboard 状态"""
    
    def __init__(self, broker='broker.emqx.io', port=1883, topic_prefix='adas/v1'):
        self.broker = broker
        self.port = port
        self.topic_prefix = topic_prefix
        self._client = None
        self._running = False
        
    def start(self):
        try:
            import paho.mqtt.client as mqtt
            self._client = mqtt.Client(client_id=f'adas_dashboard_{int(time.time())}')
            self._client.on_connect = self._on_connect
            self._client.on_message = self._on_message
            self._client.connect_async(self.broker, self.port, 60)
            self._client.loop_start()
            self._running = True
            print(f'[MQTT] Listening on {self.broker}:{self.port}/{self.topic_prefix}/#')
        except Exception as e:
            print(f'[MQTT] Connection failed: {e}')
            
    def stop(self):
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f'{self.topic_prefix}/#')
            state['system']['mqtt_connected'] = True
            print(f'[MQTT] Connected, subscribed to {self.topic_prefix}/#')
        else:
            print(f'[MQTT] Connection failed (rc={rc})')
            
    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
            topic = msg.topic
            
            if 'vehicle/state' in topic or 'can/feedback' in topic:
                if 'speed' in data:
                    state['vehicle']['speed'] = data['speed']
                if 'steering' in data:
                    state['vehicle']['steering'] = data.get('steering', 0)
                if 'brake' in data:
                    state['vehicle']['brake'] = data['brake']
                if 'throttle' in data:
                    state['vehicle']['throttle'] = data.get('throttle', 0)
                if 'source' in data:
                    state['vehicle']['source'] = data['source']
                state['vehicle']['timestamp'] = time.time()
                    
            elif 'safety' in topic:
                if 'state' in data:
                    state['safety']['state'] = data['state']
                if 'fault_level' in data:
                    state['safety']['fault_level'] = data['fault_level']
                if 'active_source' in data:
                    state['safety']['active_source'] = data['active_source']
                if 'aeb_active' in data:
                    state['safety']['aeb_active'] = data['aeb_active']
                    
            elif 'diagnostics' in topic:
                if 'crc_errors' in data:
                    state['can']['crc_errors'] = data['crc_errors']
                if 'frame_period_ms' in data:
                    state['can']['frame_period_ms'] = data['frame_period_ms']
                if 'loop_load_pct' in data:
                    state['can']['loop_load_pct'] = data['loop_load_pct']
                    
            elif 'alert' in topic:
                add_alert(
                    data.get('level', 'info'),
                    data.get('source', 'mqtt'),
                    data.get('message', 'Unknown alert')
                )
                
            update_history()
            update_and_broadcast()
            
        except json.JSONDecodeError:
            pass
        except Exception as e:
            print(f'[MQTT] Parse error: {e}')


# ============ 主启动 ============

def main():
    parser = argparse.ArgumentParser(description='ADAS IoT Dashboard')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=5000, help='监听端口')
    parser.add_argument('--broker', default=None, help='MQTT Broker 地址')
    parser.add_argument('--sim-only', action='store_true', help='仅使用模拟器（不连 MQTT）')
    parser.add_argument('--debug', action='store_true', help='调试模式')
    args = parser.parse_args()

    print('=' * 60)
    print('  ADAS IoT Dashboard — 实时监控平台')
    print('  ==================================')
    print(f'  模式: {"模拟器" if args.sim_only else "MQTT + 模拟器"}')
    if args.broker:
        print(f'  MQTT: {args.broker}')
    print(f'  地址: http://{args.host}:{args.port}')
    print('=' * 60)
    
    # 启动模拟器（默认开启）
    sim = SimDataGenerator()
    sim.start()
    add_alert('info', 'system', '🟢 ADAS IoT Dashboard 已启动')
    add_alert('info', 'system', '📡 模拟数据生成中（10Hz）')
    
    # 连接 MQTT（如果指定了 broker）
    mqtt_listener = None
    if args.broker and not args.sim_only:
        mqtt_listener = MqttListener(broker=args.broker)
        mqtt_listener.start()
    elif args.broker:
        print('[Dashboard] --sim-only 模式下跳过 MQTT 连接')

    # 启动 Flask-SocketIO 服务
    try:
        socketio.run(app, host=args.host, port=args.port,
                     debug=args.debug, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
        if mqtt_listener:
            mqtt_listener.stop()
        print('[Dashboard] Shutdown complete')


if __name__ == '__main__':
    main()
