#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA 桥场景库（IOT_TI HIL 闭环）。

字段说明：
  duration      仿真时长 (s)，0 = 不限
  spawn_index   自车出生点序号（Town04 高速环路）
  lead          None = 无前车；否则：
    gap0          初始车距 (m)
    profile       [(t, 目标速度 m/s)] 分段阶梯
    hard_brake    (t0, t1) 区间内前车 brake=1.0（None = 无）
  pedestrian    None = 无行人；否则：
    ahead_m           行人生成点在自车前方距离 (m)
    start_lateral_m   相对车道中心线横向起点（右手系左正；负 = 右侧路边）
    trigger_ego_gap_m 自车逼近到该距离触发横穿
    speed_mps         横穿速度
  notes         运行前打印的讲解要点
"""

SCENARIOS = {
    'lka': {
        'name': 'LKA 车道保持',
        'duration': 90.0,
        'spawn_index': 30,
        'lead': None,
        'pedestrian': None,
        'notes': [
            '无前车，自车沿高速车道巡航',
            '观察点：弯道曲率前馈 + 横向误差修正，lateral_offset 有界',
        ],
    },
    'acc': {
        'name': 'ACC 自适应巡航',
        'duration': 90.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 50.0,
            'profile': [(0.0, 8.0), (20.0, 5.0), (40.0, 10.0), (60.0, 7.0)],
            'hard_brake': None,
        },
        'pedestrian': None,
        'notes': [
            '前车 8→5→10→7 m/s 变速，自车按时距跟随',
            '观察点：跟车距离随速度自适应，无追尾、无振荡',
        ],
    },
    'aeb': {
        'name': 'AEB 前车急停',
        'duration': 60.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 60.0,
            'profile': [(0.0, 9.0), (15.0, 0.0), (30.0, 8.0)],
            'hard_brake': (15.0, 30.0),
        },
        'pedestrian': None,
        'notes': [
            't=15s 前车全力急刹，SoC 端 TTC 触发 AEB',
            '观察点：aeb 节点先 WARNING 后 BRAKING，gate 放行紧急制动',
        ],
    },
    'aeb_pedestrian': {
        'name': 'AEB 横穿行人',
        'duration': 60.0,
        'spawn_index': 30,
        'lead': None,
        'pedestrian': {
            'ahead_m': 90.0,
            'start_lateral_m': -6.0,
            'trigger_ego_gap_m': 40.0,
            'speed_mps': 1.5,
        },
        'notes': [
            '自车逼近 40m 时行人从右侧横穿',
            '观察点：object_tracker 上报 CLASS_PEDESTRIAN，AEB 紧急制动',
        ],
    },
    'acc_stop_and_go': {
        'name': 'ACC 停车再走',
        'duration': 90.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 35.0,
            'profile': [(0.0, 8.0), (20.0, 0.0), (45.0, 8.0)],
            'hard_brake': (20.0, 45.0),
        },
        'pedestrian': None,
        'notes': [
            '前车 8→0→8 m/s，配合 hard_brake 区间强制停车',
            '观察点：自车起停跟随时距、起步响应、起步无冲撞',
        ],
    },
    'acc_slow_truck': {
        'name': 'ACC 慢速卡车',
        'duration': 90.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 25.0,
            'profile': [(0.0, 3.0)],
            'hard_brake': None,
        },
        'pedestrian': None,
        'notes': [
            '前车以 3 m/s（约 11 km/h）匀速',
            '观察点：长跟时距、无误加速、目标车速收敛到 11 km/h',
        ],
    },
    'aeb_stationary': {
        'name': 'AEB 静止障碍物',
        'duration': 30.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 40.0,
            'profile': [(0.0, 0.0)],
            'hard_brake': None,
        },
        'pedestrian': None,
        'notes': [
            '前车起步即静止，模拟丢车 / 抛锚 / 大货静止',
            '观察点：AEB 对静态障碍的 TTC 触发（CARLA 0.9.16 静态目标有' \
            ' tracker 检出概率，必要时叠加 0x301 FREEZE_SEQ 验证 MCU 旁路）',
        ],
    },
    'free': {
        'name': '自由巡航（无脚本演员）',
        'duration': 0.0,
        'spawn_index': 30,
        'lead': None,
        'pedestrian': None,
        'notes': ['长跑/调试用，Ctrl-C 结束'],
    },
}

ORDER = ['lka', 'acc', 'acc_stop_and_go', 'acc_slow_truck',
         'aeb', 'aeb_stationary', 'aeb_pedestrian', 'free']
