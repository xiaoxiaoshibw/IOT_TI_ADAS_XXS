#!/usr/bin/env python3
"""
微信小程序订阅用户存储 — 一个纯文件的小 openid 表。
=================================================
小程序前端 wx.login() 拿到 code → 打给 Dashboard 的 /api/wechat/bind →
后端用 code 换 openid → 存这里。notifier.py 推送订阅消息时读这张表，
给所有已绑定的 openid 逐个发。

用不到数据库：openid 列表量级很小（演示/单车场景），JSON 文件 + 进程内锁
足够，换 SQLite 纯属过度设计。
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import List


class WechatSubscriberStore:
    def __init__(self, path: str | Path = None):
        self.path = Path(path) if path else Path(__file__).resolve().parent / "wechat_subscribers.json"
        self._lock = threading.Lock()

    def _read(self) -> List[str]:
        try:
            with open(self.path, encoding="utf-8") as f:
                data = json.load(f)
                return list(dict.fromkeys(data)) if isinstance(data, list) else []
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def add(self, openid: str) -> bool:
        """新增一个 openid（已存在则忽略）。返回是否新增成功。"""
        if not openid:
            return False
        with self._lock:
            openids = self._read()
            if openid in openids:
                return False
            openids.append(openid)
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(openids, f, ensure_ascii=False, indent=2)
            return True

    def remove(self, openid: str) -> bool:
        with self._lock:
            openids = self._read()
            if openid not in openids:
                return False
            openids.remove(openid)
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(openids, f, ensure_ascii=False, indent=2)
            return True

    def list_all(self) -> List[str]:
        with self._lock:
            return self._read()
