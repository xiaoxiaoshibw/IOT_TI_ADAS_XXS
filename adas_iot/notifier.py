#!/usr/bin/env python3
"""
ADAS IoT 告警实际推送 — Webhook / 邮件 / 微信小程序订阅消息
============================================================
Dashboard 里的告警日志只有打开网页才能看到；这个模块把 CRITICAL 级告警
真正推出去（Slack/企业微信/钉钉风格 Webhook、SMTP 邮件、或微信小程序订阅
消息），不用守着网页。

设计要点：
  · 默认关闭（未配置任何一个渠道时 AlertNotifier.enabled=False，
    notify() 直接是 no-op），不影响没有配置这个功能的部署。
  · 网络 I/O 全部丢进后台线程 fire-and-forget，绝不阻塞 add_alert() 的
    调用方（10Hz 模拟循环 / MQTT 回调线程都不能被一次慢 HTTP 请求卡住）。
  · 限流：同一 (level, source) 组合在冷却窗口内只推一次，防止故障风暴
    刷爆 Slack/邮箱/小程序（例如 CRC 错误连续跳动）。
  · 只依赖标准库 + requests（项目已有依赖），SMTP 用 smtplib（标准库）。

微信小程序订阅消息说明：
  这是"一次性订阅消息"机制（不是旧的模板消息，那个已下线）——小程序前端
  调 wx.requestSubscribeMessage() 拿用户授权、wx.login() 拿 code，code
  经 dashboard 的 /api/wechat/bind 换成 openid 存进 wechat_store.py；
  本模块推送时读全部已绑定 openid 逐个发。
  订阅消息模板字段名（thing1/phrase2/time3...）由你在小程序后台选完模板
  属性后自动生成，因人而异，所以 field_keys 是可配置的，不能硬编码。
"""

from __future__ import annotations

import smtplib
import threading
import time
from email.mime.text import MIMEText
from typing import Optional

import requests

from wechat_store import WechatSubscriberStore


class AlertNotifier:
    def __init__(self, config: Optional[dict] = None):
        config = config or {}
        webhook_cfg = config.get("webhook", {}) or {}
        email_cfg = config.get("email", {}) or {}

        self.webhook_url: str = webhook_cfg.get("url", "") or ""
        self.webhook_format: str = webhook_cfg.get("format", "slack")  # slack | raw

        self.email_enabled: bool = bool(email_cfg.get("enabled", False))
        self.smtp_host: str = email_cfg.get("smtp_host", "")
        self.smtp_port: int = int(email_cfg.get("smtp_port", 587))
        self.smtp_user: str = email_cfg.get("smtp_user", "")
        self.smtp_password: str = email_cfg.get("smtp_password", "")
        self.email_from: str = email_cfg.get("from", self.smtp_user)
        self.email_to: str = email_cfg.get("to", "")

        wechat_cfg = config.get("wechat", {}) or {}
        self.wechat_appid: str = wechat_cfg.get("appid", "") or ""
        self.wechat_secret: str = wechat_cfg.get("secret", "") or ""
        self.wechat_template_id: str = wechat_cfg.get("template_id", "") or ""
        self.wechat_page: str = wechat_cfg.get("page", "")
        # 模板字段名由小程序后台选完属性后自动生成（thing1/phrase2/time3...），
        # 因人而异，这里给一组常见默认值，实际以你模板详情页显示的为准。
        self.wechat_field_keys: dict = wechat_cfg.get(
            "field_keys",
            {"level": "phrase1", "message": "thing2", "time": "time3"},
        )
        self._wechat_store = WechatSubscriberStore()
        self._wechat_token: str = ""
        self._wechat_token_expiry: float = 0.0
        self._wechat_lock = threading.Lock()

        # 只有 CRITICAL 才推送（可通过 config 覆盖，比如竞赛演示想连 WARNING 也推）。
        self.min_level: str = (config.get("min_level") or "critical").lower()
        self.cooldown_s: float = float(config.get("cooldown_s", 60.0))

        self._last_sent: dict[tuple[str, str], float] = {}
        self._lock = threading.Lock()

    @property
    def wechat_enabled(self) -> bool:
        return bool(self.wechat_appid and self.wechat_secret and self.wechat_template_id)

    @property
    def enabled(self) -> bool:
        return bool(self.webhook_url) or self.email_enabled or self.wechat_enabled

    _LEVEL_RANK = {"info": 0, "warning": 1, "critical": 2}

    def _should_send(self, level: str, source: str) -> bool:
        if self._LEVEL_RANK.get(level.lower(), 0) < self._LEVEL_RANK.get(self.min_level, 2):
            return False
        key = (level.lower(), source)
        now = time.monotonic()
        with self._lock:
            last = self._last_sent.get(key, 0.0)
            if now - last < self.cooldown_s:
                return False
            self._last_sent[key] = now
        return True

    def notify(self, level: str, source: str, message: str, vehicle_state: Optional[dict] = None):
        """非阻塞：分级/限流判断在调用线程做（很快），实际网络 I/O 丢后台线程。"""
        if not self.enabled:
            return
        if not self._should_send(level, source):
            return
        threading.Thread(
            target=self._send_all,
            args=(level, source, message, vehicle_state),
            daemon=True,
        ).start()

    def _send_all(self, level: str, source: str, message: str, vehicle_state: Optional[dict]):
        text = self._format_text(level, source, message, vehicle_state)
        if self.webhook_url:
            try:
                self._send_webhook(level, text)
            except Exception as exc:  # noqa: BLE001 - 推送失败不影响主流程
                print(f"[Notifier] Webhook 推送失败: {exc}")
        if self.email_enabled:
            try:
                self._send_email(level, source, text)
            except Exception as exc:  # noqa: BLE001
                print(f"[Notifier] 邮件推送失败: {exc}")
        if self.wechat_enabled:
            try:
                self._send_wechat(level, source, message)
            except Exception as exc:  # noqa: BLE001
                print(f"[Notifier] 微信小程序推送失败: {exc}")

    @staticmethod
    def _format_text(level: str, source: str, message: str, vehicle_state: Optional[dict]) -> str:
        prefix = {"critical": "🔴 CRITICAL", "warning": "🟡 WARNING", "info": "ℹ️ INFO"}.get(
            level.lower(), level.upper()
        )
        line = f"[ADAS IoT] {prefix} · {source} · {message}"
        if vehicle_state:
            line += (
                f"\n速度={vehicle_state.get('speed', '?')}m/s "
                f"安全状态={vehicle_state.get('safety_state', '?')}"
            )
        return line

    def _send_webhook(self, level: str, text: str):
        if self.webhook_format == "slack":
            payload = {"text": text}
        else:
            payload = {"level": level, "message": text, "ts": time.time()}
        resp = requests.post(self.webhook_url, json=payload, timeout=5.0)
        resp.raise_for_status()

    def _send_email(self, level: str, source: str, text: str):
        if not (self.smtp_host and self.email_to):
            return
        msg = MIMEText(text, "plain", "utf-8")
        msg["Subject"] = f"[ADAS IoT] {level.upper()} 告警 — {source}"
        msg["From"] = self.email_from
        msg["To"] = self.email_to
        with smtplib.SMTP(self.smtp_host, self.smtp_port, timeout=10.0) as smtp:
            smtp.starttls()
            if self.smtp_user:
                smtp.login(self.smtp_user, self.smtp_password)
            smtp.sendmail(self.email_from, [self.email_to], msg.as_string())

    def _get_wechat_access_token(self) -> str:
        """access_token 有效期 7200s，官方要求缓存复用、不能每次请求都换新的。
        提前 5 分钟过期，避免临界点用到刚好失效的 token。
        """
        with self._wechat_lock:
            if self._wechat_token and time.monotonic() < self._wechat_token_expiry:
                return self._wechat_token
            resp = requests.get(
                "https://api.weixin.qq.com/cgi-bin/token",
                params={
                    "grant_type": "client_credential",
                    "appid": self.wechat_appid,
                    "secret": self.wechat_secret,
                },
                timeout=5.0,
            )
            resp.raise_for_status()
            data = resp.json()
            if "access_token" not in data:
                raise RuntimeError(f"获取 access_token 失败: {data}")
            self._wechat_token = data["access_token"]
            self._wechat_token_expiry = time.monotonic() + data.get("expires_in", 7200) - 300
            return self._wechat_token

    def _send_wechat(self, level: str, source: str, message: str):
        openids = self._wechat_store.list_all()
        if not openids:
            return
        token = self._get_wechat_access_token()
        # 订阅消息字段有长度限制：thing≤20字符，phrase≤5字符，time 是日期时间字符串。
        level_label = {"critical": "危险", "warning": "警告", "info": "提示"}.get(
            level.lower(), level[:5]
        )
        data = {
            self.wechat_field_keys.get("level", "phrase1"): {"value": level_label[:5]},
            self.wechat_field_keys.get("message", "thing2"): {
                "value": f"{source}: {message}"[:20]
            },
            self.wechat_field_keys.get("time", "time3"): {
                "value": time.strftime("%Y-%m-%d %H:%M:%S")
            },
        }
        for openid in openids:
            payload = {
                "touser": openid,
                "template_id": self.wechat_template_id,
                "data": data,
            }
            if self.wechat_page:
                payload["page"] = self.wechat_page
            resp = requests.post(
                "https://api.weixin.qq.com/cgi-bin/message/subscribe/send",
                params={"access_token": token},
                json=payload,
                timeout=5.0,
            )
            result = resp.json()
            if result.get("errcode", 0) != 0:
                # 43101=用户未授权该模板（一次性订阅已用掉，需要用户再次触发
                # wx.requestSubscribeMessage 才能再收到一次）——这是订阅消息
                # 机制本身的限制，不是推送失败，只打日志不重试。
                print(f"[Notifier] 微信推送 {openid} 失败: {result}")
