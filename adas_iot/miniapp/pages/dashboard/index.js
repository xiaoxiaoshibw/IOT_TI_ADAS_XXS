const api = require("../../utils/api");

const DEFAULT_VEHICLE = {
  speedText: "0.0",
  speedKmhText: "0.0",
  steeringText: "0.0",
  brakeText: "0",
  throttleText: "0",
  brakePct: 0,
  throttlePct: 0,
  sourceText: "-"
};

const KNOWN_SAFETY_STATES = [
  "INIT",
  "STANDBY",
  "ACTIVE",
  "DEGRADED",
  "EMERGENCY_BRAKE",
  "FAILSAFE",
  "MRM",
  "FAULT_LOCK"
];

const SAFETY_LABEL_OVERRIDE = {
  EMERGENCY_BRAKE: "AEB",
  FAULT_LOCK: "FAULT"
};

const DANGER_STATES = ["EMERGENCY", "EMERGENCY_BRAKE", "FAILSAFE", "FAULT_LOCK"];
const WARN_STATES = ["MRM", "DEGRADED"];

const POLL_BACKOFF_STEPS = [
  { upTo: 2, ms: 1000 },
  { upTo: 5, ms: 3000 },
  { upTo: 9, ms: 5000 },
  { upTo: Infinity, ms: -1 }
];

const SPARKLINE_MAX_POINTS = 60;

function numberValue(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function percentValue(value) {
  const numeric = numberValue(value, 0);
  const pct = numeric <= 1 ? numeric * 100 : numeric;
  return Math.max(0, Math.min(100, pct));
}

function formatFixed(value, digits = 1) {
  return numberValue(value, 0).toFixed(digits);
}

function firstValue(values, fallback) {
  for (let index = 0; index < values.length; index += 1) {
    if (values[index] !== undefined && values[index] !== null) {
      return values[index];
    }
  }
  return fallback;
}

function pad2(value) {
  const text = String(value);
  return text.length >= 2 ? text : `0${text}`;
}

function pickBackoffMs(failureCount) {
  for (let index = 0; index < POLL_BACKOFF_STEPS.length; index += 1) {
    if (failureCount <= POLL_BACKOFF_STEPS[index].upTo) {
      return POLL_BACKOFF_STEPS[index].ms;
    }
  }
  return -1;
}

function normalizeVehicle(raw = {}) {
  const speed = firstValue([raw.speed, raw.speed_mps], 0);
  const steering = firstValue([raw.steering, raw.steering_deg], 0);
  const brake = firstValue([raw.brake, raw.brake_pct], 0);
  const throttle = firstValue([raw.throttle, raw.throttle_pct], 0);
  const brakePct = percentValue(brake);
  const throttlePct = percentValue(throttle);
  const source = raw.source || raw.active_source || "none";
  const sourceMap = {
    primary: "主源",
    backup: "备源",
    safety: "安全仲裁",
    ros2: "ROS2",
    none: "-"
  };

  return {
    speedText: formatFixed(speed, 1),
    speedKmhText: formatFixed(numberValue(speed) * 3.6, 1),
    steeringText: formatFixed(steering, 1),
    brakeText: formatFixed(brakePct, 0),
    throttleText: formatFixed(throttlePct, 0),
    brakePct,
    throttlePct,
    sourceText: sourceMap[source] || source
  };
}

function normalizeSafety(raw = {}) {
  const state = raw.state || raw.safety_level || raw.hil_state || "INIT";
  const levelClass = DANGER_STATES.includes(state) ? "danger" : WARN_STATES.includes(state) ? "warn" : "ok";

  let cellList = KNOWN_SAFETY_STATES;
  if (state && KNOWN_SAFETY_STATES.indexOf(state) === -1) {
    cellList = [state].concat(KNOWN_SAFETY_STATES);
  }
  const cells = cellList.map((name) => {
    let cls = "";
    if (state === name) {
      cls = DANGER_STATES.includes(name) ? "danger" : WARN_STATES.includes(name) ? "warn" : "active";
    }
    return {
      name,
      label: SAFETY_LABEL_OVERRIDE[name] || name,
      cls
    };
  });

  return {
    state,
    levelClass,
    cells,
    faultLevel: firstValue([raw.fault_level], 0),
    activeSource: raw.active_source || "-",
    aebText: raw.aeb_active ? "已触发" : "未触发"
  };
}

function normalizeCan(raw = {}) {
  return {
    framePeriodText: formatFixed(firstValue([raw.frame_period_ms, raw.period_mean], 0), 2),
    crcErrors: firstValue([raw.crc_errors, raw.crc_error_cnt], 0),
    seqErrors: firstValue([raw.seq_errors], 0),
    loopLoadText: formatFixed(firstValue([raw.loop_load_pct, raw.loop_load], 0), 1)
  };
}

function normalizeSystem(raw = {}) {
  return {
    mqttText: raw.mqtt_connected ? "MQTT 已连接" : "MQTT 未连接"
  };
}

function normalizeAlerts(raw = []) {
  const levelMap = {
    info: { text: "提示", cls: "info" },
    warning: { text: "警告", cls: "warning" },
    critical: { text: "危险", cls: "critical" },
    danger: { text: "危险", cls: "critical" }
  };

  return raw.slice(0, 20).map((item, index) => {
    const level = String(item.level || "info").toLowerCase();
    const mapped = levelMap[level] || levelMap.info;
    return {
      id: `${item.time || item.timestamp || index}-${index}`,
      key: `${item.time || ""}|${item.source || ""}|${item.message || ""}`,
      time: item.time || "--:--",
      source: item.source || "system",
      message: item.message || "未知告警",
      levelText: mapped.text,
      levelClass: mapped.cls,
      level
    };
  });
}

Page({
  data: {
    online: false,
    statusText: "等待连接",
    lastUpdatedText: "",
    vehicle: DEFAULT_VEHICLE,
    safety: normalizeSafety(),
    can: normalizeCan(),
    system: normalizeSystem(),
    alerts: [],
    recentNewAlert: null,
    failureCount: 0,
    pollStopped: false
  },

  onLoad() {
    this.timer = null;
    this.bannerTimer = null;
    this.seenAlertKeys = new Set();
    this.sparkline = null;
    this.sparklineSpeed = [];
  },

  onReady() {
    this.initSparklineCanvas();
  },

  onShow() {
    const start = () => {
      this.scheduleNextPoll(pickBackoffMs(this.data.failureCount || 0));
    };
    this.fetchState().then(start).catch(start);
  },

  onHide() {
    this.stopPolling();
  },

  onUnload() {
    this.stopPolling();
    if (this.bannerTimer) {
      clearTimeout(this.bannerTimer);
      this.bannerTimer = null;
    }
  },

  onPullDownRefresh() {
    const finish = () => {
      wx.stopPullDownRefresh();
      this.scheduleNextPoll(pickBackoffMs(this.data.failureCount || 0));
    };
    this.fetchState().then(finish).catch(finish);
  },

  stopPolling() {
    if (this.timer) {
      clearTimeout(this.timer);
      this.timer = null;
    }
  },

  scheduleNextPoll(delayMs) {
    this.stopPolling();
    if (delayMs < 0) {
      this.setData({ pollStopped: true });
      return;
    }
    this.setData({ pollStopped: false });
    this.timer = setTimeout(() => {
      const reschedule = () => {
        const next = pickBackoffMs(this.data.failureCount || 0);
        this.scheduleNextPoll(next);
      };
      this.fetchState({ silent: true }).then(reschedule).catch(reschedule);
    }, delayMs);
  },

  fetchState(options = {}) {
    return api.request("/api/state").then((state) => {
      const now = new Date();
      const alerts = normalizeAlerts(state.alerts || []);
      this.detectNewAlerts(alerts);

      const histSpeed = (state.history && Array.isArray(state.history.speed)) ? state.history.speed : [];
      if (histSpeed.length > 0) {
        this.sparklineSpeed = histSpeed.slice(-SPARKLINE_MAX_POINTS);
        this.drawSparkline();
      }

      this.setData({
        online: true,
        statusText: "数据接收中",
        lastUpdatedText: `${pad2(now.getHours())}:${pad2(now.getMinutes())}:${pad2(now.getSeconds())}`,
        vehicle: normalizeVehicle(state.vehicle || {}),
        safety: normalizeSafety(state.safety || {}),
        can: normalizeCan(state.can || {}),
        system: normalizeSystem(state.system || {}),
        alerts,
        failureCount: 0
      });
    }).catch((error) => {
      const failureCount = (this.data.failureCount || 0) + 1;
      const backoffMs = pickBackoffMs(failureCount);
      const stopped = backoffMs < 0;
      this.setData({
        online: false,
        statusText: stopped
          ? "已暂停轮询，下拉刷新恢复"
          : `连接失败 (${failureCount})`,
        lastUpdatedText: error.errMsg || error.message || "请检查后端地址",
        failureCount
      });
      if (!options.silent && !stopped) {
        wx.showToast({
          title: "连接失败",
          icon: "none"
        });
      }
      if (stopped && !options.silent) {
        wx.showToast({
          title: "连续失败，已暂停轮询",
          icon: "none"
        });
      }
    });
  },

  detectNewAlerts(alerts) {
    if (!this.seenAlertKeys || !(this.seenAlertKeys instanceof Set)) {
      this.seenAlertKeys = new Set();
    }
    const newOnes = alerts.filter((item) => !this.seenAlertKeys.has(item.key));

    for (let index = 0; index < alerts.length; index += 1) {
      this.seenAlertKeys.add(alerts[index].key);
    }
    if (this.seenAlertKeys.size > 200) {
      const trimmed = new Set(alerts.map((item) => item.key));
      this.seenAlertKeys = trimmed;
    }

    if (newOnes.length === 0) {
      return;
    }
    const now = Date.now();
    if (this.data.recentNewAlert && now - (this._lastBannerAt || 0) < 4000) {
      return;
    }
    this._lastBannerAt = now;
    const top = newOnes[0];
    this.setData({ recentNewAlert: top });
    if (top.level === "critical" || top.level === "warning") {
      wx.vibrateLong({
        fail: () => {}
      });
    }
    if (this.bannerTimer) {
      clearTimeout(this.bannerTimer);
    }
    this.bannerTimer = setTimeout(() => {
      this.setData({ recentNewAlert: null });
      this.bannerTimer = null;
    }, 3500);
  },

  dismissAlertBanner() {
    if (this.bannerTimer) {
      clearTimeout(this.bannerTimer);
      this.bannerTimer = null;
    }
    this.setData({ recentNewAlert: null });
  },

  initSparklineCanvas() {
    const query = wx.createSelectorQuery();
    query.select("#sparkline").fields({ node: true, size: true }).exec((res) => {
      if (!res || !res[0] || !res[0].node) {
        return;
      }
      const canvas = res[0].node;
      const dpr = (wx.getSystemInfoSync && wx.getSystemInfoSync().pixelRatio) || 1;
      canvas.width = Math.max(1, Math.round(res[0].width * dpr));
      canvas.height = Math.max(1, Math.round(res[0].height * dpr));
      const ctx = canvas.getContext("2d");
      if (!ctx) {
        return;
      }
      ctx.scale(dpr, dpr);
      this.sparkline = {
        canvas,
        ctx,
        width: res[0].width,
        height: res[0].height
      };
      this.drawSparkline();
    });
  },

  drawSparkline() {
    if (!this.sparkline) {
      return;
    }
    const { ctx, width, height } = this.sparkline;
    const speed = this.sparklineSpeed || [];
    if (speed.length < 2 || width <= 0 || height <= 0) {
      return;
    }
    ctx.clearRect(0, 0, width, height);

    const maxVal = Math.max.apply(null, speed.concat([1]));
    const minVal = Math.min.apply(null, speed.concat([0]));
    const span = Math.max(1e-3, maxVal - minVal);
    const stepX = width / (speed.length - 1);

    ctx.beginPath();
    ctx.moveTo(0, height);
    for (let index = 0; index < speed.length; index += 1) {
      const x = index * stepX;
      const y = height - ((speed[index] - minVal) / span) * (height - 4) - 2;
      if (index === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.lineTo(width, height);
    ctx.closePath();
    ctx.fillStyle = "rgba(15, 118, 110, 0.15)";
    ctx.fill();

    ctx.beginPath();
    for (let index = 0; index < speed.length; index += 1) {
      const x = index * stepX;
      const y = height - ((speed[index] - minVal) / span) * (height - 4) - 2;
      if (index === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.strokeStyle = "#0f766e";
    ctx.lineWidth = 1.5;
    ctx.lineJoin = "round";
    ctx.stroke();
  },

  onAction(event) {
    const action = (event && event.currentTarget && event.currentTarget.dataset && event.currentTarget.dataset.action) || "";
    if (action === "bind-wechat") {
      this.bindWechat();
    }
  },

  goSettings() {
    wx.navigateTo({
      url: "/pages/settings/index"
    });
  },

  bindWechat() {
    const templateId = api.getTemplateId();
    if (!templateId) {
      wx.showToast({
        title: "先填写模板 ID",
        icon: "none"
      });
      this.goSettings();
      return;
    }

    wx.requestSubscribeMessage({
      tmplIds: [templateId],
      success: (subscribeResult) => {
        const result = subscribeResult[templateId];
        if (result === "accept") {
          this.loginAndBind();
          return;
        }
        const messageMap = {
          reject: "已拒绝订阅，无法接收告警",
          ban: "订阅消息已被微信后台封禁",
          filter: "模板已失效或被过滤",
          unsubscribed: "未完成订阅"
        };
        wx.showToast({
          title: messageMap[result] || `订阅未通过：${result || "未知"}`,
          icon: "none",
          duration: 2500
        });
      },
      fail: () => {
        wx.showToast({
          title: "订阅授权失败",
          icon: "none"
        });
      }
    });
  },

  loginAndBind() {
    wx.login({
      success: (loginResult) => {
        if (!loginResult.code) {
          wx.showToast({
            title: "登录失败",
            icon: "none"
          });
          return;
        }
        api.request("/api/wechat/bind", {
          method: "POST",
          data: {
            code: loginResult.code
          }
        }).then((result) => {
          wx.showToast({
            title: result && result.new_subscriber ? "绑定成功" : "已绑定",
            icon: "success"
          });
        }).catch(() => {
          wx.showToast({
            title: "绑定接口失败",
            icon: "none"
          });
        });
      },
      fail: () => {
        wx.showToast({
          title: "微信登录失败",
          icon: "none"
        });
      }
    });
  }
});