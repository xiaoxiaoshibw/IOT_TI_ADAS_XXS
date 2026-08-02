const api = require("../../utils/api");

function cleanUrl(url) {
  return String(url || "").trim().replace(/\/+$/, "");
}

function isInsecureUrl(url) {
  return /^http:\/\//i.test(cleanUrl(url));
}

function extractServerUrl(text) {
  if (!text) {
    return null;
  }
  const match = String(text).match(/^(https?:\/\/[^\s/?#]+)/i);
  if (!match) {
    return null;
  }
  return cleanUrl(match[1]);
}

Page({
  data: {
    serverUrl: "",
    templateId: "",
    bindUrl: "",
    insecure: false,
    testing: false,
    debugInfo: null,
    debugLoading: false,
    debugError: ""
  },

  onLoad() {
    const serverUrl = api.getServerUrl();
    this.setData({
      serverUrl,
      templateId: api.getTemplateId(),
      bindUrl: `${serverUrl}/api/wechat/bind`,
      insecure: isInsecureUrl(serverUrl)
    });
  },

  onServerInput(event) {
    const serverUrl = event.detail.value;
    this.setData({
      serverUrl,
      bindUrl: `${cleanUrl(serverUrl)}/api/wechat/bind`,
      insecure: isInsecureUrl(serverUrl)
    });
  },

  onTemplateInput(event) {
    this.setData({
      templateId: event.detail.value
    });
  },

  persistSettings(showToast) {
    const serverUrl = cleanUrl(this.data.serverUrl);
    if (!serverUrl) {
      wx.showToast({
        title: "地址不能为空",
        icon: "none"
      });
      return false;
    }

    const insecure = isInsecureUrl(serverUrl);
    api.setServerUrl(serverUrl);
    api.setTemplateId(this.data.templateId);
    this.setData({
      serverUrl,
      templateId: api.getTemplateId(),
      bindUrl: `${serverUrl}/api/wechat/bind`,
      insecure
    });
    if (showToast) {
      wx.showToast({
        title: "已保存",
        icon: "success"
      });
      if (insecure) {
        setTimeout(() => {
          wx.showModal({
            title: "HTTP 仅供本地调试",
            content: "微信小程序要求 request 合法域名为 HTTPS，公网部署前请改为 https://。本地调试请在微信开发者工具中关闭域名校验。",
            showCancel: false,
            confirmText: "知道了"
          });
        }, 400);
      }
    }
    return true;
  },

  save() {
    this.persistSettings(true);
  },

  resetDefault() {
    const defaultUrl = getApp().globalData.defaultServerUrl;
    this.setData({
      serverUrl: defaultUrl,
      bindUrl: `${defaultUrl}/api/wechat/bind`,
      insecure: isInsecureUrl(defaultUrl)
    });
  },

  testConnection() {
    if (!this.persistSettings(false)) {
      return;
    }
    this.setData({
      testing: true
    });
    api.request("/api/state", {
      timeout: 4000
    }).then(() => {
      wx.showToast({
        title: "连接正常",
        icon: "success"
      });
    }).catch(() => {
      wx.showToast({
        title: "连接失败",
        icon: "none"
      });
    }).then(() => {
      this.setData({
        testing: false
      });
    }).catch(() => {
      this.setData({
        testing: false
      });
    });
  },

  scanQrCode() {
    if (!wx.scanCode) {
      wx.showToast({
        title: "当前微信版本不支持扫码",
        icon: "none"
      });
      return;
    }
    wx.scanCode({
      onlyFromCamera: false,
      scanType: ["qr_code"],
      success: (res) => {
        const scanned = res && res.result;
        const url = extractServerUrl(scanned);
        if (!url) {
          wx.showToast({
            title: "二维码不是有效的 URL",
            icon: "none",
            duration: 2500
          });
          return;
        }
        this.setData({
          serverUrl: url,
          bindUrl: `${url}/api/wechat/bind`,
          insecure: isInsecureUrl(url)
        });
        this.persistSettings(false);
        wx.showToast({
          title: `已配 ${url}`,
          icon: "success",
          duration: 2000
        });
        this.fetchDebugInfo();
      },
      fail: (err) => {
        if (err && err.errMsg && /cancel/i.test(err.errMsg)) {
          return;
        }
        wx.showToast({
          title: "扫码失败",
          icon: "none"
        });
      }
    });
  },

  fetchDebugInfo() {
    if (!this.data.serverUrl) {
      wx.showToast({
        title: "先填或扫码得到地址",
        icon: "none"
      });
      return;
    }
    this.setData({
      debugLoading: true,
      debugError: ""
    });
    api.request("/debug/info", { timeout: 4000 }).then((info) => {
      const lanIps = Array.isArray(info && info.lan_ips) ? info.lan_ips : [];
      const port = (info && info.port) || "";
      const urlItems = lanIps.map((ip) => ({
        ip,
        url: port ? `http://${ip}:${port}/` : `http://${ip}/`
      }));
      this.setData({
        debugInfo: {
          hostname: (info && info.hostname) || "",
          port: String(port),
          items: urlItems
        },
        debugLoading: false
      });
    }).catch((err) => {
      this.setData({
        debugLoading: false,
        debugError: (err && (err.errMsg || err.message)) || "拉取失败"
      });
    });
  },

  copyDebugUrl(event) {
    const url = event && event.currentTarget && event.currentTarget.dataset && event.currentTarget.dataset.url;
    if (!url) {
      return;
    }
    wx.setClipboardData({
      data: url,
      success: () => {
        wx.showToast({
          title: "已复制",
          icon: "success"
        });
      },
      fail: () => {
        wx.showToast({
          title: "复制失败",
          icon: "none"
        });
      }
    });
  }
});