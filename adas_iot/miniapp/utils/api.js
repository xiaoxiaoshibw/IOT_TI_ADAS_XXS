const app = getApp();

function trimTrailingSlash(url) {
  return String(url || "").trim().replace(/\/+$/, "");
}

function getServerUrl() {
  const stored = wx.getStorageSync("serverUrl");
  return trimTrailingSlash(stored || app.globalData.defaultServerUrl);
}

function setServerUrl(url) {
  wx.setStorageSync("serverUrl", trimTrailingSlash(url));
}

function getTemplateId() {
  return String(wx.getStorageSync("wechatTemplateId") || "").trim();
}

function setTemplateId(templateId) {
  wx.setStorageSync("wechatTemplateId", String(templateId || "").trim());
}

function request(path, options = {}) {
  const serverUrl = getServerUrl();
  return new Promise((resolve, reject) => {
    const header = options.header || {};
    header["content-type"] = header["content-type"] || "application/json";
    wx.request({
      url: `${serverUrl}${path}`,
      method: options.method || "GET",
      data: options.data || undefined,
      timeout: options.timeout || 5000,
      header,
      success(res) {
        const ok = res.statusCode >= 200 && res.statusCode < 300;
        if (!ok) {
          reject(new Error(`HTTP ${res.statusCode}`));
          return;
        }
        let data = res.data;
        if (typeof data === "string") {
          try {
            data = JSON.parse(data);
          } catch (error) {
            reject(new Error("响应不是有效 JSON"));
            return;
          }
        }
        resolve(data);
      },
      fail(error) {
        reject(error);
      }
    });
  });
}

module.exports = {
  getServerUrl,
  setServerUrl,
  getTemplateId,
  setTemplateId,
  request
};
