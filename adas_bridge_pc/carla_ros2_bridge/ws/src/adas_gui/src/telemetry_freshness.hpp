#ifndef ADAS_GUI__TELEMETRY_FRESHNESS_HPP_
#define ADAS_GUI__TELEMETRY_FRESHNESS_HPP_

// GUI 线程单例：所有面板共享同一份"遥测新鲜度"事实表。
//
// 之前 MainWindow / SafetyPanel 各自维护一份 last_status_ms_ 等字段，
// 同一信号两处更新时间可能错开（queued signal 顺序未严格保证），
// 偶发出现"A 面板说新鲜、B 面板说断流"的怪现象。抽到单例后只有一份事实，
// 阈值改一处生效（kMcuStaleMs 等常量保留在原处，仅这里集中存储时间戳）。

#include <array>

#include <QDateTime>
#include <QtGlobal>

namespace adas::gui {

class TelemetryFreshness {
 public:
  // 与 RosBridge 信号一一对应；新增通道在这里加枚举值即可。
  enum Channel {
    Mcu,        // /adas/mcu/status
    Actuation,  // /adas/mcu/actuation_feedback
    Ego,        // /adas/localization/kinematic_state
    Nav,        // /adas/navigation/status
    Behavior,   // /adas/planning/behavior
    Aeb,        // /adas/control/aeb/status
    Lead,       // /adas/perception/objects (lead distance)
    Count,
  };

  // GUI 单线程，无需线程安全。
  static TelemetryFreshness& instance();

  // 收到一条有效消息时调用，记录时间戳。
  void markFresh(Channel c);

  // 自上次 markFresh 起的毫秒数；从未收到返回 -1。
  qint64 ageMs(Channel c) const;

  // ageMs >= 0 且 ageMs <= limit_ms。
  bool isFresh(Channel c, qint64 limit_ms) const;

  // 测试用：清空所有时间戳。
  void resetForTest();

 private:
  TelemetryFreshness() = default;
  std::array<qint64, Count> last_ms_{};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__TELEMETRY_FRESHNESS_HPP_