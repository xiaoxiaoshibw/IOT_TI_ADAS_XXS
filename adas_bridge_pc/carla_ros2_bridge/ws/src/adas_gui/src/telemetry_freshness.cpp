#include "telemetry_freshness.hpp"

namespace adas::gui {

TelemetryFreshness::TelemetryFreshness() { last_ms_.fill(-1); }

TelemetryFreshness& TelemetryFreshness::instance() {
  static TelemetryFreshness inst;
  return inst;
}

void TelemetryFreshness::markFresh(Channel c) {
  last_ms_[static_cast<int>(c)] = QDateTime::currentMSecsSinceEpoch();
}

qint64 TelemetryFreshness::ageMs(Channel c) const {
  const qint64 t = last_ms_[static_cast<int>(c)];
  if (t < 0) return -1;
  return QDateTime::currentMSecsSinceEpoch() - t;
}

bool TelemetryFreshness::isFresh(Channel c, qint64 limit_ms) const {
  const qint64 a = ageMs(c);
  return a >= 0 && a <= limit_ms;
}

void TelemetryFreshness::resetSession() {
  last_ms_.fill(-1);
}

void TelemetryFreshness::resetForTest() { resetSession(); }

}  // namespace adas::gui
