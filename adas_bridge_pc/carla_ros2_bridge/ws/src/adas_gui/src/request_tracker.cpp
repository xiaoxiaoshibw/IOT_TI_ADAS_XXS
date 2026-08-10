#include "request_tracker.hpp"

namespace adas::gui {

bool RequestTracker::begin(const QString& operation, const QString& request_id,
                           qint64 now_ms, qint64 timeout_ms) {
  if (operation.isEmpty() || request_id.isEmpty() || timeout_ms <= 0 ||
      pending_.contains(operation)) {
    return false;
  }
  pending_.insert(operation, {operation, request_id, RequestState::Sent,
                              now_ms + timeout_ms, {}});
  return true;
}

bool RequestTracker::finish(const QString& operation, const QString& request_id,
                            RequestState state, const QString& detail) {
  auto it = pending_.find(operation);
  if (it == pending_.end() || it->request_id != request_id) return false;
  it->state = state;
  it->detail = detail;
  pending_.erase(it);
  return true;
}

QList<RequestRecord> RequestTracker::expire(qint64 now_ms) {
  QList<RequestRecord> expired;
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (now_ms < it->deadline_ms) {
      ++it;
      continue;
    }
    it->state = RequestState::TimedOut;
    it->detail = QStringLiteral("request timed out");
    expired.append(*it);
    it = pending_.erase(it);
  }
  return expired;
}

bool RequestTracker::pending(const QString& operation) const {
  return pending_.contains(operation);
}

QString RequestTracker::requestId(const QString& operation) const {
  const auto it = pending_.constFind(operation);
  return it == pending_.cend() ? QString() : it->request_id;
}

void RequestTracker::cancelAll(const QString&) { pending_.clear(); }

}  // namespace adas::gui
