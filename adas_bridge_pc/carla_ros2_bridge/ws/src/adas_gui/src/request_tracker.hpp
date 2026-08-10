#ifndef ADAS_GUI__REQUEST_TRACKER_HPP_
#define ADAS_GUI__REQUEST_TRACKER_HPP_

#include <QHash>
#include <QList>
#include <QString>

namespace adas::gui {

enum class RequestState { Sent, Acknowledged, Failed, TimedOut };

struct RequestRecord {
  QString operation;
  QString request_id;
  RequestState state{RequestState::Sent};
  qint64 deadline_ms{0};
  QString detail;
};

// Operation-keyed request registry. A key has at most one in-flight request;
// response IDs must match so a late response cannot unlock a newer request.
class RequestTracker {
 public:
  bool begin(const QString& operation, const QString& request_id,
             qint64 now_ms, qint64 timeout_ms);
  bool finish(const QString& operation, const QString& request_id,
              RequestState state, const QString& detail = {});
  QList<RequestRecord> expire(qint64 now_ms);
  bool pending(const QString& operation) const;
  QString requestId(const QString& operation) const;
  void cancelAll(const QString& detail = {});

 private:
  QHash<QString, RequestRecord> pending_;
};

}  // namespace adas::gui

#endif  // ADAS_GUI__REQUEST_TRACKER_HPP_
