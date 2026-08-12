#ifndef ADAS_GUI__SESSION_CONTRACT_HPP_
#define ADAS_GUI__SESSION_CONTRACT_HPP_

#include <QString>

namespace adas::gui {

inline bool accepts_run_id(const QString& current, const QString& incoming) {
  return !current.isEmpty() && !incoming.isEmpty() && current == incoming;
}

enum class RouteUpdate { Ignore, Replace, Clear };

inline RouteUpdate route_update_for(const QString& current_run_id,
                                    const QString& incoming_run_id,
                                    bool status_valid,
                                    int point_count) {
  if (!accepts_run_id(current_run_id, incoming_run_id)) {
    return RouteUpdate::Ignore;
  }
  return status_valid && point_count >= 2 ? RouteUpdate::Replace
                                          : RouteUpdate::Clear;
}

inline bool map_identity_changed(const QString& old_map_id,
                                 const QString& old_map_hash,
                                 const QString& new_map_id,
                                 const QString& new_map_hash) {
  if (old_map_id.isEmpty() && old_map_hash.isEmpty()) return false;
  if (!old_map_id.isEmpty() && old_map_id != new_map_id) return true;
  return !old_map_hash.isEmpty() && !new_map_hash.isEmpty() &&
         old_map_hash != new_map_hash;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__SESSION_CONTRACT_HPP_
