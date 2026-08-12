// 配合 run_id_session.hpp；adas_gui CMake 把 .cpp 加进可执行文件编译清单。
#include "run_id_session.hpp"

namespace adas_gui {
QString RunIdSession::current_ = QString();
}  // namespace adas_gui
