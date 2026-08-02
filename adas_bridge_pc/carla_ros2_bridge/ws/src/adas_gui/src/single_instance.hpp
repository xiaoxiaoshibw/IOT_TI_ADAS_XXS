#ifndef ADAS_GUI__SINGLE_INSTANCE_HPP_
#define ADAS_GUI__SINGLE_INSTANCE_HPP_

// 系统级单实例互斥锁：本机同一时刻只能有一个 GUI、一个 CARLA（PC 是团队
// 共享机器，见 hil-operational-gotchas 记忆——两个独立 CARLA 客户端各自当
// 自己是唯一的 tick 发起者会互相踩，还可能连带把对方杀掉）。
//
// 用 flock(2) 而不是"看有没有这个 pid 的进程"之类的检测，是因为 flock 天然
// 处理两个关键场景：
//   1. GUI 退出时，CARLA 子进程不继承 GUI 锁；新 GUI 会通过端口探测复用仍在
//      运行的外部 CARLA，不会被子进程遗留的 GUI 锁阻断。
//   2. 崩溃安全：进程被 SIGKILL 或宕机，OS 关闭其所有 fd 时锁自动释放，
//      不会留下"锁死但进程已经不在了"的僵尸锁（不像 pidfile 那样需要额外
//      的陈旧 pid 检测逻辑）。
//
// 锁文件路径是全局约定，CLI 脚本（start_carla.sh）和本 GUI 必须用同一个
// 路径才能互斥住彼此，改路径要两边一起改。

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <utility>

#include <QString>

namespace adas::gui {

inline QString carla_lock_path() { return QStringLiteral("/tmp/adas_carla.lock"); }
inline QString gui_lock_path() { return QStringLiteral("/tmp/adas_gui.lock"); }

class SingleInstanceLock {
 public:
  explicit SingleInstanceLock(QString path) : path_(std::move(path)) {}
  ~SingleInstanceLock() { unlock(); }

  SingleInstanceLock(const SingleInstanceLock&) = delete;
  SingleInstanceLock& operator=(const SingleInstanceLock&) = delete;

  // 非阻塞加锁：已被别的进程占用时立即返回 false，不等待、不阻塞 UI 线程。
  bool tryLock() {
    if (fd_ >= 0) return true;  // 本进程已经持有
    const int fd = ::open(path_.toLocal8Bit().constData(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
    if (fd < 0) return false;
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd);
      return false;
    }
    fd_ = fd;
    return true;
  }

  // 主动释放（例如托管的进程已确认退出，允许后续重新启动）。析构里也会调，
  // 这里额外暴露出来是因为 GUI 进程存活期间可能反复 start/stop CARLA。
  void unlock() {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool held() const { return fd_ >= 0; }

 private:
  QString path_;
  int fd_{-1};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__SINGLE_INSTANCE_HPP_
