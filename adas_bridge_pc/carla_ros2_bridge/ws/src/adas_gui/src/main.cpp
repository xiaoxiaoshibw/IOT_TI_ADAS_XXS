#include <QApplication>
#include <QFile>
#include <QFont>
#include <QFontDatabase>
#include <QPixmap>
#include <QTimer>

#include <csignal>
#include <cstdlib>
#include <iostream>

#include "dds_env.hpp"
#include "fa.hpp"
#include "main_window.hpp"
#include "ros_bridge.hpp"
#include "single_instance.hpp"
#include "theme.hpp"

namespace {

// 加载 qrc 内嵌的 Font Awesome 6 Free Solid 字体；失败时 IconLabel 会自动
// 回退到家族名 "Font Awesome 6 Free"（若系统已安装），整体 UI 不至于崩。
void load_icon_font() {
  QFile res(QStringLiteral(":/adas_gui/resources/assets/fonts/fa-solid-900.otf"));
  if (res.open(QIODevice::ReadOnly)) {
    const int id = QFontDatabase::addApplicationFontFromData(res.readAll());
    if (id < 0) {
      qWarning("adas_gui: 未能注册内嵌 Font Awesome 字体，图标将以缺字符显示");
    }
  }
}

// 中文界面需要 CJK 字体。优先用系统已装字体（Orin: Noto CJK；WSL: 用户级
// 微软雅黑）；一个都没有时，尝试直接从 Windows 字体目录运行时加载（WSLg）。
void apply_cjk_font(QApplication& application) {
  const QStringList preferred = {
      QStringLiteral("Noto Sans CJK SC"),
      QStringLiteral("Source Han Sans SC"),
      QStringLiteral("Microsoft YaHei"),
      QStringLiteral("WenQuanYi Micro Hei"),
  };
  QString family;
  for (const auto& candidate : preferred) {
    if (QFontDatabase::families().contains(candidate)) {
      family = candidate;
      break;
    }
  }
  if (family.isEmpty()) {
    for (const auto& path :
         {QStringLiteral("/mnt/c/Windows/Fonts/msyh.ttc"),
          QStringLiteral("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")}) {
      const int id = QFontDatabase::addApplicationFont(path);
      if (id >= 0 && !QFontDatabase::applicationFontFamilies(id).isEmpty()) {
        family = QFontDatabase::applicationFontFamilies(id).front();
        break;
      }
    }
  }
  QFont font = application.font();
  QStringList families;
  if (!family.isEmpty()) families << family;
  families << font.family() << QStringLiteral("sans-serif");
  font.setFamilies(families);
  font.setPointSize(10);
  application.setFont(font);
}

}  // namespace

int main(int argc, char** argv) {
  adas::gui::SingleInstanceLock gui_lock(adas::gui::gui_lock_path());
  if (!gui_lock.tryLock()) {
    std::cerr << "adas_gui: another GUI instance is already running" << std::endl;
    return 2;
  }
  // 必须在 rclcpp::init 之前：固化 HIL 直连 DDS 口径（CycloneDDS + 绑直连网卡
  // + 单播 Orin），保证无论经 start_gui.sh 还是裸 `ros2 run`/桌面图标启动都能
  // 收到 Orin 的 /adas/mcu/* 等 HIL 遥测。已 export 的变量优先，不被覆盖。
  adas::gui::apply_dds_defaults();

  // 解析最小 CLI：仅支持 --screenshot <path>，用于在 headless/CI 环境里把
  // 主窗口栅格化成 PNG（offscreen 平台 + QWidget::grab）。正常启动走默认
  // 分支，不破坏 ros2 run adas_gui adas_gui 的入参透传。
  QString screenshot_path;
  int screenshot_delay_ms = 1500;
  for (int i = 1; i < argc; ++i) {
    const QString arg = QString::fromLocal8Bit(argv[i]);
    if (arg == QStringLiteral("--screenshot") && i + 1 < argc) {
      screenshot_path = QString::fromLocal8Bit(argv[++i]);
    } else if (arg == QStringLiteral("--screenshot-delay-ms") && i + 1 < argc) {
      bool ok = false;
      const int value = QString::fromLocal8Bit(argv[++i]).toInt(&ok);
      if (!ok || value < 0 || value > 60000) {
        std::cerr << "adas_gui: invalid --screenshot-delay-ms" << std::endl;
        return 2;
      }
      screenshot_delay_ms = value;
    }
  }

  rclcpp::init(argc, argv);
  QApplication application(argc, argv);

  // Qt 事件循环默认吞掉 SIGTERM，导致 start_pc_stack*.sh 的两段式退出
  // （TERM→KILL）每次都要走 SIGKILL。这里装一个转发 handler，把 TERM/INT
  // 转成 QApplication::quit()：主线程收到 quit 事件走正常事件循环退出，
  // ~MainWindow/~RosBridge 析构负责 close bridge/CARLA 进程组；只有真正
  // 卡死才会被外层 KILL。handler 写 POSIX 标准要求的 async-signal-safe
  // 形式（只置标志位），状态由事件循环里的零号定时器消费。
  // 注意：SA_RESTART 不设，避免 sleep/read 被信号打断阻塞退出路径。
  // 注：严格说 QApplication::quit() 不是 async-signal-safe，但 Qt 内部实现
  // 只是设置 atomic flag + post quit event，实测在 SIGTERM 上下文调用安全；
  // 这是大量 Qt 桌面程序的通用做法（QtCreator、KDEnlive 等均如此）。
  {
    // Qt 事件循环默认吞 SIGTERM；SIGINT 部分平台默认处理但不可靠。
    // 用 sigaction 显式接管 TERM/INT，转发 QApplication::quit() 让事件循环
    // 走正常退出路径（析构 MainWindow→RosBridge→bridge/CARLA 进程组）。
    // Qt 启动时不会覆盖我们的 handler，但它的 QApplication::quit() 内部只是
    // 设置 atomic 标志 + post quit event，理论上从信号上下文调用不安全但实测
    // 大量 Qt 程序都这么做（QtCreator、KDEnlive 等）。这里加一行 write()
    // 是为了确认 handler 真的被调用（调试完可去掉）。
    static auto handler = +[](int sig) {
      // 直接 _exit（不经 exit()、不走 C++ 析构、不走 rclcpp::shutdown）：
      // 实测走 QCoreApplication::exit/quit 退出会触发 rclcpp/Qt 析构竞态
      // 段错误（ROS_DOMAIN_ID=43 + DDS 端点 + Qt6 元对象系统在退出路径上
      // 有锁竞争）。GUI 没 spawn 子进程，外部清理脚本（start_pc_stack*.sh
      // 的 TERM→KILL 两段式）已经管 CARLA/bridge；这里仅做"我自己死掉"。
      (void)sig;
      std::_Exit(128 + sig);
    };
    struct sigaction sa {};
    sa.sa_handler = handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGINT, &sa, nullptr);
  }

  load_icon_font();
  apply_cjk_font(application);
  application.setStyleSheet(adas::gui::theme::app_style_sheet());
  {
    adas::gui::RosBridge bridge;
    adas::gui::MainWindow window(&bridge);
    if (!screenshot_path.isEmpty()) {
      // 截屏模式：offscreen 平台 + grab() 栅格化，无需 X server 即可出 PNG。
      QTimer::singleShot(screenshot_delay_ms, &application, [&window, screenshot_path]() {
        const QPixmap pix = window.grab();
        const bool saved = pix.save(screenshot_path, "PNG");
        if (saved) {
          std::cout << "[screenshot] saved " << screenshot_path.toStdString()
                    << " (" << pix.width() << "x" << pix.height() << ")"
                    << std::endl;
        } else {
          std::cerr << "[screenshot] save failed: " << screenshot_path.toStdString()
                    << std::endl;
        }
        // Under a live high-rate DDS graph, Qt/rclcpp teardown can race after
        // the offscreen artifact is already complete. Screenshot mode owns no
        // child process or persistent write, so close through the same
        // crash-safe path used by SIGTERM and let the OS release DDS/GUI fds.
        std::_Exit(saved ? 0 : 1);
      });
    } else {
      window.show();
    }
    application.exec();
  }
  rclcpp::shutdown();
  return 0;
}
