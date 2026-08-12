// P0.C 会话隔离：每次 GUI 启动新的"运行会话"时生成一个 UUID v4 作为 run_id。
// SoC / Bridge / CARLA 链路只接受与之匹配的导航与地图消息。
// 同一 GUI 进程内的 run_id 自生成起稳定；GUI 重启或用户显式开新会话时重新生成。
// 空 run_id 从不构成握手，也不得被任何消费端视为通配符。
//
// 线程模型：所有方法都假定在 Qt GUI 主线程里调用。begin()/end() 仅在启动
// 与显式开新会话时触发；current()/current_std() 每次构造 ROS 请求时被
// RosBridge 调用，频率远低于主消息循环，因此不需要 atomic。

#pragma once

#include <QString>
#include <QUuid>

#include <string>

namespace adas_gui {

class RunIdSession {
 public:
  // 生成新的 UUID v4 作为当前 run_id。
  static QString begin() {
    const QString id = generate_uuid_v4();
    current_ = id;
    return id;
  }

  // 显式结束当前会话。后续 current() 返回空串，消费者视为会话结束。
  static void end() {
    current_.clear();
  }

  // 当前 run_id。空 = 当前进程尚未启动新会话。
  static QString& mutable_current() { return current_; }
  static QString current() { return current_; }

  // std::string 形式，便于直接写入 ROS msg 的 string 字段。
  static std::string current_std() {
    return current_.toStdString();
  }

 private:
  static QString generate_uuid_v4() {
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
  }
  static QString current_;
};

}  // namespace adas_gui
