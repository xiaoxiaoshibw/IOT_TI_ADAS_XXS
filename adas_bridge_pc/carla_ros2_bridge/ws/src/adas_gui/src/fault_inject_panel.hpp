#ifndef ADAS_GUI__FAULT_INJECT_PANEL_HPP_
#define ADAS_GUI__FAULT_INJECT_PANEL_HPP_

// 故障注入面板（审计整改 TOP10-2）：把 0x301 帧注入入口嵌入 GUI。
// 通过回调函数向桥节点发送 JSON 命令 `{"cmd":N,"param":N,"label":"...",...}`；
// 桥节点订阅 `/adas/_debug/fault_inject_cmd` 后通过 PC CANalyst-II 发送 0x301 帧
// 到 MCU。当前 GUI 端只负责构造命令并发出；具体链路在后续版本由 bridge_node
// 订阅实现。
//
// INJ_CMD 定义参考 `ADAS0.0.2/MCU/include/adas_can_protocol.h:275-289`
//   INJ_CMD_NONE                = 0
//   INJ_CMD_DROP_CTRL           = 1   （CAN 掉线 / 主源停止）
//   INJ_CMD_FREEZE_SEQ          = 2   （SEQ 冻结）
//   INJ_CMD_CORRUPT_CRC         = 3   （CRC 错）
//   INJ_CMD_FORCE_MRM           = 4   （强制 MRM）
//   INJ_CMD_FORCE_EMERGENCY     = 5   （强制急停）
//   INJ_CMD_FORCE_FAULT_LOCK    = 6   （强制 FAULT_LOCK）
//   INJ_CMD_DROP_ALL            = 7   （双源失效）
//   INJ_CMD_RECOVER             = 8   （手动恢复）
//   INJ_CMD_INJECT_SELF_TEST_FAIL = 9 （自检失败）
//
// 视觉：4 个一级按钮 + 4 个二级按钮（按"安全等级"分组），下方实时显示注入历史。

#include <QGroupBox>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QWidget>

#include <functional>

namespace adas::gui {

class FaultInjectPanel : public QWidget {
  Q_OBJECT

 public:
  using InjectCallback = std::function<void(int cmd, int param, const QString& label)>;

  explicit FaultInjectPanel(InjectCallback callback, QWidget* parent = nullptr);
  ~FaultInjectPanel() override = default;

 private slots:
  void onClearClicked();

 private:
  void build_ui();
  void publish_inject_command(int cmd, int param, const QString& label);
  void log_event(const QString& line);

  InjectCallback callback_;

  QPushButton* btn_drop_ctrl_;       // 1: 主源停止
  QPushButton* btn_freeze_seq_;      // 2: SEQ 冻结
  QPushButton* btn_corrupt_crc_;     // 3: CRC 错
  QPushButton* btn_force_mrm_;       // 4: 强制 MRM
  QPushButton* btn_force_estop_;     // 5: 强制急停
  QPushButton* btn_force_lock_;      // 6: 强制 FAULT_LOCK
  QPushButton* btn_drop_all_;        // 7: 双源失效
  QPushButton* btn_inject_st_fail_;  // 9: 自检失败
  QPushButton* btn_recover_;         // 8: 手动恢复
  QPushButton* btn_clear_;
  QTextEdit* log_view_;
};

}  // namespace adas::gui

#endif  // ADAS_GUI__FAULT_INJECT_PANEL_HPP_