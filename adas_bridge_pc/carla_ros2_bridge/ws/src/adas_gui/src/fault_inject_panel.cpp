#include "fault_inject_panel.hpp"

#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>

#include "format.hpp"
#include "theme.hpp"

namespace adas::gui {
namespace {

// 按钮在网格里水平伸展填满单元格（Expanding 横向策略）；纵向保持固定高度
// 防止 panel 高度变化时按钮被拉伸或挤压。
QPushButton* make_inject_button(const QString& text, const QString& tooltip,
                                QWidget* parent) {
  auto* btn = new QPushButton(text, parent);
  btn->setToolTip(tooltip);
  btn->setMinimumHeight(34);
  btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  btn->setStyleSheet(QStringLiteral(
      "QPushButton{background:%1;color:%2;border:1px solid %3;border-radius:6px;"
      "font-size:12px;font-weight:600;padding:6px 8px;}"
      "QPushButton:hover{background:%4;}"
      "QPushButton:pressed{background:%5;}")
      .arg(theme::kMdSurfaceContainerHigh, theme::kMdOnSurface,
           theme::kCardBorder, theme::kMdPrimaryContainer,
           theme::kMdPrimary));
  return btn;
}

// 高危注入按钮（FORCE_* / DROP_ALL / INJECT_SELF_TEST_FAIL）在执行前要二次
// 确认：bench 上单击 = 0x301 帧 → MCU 即时动作（急停/锁死/FAILSAFE）。
// 中性按钮（CORRUPT_CRC / FREEZE_SEQ / RECOVER / CLEAR）仅触发观测类状态，
// 不需要二次确认。
bool confirm_dangerous(const QString& title, const QString& body) {
  QMessageBox box;
  box.setIcon(QMessageBox::Warning);
  box.setWindowTitle(title);
  box.setText(QStringLiteral("<b>%1</b>").arg(title));
  box.setInformativeText(body);
  box.setStandardButtons(QMessageBox::Cancel | QMessageBox::Yes);
  box.setDefaultButton(QMessageBox::Cancel);
  box.button(QMessageBox::Yes)->setText(QStringLiteral("确认注入"));
  box.button(QMessageBox::Cancel)->setText(QStringLiteral("取消"));
  return box.exec() == QMessageBox::Yes;
}

}  // namespace

FaultInjectPanel::FaultInjectPanel(InjectCallback callback, QWidget* parent)
    : QWidget(parent), callback_(std::move(callback)) {
  build_ui();
}

void FaultInjectPanel::build_ui() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  // 标题 + 警示（始终可见，不进 scroll，让用户任何时候都看到当前面板说明）
  auto* title_box = new QGroupBox(QStringLiteral("故障注入（0x301）"), this);
  auto* title_layout = new QVBoxLayout(title_box);
  auto* title_label = new QLabel(QStringLiteral(
      "桥接节点通过 PC CANalyst-II 发 0x301 帧到 MCU；"
      "仅当 MCU 烧录 ADAS_TEST_BUILD=1 时生效，生产固件忽略。"));
  title_label->setWordWrap(true);
  title_label->setStyleSheet(QStringLiteral("color:%1;font-size:10px;")
                                 .arg(theme::kTextSecondary));
  title_layout->addWidget(title_label);
  root->addWidget(title_box);

  // 按钮矩阵：包到 QScrollArea，纵向空间不足时可滚，避免按钮被剪
  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* scroll_content = new QWidget();
  auto* scroll_layout = new QVBoxLayout(scroll_content);
  scroll_layout->setContentsMargins(0, 0, 0, 0);
  scroll_layout->setSpacing(4);

  // 按钮 label 缩成纯中文，英文名 + INJ_CMD_xx 进 tooltip（避免 145px 按钮
  // 里塞不下"主源停止 (DROP_CTRL)"被 ellipsis）。危险按钮的强制语义保留在
  // tooltip，方便与 fault_catalog 对照。
  btn_drop_ctrl_ = make_inject_button(
      QStringLiteral("主源停止"),
      QStringLiteral("INJ_CMD_DROP_CTRL = 1\n60 ms 后 MCU 应切源/FAILSAFE"), this);
  btn_freeze_seq_ = make_inject_button(
      QStringLiteral("SEQ 冻结"),
      QStringLiteral("INJ_CMD_FREEZE_SEQ = 2\n5 帧后 MCU 应报 seq_stall"), this);
  btn_corrupt_crc_ = make_inject_button(
      QStringLiteral("CRC 错"),
      QStringLiteral("INJ_CMD_CORRUPT_CRC = 3\nCRC 错误计数累加"), this);
  btn_force_mrm_ = make_inject_button(
      QStringLiteral("强制 MRM"),
      QStringLiteral("INJ_CMD_FORCE_MRM = 4\n立即进入 MRM 4.8 m/s² 减速"), this);
  btn_force_estop_ = make_inject_button(
      QStringLiteral("强制急停"),
      QStringLiteral("INJ_CMD_FORCE_EMERGENCY = 5\n立即全力制动"), this);
  btn_force_lock_ = make_inject_button(
      QStringLiteral("强制 FAULT_LOCK"),
      QStringLiteral("INJ_CMD_FORCE_FAULT_LOCK = 6\n闩锁；需重启恢复"), this);
  btn_drop_all_ = make_inject_button(
      QStringLiteral("双源失效"),
      QStringLiteral("INJ_CMD_DROP_ALL = 7\nMCU 进入 FAILSAFE"), this);
  btn_inject_st_fail_ = make_inject_button(
      QStringLiteral("自检失败"),
      QStringLiteral("INJ_CMD_INJECT_SELF_TEST_FAIL = 9\n强制 FAULT_LOCK"), this);
  btn_recover_ = make_inject_button(
      QStringLiteral("手动恢复"),
      QStringLiteral("INJ_CMD_RECOVER = 8\n解除注入并清除非自检故障"), this);
  btn_clear_ = make_inject_button(
      QStringLiteral("清空日志"),
      QStringLiteral("仅清空本地显示"), this);

  auto* grid = new QGridLayout();
  grid->setSpacing(4);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->addWidget(btn_drop_ctrl_,     0, 0);
  grid->addWidget(btn_freeze_seq_,    0, 1);
  grid->addWidget(btn_corrupt_crc_,   1, 0);
  grid->addWidget(btn_force_mrm_,     1, 1);
  grid->addWidget(btn_force_estop_,   2, 0);
  grid->addWidget(btn_force_lock_,    2, 1);
  grid->addWidget(btn_drop_all_,      3, 0);
  grid->addWidget(btn_inject_st_fail_, 3, 1);
  grid->addWidget(btn_recover_,       4, 0);
  grid->addWidget(btn_clear_,         4, 1);
  scroll_layout->addLayout(grid);
  scroll->setWidget(scroll_content);
  root->addWidget(scroll, 1);  // stretch=1：随右栏高度伸展

  // 日志：留在 scroll 外、随高度伸展（panel 高时多占、低时收到 minHeight）
  log_view_ = new QTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMinimumHeight(80);
  log_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  log_view_->setStyleSheet(QStringLiteral(
      "QTextEdit{background:%1;color:%2;border:1px solid %3;border-radius:6px;"
      "font-family:'Courier New',monospace;font-size:10px;}")
      .arg(theme::kMdSurfaceContainerHigh, theme::kMdOnSurface,
           theme::kCardBorder));
  root->addWidget(log_view_, 1);  // 与 scroll 平分额外高度

  // 信号
  connect(btn_drop_ctrl_, &QPushButton::clicked, this, [this]() {
    publish_inject_command(1, 0, QStringLiteral("主源停止"));
  });
  connect(btn_freeze_seq_, &QPushButton::clicked, this, [this]() {
    publish_inject_command(2, 0, QStringLiteral("SEQ 冻结"));
  });
  connect(btn_corrupt_crc_, &QPushButton::clicked, this, [this]() {
    publish_inject_command(3, 0, QStringLiteral("CRC 错"));
  });
  connect(btn_force_mrm_, &QPushButton::clicked, this, [this]() {
    if (!confirm_dangerous(QStringLiteral("强制 MRM"),
                          QStringLiteral("将向 MCU 发 INJ_CMD_FORCE_MRM = 4；"
                                         "车辆立即进入 4.8 m/s² 减速的最小风险机动（MRM）。"
                                         "继续注入？"))) {
      log_event(QStringLiteral("× 取消：强制 MRM"));
      return;
    }
    publish_inject_command(4, 0, QStringLiteral("强制 MRM"));
  });
  connect(btn_force_estop_, &QPushButton::clicked, this, [this]() {
    if (!confirm_dangerous(QStringLiteral("强制急停"),
                          QStringLiteral("将向 MCU 发 INJ_CMD_FORCE_EMERGENCY = 5；"
                                         "车辆立即全力制动（emergency brake）。"
                                         "继续注入？"))) {
      log_event(QStringLiteral("× 取消：强制急停"));
      return;
    }
    publish_inject_command(5, 0, QStringLiteral("强制急停"));
  });
  connect(btn_force_lock_, &QPushButton::clicked, this, [this]() {
    if (!confirm_dangerous(QStringLiteral("强制 FAULT_LOCK"),
                          QStringLiteral("将向 MCU 发 INJ_CMD_FORCE_FAULT_LOCK = 6；"
                                         "MCU 进入闩锁态，必须重启才能恢复。"
                                         "继续注入？"))) {
      log_event(QStringLiteral("× 取消：强制 FAULT_LOCK"));
      return;
    }
    publish_inject_command(6, 0, QStringLiteral("强制 FAULT_LOCK"));
  });
  connect(btn_drop_all_, &QPushButton::clicked, this, [this]() {
    if (!confirm_dangerous(QStringLiteral("双源失效"),
                          QStringLiteral("将向 MCU 发 INJ_CMD_DROP_ALL = 7；"
                                         "主/备源同时停止心跳，MCU 进入 FAILSAFE。"
                                         "继续注入？"))) {
      log_event(QStringLiteral("× 取消：双源失效"));
      return;
    }
    publish_inject_command(7, 0, QStringLiteral("双源失效"));
  });
  connect(btn_inject_st_fail_, &QPushButton::clicked, this, [this]() {
    if (!confirm_dangerous(QStringLiteral("自检失败注入"),
                          QStringLiteral("将向 MCU 发 INJ_CMD_INJECT_SELF_TEST_FAIL = 9；"
                                         "强制 MCU 报告自检失败并进入 FAULT_LOCK。"
                                         "继续注入？"))) {
      log_event(QStringLiteral("× 取消：自检失败"));
      return;
    }
    publish_inject_command(9, 0, QStringLiteral("自检失败"));
  });
  connect(btn_recover_, &QPushButton::clicked, this, [this]() {
    publish_inject_command(8, 0, QStringLiteral("手动恢复"));
  });
  connect(btn_clear_, &QPushButton::clicked, this, &FaultInjectPanel::onClearClicked);

  log_event(QStringLiteral("面板就绪 — 等待桥节点订阅 /adas/_debug/fault_inject_cmd"));
}

void FaultInjectPanel::onClearClicked() {
  log_view_->clear();
}

void FaultInjectPanel::publish_inject_command(int cmd, int param,
                                              const QString& label) {
  if (callback_) callback_(cmd, param, label);
  log_event(QStringLiteral("→ INJ cmd=%1 param=%2 label=%3")
                .arg(cmd)
                .arg(param)
                .arg(label));
}

void FaultInjectPanel::log_event(const QString& line) {
  const auto ts = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
  log_view_->append(QStringLiteral("[%1] %2").arg(ts, line));
}

}  // namespace adas::gui