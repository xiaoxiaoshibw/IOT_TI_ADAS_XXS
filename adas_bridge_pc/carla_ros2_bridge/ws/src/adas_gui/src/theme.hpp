#ifndef ADAS_GUI__THEME_HPP_
#define ADAS_GUI__THEME_HPP_

// MD3（Material Design 3）暗色风格主题：所有颜色/样式只在这里定义，各控件
// 不再散落字面量。语义分级色与 format.hpp::state_color 一一对应：
//   绿=正常、黄=降级/预警、红=危险/闭锁、灰=无数据。
// 兼容层：保留旧常量名（kOk/kWarn/kDanger/kStale/kAccent/kWindowBg/kCardBg
// kCardBorder/kMapBg/kTextPrimary/kTextSecondary/kLaneJunction/kEgo/kEgoTrail
// kGoal/kSnapPreview 等）与 value_style/banner_style 签名，避免下游 139+ 处
// 调用点改动；值与旧版一致，状态机染色不会因切换风格而变。

#include <QColor>
#include <QString>

namespace adas::gui::theme {

// ====== 语义分级色（与 format.hpp::state_color 字面一致，勿乱改） ======
inline constexpr const char* kOk = "#2e7d32";        // 正常（绿）
inline constexpr const char* kWarn = "#f9a825";     // 降级/预警（黄）
inline constexpr const char* kDanger = "#c62828";   // 危险/闭锁（红）
inline constexpr const char* kStale = "#616161";    // 无数据/断流（灰）
inline constexpr const char* kAccent = "#42a5f5";    // 路线/强调（蓝）

// ====== 界面基色（保留旧名，值随 MD3 dark scheme 微调） ======
inline constexpr const char* kWindowBg = "#0b0f14";
inline constexpr const char* kCardBg = "#151b21";
inline constexpr const char* kCardBorder = "#27313a";
inline constexpr const char* kMapBg = "#0d1419";
inline constexpr const char* kTextPrimary = "#edf3f7";
inline constexpr const char* kTextSecondary = "#91a0ad";

// 地图元素（不变）
inline constexpr const char* kLaneNormal = "#5c6370";
inline constexpr const char* kLaneJunction = "#6d5d2a";
inline constexpr const char* kEgo = "#66bb6a";
inline constexpr const char* kEgoTrail = "#33691e";
inline constexpr const char* kGoal = "#ef5350";
inline constexpr const char* kSnapPreview = "#80cbc4";

// ====== MD3 dark scheme token 扩展 ======
inline constexpr const char* kMdPrimary = "#66ddd2";
inline constexpr const char* kMdOnPrimary = "#04201e";
inline constexpr const char* kMdPrimaryContainer = "#123b3d";
inline constexpr const char* kMdOnPrimaryContainer = "#b8f4ee";
inline constexpr const char* kMdSecondary = "#aebdc8";
inline constexpr const char* kMdSecondaryContainer = "#202a32";
inline constexpr const char* kMdOnSecondaryContainer = "#d8e4eb";
inline constexpr const char* kMdTertiary = "#d8bc78";
inline constexpr const char* kMdTertiaryContainer = "#3a3020";
inline constexpr const char* kMdError = "#ffb4ab";
inline constexpr const char* kMdOnError = "#690005";
inline constexpr const char* kMdErrorContainer = "#93000a";
inline constexpr const char* kMdOnErrorContainer = "#ffdad6";
inline constexpr const char* kMdSurface = "#0b0f14";
inline constexpr const char* kMdSurfaceContainerLow = "#10161b";
inline constexpr const char* kMdSurfaceContainer = "#151b21";
inline constexpr const char* kMdSurfaceContainerHigh = "#1b242b";
inline constexpr const char* kMdSurfaceContainerHighest = "#253039";
inline constexpr const char* kMdOutline = "#3a4853";
inline constexpr const char* kMdOutlineVariant = "#27313a";
inline constexpr const char* kMdOnSurface = "#edf3f7";
inline constexpr const char* kMdOnSurfaceVariant = "#bac6ce";
inline constexpr const char* kMdInverseSurface = "#f4f7f9";

// 用于状态 chip 容器：分级色淡化（容器色），保留语义而不刺眼
inline QString color_with_alpha(const char* hex, int alpha) {
  QColor c = QColor::fromString(QString::fromLatin1(hex));
  if (!c.isValid()) return QString::fromLatin1(hex);
  return QString("rgba(%1,%2,%3,%4)")
      .arg(c.red()).arg(c.green()).arg(c.blue()).arg(alpha);
}

inline QString value_style(const char* color, bool bold = false) {
  return QString("color:%1;%2").arg(color, bold ? "font-weight:bold;" : "");
}

inline QString banner_style(const QString& background, const char* text = "white") {
  return QString(
      "background:%1;color:%2;font-size:20px;font-weight:700;border-radius:8px;"
      "padding:8px 18px;"
      "letter-spacing:0px;")
      .arg(background, text);
}

// MD3 字号 scale（ldpi 桌面）：title 22 / headline-7 18 / label-large 14 / body 13
inline QString type_scale(const char* role) {
  QString base = "color:%1;";
  if (QString::fromLatin1(role) == "title")            return base + "font-size:18px;font-weight:600;letter-spacing:0px;";
  if (QString::fromLatin1(role) == "headline")         return base + "font-size:16px;font-weight:600;";
  if (QString::fromLatin1(role) == "label-large")       return base + "font-size:14px;font-weight:600;letter-spacing:0px;";
  if (QString::fromLatin1(role) == "label")            return base + "font-size:12px;font-weight:600;letter-spacing:0px;";
  if (QString::fromLatin1(role) == "body")             return base + "font-size:13px;";
  if (QString::fromLatin1(role) == "body-small")       return base + "font-size:11px;";
  if (QString::fromLatin1(role) == "speed-big")        return base + "font-size:32px;font-weight:700;";
  if (QString::fromLatin1(role) == "speed-unit")       return base + "font-size:11px;font-weight:600;letter-spacing:0px;";
  if (QString::fromLatin1(role) == "state-name")        return base + "font-size:16px;font-weight:600;letter-spacing:0px;";
  return base + "font-size:13px;";
}

// MD3 chip：圆角 pill 形状态 chip 用样式表，state=ok/warn/danger/stale/primary/neutral
inline QString chip_style(const QString& state_token, bool filled = false) {
  QString bg, fg, border;
  if (state_token == "ok") { bg = color_with_alpha("#2e7d32", filled ? 180 : 70);  fg = "#a5d6a7"; border = "#2e7d32"; }
  else if (state_token == "warn")    { bg = color_with_alpha("#f9a825", filled ? 180 : 70);  fg = "#ffe082"; border = "#f9a825"; }
  else if (state_token == "danger")  { bg = color_with_alpha("#c62828", filled ? 220 : 90);  fg = "#ef9a9a"; border = "#c62828"; }
  else if (state_token == "stale")   { bg = color_with_alpha("#616161", filled ? 200 : 60);  fg = "#bdbdbd"; border = "#616161"; }
  else if (state_token == "primary") { bg = color_with_alpha("#00497c", filled ? 220 : 90); fg = "#a6c8ff"; border = "#00497c"; }
  else /*neutral*/                   { bg = color_with_alpha("#44474e", filled ? 200 : 70);  fg = "#c3c6cf"; border = "#44474e"; }
  return QString(
        "QFrame#StatusChip, QLabel { background:%1; color:%2; border:1px solid %3;"
        "border-radius:6px; padding:5px 10px;"
        "font-size:13px; font-weight:600; }")
      .arg(bg, fg, border);
}

// 全局样式表：卡片化 QGroupBox、MD3 按钮、暗色表单/列表/标签页。
inline QString app_style_sheet() {
  return QString(R"(
QMainWindow, QWidget#root, QWidget#sidebarContent {
  background:%1; color:%2;
}
QWidget { letter-spacing:0px; }
QLabel { background:transparent; color:%2; }
QScrollArea, QScrollArea > QWidget > QWidget {
  background:transparent; border:none;
}

/* Operational cards: compact, flat and easy to scan. */
QGroupBox {
  background:%3; border:1px solid %4; border-radius:8px;
  margin:0; padding:0; font-size:13px; font-weight:600;
}
#sectionIcon QLabel, QLabel#sectionIcon { color:%6; }

QListWidget, QPlainTextEdit, QComboBox, QSpinBox, QLineEdit {
  background:%7; border:1px solid %4; border-radius:6px; color:%2;
  padding:5px 8px; min-height:24px;
  selection-background-color:%8; selection-color:%2;
}
QListWidget { padding:2px; }
QListWidget::item { border-radius:4px; padding:6px 8px; }
QListWidget::item:selected { background:%8; color:%2; }
QComboBox:hover, QSpinBox:hover, QLineEdit:hover { border-color:%6; }
QComboBox:focus, QSpinBox:focus, QLineEdit:focus { border:1px solid %6; }
QComboBox QAbstractItemView {
  background:%7; color:%2; border:1px solid %4; border-radius:6px;
  padding:4px; selection-background-color:%8;
}
QComboBox::drop-down { border:none; width:22px; }
QSpinBox::up-button, QSpinBox::down-button {
  width:14px; border:none; background:transparent;
}
QLineEdit { padding:6px 9px; }

QPushButton {
  background:%9; border:1px solid %4; border-radius:6px;
  padding:7px 10px; color:%2; font-weight:600; min-height:22px;
}
QPushButton:hover { background:%10; border-color:%6; }
QPushButton:pressed { background:%7; }
QPushButton:disabled {
  color:#66737e; background:#12181d; border-color:#202830;
}
QPushButton#primaryButton {
  background:%6; border-color:%6; color:#04201e;
}
QPushButton#primaryButton:hover { background:#83e8df; border-color:#83e8df; }
QPushButton#primaryButton:pressed { background:#45bfb5; border-color:#45bfb5; }
QPushButton#secondaryButton { background:%9; border-color:%4; color:%2; }
QPushButton#secondaryButton:hover { background:%10; }
QPushButton#secondaryButton:pressed { background:%7; }
QPushButton#outlinedButton {
  background:transparent; border:1px solid %4; color:%5; border-radius:6px;
}
QPushButton#outlinedButton:hover { background:%9; color:%2; border-color:%6; }
QPushButton#outlinedButton:pressed { background:%7; }
QPushButton#iconButton {
  background:transparent; border:1px solid transparent; border-radius:6px;
  padding:5px; min-width:24px; min-height:24px; color:%5;
}
QPushButton#iconButton:hover { background:%9; border-color:%4; color:%2; }
QPushButton#iconButton:pressed { background:%7; }
QPushButton#textButton {
  background:transparent; border:none; color:%6; padding:6px 8px;
}
QPushButton#textButton:hover { background:%9; }
QPushButton#dangerButton {
  background:#321c20; border-color:#6b3037; color:#ffb8bd;
}
QPushButton#dangerButton:hover { background:#4a2228; border-color:#a94a55; }
QPushButton#dangerButton:pressed { background:#251418; }
QPushButton#dangerButton:disabled {
  background:#171416; color:#665a5d; border-color:#292326;
}

QProgressBar {
  background:%7; border:1px solid %4; border-radius:5px;
  text-align:center; color:%2; min-height:18px; padding:0;
}
QProgressBar::chunk { border-radius:4px; margin:1px; background:%6; }
QCheckBox { spacing:7px; color:%2; }
QCheckBox::indicator {
  width:16px; height:16px; border-radius:4px;
  border:1px solid %4; background:%7;
}
QCheckBox::indicator:checked { background:%6; border-color:%6; }
QRadioButton::indicator {
  width:16px; height:16px; border-radius:8px;
  border:1px solid %4; background:%7;
}
QRadioButton::indicator:checked { background:%6; border-color:%6; }

QSplitter::handle { background:%1; }
QSplitter::handle:hover { background:%6; }
QStatusBar {
  background:%3; border-top:1px solid %4; color:%5; min-height:28px;
}
QStatusBar::item { border:none; }
QStatusBar QLabel { padding:2px 7px; }

QTabWidget::pane {
  border:1px solid %4; border-radius:8px; background:%3; padding:3px;
}
QTabBar { background:transparent; }
QTabBar::tab {
  background:transparent; border:none; border-bottom:2px solid transparent;
  padding:7px 14px; margin:0 2px; color:%5;
  font-weight:600; font-size:12px;
}
QTabBar::tab:selected { color:%2; border-bottom:2px solid %6; }
QTabBar::tab:!selected:hover { color:%2; }
QToolTip {
  background:%10; color:%2; border:1px solid %4;
  border-radius:6px; padding:5px 8px;
}

QScrollBar:vertical { background:transparent; width:8px; margin:2px; }
QScrollBar::handle:vertical { background:%4; border-radius:3px; min-height:24px; }
QScrollBar::handle:vertical:hover { background:%5; }
QScrollBar::add-line, QScrollBar::sub-line { height:0; width:0; }
QScrollBar:horizontal { background:transparent; height:8px; margin:2px; }
QScrollBar::handle:horizontal { background:%4; border-radius:3px; min-width:24px; }
QScrollBar::handle:horizontal:hover { background:%5; }
)")
      .arg(kWindowBg, kTextPrimary, kCardBg, kCardBorder, kTextSecondary,
           kMdPrimary,
           kMdSurfaceContainerLow,
           kMdPrimaryContainer,
           kMdSecondaryContainer,
           kMdSurfaceContainerHigh);
}

}  // namespace adas::gui::theme

#endif  // ADAS_GUI__THEME_HPP_
