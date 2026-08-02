#ifndef ADAS_GUI__FA_HPP_
#define ADAS_GUI__FA_HPP_

// Font Awesome 6 Free Solid 图标字体（私用区码点 → 图标）映射与加载。
// 字体文件通过 adas_gui.qrc 嵌入（:/adas_gui/assets/fonts/fa-solid-900.otf），
// 由 QFontDatabase::addApplicationFont 在 main.cpp 启动时加载，加载成功后
// 用 fontFamily() 取得实际家族名（"Font Awesome 6 Free"）插入风格回退链。
//
// fa::icon("play") → QString(QChar(0xf04b))，可直接 setText 到任意 QLabel。
// 与已经存在的图标字体（如系统安装的）冲突时，font-family 仅在 IconLabel/FA
// 渲染时启用，正常文本仍使用 CJK 字体。

#include <QString>
#include <QStringView>
#include <QHash>

namespace adas::gui::fa {

// Font Awesome 6 Free Solid 实际家族名（addApplicationFont 后由
// applicationFontFamilies() 返回；6.x 版本统一为该名）。
inline constexpr const char* kFamily = "Font Awesome 6 Free";

// 私用区前缀：FA6 solid 全部位于 U+F0000..U+F8FF 与扩展 U+E000..U+E5FF。
// 这里只嵌入项目实际使用到的图标子集，避免无谓编译。
inline QString icon(QStringView name) {
  static const QHash<QString, QString> table = {
    // 启动控制
    {QStringLiteral("play"),              QStringLiteral("\uf04b")},
    {QStringLiteral("stop"),              QStringLiteral("\uf04d")},
    {QStringLiteral("car"),               QStringLiteral("\uf1b9")},
    {QStringLiteral("sitemap"),           QStringLiteral("\uf0e8")},
    {QStringLiteral("circle-stop"),       QStringLiteral("\uf28d")},
    {QStringLiteral("folder-open"),        QStringLiteral("\uf07c")},
    // 地图工具栏
    {QStringLiteral("expand"),            QStringLiteral("\uf065")},
    {QStringLiteral("eraser"),            QStringLiteral("\uf12d")},
    {QStringLiteral("ban"),                QStringLiteral("\uf05e")},
    {QStringLiteral("location-crosshairs"),QStringLiteral("\uf601")},
    {QStringLiteral("map"),               QStringLiteral("\uf279")},
    {QStringLiteral("route"),             QStringLiteral("\uf4d7")},
    {QStringLiteral("crosshairs"),        QStringLiteral("\uf05b")},
    {QStringLiteral("compass"),            QStringLiteral("\uf14e")},
    // 安全仪表
    {QStringLiteral("microchip"),         QStringLiteral("\uf2db")},
    {QStringLiteral("gauge-high"),        QStringLiteral("\uf625")},
    {QStringLiteral("list-check"),        QStringLiteral("\uf0ae")},
    {QStringLiteral("lock"),              QStringLiteral("\uf023")},
    {QStringLiteral("bolt"),              QStringLiteral("\uf0e7")},
    {QStringLiteral("gears"),             QStringLiteral("\uf085")},
    {QStringLiteral("sliders"),           QStringLiteral("\uf1de")},
    {QStringLiteral("flag"),              QStringLiteral("\uf024")},
    {QStringLiteral("flag-checkered"),    QStringLiteral("\uf11e")},
    {QStringLiteral("clock"),             QStringLiteral("\uf017")},
    {QStringLiteral("circle-check"),      QStringLiteral("\uf058")},
    {QStringLiteral("circle-exclamation"),QStringLiteral("\uf06a")},
    {QStringLiteral("circle-info"),       QStringLiteral("\uf05a")},
    {QStringLiteral("triangle-exclamation"),QStringLiteral("\uf071")},
    {QStringLiteral("xmark"),             QStringLiteral("\uf00d")},
    {QStringLiteral("gear"),              QStringLiteral("\uf013")},
    {QStringLiteral("chart-line"),        QStringLiteral("\uf201")},
    {QStringLiteral("wifi"),              QStringLiteral("\uf1eb")},
    {QStringLiteral("satellite-dish"),    QStringLiteral("\uf7c0")},
    {QStringLiteral("tower-broadcast"),   QStringLiteral("\uf519")},
    {QStringLiteral("battery-full"),      QStringLiteral("\uf240")},
    {QStringLiteral("battery-half"),      QStringLiteral("\uf242")},
    {QStringLiteral("battery-empty"),     QStringLiteral("\uf244")},
    {QStringLiteral("signal"),            QStringLiteral("\uf012")},
    {QStringLiteral("road"),              QStringLiteral("\uf018")},
    {QStringLiteral("road-circle-check"),QStringLiteral("\ue564")},
    {QStringLiteral("road-circle-xmark"),QStringLiteral("\ue566")},
  };
  auto it = table.find(name.toString());
  return it != table.end() ? it.value() : QStringLiteral("?");
}

}  // namespace adas::gui::fa

#endif  // ADAS_GUI__FA_HPP_