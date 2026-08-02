#ifndef ADAS_GUI__ICONS_HPP_
#define ADAS_GUI__ICONS_HPP_

// 内嵌 SVG 图标库，替代 Font Awesome 字体（在 Ubuntu 上字符形如 ☒ 易渲染失败）。
// 命名沿用旧 fa:: 字典键名，便于上游调用点最小改动：icons::get("microchip") 等。
// SVG 在 .qrc 内以文本资源携带，运行时由 QSvgRenderer 转 QPixmap，不依赖字体。
// 颜色固定为暗色主题下的浅色（#e4e6ea）；危险类用 #ef9a9a；正常类 #66bb6a。

#include <QIcon>
#include <QString>
#include <QStringView>

class QPixmap;

namespace adas::gui::icons {

// 取 24x24 视口的图标；按设备像素比渲染，缓存复用。
QIcon get(QStringView name);

// 直接取单色 pixmap（用于自绘控件 IconLabel 等）。
QPixmap pixmap(QStringView name, int px, qreal device_pixel_ratio = 1.0);

}  // namespace adas::gui::icons

#endif  // ADAS_GUI__ICONS_HPP_