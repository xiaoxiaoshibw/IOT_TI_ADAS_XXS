#include "icons.hpp"

#include <QHash>
#include <QPixmap>
#include <QPainter>
#include <QSvgRenderer>

namespace adas::gui::icons {
namespace {

// 单色实心 SVG：viewBox 24x24。颜色在 path 上写死；使用 #e4e6ea 默认浅色，
// 红色 #ff6b6b，绿色 #66bb6a，黄色 #f9a825。每张 SVG 是独立的字符串。
const QHash<QString, QString>& svg_table() {
  static const QHash<QString, QString> table = {
    {QStringLiteral("play"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#e4e6ea' d='M8 5v14l11-7z'/></svg>")},
    {QStringLiteral("stop"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<rect x='6' y='6' width='12' height='12' rx='2' fill='#ff6b6b'/></svg>")},
    {QStringLiteral("car"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#e4e6ea' d='M5 11l1.5-4.5A2 2 0 0 1 8.4 5h7.2a2 2 0 0 1 1.9 1.5L19 11"
     "h1a1 1 0 0 1 1 1v4a1 1 0 0 1-1 1h-1v1a1 1 0 0 1-1 1h-1a1 1 0 0 1-1-1v-1H9v1"
     "a1 1 0 0 1-1 1H7a1 1 0 0 1-1-1v-1H5a1 1 0 0 1-1-1v-4a1 1 0 0 1 1-1z'/>"
     "<path fill='#14181c' d='M7.5 11h9l-1-3H8.5z'/></svg>")},
    {QStringLiteral("sitemap"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#e4e6ea' stroke-width='1.8' stroke-linecap='round'>"
     "<rect x='9' y='3' width='6' height='4' rx='1' fill='#e4e6ea'/>"
     "<rect x='3' y='17' width='6' height='4' rx='1' fill='#e4e6ea'/>"
     "<rect x='9' y='17' width='6' height='4' rx='1' fill='#e4e6ea'/>"
     "<rect x='15' y='17' width='6' height='4' rx='1' fill='#e4e6ea'/>"
     "<path d='M12 7v4M4 17v-3h16v3M12 14v3'/></g></svg>")},
    {QStringLiteral("circle-stop"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='none' stroke='#ff6b6b' stroke-width='2'/>"
     "<rect x='8.5' y='8.5' width='7' height='7' rx='1.2' fill='#ff6b6b'/></svg>")},
    {QStringLiteral("folder-open"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#e4e6ea' d='M3 6a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v1H3z'/>"
     "<path fill='#a6c8ff' d='M3 9h18l-2 9a2 2 0 0 1-2 1H5a2 2 0 0 1-2-1z'/></svg>")},
    {QStringLiteral("sliders"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g stroke='#e4e6ea' stroke-width='1.6' stroke-linecap='round' fill='none'>"
     "<path d='M4 7h8M16 7h4M4 17h4M12 17h8'/></g>"
     "<g fill='#e4e6ea'><circle cx='13' cy='7' r='2.2'/>"
     "<circle cx='9' cy='17' r='2.2'/></g></svg>")},
    {QStringLiteral("signal"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='#e4e6ea'><rect x='3' y='14' width='3' height='6' rx='1'/>"
     "<rect x='8' y='10' width='3' height='10' rx='1'/>"
     "<rect x='13' y='6' width='3' height='14' rx='1'/>"
     "<rect x='18' y='3' width='3' height='17' rx='1'/></g></svg>")},
    {QStringLiteral("list-check"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#66bb6a' stroke-width='2' stroke-linecap='round'>"
     "<path d='M5 6l1.5 1.5L9 5M5 14l1.5 1.5L9 13'/></g>"
     "<g stroke='#e4e6ea' stroke-width='1.6' stroke-linecap='round' fill='none'>"
     "<path d='M12 6h7M12 14h7'/></g></svg>")},
    {QStringLiteral("microchip"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<rect x='7' y='7' width='10' height='10' rx='1.5' fill='#e4e6ea'/>"
     "<rect x='10' y='10' width='4' height='4' rx='0.5' fill='#14181c'/>"
     "<g stroke='#e4e6ea' stroke-width='1.5' stroke-linecap='round'>"
     "<path d='M4 9h3M4 12h3M4 15h3M17 9h3M17 12h3M17 15h3"
     "M9 4v3M12 4v3M15 4v3M9 17v3M12 17v3M15 17v3'/></g></svg>")},
    {QStringLiteral("gauge-high"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='none' stroke='#e4e6ea' stroke-width='2' stroke-linecap='round' "
     "d='M4 16a8 8 0 1 1 16 0'/>"
     "<path fill='#66bb6a' d='M12 16l5-4 1 1-5 4z'/>"
     "<circle cx='12' cy='16' r='1.6' fill='#e4e6ea'/></svg>")},
    {QStringLiteral("lock"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='none' stroke='#e4e6ea' stroke-width='2' d='M8 11V8a4 4 0 1 1 8 0v3'/>"
     "<rect x='5' y='11' width='14' height='9' rx='2' fill='#e4e6ea'/></svg>")},
    {QStringLiteral("bolt"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#f9a825' d='M13 2L4 14h6l-1 8 9-12h-6z'/></svg>")},
    {QStringLiteral("gears"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='#e4e6ea'><circle cx='9' cy='9' r='4'/>"
     "<circle cx='17' cy='15' r='3'/>"
     "<g stroke='#e4e6ea' stroke-width='1.4' stroke-linecap='round'>"
     "<path d='M9 3v2M9 13v2M3 9h2M13 9h2"
     "M5 5l1.4 1.4M11.6 11.6L13 13M5 13l1.4-1.4'/></g>"
     "<g stroke='#e4e6ea' stroke-width='1.3' stroke-linecap='round'>"
     "<path d='M17 11v1M17 18v1M14 15h1M19 15h1'/></g></g></svg>")},
    {QStringLiteral("shield"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#66bb6a' d='M12 3l8 3v6c0 5-3.5 8-8 9-4.5-1-8-4-8-9V6z'/>"
     "<path fill='none' stroke='#14181c' stroke-width='1.6' stroke-linecap='round' "
     "stroke-linejoin='round' d='M9 12l2 2 4-4'/></svg>")},
    {QStringLiteral("flag"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path stroke='#e4e6ea' stroke-width='1.8' fill='none' d='M6 3v18'/>"
     "<path fill='#e4e6ea' d='M6 4h12l-2 4 2 4H6z'/></svg>")},
    {QStringLiteral("flag-checkered"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path stroke='#e4e6ea' stroke-width='1.8' fill='none' d='M6 3v18'/>"
     "<path fill='#e4e6ea' d='M6 4h12v8H6z'/>"
     "<g fill='#14181c'><rect x='9' y='4' width='3' height='2'/>"
     "<rect x='15' y='4' width='3' height='2'/><rect x='6' y='6' width='3' height='2'/>"
     "<rect x='12' y='6' width='3' height='2'/><rect x='9' y='8' width='3' height='2'/>"
     "<rect x='15' y='8' width='3' height='2'/><rect x='6' y='10' width='3' height='2'/>"
     "<rect x='12' y='10' width='3' height='2'/></g></svg>")},
    {QStringLiteral("clock"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='none' stroke='#e4e6ea' stroke-width='2'/>"
     "<path d='M12 7v5l3 2' fill='none' stroke='#e4e6ea' stroke-width='2' "
     "stroke-linecap='round'/></svg>")},
    {QStringLiteral("circle-check"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='#66bb6a'/>"
     "<path fill='#14181c' d='M10 14.2L7.2 11.4l-1.4 1.4L10 17l8-8-1.4-1.4z'/></svg>")},
    {QStringLiteral("circle-exclamation"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='#f9a825'/>"
     "<path fill='#14181c' d='M11 7h2v6h-2zM11 15h2v2h-2z'/></svg>")},
    {QStringLiteral("circle-info"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='none' stroke='#a6c8ff' stroke-width='2'/>"
     "<path fill='#a6c8ff' d='M11 10h2v8h-2zM11 6h2v2h-2z'/></svg>")},
    {QStringLiteral("triangle-exclamation"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#ff6b6b' d='M12 3l10 18H2z'/>"
     "<path fill='#14181c' d='M11 9h2v6h-2zM11 16h2v2h-2z'/></svg>")},
    {QStringLiteral("xmark"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path stroke='#e4e6ea' stroke-width='2.4' stroke-linecap='round' "
     "d='M6 6l12 12M18 6L6 18'/></svg>")},
    {QStringLiteral("gear"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='#e4e6ea'><circle cx='12' cy='12' r='3.4'/>"
     "<path d='M12 2l1.6 3 1.4-0.4-0.4 3.2 3.2-0.4-0.4 1.4 3 1.6-3 1.6 0.4 1.4-3.2-0.4"
     "0.4 3.2-1.4-0.4L12 22l-1.6-3-1.4 0.4 0.4-3.2-3.2 0.4 0.4-1.4-3-1.6 3-1.6"
     "-0.4-1.4 3.2 0.4-0.4-3.2 1.4 0.4z'/></g></svg>")},
    {QStringLiteral("chart-line"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g stroke='#e4e6ea' stroke-width='1.8' fill='none' stroke-linecap='round'>"
     "<path d='M4 4v16h16'/></g>"
     "<path fill='#66bb6a' d='M7 14l3-3 3 2 5-6v8z'/></svg>")},
    {QStringLiteral("wifi"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#e4e6ea' stroke-width='1.8' stroke-linecap='round'>"
     "<path d='M2 8a16 16 0 0 1 20 0M5 12a11 11 0 0 1 14 0M8 16a6 6 0 0 1 8 0'/></g>"
     "<circle cx='12' cy='20' r='1.6' fill='#e4e6ea'/></svg>")},
    {QStringLiteral("expand"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g stroke='#e4e6ea' stroke-width='1.8' stroke-linecap='round' fill='none'>"
     "<path d='M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5'/></g></svg>")},
    {QStringLiteral("eraser"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#e4e6ea' d='M3 17L13 7l5 5-10 10H3z'/>"
     "<path fill='#a6c8ff' d='M13 7l4-4 5 5-4 4z'/></svg>")},
    {QStringLiteral("ban"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='none' stroke='#ff6b6b' stroke-width='2.4'/>"
     "<path stroke='#ff6b6b' stroke-width='2.4' stroke-linecap='round' "
     "d='M6 6l12 12'/></svg>")},
    {QStringLiteral("location-crosshairs"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#e4e6ea' stroke-width='1.6' stroke-linecap='round'>"
     "<circle cx='12' cy='12' r='3.5'/><path d='M12 2v3M12 19v3M2 12h3M19 12h3'/></g>"
     "<circle cx='12' cy='12' r='1.4' fill='#e4e6ea'/></svg>")},
    {QStringLiteral("crosshairs"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#e4e6ea' stroke-width='1.6'>"
     "<circle cx='12' cy='12' r='6'/><circle cx='12' cy='12' r='1'/></g>"
     "<g stroke='#e4e6ea' stroke-width='1.4' stroke-linecap='round'>"
     "<path d='M12 1v5M12 18v5M1 12h5M18 12h5'/></g></svg>")},
    {QStringLiteral("compass"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<circle cx='12' cy='12' r='9' fill='none' stroke='#e4e6ea' stroke-width='2'/>"
     "<path fill='#a6c8ff' d='M16 8l-5 2-1 5 5-2z'/></svg>")},
    {QStringLiteral("map"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#a6c8ff' d='M3 6l6-2 6 2 6-2v14l-6 2-6-2-6 2z'/>"
     "<path fill='#14181c' d='M9 4v14M15 6v14'/></svg>")},
    {QStringLiteral("route"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='#a6c8ff'><circle cx='6' cy='6' r='2.5'/><circle cx='18' cy='18' r='2.5'/></g>"
     "<path fill='none' stroke='#a6c8ff' stroke-width='2' stroke-linecap='round' "
     "d='M6 8.5c0 4 12 3 12 7.5'/></svg>")},
    {QStringLiteral("road"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='#e4e6ea' d='M5 3l-2 18h6L7 3zM19 3l-2 18h6L21 3z'/>"
     "<g stroke='#f9a825' stroke-width='1.4' fill='none'>"
     "<path d='M12 3v3M12 9v3M12 15v3'/></g></svg>")},
    {QStringLiteral("chevron-down"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='none' stroke='#e4e6ea' stroke-width='2' stroke-linecap='round' "
     "stroke-linejoin='round' d='M6 9l6 6 6-6'/></svg>")},
    {QStringLiteral("chevron-right"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='none' stroke='#e4e6ea' stroke-width='2' stroke-linecap='round' "
     "stroke-linejoin='round' d='M9 6l6 6-6 6'/></svg>")},
    {QStringLiteral("chevron-up"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='none' stroke='#e4e6ea' stroke-width='2' stroke-linecap='round' "
     "stroke-linejoin='round' d='M6 15l6-6 6 6'/></svg>")},
    {QStringLiteral("chevron-left"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<path fill='none' stroke='#e4e6ea' stroke-width='2' stroke-linecap='round' "
     "stroke-linejoin='round' d='M15 6l-6 6 6 6'/></svg>")},
    {QStringLiteral("battery-full"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<rect x='3' y='8' width='16' height='8' rx='1.6' fill='none' stroke='#e4e6ea' "
     "stroke-width='1.5'/><rect x='20' y='10' width='2' height='4' rx='0.6' fill='#e4e6ea'/>"
     "<rect x='4.6' y='9.6' width='12.8' height='4.8' rx='0.5' fill='#66bb6a'/></svg>")},
    {QStringLiteral("battery-half"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<rect x='3' y='8' width='16' height='8' rx='1.6' fill='none' stroke='#e4e6ea' "
     "stroke-width='1.5'/><rect x='20' y='10' width='2' height='4' rx='0.6' fill='#e4e6ea'/>"
     "<rect x='4.6' y='9.6' width='7' height='4.8' rx='0.5' fill='#f9a825'/></svg>")},
    {QStringLiteral("battery-empty"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<rect x='3' y='8' width='16' height='8' rx='1.6' fill='none' stroke='#ff6b6b' "
     "stroke-width='1.5'/><rect x='20' y='10' width='2' height='4' rx='0.6' fill='#e4e6ea'/></svg>")},
    {QStringLiteral("tower-broadcast"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#e4e6ea' stroke-width='1.6' stroke-linecap='round'>"
     "<path d='M5 19V8a3 3 0 0 1 6 0v11M5 19h6M5 13h6'/>"
     "<path d='M14 19V11a4 4 0 0 1 4 4'/></g>"
     "<circle cx='14' cy='15' r='1.2' fill='#e4e6ea'/></svg>")},
    {QStringLiteral("satellite-dish"),
     QStringLiteral("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'>"
     "<g fill='none' stroke='#e4e6ea' stroke-width='1.6' stroke-linecap='round'>"
     "<path d='M5 19c0-7 6-12 14-12M5 14c0-4 4-7 9-7M7 18a5 5 0 0 1 6-6'/></g>"
     "<circle cx='11' cy='13' r='1.5' fill='#e4e6ea'/></svg>")},
  };
  return table;
}

}  // namespace

QPixmap pixmap(QStringView name, int px, qreal dpr) {
  const auto& table = svg_table();
  const auto it = table.find(name.toString());
  if (it == table.end()) {
    return QPixmap();
  }
  QSvgRenderer renderer(it.value().toUtf8());
  QPixmap pm(QSize(px, px) * dpr);
  pm.setDevicePixelRatio(dpr);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing, true);
  renderer.render(&p, QRectF(0, 0, px, px));
  return pm;
}

QIcon get(QStringView name) {
  static QHash<QString, QIcon> cache;
  const QString key = name.toString();
  const auto it = cache.constFind(key);
  if (it != cache.constEnd()) return it.value();
  const QPixmap pm = pixmap(name, 24, 1.5);
  QIcon icon;
  if (!pm.isNull()) icon.addPixmap(pm);
  cache.insert(key, icon);
  return icon;
}

}  // namespace adas::gui::icons