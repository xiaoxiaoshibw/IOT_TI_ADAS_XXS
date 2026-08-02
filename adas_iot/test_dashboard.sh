#!/usr/bin/env bash
# ADAS IoT 一键启动脚本
# =====================
# 启动 MQTT 桥接器（数据源） + Web Dashboard
#
# 使用方法：
#   ./test_dashboard.sh              # 启动完整演示（桥接器 + Dashboard）
#   ./test_dashboard.sh --dashboard  # 仅启动 Dashboard（需要外部 MQTT 数据源）
#   ./test_dashboard.sh --bridge     # 仅启动桥接器（模拟模式）
#   ./test_dashboard.sh --help       # 帮助信息

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 颜色
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_banner() {
    echo -e "${CYAN}"
    echo '    ╔══════════════════════════════════════════════╗'
    echo '    ║        ADAS 物联网远程监控平台 v1.0          ║'
    echo '    ║   ADAS IoT Remote Monitoring Platform        ║'
    echo '    ╚══════════════════════════════════════════════╝'
    echo -e "${NC}"
}

print_usage() {
    echo "使用方法: $0 [OPTION]"
    echo "选项:"
    echo "  --dashboard    仅启动 Web Dashboard（需外部 MQTT 数据源）"
    echo "  --bridge       仅启动 MQTT 桥接器（模拟模式）"
    echo "  --help         显示帮助信息"
    echo ""
    echo "默认: 同时启动桥接器和 Dashboard（完整演示模式）"
}

# 检查依赖
check_deps() {
    local missing=0
    for pkg in flask flask_socketio eventlet paho_mqtt; do
        if ! python3 -c "import ${pkg%_*}" 2>/dev/null; then
            echo -e "${YELLOW}⚠ 缺少依赖: $pkg${NC}"
            missing=1
        fi
    done
    if [ $missing -eq 1 ]; then
        echo -e "${YELLOW}安装依赖中...${NC}"
        pip3 install -r requirements.txt --break-system-packages 2>&1 | tail -3
    fi
}

# 启动桥接器（后台）
start_bridge() {
    echo -e "${GREEN}▶ 启动 MQTT 桥接器（模拟模式）...${NC}"
    python3 mqtt_bridge.py --sim --config config.yaml &
    BRIDGE_PID=$!
    echo -e "   PID: $BRIDGE_PID"
    sleep 2
}

# 启动 Dashboard
start_dashboard() {
    echo -e "${GREEN}▶ 启动 Web Dashboard...${NC}"
    cd dashboard
    python3 app.py &
    DASH_PID=$!
    echo -e "   PID: $DASH_PID"
    cd ..
}

# 清理
cleanup() {
    echo ""
    echo -e "${YELLOW}⏹  正在关闭服务...${NC}"
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null && echo "   MQTT 桥接器已停止"
    [ -n "$DASH_PID" ] && kill $DASH_PID 2>/dev/null && echo "   Dashboard 已停止"
    echo -e "${GREEN}✓ 已全部关闭${NC}"
    exit 0
}

# ===== 主流程 =====
print_banner

# 参数解析
MODE="both"
case "$1" in
    --dashboard) MODE="dashboard" ;;
    --bridge)    MODE="bridge" ;;
    --help|-h)   print_usage; exit 0 ;;
    "")          MODE="both" ;;
    *)           echo -e "${RED}未知参数: $1${NC}"; print_usage; exit 1 ;;
esac

check_deps

# 注册清理
trap cleanup SIGINT SIGTERM

if [ "$MODE" = "both" ] || [ "$MODE" = "bridge" ]; then
    start_bridge
fi

if [ "$MODE" = "both" ] || [ "$MODE" = "dashboard" ]; then
    start_dashboard
fi

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   ✅ 系统运行中                              ║${NC}"
echo -e "${GREEN}║                                            ║${NC}"
echo -e "${GREEN}║   📊 Dashboard: http://localhost:5000      ║${NC}"
echo -e "${GREEN}║   📡 MQTT Broker: broker.emqx.io:1883     ║${NC}"
echo -e "${GREEN}║   🔑 Topic: adas/v1/#                       ║${NC}"
echo -e "${GREEN}║                                            ║${NC}"
echo -e "${GREEN}║   Press Ctrl+C 停止所有服务                 ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
echo ""

# 等待子进程
wait
