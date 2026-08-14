#!/usr/bin/env bash
# 安装 rc_sim 开机自启服务
# 用法: sudo ./install_autostart.sh <real1|real2>
set -euo pipefail

TARGET="${1:-}"
SERVICE="rc_sim_${TARGET}.service"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_SRC="${SRC_DIR}/${SERVICE}"

if [[ "$TARGET" != "real1" && "$TARGET" != "real2" ]]; then
  echo "用法: sudo $0 <real1|real2>" >&2
  exit 1
fi
if [[ "$(id -u)" -ne 0 ]]; then
  echo "请用 sudo 运行" >&2
  exit 1
fi
if [[ ! -f "$UNIT_SRC" ]]; then
  echo "找不到单元文件: $UNIT_SRC" >&2
  exit 1
fi

install -m 644 "$UNIT_SRC" "/etc/systemd/system/${SERVICE}"
systemctl daemon-reload
systemctl enable --now "$SERVICE"

echo "已安装并启动 ${SERVICE}"
echo "查看状态: systemctl status ${SERVICE}"
echo "查看日志: journalctl -u ${SERVICE} -f"
