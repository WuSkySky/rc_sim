#!/usr/bin/env bash

set -euo pipefail

# ==================== 配置 ====================

ROBOT_USER="jetson"
ROBOT_HOST="192.168.1.114"
ROBOT_PASSWORD='yahboom'
REMOTE_WS="/home/jetson/workspaces/rc_sim"

# ==================== 配置结束 ====================

LOCAL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SSHPASS="${ROBOT_PASSWORD}"

# 确保机器人端目录存在
sshpass -e ssh \
    -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new \
    "${ROBOT_USER}@${ROBOT_HOST}" \
    "mkdir -p '${REMOTE_WS}'"

# 测试同步，不真正修改文件
sshpass -e rsync -a \
    --itemize-changes \
    --delete-delay \
    --partial \
    --human-readable \
    --stats \
    --exclude='/build/' \
    --exclude='.git/' \
    --exclude='/install/' \
    --exclude='/log/' \
    --exclude='/.vscode/' \
    --exclude='/.cache/' \
    --exclude='__pycache__/' \
    --filter='P /src/rc2026_field/COLCON_IGNORE' \
    --filter='P /src/robot_r2_description/COLCON_IGNORE' \
    -e "ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new" \
    "${LOCAL_WS}/" \
    "${ROBOT_USER}@${ROBOT_HOST}:${REMOTE_WS}/"