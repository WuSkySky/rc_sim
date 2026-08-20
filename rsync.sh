#!/usr/bin/env bash

set -euo pipefail

# ==================== 配置 ====================

ROBOT_USER="jetson"
ROBOT_HOSTS="${ROBOT_HOSTS:-10.42.0.2 10.42.0.3}"
ROBOT_PASSWORD='yahboom'
REMOTE_WS="/home/jetson/workspaces/rc_sim"

# ==================== 配置结束 ====================

LOCAL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SSHPASS="${ROBOT_PASSWORD}"

sync_host() {
    local robot_host="$1"

    echo "[${robot_host}] 开始同步"

    # 确保机器人端目录存在
    sshpass -e ssh \
        -o ConnectTimeout=5 \
        -o StrictHostKeyChecking=accept-new \
        "${ROBOT_USER}@${robot_host}" \
        "mkdir -p '${REMOTE_WS}'"

    # 双系统可能让本地文件时间早于实车端 install 文件。使用内容校验判断
    # 是否需要传输，并让已传输文件采用实车端时间，避免后续 Python 包构建
    # 因时间戳误判而跳过 launch/config 等 data_files 的安装。
    sshpass -e rsync -a --checksum --no-times \
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
        --exclude='/.idea/' \
        --exclude='__pycache__/' \
        --exclude='*.pyc' \
        --exclude='*.pyo' \
        --exclude='.pytest_cache/' \
        --exclude='.mypy_cache/' \
        --exclude='.ruff_cache/' \
        --exclude='.pytype/' \
        --exclude='.tox/' \
        --exclude='.nox/' \
        --exclude='.cache/' \
        --exclude='.ccache/' \
        --exclude='.colcon/' \
        --exclude='*.egg-info/' \
        --exclude='.eggs/' \
        --exclude='.coverage' \
        --exclude='.coverage.*' \
        --exclude='htmlcov/' \
        --exclude='*.swp' \
        --exclude='*.swo' \
        --exclude='src/robot_r2_detect/model' \
        --exclude='*~' \
        --exclude='.DS_Store' \
        --filter='P /src/rc2026_field/COLCON_IGNORE' \
        --filter='P /src/robot_r2_description/COLCON_IGNORE' \
        --filter='P /src/sim_to_real/odin/odin_ros_driver/config/calib.yaml' \
        -e "ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new" \
        "${LOCAL_WS}/" \
        "${ROBOT_USER}@${robot_host}:${REMOTE_WS}/"
}

read -r -a robot_hosts <<< "${ROBOT_HOSTS}"
if ((${#robot_hosts[@]} == 0)); then
    echo "ROBOT_HOSTS 不能为空" >&2
    exit 1
fi

pids=()
for robot_host in "${robot_hosts[@]}"; do
    sync_host "${robot_host}" &
    pids+=("$!")
done

status=0
for index in "${!pids[@]}"; do
    if wait "${pids[index]}"; then
        echo "[${robot_hosts[index]}] 同步完成"
    else
        echo "[${robot_hosts[index]}] 同步失败" >&2
        status=1
    fi
done

exit "${status}"
