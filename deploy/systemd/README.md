# rc_sim 开机自启（systemd）

每台 Jetson 一个服务，开机后自动拉起对应 launch：

- `rc_sim_real1.service`：jetson1（10.42.0.2）控制侧，启动 `bringup real1.launch.py`
- `rc_sim_real2.service`：jetson2（10.42.0.3）视觉侧，启动 `bringup real2.launch.py`

## 安装

在对应机器上执行（需 sudo）：

```bash
cd ~/workspaces/rc_sim/deploy/systemd
sudo ./install_autostart.sh real1   # jetson1
sudo ./install_autostart.sh real2   # jetson2
```

## 常用命令

| 操作 | 命令 |
|---|---|
| 立即启动 | `sudo systemctl start rc_sim_real1` |
| 停止 | `sudo systemctl stop rc_sim_real1` |
| 重启 | `sudo systemctl restart rc_sim_real1` |
| 查看状态 | `systemctl status rc_sim_real1` |
| 实时日志 | `journalctl -u rc_sim_real1 -f` |
| 禁用开机自启 | `sudo systemctl disable rc_sim_real1` |
| 启用开机自启 | `sudo systemctl enable rc_sim_real1` |

## 卸载

```bash
sudo systemctl disable --now rc_sim_real1
sudo rm /etc/systemd/system/rc_sim_real1.service
sudo systemctl daemon-reload
```

## 说明

- 启动时序：`After=network.target`（real2 额外 `After=nvargus-daemon.service` 等 MIPI 相机栈）→ `ExecStartPre=/bin/sleep 6` 阻塞等待设备供电/枚举尾巴 → `source` ROS 与工作区环境 → `exec ros2 launch`。
- 崩溃自动重启由 `Restart=on-failure` + `RestartSec=5` 兜底，无看门狗/心跳逻辑。
- 路径写死为 `/home/jetson/workspaces/rc_sim`，如部署路径变化需同步修改 unit 中的 `ExecStart` 与 `Environment`。
- `ROS_DOMAIN_ID=99` 必须显式写在 unit 中（systemd 不读取 `.bashrc`）。
