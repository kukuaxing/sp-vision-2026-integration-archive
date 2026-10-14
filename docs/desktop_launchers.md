# 桌面自瞄入口

小电脑工程为 `/home/rcsdart/rm27`。桌面有“启动自瞄”和“关闭自瞄”，安装命令为：

```bash
cd /home/rcsdart/rm27
/usr/bin/python3 scripts/install_desktop.py
```

点击“启动自瞄”后，先检查配置、模型/SDK 校验值、程序和运行库、相机枚举、串口路径与权限、协议兼容性、串口占用及重复运行。失败时弹出错误，不进入颜色选择。`configs/standard.yaml` 的 `board_protocol` 记录经核对的下位机协议；该键只供检查，不会为程序增加 RMV 支持。未声明协议或与正式入口不匹配时提前拒绝启动。硬件枚举通过不代表实际通信通过，协议匹配后启动程序继续等待下位机反馈，10 秒未收到反馈则停止并报错。

自检通过后选择敌方装甲板红色或蓝色，默认选中上次保存值；首次使用默认读取 `configs/standard.yaml`。确认“启动”后保存颜色；取消不修改。颜色记录位于 `logs/field-state/field_autoaim_enemy_color`，只覆盖运行缓存 YAML 的 `enemy_color`，不改动正式配置中的控制和发射开关。

“关闭自瞄”检查 PID 对应的真实可执行文件，只关闭本工程经启动器启动的程序。先发送 SIGINT，10 秒仍未退出再发送 SIGTERM；若仍未退出，明确报错并保留 PID，不结束其他工程。

相关命令：

```bash
# 桌面相同行为：自检，然后选择颜色。
bash tools/field_start_autoaim.sh --select-color
# 后续自启动使用：自检，沿用保存的颜色，不弹颜色选择框。
bash tools/field_start_autoaim.sh
# 仅自检，不改颜色、不启动。
bash tools/field_start_autoaim.sh --check
bash tools/field_stop_autoaim.sh
```

本次只创建桌面入口，未启用 rm27 开机自启动。用户已授权停用旧飞镖工程的 `~/.config/autostart/autoaim.sh.desktop` 并停止它等待 `/dev/ttyUSB0` 的进程，旧工程文件保留。以后设置车载自启动时还需处理设备上电等待，以及图形桌面/无显示运行方式。

正式运行入口目前仍为 SP 协议，rm27 的下位机声明为 RMV，启动自检会明确报出这项不匹配；控制和发射开关保持部署时的关闭状态。桌面入口不代表实车协议和标定已经验收。

`python3 scripts/check_deployment.py` 检查软件并报告硬件与协议状态；`--require-hardware` 要求设备枚举和权限通过；`--require-runtime` 同时要求协议与正式入口匹配，供桌面启动器使用。报告分别列出 `software_ready`、`hardware_ready` 和 `runtime_ready`。协议来源是配置声明，本次检查不打开串口进行握手，也不证明安装与标定已经验收。标定配置现已选择 RMV 接收器，报告同时列出姿态协议是否匹配及安装确认标志；若选择 HI91 会另列与 RMV 的不匹配警告。正式 standard 仍使用 SP，因此入口继续拒绝启动。

自检记录：`logs/xuc-bringup/desktop_preflight.json`；运行日志路径记录：`logs/xuc-bringup/field_autoaim.last_log`。

离线验证：`/usr/bin/python3 scripts/check_desktop_launchers.py`。它仅在系统临时目录内使用模拟程序和对话框，覆盖自检失败、协议不匹配、未声明协议、取消、颜色持久化、重复启动、无反馈退出、正常关闭和拒绝关闭无关进程；协议拒绝场景使用实际检查函数，不打开实际相机或串口。
