# 启动器与副本隔离

`tools/field_start_autoaim.sh` 与 `tools/field_stop_autoaim.sh` 相对自身定位项目，使用本副本的 `build/standard`、配置、PID 和日志。它们不会修改原项目的参数。

```bash
cd /home/rcsdart/rm27_1018
# 只检查，不启动相机或串口程序：
bash tools/field_start_autoaim.sh --check
```

原有桌面入口和系统自启动没有切换到副本。本次未运行安装脚本。`scripts/install_desktop.py` 已按项目目录名区分入口文件和显示名称，用户日后显式安装副本入口时不会覆盖原 rm27 的入口。

正式后端为 Sentinel V2，不存在旧 SP/RMV 声明不匹配的历史阻碍；实际设备数据、安装与标定是否可用仍需单独验证。当前控制/摩擦轮/发射配置为开启，不能把“复制完成”当作允许启动现场动作。

运行缓存位于 `logs/xuc-bringup/`，颜色保存在 `logs/field-state/field_autoaim_enemy_color`；唯一参数源仍为 `configs/standard.yaml`。复制来的旧 PID、锁、运行缓存和 last_log 指针已删除，现场历史日志保留。
