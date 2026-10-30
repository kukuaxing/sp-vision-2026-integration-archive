# 构建与维护

本副本：`/home/rcsdart/rm27_1018`。原项目 `/home/rcsdart/rm27` 不变。

## 编译

```bash
cd /home/rcsdart/rm27_1018
bash scripts/build.sh standard capture calibrate_camera calibrate_handeye calibrate_robotworld_handeye verify_handeye xuc_board_probe
```

默认最多两个编译任务，构建目录为 `build/`。复制来的旧 CMake 缓存含原工程绝对路径，已清理后重新生成。不要复制旧二进制冒充本副本的构建结果。依赖与本机模型/SDK见 [依赖说明](dependencies.md)。

## 不接实车的检查

```bash
source scripts/runtime_env.sh
python3 scripts/check_deployment.py
python3 scripts/check_offline.py --with-vision
python3 scripts/check_desktop_launchers.py
git diff --check
```

部署检查只校验资源、运行库、静态入口与设备枚举，不打开相机或串口；它不证明通信、安装或命中通过。离线测试使用合成数据和私有 PTY，不操作车辆。

## 入口与版本

副本中的启动/停止脚本相对自身定位目录，日志写入本副本。原桌面入口和系统自启动仍指向原项目；本次不安装入口或切换服务。只有用户自行启动现场测试。

Git 元数据和原工作区改动已保留，未提交或推送。查看本次具体差异与清理结果应使用 `logs/maintenance/`，因为 `git diff` 也包含复制前已有的未提交改动。
