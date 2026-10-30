# rm27_1018 维护结果（2026-10-18）

本副本按用户明确要求由 `/home/rcsdart/rm27` 完整复制，原目录、桌面和自启动没有切换。复制时 10044 个文件逐项哈希一致；原始数据、日志、标定、模型、SDK、依赖和 Git 元数据保留。

## 范围与结果

全文件清单、按语言识别的函数长度/复杂度和前后差异保存在 `logs/maintenance/`。超过100行只是审查入口，不是证明算法有问题或性能收益的依据。本轮做保持行为的结构重构，不改瞄准/预测/开火判定算法，不调整任何参数值，不修改固件，不启动硬件。没有把机械压行或删注释当成优化。

自有长函数共审查列出 50 个，其中 7 处做职责拆分；当前仍有 45 个超过100行，包含已抽取配置但仍较长的主循环。其余列明保留原因，没有声称已把所有长函数改到100行以内，也不声称提高命中率或实时性能。第三方/SDK、测试、固件和导航分别列入扫描清单，未为行数门槛改变其算法。

## 清理与文档

- 删除 3462 个旧缓存/过时文档文件，合计 244.96 MiB；冷编译新生成的有效构建产物不计为净节省。
- 旧 build 中两份调试证据包移到 `logs/maintenance/build_evidence/`，未删除；数据、模型、SDK、固件和历史实测日志保留。
- 删除被当前状态/配置/构建说明取代的 `docs/deployment.md` 和 `docs/usb_camera_diagnosis.md`；重写状态、配置、构建、现场验收和入口说明，清除协议交替、旧目录和旧许可状态混淆。
- RMV 参考文档仍与保留的兼容代码有关，明确标成旧后端范围；未把有效协议资料当缓存删除。
- 新副本脚本相对目录定位；导航少量活动绝对路径已改为副本，历史证据路径不改写。桌面安装器按目录名隔离入口，但本次没有安装入口。
- 当前值标在 `configs/standard.yaml`，[参数索引](current_parameters.md)列出58个关键项；所有解析值与复制源一致。

## 验证

- 主程序及6个标定/诊断工具冷编译通过，最终源码重新链接通过。
- 49/49现有离线CTest通过；启动器隔离测试、42项回放测试通过（回放测试也包含在CTest中，不重复计数）。
- 原配置表达式与抽取配置在4个场景中各117项完全一致。
- 原工程现存文件回放基准程序与副本新程序处理相同8帧/9个检测，类别、颜色、置信度、角点等输出精确一致，保存图像哈希一致；不比较运行时间，不代替真实图像全场景验收。
- 7项模型/SDK资源哈希、源码制品、部署自检和差异格式检查通过。

## 长函数逐项处理

| 文件 / 函数 | 原行数 | 现行数 | 处理 |
|---|---:|---:|---|
| `simulation/src/gestalt.cpp` / `main` | 3644 | 3644 | 保留：独立仿真入口，不改变既有仿真算法 |
| `src/standard.cpp` / `main` | 2743 | 2223 | 配置读取独立到不可变 StandardSettings，保留控制主循环 |
| `src/rmv_debug.cpp` / `run_rmv_debug` | 372 | 372 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `tasks/auto_aim/gyro_phase_scheduler.hpp` / `auto_aim::GyroPhaseScheduler::observe` | 271 | 271 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `src/auto_aim_debug_mpc.cpp` / `main` | 233 | 233 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `calibration/capture.cpp` / `capture_loop` | 222 | 222 | 保留：独立标定流程，当前未做标定算法变更 |
| `src/standard_mpc.cpp` / `main` | 216 | 216 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `tasks/auto_aim/tracker.cpp` / `auto_aim::Tracker::update_target` | 210 | 74 | 候选收集、原分配搜索、刚体配对判断分开；代价与搜索顺序不变 |
| `calibration/verify_handeye.cpp` / `compute_report` | 204 | 204 | 保留：独立标定流程，当前未做标定算法变更 |
| `tasks/auto_aim/yolos/yolo26.cpp` / `auto_aim::YOLO26::YOLO26` | 197 | 10 | 参数、角点配置、CPU、输入区、模型及日志初始化分开 |
| `io/daheng.cpp` / `io::Daheng::set_camera_params` | 197 | 197 | 保留：非当前海康/串口姿态硬件路径 |
| `tasks/auto_aim/yolos/yolo26.cpp` / `auto_aim::YOLO26::parse` | 193 | 193 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/yolos/yolov8_trt.cpp` / `auto_aim::YOLOV8_TRT::parse` | 165 | 165 | 保留：当前 CPU/OpenVINO 未启用的 TensorRT 分支 |
| `src/sentry_debug.cpp` / `main` | 165 | 165 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `tasks/auto_aim/yolos/yolo26.cpp` / `auto_aim::YOLO26::refine_color` | 159 | 90 | 原灯条采样独立；像素遍历、统计和颜色记忆不变 |
| `tasks/auto_aim/yolos/yolov8_trt.cpp` / `auto_aim::YOLOV8_TRT::YOLOV8_TRT` | 157 | 157 | 保留：当前 CPU/OpenVINO 未启用的 TensorRT 分支 |
| `tasks/auto_aim/yolos/yolov8_trt.cpp` / `auto_aim::YOLOV8_TRT::detect` | 154 | 154 | 保留：当前 CPU/OpenVINO 未启用的 TensorRT 分支 |
| `tasks/auto_aim/tracker.cpp` / `auto_aim::Tracker::publish_markers` | 152 | 152 | 保留：可视化输出，长度本身不是算法瓶颈证据 |
| `src/uav_debug.cpp` / `main` | 148 | 148 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `tasks/auto_aim/yolos/yolo11.cpp` / `auto_aim::YOLO11::parse` | 145 | 145 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `src/mt_auto_aim_debug.cpp` / `main` | 145 | 145 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `src/rmv_debug.cpp` / `run_rmv_shot_test` | 144 | 144 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `src/auto_buff_debug_mpc.cpp` / `main` | 141 | 141 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `tasks/auto_aim/solver.cpp` / `auto_aim::Solver::solve` | 137 | 104 | 诊断与重投影误差计算独立；PnP/门槛/变换不变 |
| `tasks/auto_buff/buff_target.cpp` / `auto_buff::BigTarget::update` | 134 | 134 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `src/auto_buff_debug.cpp` / `main` | 134 | 134 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `scripts/flash_sentinel.py` / `main` | 133 | 133 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `io/hikrobot/hikrobot.cpp` / `io::HikRobot::capture_start` | 132 | 15 | 设备打开、配置、采集线程职责分开 |
| `tasks/auto_aim/rmv_rotating.hpp` / `auto_aim::RmvRotatingAim::update` | 130 | 130 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `io/dm_imu/dm_imu.cpp` / `io::DM_IMU::get_imu_data_thread` | 129 | 129 | 保留：非当前海康/串口姿态硬件路径 |
| `tasks/auto_buff/yolo11_buff.cpp` / `auto_buff::YOLO11_BUFF::get_multicandidateboxes` | 124 | 124 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/yolos/yolo11.cpp` / `auto_aim::YOLO11::YOLO11` | 122 | 122 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `scripts/report_rmv_rpm_delay.py` / `analyze` | 121 | 121 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `calibration/capture.cpp` / `run` | 120 | 120 | 保留：独立标定流程，当前未做标定算法变更 |
| `tasks/auto_aim/target.cpp` / `auto_aim::Target::update_batch` | 118 | 118 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_buff/buff_target.cpp` / `auto_buff::SmallTarget::update` | 117 | 117 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tools/replay/recording.py` / `_data` | 115 | 17 | 按事件类型分派到原验证分支，错误及顺序不变 |
| `io/rmv/debug_link.hpp` / `io::rmv::DebugLink::run` | 115 | 115 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/yolo.cpp` / `auto_aim::YOLO::YOLO` | 114 | 114 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/detector.cpp` / `auto_aim::Detector::detect` | 112 | 112 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `io/xuc.cpp` / `io::XucSender::XucSender` | 112 | 112 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `scripts/check_deployment.py` / `main` | 108 | 108 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `src/mt_standard.cpp` / `main` | 108 | 108 | 保留：非当前 standard/Sentinel 入口；不在未覆盖的旧后端中机械拆分 |
| `scripts/check_rmv_pty.py` / `main` | 105 | 105 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/detector.cpp` / `auto_aim::Detector::lightbar_points_corrector` | 104 | 104 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/yolos/yolo26.cpp` / `auto_aim::YOLO26::detect` | 103 | 103 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tasks/auto_aim/yolos/yolo11.cpp` / `auto_aim::YOLO11::detect` | 102 | 102 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `tools/replay/recording.py` / `inspect` | 102 | 102 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |
| `io/daheng.cpp` / `io::Daheng::open` | 102 | 102 | 保留：非当前海康/串口姿态硬件路径 |
| `tasks/auto_aim/tracker.cpp` / `auto_aim::Tracker::track` | 101 | 90 | 保留并列明：数值/状态/协议核心或兼容路径，本轮无改算法所需的新行为基准 |

## 实车与版本限制

当前相机断流、无姿态回传和既有跟踪丢失等问题没有由本次结构整理实车复测。参数索引不是新测量。没有提交/推送Git，副本保留了复制前的未提交改动，审阅本次改动应以 `logs/maintenance/baseline_manifest.json` 和 `result.json` 为准，不能仅凭整仓库 git diff。
