# 队友视觉方案适配与小电脑离线验收

日期：2026-10-29。部署目录：`/home/rcsdart/rm27`；本地维护目录：`H:/rm/调车/sp-vision-2026-integration-archive`。

## 实施范围

来源为 Reimuaa/Fujian_AutoAim 的标签 `26.10.25final`，固定提交 `bb0240bbdbdce95e35fbd7eaa442f167e302ce31`。按 MIT 许可保留来源和许可文本，见 `third_party_fujian_autoaim.md`。这次移植旋转靶的核心估计、预测和质量检验，保留本机已有能力，不用整份队友工程覆盖当前工程。

小电脑从机器人取下，相机和电控未连接；本次没有启动实体视觉控制、运动或发射，没有刷写固件。`rm27-mission.service` 因缺少相机和串口自检失败属于当前拆机状态，不通过绕过自检强行启动。

## 已实现并接入生产入口

1. **固定旋转中心联合拟合（V01）**：导入 `FixedCenterModel`，以已同步到本机 NAV 坐标的装甲板位置、物理面编号和时间为输入，联合估计中心、相位、角速度和两种半径。最近一段观测用于后续检验，合格后全窗口拟合并等待接管稳定。拟合周期受限，不因相机帧率增加无限计算。
2. **避免取得模型资格的循环依赖**：拟合可以在旧高速分类尚未可信时独立积累观测；短暂换板缺观测保留样本窗口，当前帧没有有效输出仍禁止发射。导航期间禁止固定中心拟合，进入任务后重新积累。
3. **同一物理板的未来几何（V02）**：高速 Aimer 每次固定一个板编号迭代弹丸飞行时间，在最终预计到达点检查选面窗口，时间迭代容差为 10 微秒。当前处理年龄、配置预测延迟和飞行时间各计一次。发射前比较预测板方位与真实云台姿态，增加未来板的垂直几何检验，最后仍重查姿态代次、帧龄、AUTO 和发送许可。
4. **顺逆时针独立参数（V03）**：增加 `high_speed_delay_time_cw/ccw`，按世界 +Z 轴俯视，正世界角速度为逆时针。两者暂保留本机原有的 0.330 秒基线，不复制队友的 385/325 毫秒。它们不代表已测定的机械出膛延迟。
5. **逐面预测检验及有界补偿（V04）**：导入 `FacePredictionResidual`，只用后续同一物理板观测评分。按独立经过统计，补偿和历史质量分别管理；当前观测失效时，历史质量不能单独授权。基础补偿不超过 0.3°，相位相关总补偿不超过 0.6°；这是视觉预测一致性，不是实弹反馈。
6. **预测分支隔离（V07）**：拟合结果只生成预测副本，不写回原 Tracker 的滤波状态、协方差或数据关联。静止/低速保留原控制与发射路径。切换预测源和残差模型时清空连续帧资格。
7. **每面每次经过只提交一次**：按连续相位区分同一物理面的不同周次，只在串口完整成功写出后消费机会。被限频、超时或写失败的请求不能计作成功发射。
8. **俯仰和质量门控**：增加未来板垂直残差、俯仰跟踪/稳定窗口和历史预测误差门控。保留原有身份确认、双轴新鲜观测、云台限速、Sentinel 有效期等条件。

## 通信、导航与现场参数

保留 `io/xuc.cpp`、`io/sentinel/` 和固件中的现有通信实现：Sentinel V2 A6/6A、921600 波特率、MCU 姿态历史及时间映射、参考帧/许可/有效期、唯一串口发送所有者。队友的旧 30 字节收包边界、IMU 符号和经验 +15ms 没有替换这些内容。

`MissionBridge` 只增加只读的任务阶段查询，供固定中心模型在导航期间失效；IPC schema 2、host_tx 计数、导航命令、返航条件及双上启动条件保持原接口。上位机完整成功下发计数继续作为任务计数，不使用下位机拨弹计数。

本轮保留的本机调参值包括：`xuc_direct_yaw_impact_correction_deg=-1.95`、旧周期偏移 `388ms`、旧 `every_n_faces=1`、弹速 `22m/s`、俯仰微调 `-1.24874°`，以及内外参、摩擦轮和电控 PID。先前聊天中的 410ms/2 不是本次在线读取到的最新值，未恢复它们。

默认高速策略改为 `teammate_prediction`。**该分支不使用旧周期偏移、每隔 N 面及旧事件延迟 640ms 来决定发令时刻**；这些参数保留给旧策略对照。没有导入队友的每发 1000ms 释放间隔，发送器仍保留本机 200ms 去重下限，实际请求还受未来几何、逐面资格和质量门控限制。

## 性能、几何与统计工具

- **V08**：保留有界最新帧采集；区分 `[CAPTURE]` 采集线程交付帧率、`[VISION FPS]` 主循环速率及 `[PIPELINE]` 预处理/推理/后处理耗时和帧龄。关闭默认投影显示与绘图 UDP，详细诊断默认每 3 帧采样一次。要录完整回放数据可将 `vision_diagnostics_stride` 改为 1。
- **CPU 核绑定缺陷**：i7-1280P 的 4.8GHz 核仅有两个物理核。旧频率回退仅选逻辑核 4/5/6/7，漏掉四个 4.7GHz 性能核。新增显式 `yolo26_cpu_ids: [0,2,4,6,8,10]`，每个性能核选一个逻辑核；这是本机拓扑，不应复制到其他电脑。首次移植时默认四线程；清除未使用分类器后已重新实测，当前默认 `infer_threads: 6`，最新结果见下文“运行精简”。
- **GPU 可用性**：从 Ubuntu 官方软件源安装 `intel-opencl-icd` 及依赖，OpenVINO 已识别 Intel 核显。推理代码按设备区分属性，不再向 GPU 无条件传入 CPU 线程数。当前模型测试下 GPU 的中位耗时较高，正式配置仍为 CPU。
- **V10**：在原 1° yaw 粗搜索的最优候选附近增加 0.1° 局部搜索，只有重投影代价改善才更新，不使用全域单峰假设。
- **V11/V12**：增加关联前创新、创新协方差对角及 4 维 NIS 旁路记录，使用 LDLT 解算；字段明确为最低几何代价候选，不冒充最终关联的统计真值。旁路评分不改变原关联决策，不未经观测标定就修改 R/Q 或卡方门限。
- **V13**：维持已有多板堆叠更新；固定中心预测副本不把拟合相位再次作为独立测量回灌原 EKF。共享姿态及多板误差相关性仍需现场数据分析，不虚构交叉协方差。
- **V16**：新增线性模型下 1/2/4/8 次迭代退化一致性、双旋向物理板身份、预测时间、模型失效、真实写出后提交机会、未来观测因果性等离线检查。

## 使用与调参

唯一正式配置仍为 `configs/standard.yaml`：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `xuc_high_speed_strategy` | `teammate_prediction` | 新高速预测；设为 `periodic` 可对照旧周期策略 |
| `high_speed_delay_time_cw` | `0.330` 秒 | 顺时针预测延迟，随后另算飞行时间 |
| `high_speed_delay_time_ccw` | `0.330` 秒 | 逆时针预测延迟，随后另算飞行时间 |
| `xuc_high_speed_prediction_quality_enabled` | `true` | 要求已有独立经过的预测误差合格；不宜为出弹随意关闭 |
| `xuc_high_speed_prediction_quality_max_error_deg` | `0.45` | 历史视觉误差门限，不能解释成真实命中误差 |
| `xuc_high_speed_pitch_variation_deg` | `0.15` | 俯仰稳定窗口的允许变化 |
| `infer_threads` | `6` | 清除无用模型后重新对比选择，保持同步单请求 |
| `device` | `CPU` | 可离线切换 GPU 比较，当前不作为默认 |
| `competition_headless` | `true` | 手动调试需画面时设为 false；任务启动器仍有其原有无界面策略 |
| `vision_diagnostics_stride` | `3` | 详细日志采样间隔，完整记录用 1 |
| `vision_plot_enabled` | `false` | 需要外部绘图调试时再启用 |

首次看到旋转靶后，固定中心至少需要约 2.7 秒跨度，随后还需模型接管与逐面历史成熟；**首发可能等待约 7—10 秒或更久**，不是固定倒计时。丢帧、换目标、换向、误差超限时可能一直不发射；应看 `[TEAMMATE]` 的 `ready/quality_cycles/quality_ready` 及诊断 `fire_gates`，不能仅用等待时间判断完成。进入任务前的导航观测不用于固定中心积累。

## 离线验证与复现

在小电脑根目录：

```bash
JOBS=2 bash scripts/build.sh standard mission_link_probe
cmake -S . -B build/native-offline -DSP_BUILD_OFFLINE_ONLY=ON \
  -DSP_OFFLINE_WITH_VISION=ON -DSP_OFFLINE_WITH_INFERENCE=ON \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build/native-offline -j2
ctest --test-dir build/native-offline --output-on-failure -j2
python3 -m unittest discover -s navigation/mission -p 'test_*.py' -v
python3 navigation/mission/test_virtual_link.py build/mission_link_probe
```

`vision_benchmark` 仅打开明确指定的本地文件，模型为生产 YOLO26；图像预先解码，预热后按低延迟同步模式测量，包含检测后处理，不含取图/姿态/导航/控制。它不能给出整机 FPS 或命中率。

```bash
build/native-offline/bin/vision_benchmark configs/standard.yaml \
  data/offline-input/test.mp4 CPU 6 logs/vision-benchmark/cpu.json
build/native-offline/bin/vision_benchmark configs/standard.yaml \
  data/offline-input/test.mp4 GPU 4 logs/vision-benchmark/gpu.json
```

历史文件回放 `teammate_vision_replay` 输入已有 JSONL 的时间、旧板身份和主观测，能检查拟合成熟及拒绝行为；没有重新检测原始视频、重建完整姿态链或实弹真值。两份现场历史文件分别产生 18 次和 12 次合格拟合；无有效目标的记录没有取得资格。详细结果见 `logs/teammate-historical-replay.json`。

编译、回归和性能结果分别记录在 `logs/teammate-build.log`、`logs/teammate-tests.log`、`logs/teammate-mission-tests.log`、`logs/teammate-mission-pty.log`、`logs/vision-benchmark/`。本轮 52 项 CTest 和 14 项导航任务测试全部通过；虚拟串口验证了恰好 50 次上位机成功发令、过期 IPC 撤销和帧 CRC，不打开实体设备。

最终验收状态见工作区 `H:/rm/调车/视觉改进方案.md`，小电脑同步为 `docs/vision_improvement_plan.md`。不把程序编译成功当成现场命中验收。

首次移植、尚未移除无用分类器时，同批 24 帧文件图像预热后 150 次测试：CPU 四线程检测中位约 11.3—11.4ms，六线程约 9.24ms，GPU 约 14.5—15.2ms；六线程 P95 约 19.9—24.3ms，波动较大。后一次输出一致性检查中 CPU/GPU 检测结果也未完全一致，匹配到的角点最大差约 0.76px，置信度最大差约 0.075。保留 CPU 默认，GPU 还需更全面的精度与整机负载验收；不宣称硬件升级已经提高命中率。

## 仍需整车完成

V05/V15 的实际出膛间隔、散布、逐发弹速和真实命中统计；V06 的曝光/IMU 实际时间偏移；V09 的曝光、对焦和标定验收；V11/V12/V13 的噪声标定及关联效果对照；V14 的真实执行跟踪和是否需要 MPC。当前缺少连接的相机、电控、机构及靶场，不能通过修改软件替代这些测量。

机械维修、支架拆装后须复核内外参和枪口零位。没有把队友的相机标定、角度符号、摩擦轮 PID、装甲尺寸、出膛时间或上科大公开代码中的单位/滤波问题复制到本机。

## 2026-10-29 运行精简与线程优化

正式源码和程序已同步在 `/home/rcsdart/rm27`。本轮主要改动：

- 从正式入口移除完全没有输入的 `CommandGener` 实例，其原循环每2ms唤醒；保留其他入口需要的模块。
- `competition_headless: true` 时不初始化ROS、不创建可视化节点/1ms轮询线程，也不发布TF和Marker。导航自身ROS链路照常使用。
- 移除YOLO26中没有任何调用的传统 `Detector` 成员，因此不再通过它加载ONNX/OpenVINO分类模型和第二套推理资源。传统检测器的独立功能仍保留。文件回放工具也去掉相应虚假模型依赖。
- 移除未使用的录像器对象；只在启用绘图时创建UDP绘图器；关闭采样保存时不再创建imgs目录。
- 大量transport/overlay JSON只在真正的诊断采样帧或启用绘图时生成，且放在串口发送和发射机会提交之后。发送结果直接取类型明确的SendResult，不再反读JSON。实际发令的目标归属与逐发日志仍保留。
- 关联前S/NIS旁路仅在诊断采样帧计算，不改正式关联或滤波；3600帧双路跟踪对照的状态、面编号、身份、更新资格和协方差一致。
- Aimer改为接收目标引用/指针，正式入口不再为选中的目标构造临时链表。普通弹道迭代按需复制，取消预先复制10份滤波状态。
- 固定中心模式只根据既有相位/角速度采样未来物理面，不复制滤波器和传播协方差。其他模型保留原预测。双旋向、跨相位边界和交错高度均与原全对象预测核对。
- 去掉海康SDK阻塞取帧前额外的1ms sleep；最新一帧队列、SDK像素转换与时间戳定义保留。
- 比赛模式过滤debug日志，信息日志按性能报告周期刷新；警告立即刷新、实际发令记录立即刷新。日志对象首次创建改为线程安全，不新增异步日志线程。
- OpenCV/OMP均限制为1线程，OpenVINO由唯一正式 `infer_threads` 配置明确控制，移除重复的全局TBB上限设置。CPU默认改为6，单流、同步单请求及原性能核集合保留。

同一输入24帧预先解码、预热20次，每次测150次完整生产检测；修改后六线程重复3轮：

| 项目 | 清理前，CPU4 | 清理后，CPU6 |
| --- | --- | --- |
| 检测进程线程数 | 42 | 8 |
| 预热后常驻内存 | 667—709 MiB | 309—310 MiB |
| 检测中位耗时 | 10.41—10.46 ms | 8.35—8.38 ms |
| 检测P95 | 16.78—18.41 ms | 8.93—9.36 ms |

清理后CPU4中位11.13—11.22ms/P95 11.56—11.66ms；CPU8实际仍为8个进程线程、中位8.34ms/P95 9.45ms，没有值得增加预算的收益，选择6。上述进程线程数不包括完整机器人所需的相机、串口、任务监管和导航进程。

所有对照的24帧类别、置信度、角点及150次检测数量完全一致。静止、慢转、两种高速旋向共1200组瞄准输出与修改前完全一致；高速固定中心采样的Release中位耗时从约4.1—4.2µs降为1.27µs，该微秒级收益不能当作整个视觉链路的提速比例。

52项CTest通过；另对修改后的回放脚本重跑回归及真实模型空图/黑图两次一致性检查通过。去掉临时配置中的 `classify_model` 后仍可完成同视频推理；虚拟串口恰好50次host_tx通过；正式程序只读配置检查与源码构建身份核对通过。

证据为 `logs/thread-optimization-result.json`、`logs/thread-opt-*.json/log`。离线构建需指定 `-DCMAKE_BUILD_TYPE=Release`；`aim_benchmark configs/standard.yaml OUTPUT_JSON` 不连接任何硬件。

原角度补偿、预测延时、PID、通信协议、许可、返航计数和自启动安全逻辑没有改变。未启动实体自瞄/导航或发射。相机和机器人尚未接回，整机FPS、帧龄与导航同时运行时的尾部延迟仍需实测。
