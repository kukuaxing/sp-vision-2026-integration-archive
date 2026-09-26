# 统一记录与只读事件回放

`tools/replay/` 提供 Python 3.10+ 标准库实现的 Writer、Reader、完整性检查和确定性事件回放。默认演示由代码生成 12 张 32×24 的 PGM 小图及 73 条事件，明确标记 `source=synthetic`。没有调用识别模型、估计器或规划器，没有打开相机、串口、网络或控制设备。

它解决的是数据能否完整、明确、按同一顺序交给后续算法的问题。`python -m tools.replay replay` 是**事件回放**，不是完整自瞄算法回放；其 demo 的合成检测与目标状态是夹具的已知输入，不是实际模型输出。另有下文的 `scripts/replay_detector.py`，实际调用本工程的 C++ YOLO26 进行文件识别回放。

## 运行与产物

在仓库根目录：

```text
python -m tools.replay demo --output data/replay/demo/events.jsonl
python -m tools.replay inspect data/replay/demo/events.jsonl --report logs/replay/demo_report.json
python -m tools.replay replay data/replay/demo/events.jsonl
python -m unittest discover -s tests/replay -v
```

Windows 当前解释器可用 `C:\Users\32751\AppData\Local\Programs\Python\Python313\python.exe`。`demo` 可加 `--config configs/standard.yaml` 将正式配置内容快照写入 manifest，**不使用这些配置操作硬件**。未提供时写入明确标记为合成夹具的小段配置说明。

数据只放 `data/replay/`，检查报告只放 `logs/replay/`；它们已被 Git 忽略。Writer 不覆盖既有记录或图像，重复运行同一路径的 demo 会失败。检查/回放可以任意重复；要录入新的数据批次，应明确指定新的采集目录。报告路径不能位于被检查的采集目录内，避免覆盖原始图像或 JSONL。

检查失败返回状态码 2，不跳过损坏数据、不补造图像或时间戳，也不输出新的“通过”报告。已有旧报告不会被当作本次成功；自动调用方必须检查本次退出状态。

## 文件格式 `sp-vision-events-1`

一个采集目录包含 `events.jsonl` 和相对路径引用的图像。JSONL 使用 UTF-8，每行必须有换行，单行最大 8 MiB。不允许重复 JSON 键、NaN、Infinity、未知字段或跳跃的事件序号。整数时间以微秒表示，角度为 rad，速度为 rad/s 或 m/s，长度为 m。所有数值必须有限。

首行为 manifest，字段固定：

| 字段 | 内容 |
|---|---|
| `schema`, `kind`, `time_unit` | 分别为 `sp-vision-events-1`、`manifest`、`us` |
| `run_id`, `source` | 本次采集标识；来源为 `synthetic`、`captured` 或 `imported` |
| `runtime` | `application`、`revision`、`build`、`platform` 四个非空字符串；实录应填写真实提交/构建信息，不要沿用夹具值 |
| `clocks` | 时间域名称到初始 epoch 字符串的映射，必须声明 `host_monotonic` |
| `frames` | `world`、`gimbal`、`camera`、`chassis` 各坐标系的明确文字定义 |
| `configs` | 每项包含 `name`、`content_utf8`、`sha256`；哈希校验 UTF-8 内容原始字节，名称不重复 |

每条普通事件字段固定：

```json
{"kind":"pose","seq":1,"stream":"pose/small_y","host_us":1000000,"stamp":{"domain":"mcu_monotonic","epoch":"mcu-boot-1","time_us":5000000,"mapped_host_us":1000000,"uncertainty_us":100},"data":{"sensor_id":"camera_support","from_frame":"gimbal","to_frame":"world","quaternion_wxyz":[1,0,0,0],"angular_velocity_rad_s":null}}
```

- `seq` 从 1 连续递增；`host_us` 是录入/接收时间，全局不倒退。多线程生产者应先经过单一排序/写入队列；Writer 本身不提供线程安全或异步落盘。
- `stamp.time_us` 是数据实际对应的测量/生成时刻，不能把接收时刻写成真实相机曝光时刻。检测的 `stamp` 必须对应引用图像的测量时刻，处理结束时间放在 `host_us`。目标状态可使用自身预测/估计时刻，并用 `image_id` 表示关联图像。
- `stamp.domain`、`stamp.epoch` 必须已声明。主机时间必须恒等映射、`uncertainty_us=0`，测量不能晚于录入时间。
- MCU/相机时间不能直接当主机时间。已知时钟转换时，成对写入 `mapped_host_us`、`uncertainty_us`；未知时两者均为 null。映射时刻晚于接收时刻且超出不确定度时拒绝。
- 同一 `stream/domain/epoch` 内，源时间和已有映射时间均不倒退。相等允许记录，但报告单列重复时间戳，不能把它解释为新的有效测量。
- MCU/相机重启须先记录 `clock_epoch`。旧 epoch 立即失效，不能再次回到已退役的 epoch。主机单调时钟重启需要另开采集文件。

最后一行必须是结束记录：

```json
{"kind":"end","event_count":73,"recording_sha256":"64个小写十六进制字符","complete":true}
```

`recording_sha256` 覆盖结束行之前的**全部原始 UTF-8 字节，包含每行换行**，不包含结束行自身。这使其他语言的 Writer 不必复刻 Python 浮点 JSON 格式。结束行计数必须与实际事件数一致，之后不能再有内容。缺少结束行、整行截断、非正常关闭的 `complete=false` 均拒绝。SHA256 用于完整性核对，不提供来源身份认证。

Python Writer 每次成功追加会 flush；关闭时写结束行。上下文因未捕获异常退出会写 `complete=false`。它没有提供 `fsync` 或断电原子提交保证，崩溃/掉电文件可能不完整，Reader 会明确拒绝。可正常读取的文件不代表没有丢帧；采集侧应另外用 `health` 记录队列溢出、相机丢帧及设备断流。

## 事件载荷

以下字段均为必填；只有注明 null 的字段允许空值。

| kind | data 字段及含义 |
|---|---|
| `image` | `image_id` 唯一、`path` 相对文件、`sha256`、`width`、`height`、`encoding`（pgm/ppm/png/jpeg） |
| `detections` | 已出现的 `image_id`、`objects` 数组。每个对象为 `label`、`color`（red/blue/unknown）、`confidence` [0,1]、四个二维 `corners_px`；空数组表示本图无检测 |
| `target` | `target_id`、已出现的 `image_id`、`frame`、`position_m` 三维、`velocity_mps` 三维、`yaw_rad`、`yaw_rate_rad_s`、`confidence` |
| `pose` | `sensor_id`、`from_frame`、`to_frame`、归一化 `quaternion_wxyz`、`angular_velocity_rad_s` 三维或 null；不自动修改或归一化姿态 |
| `command` | 唯一 `command_id`、u32 `session_id`、`reference`（world_los/gimbal_joint）、`yaw_rad`、`pitch_rad`、`valid_until_host_us`、布尔 `control`、布尔 `fire_requested`；生成时刻和有效期必须使用主机时间 |
| `feedback` | u32 `session_id`、`reference`、`accepted_command_id` 或 null、`applied_command_id` 或 null、`yaw_rad`、`pitch_rad`、`mode`（safe/manual/vision/calibration/fault）、u32 `fault_bits`、布尔 `fire_permit`、u32 `shots_fired_total` 或 null |
| `clock_epoch` | `domain`、`previous_epoch`、新的 `epoch`、`reason`；该事件自己的 stamp 必须使用主机时间 |
| `health` | `component`、`status`（ok/degraded/failed）、非空 `detail` |

`frame` 使用 manifest 已定义的四个坐标系名称。`pose` 必须说明从哪个系旋转到哪个系，不把大 Y、相机支架或未经确认的 IMU 姿态混成同一个传感器。

图像路径禁止绝对路径、`..`、空段、反斜杠、盘符/冒号、Windows 保留设备名/通配符/末尾点或空格，以及逃出采集目录的符号链接。哈希不匹配或缺图立即失败。PGM/PPM 仅支持夹具使用的 `P5/P6\n宽 高\n255\n` 紧凑 8 位格式，校验尺寸和完整像素字节数；PNG/JPEG 校验文件 SHA256，但标准库不解码像素，接入 OpenCV 后仍须检查实际尺寸和解码结果。

`feedback` 中的非空指令 ID 必须指向之前记录、同会话的指令。它不是“期望姿态”的别名。采集开始时设备仍在执行记录范围外的旧指令，应将对应 ID 写 null 并记录 health 说明，不能凭姿态相近补猜一个 ID。原始重传字节继续交给 `tools/rmv/client.py` 的 wire journal；本层只记录一次唯一控制意图。

## Python 录入与回放接口

```python
from pathlib import Path
from tools.replay import Reader, Writer, inspect, stamp

# clocks/frames/runtime 必须由实际应用填入真实值。
with Writer(Path("data/replay/capture/events.jsonl"),
            run_id="capture", source="captured", runtime=runtime,
            clocks=clocks, frames=frames,
            configs={"configs/standard.yaml": config_text}) as recording:
    digest = recording.write_image("images/frame-000001.png", encoded_image_bytes)
    recording.append("image", stream="camera/main", host_us=receive_host_us,
                     timestamp=exposure_stamp,
                     data={"image_id": "frame-000001", "path": "images/frame-000001.png",
                           "sha256": digest, "width": width, "height": height, "encoding": "png"})

report = inspect("data/replay/capture/events.jsonl")
for event in Reader("data/replay/capture/events.jsonl"):
    # 在此显式交给后续离线算法，不设置串口或发送控制的回调。
    consume_offline_event(event)
```

`Reader` 按记录顺序逐条校验后产出事件，不按系统时钟 sleep，不把落盘延迟变成播放延迟。结束行验证发生在完整迭代结束时；中途退出循环没有完成整体完整性检查，必要时先完整调用 `inspect`。`replay_digest` 对标准化后的事件 JSON 计算稳定摘要；它与含 manifest 的原始文件摘要是两种明确不同的校验值。

## 报告怎样解释

报告列出事件数量、源时间域、未映射/不确定度超过 5 ms 的样本、接收减测量时间分布、重复源时间戳、故障反馈数量，以及按传感器/坐标变换/时间域/epoch 分组的图像姿态括号覆盖。只有映射不确定度不超过 5 ms 的图像/姿态参与括号统计；不跨传感器或重启拼接，括号间隔不超过 50 ms。它检查时间样本可用性，不证明外参、安装关系或硬件曝光时间正确。

指令分别统计“已记录意图”“反馈报告已接收”“反馈报告已采用”“缺少已采用反馈”。只对同一 reference、反馈明确引用的指令计算角度差，yaw 差使用周期最短差；该统计不是端到端动态误差、稳定时间或命中率。反馈测量时刻减指令生成时刻也不等于接收 ACK 延迟。截止期之后仍被反馈引用的指令单列，可能表示 last-applied 字段保留旧值，不能直接下结论说电控超时失效。

## 正式接入还需要的工作

1. C++ 相机端记录原始图像和真实时间来源；若只有 SDK 取帧完成时间，清楚标明并保留未知曝光映射，不能凭当前时刻补造硬件时间。
2. 检测/追踪/Aimer 在各自边界追加对应事件，并写准确坐标系、角点顺序、类别标签约定、模型/标定信息。模型文件可在独立运行元信息中用配置快照记录路径与 SHA256，禁止复制权重进 Git。
3. RMV/旧 SP 的反馈适配器保留能力差异；无法确认已采用序号、真实射击计数或完整姿态时写 null/health，不虚构支持。
4. 显式初始化同一个生产算法、配置及确定性随机种子，通过 Reader 输入图像/测量，用独立输出事件和真实标注计算 A/B 指标。此项当前没有声称完成。
5. 实车录入后核对相机支架 IMU 安装、时钟转换、标定版本与真实控制响应；离线夹具不能替代这些验收。

## 实际 YOLO26 文件回放

`tests/offline/detector_replay.cpp` 链接当前工程的 YOLO26、颜色判定、角点修正及装甲分类相关源码；`scripts/replay_detector.py` 负责显式文件输入、统一记录和两次运行的一致性核对。它实际执行 OpenVINO 推理；没有把合成检测结果当作推理输出。

该入口是离线 CMake 的可选 `SP_OFFLINE_WITH_INFERENCE=ON` 目标 `detector_replay`，需要本机已有 OpenCV、OpenVINO C++ 开发依赖，以及正式配置引用的模型文件。它不下载模型，不连接相机，也不调用完整 standard 程序。

构建目标可用后，在仓库根目录运行：

```text
python scripts/replay_detector.py --blank --output data/replay/yolo26_blank --max-frames 1
python scripts/replay_detector.py --input assets/demo/test.mp4 --output data/replay/yolo26_video --max-frames 8 --stride 10 --repeat-check
```

Windows 默认二进制路径为 `build/native-offline/bin/Release/detector_replay.exe`，其他构建目录使用 `--executable`。输入必须是明确的本地普通文件，输出必须是仓库 `data/replay/` 下的新目录，报告和进程日志只写 `logs/replay/`，已有结果不会覆盖。`--max-frames` 默认 8；`--stride 10` 表示原视频每十帧取一帧，实际总数受文件长度限制。

每次运行从 `configs/standard.yaml` 派生临时运行配置，强制 CPU、一个推理线程、关闭大核绑定、关闭异步/调试窗口/样本保存。分类器也使用 CPU/一个线程。正式配置文件保持原样，内容及派生配置一起进入记录快照。Windows 推理子进程使用较低优先级，DLL 搜索路径只在子进程环境中设置。

视频使用本机已安装的 Python cv2 FFmpeg，以一个解码线程导出选定帧，再交给**同一个持续运行的 C++ YOLO26 对象**，保留颜色记忆等时序行为。C++ 本身另支持带单线程 FFmpeg 的直接视频读取；统一脚本使用已解码列表，避免要求 vcpkg 与 Python 的 OpenCV 都带视频编解码器。OpenCV 原始文件解码为 BGR，调用 YOLO26 前转为正式相机所用的 RGB；保存 PNG 仍按正常 BGR 编码，图像颜色不会被人为颠倒。

统一记录包含图像和实际检测结果，以及 `health` 中的每帧耗时；`pass1/raw.jsonl` 另外保留模型 class_id、装甲类型、关键点置信度和颜色来源/分值。配置快照中的 `detector_provenance.json` 记录二进制、输入、模型、算法源文件 SHA256 及 OpenCV/OpenVINO 版本。二进制、输入和正式配置比较运行前后的 SHA256；模型和源码哈希注明是在运行后记录。原始视频 PTS 只表示容器呈现时间，不转换成未知的实车曝光或 IMU 时间；没有 PTS 时明确使用解码完成时刻。

`--blank` 实际执行空图防护和黑图模型推理；黑图有检测会失败。`--repeat-check` 用新建的模型实例对同一组选定图像再跑一遍，比较类别、颜色、关键点、置信度及输入图像哈希，数字允许绝对/相对 `1e-5` 容差；不比较耗时。报告中的 `detector_wall_us` 包括预处理、模型推理及后处理，只能描述本机执行情况。

这证明文件路径能够执行真实检测并稳定导出结果，**不能证明检测准确率、现场泛化或命中率**。没有人工标注，不计算这些指标；没有姿态数据，不生成目标三维状态、轨迹、弹道、控制或实际反馈。

2026-09-26 本机实际验收：空图与 640×640 黑图通过，黑图零检测；`assets/demo/test.mp4` 每十帧取一帧，共 8 帧、9 个实际检测，两个独立模型实例输出的置信度和角点最大差均为 0。第一遍检测总耗时中位数约 28.8 ms，仅代表这台 Windows 机器和当前单线程配置。24 条图像/检测/耗时事件通过统一 Reader 校验，报告在 `logs/replay/yolo26_video_detector_report.json`；没有把检测数量解释成准确率。
