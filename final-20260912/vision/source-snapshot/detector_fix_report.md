# Detector 异常排查报告

日期：2026-08-05
涉及模块：`tasks/auto_aim/yolos/yolo26.cpp`、`io/hikrobot/hikrobot.cpp`、`tasks/auto_aim/detector.cpp`、`configs/standard_hikrobot.yaml`、`calibration/verify_handeye.cpp`、`CMakeLists.txt`

---

## 一、装甲板识别异常：keypoint 四边形斜穿/垂直于装甲板

### 现象
Detection 窗口中 armor 的关键点四边形方向错误（近似垂直于装甲板）、斜穿，部分角点跑到画面外（如 `(2,658)`、`(831,-177)`），无法贴合真实装甲板四角。

### 原因
YOLO26 模型输出布局为 `[1, 300, 18]`，每个关键点为 `(x, y, visibility)` **三值**：
`xyxy(4) + conf(1) + cls(1) + 4 关键点 × 3 = 18 列`。
原解析代码按每点 `(x, y)` **两值**（固定步长 2）读取，把 `visibility`(≈1.0) 误读成坐标，导致关键点被推向画面外、四边形错乱。14 列旧模型（`xyxy + conf + cls + 4×2`）才是每点两值。

### 修正
`tasks/auto_aim/yolos/yolo26.cpp` 的 `parse()`，步长随列数变化：

```cpp
// 模型输出每点 3 值 (x,y,visibility)：18 列 = 4(xyxy)+1(conf)+1(cls)+12(kpts)
// 旧 14 列模型为 2 值/点；步长按列数定，避免把 visibility 当坐标
const int kpt_step = (num_cols == 18) ? 3 : 2;
...
const float kx = (kpt_ptr[i * kpt_step] - pad_x_f) * inv_scale;
const float ky = (kpt_ptr[i * kpt_step + 1] - pad_y_f) * inv_scale;
```

### 验证
- 离线 135 张：关键点全部落在 bbox 内（0 个画面外、0 个 bbox 外），keypoint 宽高比中位 2.29（横向装甲板 >1.5）。
- 硬件实测：Detection 窗口 keypoint 四边形贴合真实装甲板四角。

---

## 二、颜色通道异常：红蓝反转

### 现象
识别框与关键点贴合正确，但装甲板颜色相对现实**红蓝互换**（打红方，模型却全报 `blue`，日志 class 稳定落在 9/10 = blue_2/blue_3）。

### 原因
`io/hikrobot/hikrobot.cpp` 手写的 Bayer→RGB `type_map` 采用**同名映射**（`PixelType_Gvsp_BayerRG8 → cv::COLOR_BayerRG2RGB`），而**相机厂商的 Bayer 命名与 OpenCV 命名差一格对角线**，导致 R/B 通道互换。证据：

1. **项目已有先例**：`daheng.cpp:212` 注释明确记录此坑——
   > 大恒 BAYERRG 对应 OpenCV 的 `COLOR_BayerBG2RGB`（命名规则不同）
2. **OpenCV 像素实验**：构造"左上 R"的 CFA，`COLOR_BayerRG2RGB` 还原为红色、`COLOR_BayerBG2RGB` 才还原为 R/B 互换，证实厂商 `BayerRG8` 实际对应 OpenCV `BayerBG`。
3. **内部自洽诊断**：修复前对真实帧跑模型，模型标签 `blue` 与框内像素偏蓝**完全一致**——整条颜色链（相机解马赛克 → 模型输入 → 标签 → 显示）一致反转，所以识别贴合不受影响，仅颜色相对现实反了。

### 修正
1. `hikrobot.cpp` 改用**官方 SDK 转换**（颜色由海康保证，不再手猜 CFA）：
   ```cpp
   cvt_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;   // 目标 RGB8
   cvt_param.pDstBuffer = rgb_img.data;                     // 独立 CV_8UC3 输出缓冲
   cvt_param.nDstBufferSize = rgb_img.total() * rgb_img.elemSize();
   unsigned int cvt_ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
   ```
   并删除手写 `type_map` / `cv::cvtColor`。输出保持 RGB，下游（YOLO26 前处理、保存、显示）无需改动。
2. `tasks/auto_aim/detector.cpp`：传统灯条检测器的 `imshow` 前补 `cv::cvtColor(..., cv::COLOR_RGB2BGR)`（相机出 RGB，OpenCV imshow 需 BGR）。

### 验证
- 诊断脚本对比：修复前 `cls=9/10 blue + 框内像素偏蓝` → 修复后 `cls=2 red + 框内像素偏红`，与真实红板一致。
- Detection 窗口（imshow）颜色正常。

---

## 三、外参验证工具 verify_handeye 与构建调整

随本次提交一并纳入工作区遗留的标定工具与构建调整（评估后确认提交）。

### verify_handeye（calibration/verify_handeye.cpp）
手眼外参验证工具，量化验证 `R_camera2gimbal / t_camera2gimbal` 是否自洽。
- **原理**：棋盘固定在世界系，每帧 `solvePnP` 得板在相机系位姿，经「外参→云台→世界」链投影到世界系；外参正确则各帧板位姿重合，**残差统计即外参质量**。
- **数据流**：`io::CBoard` 从 vcan/can0（0x150）读 IMU 四元数 → `Solver::set_R_gimbal2world`（`R_gimbal2imubody` 相似变换），复用运行时同款 Solver。
- **用法**：`./build/verify_handeye -c=configs/calibration_hikrobot.yaml -e=configs/standard_hikrobot.yaml -i=assets/img_with_q`；`--live` 实时、`--compare-roll-flip` roll 180° 对比、`-o=out.csv` 导出残差、`--display` 叠层显示（红=检测，绿=投影）。
- **配套**：`standard_hikrobot.yaml` 的 `R_camera2gimbal` 已按 2026-08-04 验证结果更新（撤销 roll 抹除）。

### CMakeLists.txt 构建调整
- 新增 `verify_handeye` target（链接 `tools io auto_aim`）。
- 移除 `xuc_link_test` target——下位机假下位机自测不再使用（源文件 `tests/xuc_link_test.cpp` 保留在仓库）。

### 杂项清理（不入 git）
删除：
- `$(ALLUSERSPROFILE)/` —— 海康 GenICam XML 缓存误拷贝到字面环境变量名的目录；
- `configs/standard_hikrobot.yaml.bak_roll180`、`configs/standard_hikrobot.yaml.bak_xuc` —— 外参/串口调试备份；
- `kernel.errors.txt` —— OpenVINO CPU kernel 编译错误日志（运行残留）；
- `scripts/imu_to_can` —— `imu_to_can.cpp` 的编译产物二进制。

---

## 四、改动文件清单
| 文件 | 改动 |
|---|---|
| `io/hikrobot/hikrobot.cpp` | 颜色修复：SDK `MV_CC_ConvertPixelType` → RGB8，删除手写 type_map |
| `tasks/auto_aim/yolos/yolo26.cpp` | keypoint 修复：`kpt_step` 随列数（18→3，14→2） |
| `tasks/auto_aim/detector.cpp` | imshow 前补 RGB→BGR 转换 |
| `configs/standard_hikrobot.yaml` | 运行参数：`force_auto_aim: true`、`min_confidence: 0.45` 等 |
| `calibration/verify_handeye.cpp` | 新增：手眼外参验证工具 |
| `CMakeLists.txt` | 新增 `verify_handeye` target；移除 `xuc_link_test` target |
