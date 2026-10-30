# 实车测试部署核对

核对时间：2026-10-30，约19:54，中国标准时间。唯一正式工程 `/home/rcsdart/rm27`。

## 结论

已有证据支持保留的视觉改进已经编入正式 `build/standard`，当前正式配置已启用对应策略。尚未通过命中非退步验证的候选没有启用。本轮没有改动正式算法、实车参数、服务状态或启动机器人；只修正了一项过时的启动器测试及其超时清理。

软件部署通过；整机连接尚未就绪。本次只读枚举没有找到海康相机或下位机串口，雷达网络也未就绪。因此不能把这份记录视为实车命中率、标定或整机任务验收。

## 已在正式程序中生效

| 项目 | 核对结果 |
| --- | --- |
| 队友视觉处理适配 | `xuc_high_speed_strategy: teammate_prediction`，保留本机Sentinel V2通信契约 |
| 固定旋转中心与同面未来预测 | 正式分支包含固定中心模型及其所有权管理、同面未来几何 |
| 历史预测残差修正 | 已编入并启用；短暂同目标缺失最多保留150ms上下文，不由缺失帧授予发射许可 |
| 一次物理经过防重复发令 | 已编入；串口完整发送后才提交对应经过 |
| 发令质量与时序检查 | 历史预测质量、俯仰稳定、图像/姿态新鲜度、参考身份和等待后的复核均在正式路径中 |
| 检测与跟踪 | 启用现有灯条角点细化、目标关联、ROI及失败后的全图重搜 |
| CPU预算 | OpenVINO CPU，6推理线程，CPU集合0/2/4/6/8/10；OpenCV和OMP各1线程；关闭绘图 |
| 实车诊断 | 已启用，每3帧抽样，最多12000条；记录构建/配置身份、几何/预测/发令信息，输出至 `logs/diagnostics/` |
| 启动入口 | 服务和桌面均指向本工程 `tools/field_start_autoaim.sh`，由唯一正式YAML生成运行缓存 |

## 明确保留关闭或仅离线的候选

- Otsu＋亚像素端点候选：已编译，但 `corner_refine_brightness_invariant_subpixel: false`。几何误差改善没有转化为稳定的命中率收益。
- 联合位姿拟合原型、居中发令机会和新增时间裕量：未部署到正式控制路径。
- FrameAdmission精确旧帧跳过：仅离线测试接入，正式循环未调用；图像留出未通过命中非退步，且缺少现场重放证据。
- 额外旋向时间补偿、固定像素/距离修正及仿真扰动参数：没有作为实车优化写入。

## 本次实际有效参数

- 顺/逆时针预测延迟均 `0.330 s`，另外计算飞行时间；这是保留的预测参数，不是新测得的出膛延迟。
- 弹速设定 `22.0 m/s`，仍需实车确认。
- 水平落点修正 `-1.95°`；另有准轴修正 `-1.30°`，静态配置报告的合计固定水平修正为 `-3.25°`。
- 俯仰修正 `-1.24874°`，旋转额外水平trim `0°`。
- 发射请求最小间隔 `200 ms`，只是下限，并不表示程序必定每200ms发一发。
- 当前 `teammate_prediction` 分支的发令时刻不由旧 `xuc_periodic_phase_offset_ms: 388` 或 `xuc_periodic_every_n_faces: 1` 决定。
- 复用既有标定，`sentinel_require_verified_pose: false`；这不代表当前安装外参已重新测量通过。若维修改变了相机/云台安装位置，应先验证几何。

## 本轮验证与身份

- `build/runtime-source-manifest.txt` 中198项均与现有文件吻合；manifest SHA256为 `99037c3230fb3990b1a9b77fcb3b5a9896a5f31581987995ff9f874f64835ddc`，与正式程序内身份吻合。该清单是构建时输入清单，新增且未接入正式循环的离线辅助文件不表示已编入正式程序。
- `build/standard` SHA256：`dbc3d740d18255383af4f211b9bb8b740bbee0c65d31b0d18c8a757f378d5cfe`。
- `configs/standard.yaml` SHA256：`e087e02c2c6ad257acef5a0808950a42f15445c4ed00714807023dad7427fd90`。
- 7项模型/依赖资源校验通过；部署检查 `software_ready: true`、协议静态兼容；`--sentinel-check` 通过且未打开设备。
- 本轮重跑13项关键CTest全部通过，4.09秒。日志 `logs/field-readiness-ctest.log`。
- 桌面启动器模拟生命周期全部通过；未打开真实设备。日志 `logs/field-readiness-launchers.log`。旧测试仍要求无心跳退出，与现有持续识别策略冲突，已更新为检查持续运行、不虚报通信成功、显式停止可清理。测试超时也增加模拟进程组清理。
- 源码卫生452项通过，测试文件空白检查通过。

## 此刻设备和服务状态

- `rm27-mission.service`：active，`WAIT_DEVICES`，0次启动尝试，无standard子进程，等待MCU串口及海康相机。
- `rm27-navigation.service`：active，`WAITING_ETHERNET`，定位未就绪，等待雷达网络。
- 下位机配置路径 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` 不存在；USB枚举未见配置中的海康相机。
- 两个服务仍保留自启动。接好设备后监管会自行准备视觉/任务会话，不能同时另开一个桌面自瞄抢占设备。

## 推荐首轮：原地视觉测试

先在小电脑终端暂停本轮完整任务监管，避免接线后自动准备导航任务。该操作不取消下次开机自启动：

```bash
sudo systemctl stop rm27-mission.service
cd /home/rcsdart/rm27
```

接好相机和电控USB后，只读自检：

```bash
bash tools/field_start_autoaim.sh --check
```

手动从小电脑桌面终端启动（存在 `DISPLAY` 且没有 `RM27_MISSION_DIR`）时，启动器在运行缓存中设置 `competition_headless: false`，显示 `reprojection` 窗口。开机任务及无DISPLAY终端保持无窗口。正式YAML及控制/发射开关不因显示模式变化而改动。

只跟随、不启动摩擦轮/发射的第一步测试：

```bash
bash tools/field_start_autoaim.sh --follow-only --select-color
```

选择靶板颜色；确认通信、静止靶指向及双旋向跟踪正常，由用户操作遥控器授予AUTO。结束当前窗口后，进行正式射击测试：

```bash
bash tools/field_start_autoaim.sh --select-color
```

先验证静止靶，再各做一组顺时针和逆时针旋转靶，分别记录实际发数、命中数、偏向及距离。首轮先保持参数不变，避免多参数同时调整。日志会自动记录；未经独立测量，不能直接把视觉距离分布当测距精度。

恢复完整任务时，先结束手动自瞄，回到正确P00位置与朝向并接回雷达，再执行：

```bash
sudo systemctl start rm27-mission.service
```

等定位与任务就绪后，先保持非AUTO，再由用户重新拨双上触发任务。当前Sentinel V2双下是ROTATION档，停止使用左上右中STOP档，不套用旧“双下停机”说明。


2026-10-30 正面灯条宽轮廓修复已编入新的正式程序；参数未变，旧指纹保留作历史记录。当前部署身份与验证见 [正面灯条修复](frontal_lightbar_fix.md)。


后续已按实车反馈收紧发射门限并修复图像姿态短缺口导致的重复学习，当前配置和程序身份见 [门限与首发等待](fire_gate_and_warmup.md) 及 `logs/fire-gate-warmup-deployment.json`；上文旧指纹为各轮历史记录。
