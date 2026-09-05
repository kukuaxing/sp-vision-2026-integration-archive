# 2026-09-05 固定靶实弹与暂停状态

## 封存时状态

- 机器人已由现场人员确认回到真正安全档；上位机 `standard` 与监控进程均已停止。
- 电控同学接管调试，视觉侧暂停，不应自动恢复 AUTO、摩擦轮或拨弹轮动作。
- 小电脑当前启用配置已恢复到 12.8 m/s 基线；2.7 m 修正配置仅作为未验证实验文件保存。
- 相机与 CH340 在停止前均能枚举；不以此替代下一次上电前的现场安全检查。

## 已烧录下位机固件

- 功能：3200 rpm 摩擦轮、600 ms 发射节拍、完整 45° 拨弹闭环锁存、摩擦轮达到目标转速 90% 并稳定 200 ms 后才允许供弹；低于 70% 暂停，恢复到 90% 后继续。
- AUTO 退出、遥控失效或串口指令过期时复位发射状态。
- 主机回归测试与 ARM Release 构建通过；烧录、校验、独立回读及复位运行成功。
- 固件：`firmware/stm32f405-xuc-v2/artifacts/flashed-phase2d5-rpm3200-cadence600-feed-latch-20260905/stm32f405_xuc_v2_rpm3200_cadence600_feed_latch.bin`
- 长度：42,692 字节。
- SHA-256：`aaaf4c33531b37dd720b811db5c4e571b807b429bac18e6f51dc37d627862571`。
- `readback_42692.bin` 的长度和 SHA-256 与固件完全相同。

## 实车结果

1. 空仓验收通过。
2. 第一轮 5 发全部完成，落点依次为：下、下、右下、右下、右下。
3. 后续一轮首次发射后相机 USB 掉线；内核记录 `usbfs -19`、USB 2-2 disconnect 与端口描述符 `-110`。机械同学重新固定相机线后继续隔离验证。
4. 固定线缆后的 12 秒 AUTO 禁射测试：摩擦轮空转、云台跟随，相机稳定且拨弹轮不动。
5. 单次空拨弹测试：相机稳定。
6. 单发实弹测试：相机稳定，实际发射 1 发，落点为下。
7. 随后两轮 5 发均全部完成；第一轮未记录落点，第二轮落点依次为：左下、左下、下、左、左下。
8. 第二轮靶距 2.7 m，弹着群中心约向左 10 cm、向下 20 cm。

## 弹道修正试验结论

- 由 2.7 m、左 10 cm、下 20 cm 计算的几何修正约为：向右 2.121°、向上 4.236°。
- 当前 12.8 m/s 模型在该距离的弹道补偿约 10.7°；按落点反算，所需总补偿约 14.6°，等效拟合弹速约 9.53 m/s。
- 9.5 m/s 空弹试验的运动方向符合预期，但物理 Pitch 约到 -14.16° 时装甲板从画面下方离开，系统因失去目标未取得有效计数。
- 该试验不能视为通过，也不能据此实弹。若要完整补偿，优先评估将相机光轴相对炮管向下调整约 4°并重新标定外参；否则只能接受部分补偿。不得绕过失去目标后的禁射保护。
- 上位机源码已加入可选的 `xuc_direct_yaw_impact_correction_deg` 支持，默认值为 0；当前启用基线配置未打开该修正。

## 配置快照

- 当前安全基线：
  - `changed-files/configs/standard_hikrobot_phase2d5_fire_single.yaml`，SHA-256 `a184bd38364e848f045c0e92f45e0d9ddba252d388a5d7c3ece57eda23be2edd`
  - `changed-files/configs/standard_hikrobot_phase2d5_fire_dryrun.yaml`，SHA-256 `a07c35e9f55157667532137353d2f35c211b6dce37d100a98d3952099d10c211`
- 未验证实验配置：
  - `changed-files/configs/standard_hikrobot_phase2d5_impact_2p7m_live_UNVALIDATED.yaml`
  - `changed-files/configs/standard_hikrobot_phase2d5_impact_2p7m_dryrun_UNVALIDATED.yaml`
- 当前上位机源码快照：`changed-files/src/standard.cpp`，SHA-256 `2c96baaea61c4233faaf1eb3884e83f06bb6060649c40cf0cfb84dc9575be094`。

## 日志索引

- `feed_latch_live5_125341.log`：发射锁存后的首轮成功 5 发。
- `feed_latch_live5_round2_retry_130541.log`：相机 USB 掉线对应轮次。
- `camera_isolation_wheels_dryrun_131005.log`：固定线缆后的摩擦轮/云台禁射隔离测试。
- `camera_isolation_empty_feed_133431.log`：单次空拨弹隔离测试。
- `camera_impact_single_live_133707.log`：固定线缆后的单发实弹测试。
- `feed_latch_live5_after_cable_fix_134025.log`：固定线缆后的首轮 5 发。
- `feed_latch_live5_repeat_134436.log`：2.7 m 落点记录轮次。
- `impact_correction_dryrun_140502.log`：9.5 m/s、水平修正试验；因目标离开视场而未通过。
- 同名 `_monitor.log` 保存对应的 USB/设备监控输出。

下一次恢复视觉调试前，必须重新确认：电控调试已结束、机器人回真正安全档、弹仓和拨弹轮状态明确、射界无人、现场人员同意视觉侧重新接管。
