# XUC Phase 2D.5 yaw 收敛修正交接

日期：2026-09-03

当前分支：`fix/xuc-v2-yaw-convergence-phase2d5-20260903`

固件代码提交：`4fb76dbba388e9fb09e2dd8b8b8ed1d253d92105`

## 当前状态

- Phase 2D.5 已完成代码修改、ARM Release 构建和主机侧测试。
- **尚未烧录到机器人，尚未实车验证。**
- 本轮停止原因是机器人正在电控同学手中，Windows 也未枚举到无线调试器。
- 继续前不要把本提交标记为真机通过。

## 实车证据和结论

1. Phase 2D.3（`4905e59`）的 AUTO 符号能实际收敛：
   - 画面目标 yaw 误差从约 `+16.82 deg` 降到 `+3.39 deg`。
   - 云台 yaw 从约 `-42.63 deg` 变到 `-31.24 deg`。
   - 但在约 `3.4 deg` 残差处停滞，未完全居中。
2. Phase 2D.4（`331f080`）把电机输出改为 `-delta` 后发散：
   - 目标误差从约 `+11.63 deg` 增大到 `+31.66 deg`。
   - 云台 yaw 从约 `-22.79 deg` 变到 `-48.90 deg`。
   - 约 2 秒后目标离开画面/被丢失。
3. 手动双向测试中，yaw 左右均能正常运动，排除单向机械卡死、单向电机或 CAN 执行故障。
4. DR16 手动操纵为“右推左转、左推右转”，证明手动 `rc.ch[0]` 映射需要单独取反。这不能用来推翻 AUTO 坐标链的已有实车证据。

## Phase 2D.5 修改

- `STM32F405/RC.cpp`
  - 手动 yaw 改为 `-rc.ch[0]`，预期右推右转、左推左转。
- `STM32F405/control.cpp`
  - AUTO 恢复经实车证明收敛的 `yaw_speed_command = delta * 1.2f`。
  - 输出上限恢复为 `180`，低于 6020 的 `260` 速度上限。
  - 静摩擦最小启动速度由 `80` 提高到 `120`。
  - 加入启停滞回：误差超过 `24` 机械刻度（约 `1.05 deg`）才启动，进入 `10` 刻度（约 `0.44 deg`）后停止。
- `STM32F405/control.h`
  - 增加 `auto_yaw_motion_active`滞回状态，模式切换时清零。

XUC 协议、AUTO 解锁安全门、串口超时保护、射击禁用逻辑均未改动。

## 构建和测试

- ARM GCC：`10.3.1 20210824`
- STM32CubeF4 commit：`b5abca20c9676b04f8d2885a668a9b653ee65705`
- Legacy HAL commit：`b6e8197518e97d3fe9243fe49a692033b9d7d734`
- `STM32_FIRMWARE_BUILD=PASS`
- `XUC_PHASE2_YAW_GATE_TEST=PASS`
- `XUC_V2_PROTOCOL_TEST=PASS`
- `PHASE2D5_SOURCE_INVARIANTS=PASS`
- 初始栈指针：`0x20020000`
- Reset vector：`0x08007789`

构建产物（本机未跟踪目录）：

`build-arm-release-phase2d5/stm32f405_xuc_v2.bin`

- 大小：`41252` bytes
- SHA256：`ea442f2a09f1e961c87b313c4458527567cb540f3b29847d7d9041733d4a011b`

## 下次继续顺序

1. 清空弹丸，禁用摩擦轮和拨弹机构，保证云台转动范围无人。
2. 连接无线调试器，核对待烧录 bin 的 SHA256 为上述值。
3. 烧录、OpenOCD verify，然后回读同长度闪存并比对 SHA256。
4. 先用 `UP+MID` 低幅摇杆复测手动方向：右推应右转，左推应左转。
5. 再用禁射配置做 AUTO 定点测试：目标从画面左、右两侧分别进入，核心判据是 `target_gimbal_deg` 的绝对值持续减小。
6. 如误差连续增大、立即丢目标、持续高频换向或异响，立即退出 AUTO，保留上位机日志，不继续提高输出。
