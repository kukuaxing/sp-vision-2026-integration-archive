# SP Vision 自瞄最终归档（2026-09-12）

本目录保存 2026-09-12 从现场小电脑和开发工作区汇总的最新可恢复版本。归档不含
SSH 私钥、构建目录、临时备份、阶段配置和海量运行日志。

## 归档内容

- `vision/source-snapshot/`：完整源码树，共 394 个文件。它以源码分支
  `feat/xuc-board-communication-20260901` 的 `ed41007501b34ab2e13a125d7d2820a180829a33`
  为基线，叠加 27 个已跟踪最终改动和 25 个选定的新源码、测试、配置及脚本文件。
- `vision/binary/standard`：2026-09-12 从小电脑当前 `build/standard` 取回的实机程序。
- `vision/config/standard_hikrobot_phase2d5_rpm6500_gyro_oneface_live_trim141_delay250_UNVALIDATED.yaml`：
  当前实车配置。
- `vision/launcher/`：现场一键启动和停止脚本。
- `AUTOAIM_FINAL_REPORT_20260912.md`：功能、参数、测试和未决风险说明。
- `SHA256SUMS.txt`：关键文件 SHA-256 校验值。

## 在小电脑恢复

将 `vision/source-snapshot/` 恢复为工程目录并按原有 CMake 流程重新构建。若直接恢复
归档二进制，应同时恢复配置和脚本，并执行：

```bash
chmod +x build/standard tools/field_start_autoaim.sh tools/field_stop_autoaim.sh
sha256sum build/standard
```

期望 `build/standard` 的 SHA-256 为：

```text
d97ba9af270b1598c0b543bcc882db2a1cfd4f3cf3f234ed2f25c7c95cfb01c6
```

现场启动前必须核对串口、CAN、摩擦轮状态和安全区域。配置中
`xuc_allow_shoot: true`，恢复后不是空发/只跟随模式。

## 版本边界

本归档代表当前代码和参数，不等于所有最新补偿都已经完成闭环实弹验收。尤其是
`xuc_direct_yaw_impact_correction_deg: -2.12` 为针对持续偏左的最新修正，归档时尚未收到
修正后的下一轮命中反馈；配置文件名保留 `UNVALIDATED` 以避免误解。
