# STM32F405 XUC V2 固件

## 机器人当前状态

机器人当前烧录的是 2026-09-04 Phase 2D.5 双轴实车版。产物位于 `artifacts/flashed-phase2d5-joint-20260904/`。

- bin SHA-256：`7708D30EE510AB4807379175C4314B1D413BD3F96CB01A3F93A92B07937A1F21`
- 长度：41988 字节
- CMSIS-DAP 烧录成功，同长度 Flash 回读 SHA-256 完全一致。
- Yaw、Pitch 和双轴实车跟随已通过；开火未测。

## 历史候选

Phase 2D.5 固件代码提交：

`4fb76dbba388e9fb09e2dd8b8b8ed1d253d92105`

归档源码 HEAD（含交接文档提交）：

`5ab5e2af13ffa3bf96f558755346d6e2e0dddf70`

`artifacts/pending-phase2d5/` 保留了 2026-09-03 当时尚未烧录的旧 D5 候选，仅供历史对比，不再是当前机器人版本。

## 校验重点

- D4 bin：`cf13b6e7ea65106bd1f43ba8be39daf64236a4e5e9e90e3df88454fd5151173d`
- D5 bin：`ea442f2a09f1e961c87b313c4458527567cb540f3b29847d7d9041733d4a011b`
- 当前双轴 D5 bin：`7708D30EE510AB4807379175C4314B1D413BD3F96CB01A3F93A92B07937A1F21`

`history/RM_Infantry-Robot-xuc-v2.bundle` 包含原固件仓库的完整本地 Git refs，可用 `git clone RM_Infantry-Robot-xuc-v2.bundle <目录>` 恢复。
