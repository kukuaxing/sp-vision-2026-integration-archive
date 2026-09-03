# STM32F405 XUC V2 固件

## 机器人当前状态

机器人当前烧录的是 Phase 2D.4，对应提交：

`331f080c9dd0d213e513a5c0d412421aca2ebae8`

产物位于 `artifacts/flashed-phase2d4/`。其中 `flash_phase2d4_20260903_144809.log` 和 `readback_phase2d4_20260903_144828.log` 记录烧录与回读验证。

## 下一候选

Phase 2D.5 固件代码提交：

`4fb76dbba388e9fb09e2dd8b8b8ed1d253d92105`

归档源码 HEAD（含交接文档提交）：

`5ab5e2af13ffa3bf96f558755346d6e2e0dddf70`

D5 构建产物位于 `artifacts/pending-phase2d5/`。它已经构建并通过主机侧测试，但尚未烧录和实车验证；继续工作前请先阅读 `source/docs/XUC_PHASE2D5_YAW_CONVERGENCE_HANDOFF_20260903.md`。

## 校验重点

- D4 bin：`cf13b6e7ea65106bd1f43ba8be39daf64236a4e5e9e90e3df88454fd5151173d`
- D5 bin：`ea442f2a09f1e961c87b313c4458527567cb540f3b29847d7d9041733d4a011b`

`history/RM_Infantry-Robot-xuc-v2.bundle` 包含原固件仓库的完整本地 Git refs，可用 `git clone RM_Infantry-Robot-xuc-v2.bundle <目录>` 恢复。
