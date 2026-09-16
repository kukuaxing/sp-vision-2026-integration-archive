# 构建与维护

小电脑工程：`/home/rm/sp_vision`；Windows 仓库：`Documents/GitHub/sp-vision-2026-integration-archive`。

## 构建

新克隆先按 [依赖说明](dependencies.md) 准备模型与SDK；现有小电脑资源未删除。

在工程根目录执行 `bash scripts/build.sh`，默认以2个任务构建 `standard`。
可用 `bash scripts/build.sh standard calibrate_camera calibrate_handeye verify_handeye` 选择目标。
依赖沿用现场 ROS 2 Humble、OpenVINO 2024.6、OpenCV、Eigen、Ceres、fmt、spdlog、yaml-cpp 与 MVS 环境。`librm/` 使用固定提交的 Git 子模块；新克隆使用 `git clone --recurse-submodules`，已有克隆使用 `git submodule update --init --recursive`。小电脑依赖完整保留，不依赖旧 worktree。

编译产物只放在 `build/`。不要为每轮修改新建构建目录、复制配置或源码文件。修改前后使用 `git status`、`git diff` 核对，不创建备份副本。

## 运行入口

现场沿用桌面“启动自瞄”和“停止自瞄”，分别指向 `tools/field_start_autoaim.sh` 和 `tools/field_stop_autoaim.sh`。脚本相对自身定位工程，不再写死旧目录。
不启用新的开机自启动。运行日志位于 `logs/xuc-bringup/日期/`，跨日期、跨颜色保留最近20份一键日志；这不是单文件大小限制。

## Git

只维护当前文件。本次按用户要求将仓库重建为单次提交，删除旧标签、旧提交及本地历史备份。以后继续在同一目录修改；提交和推送按用户要求进行，不另建归档分支、标签或副本。

## 提交前检查

运行 `python3 scripts/check_source_tree.py` 检查跟踪文件和忽略规则，再执行 `git diff --check`。`.gitignore` 不会自动移除已经跟踪的文件，产物必须从索引中取消跟踪。
