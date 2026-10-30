# 仿真入口

## 当前：福建省赛固定中心旋转靶

参数来源、真实运行结果及边界见 [固定旋转靶仿真说明](../docs/fixed_target_simulation.md)。

```bash
cd /home/rcsdart/rm27
python3 simulation/run_fixed_target.py --suite --stress --delay-sweep
python3 simulation/validate_fixed_target.py --repeat
```

先按说明编译 `fixed_target_sim`。当前按现场300血、50发预算、30次命中完成评估，不向控制器反馈理想HP。完整套件含20组400轮、多种子压力测试及延迟扫描。输出 `logs/fixed-target-simulation.json/.html`，HTML可离线播放、切换旋向和初相位。复用正式几何/跟踪/预测算法及短漏检修复，支持有限云台和误差注入；未运行真实设备或完整电控链路，假设误差下的几何命中不能作为实车命中率。

已新增图像入口，按说明第7节编译 `fixed_target_image_sim`：

```bash
python3 simulation/run_fixed_target_images.py
python3 simulation/validate_fixed_target.py --result logs/fixed-target-image-simulation.json
python3 simulation/run_fixed_target_image_stress.py
python3 simulation/validate_fixed_target.py --result logs/fixed-target-image-stress.json
```

图像按1440×1080渲染后进入现有CPU YOLO26、颜色与角点精修，不补真值检测。基础17轮含空白负对照；中等物理误差联合12轮共539/600合格几何命中（89.833%），两旋向均6/6完成，最差84%。仍是合成纹理、背景与物理分布，未测真实相机或实弹。输出 `logs/fixed-target-image-simulation.json/.html` 和 `logs/fixed-target-image-stress.json/.html`；图像HTML为离散截图，几何HTML提供运动回放。实际模型、正式配置、源码及二进制指纹附于结果JSON，完整54项离线测试通过。

继续优化使用以下独立入口，详见 [脱靶归因与时序标定](../docs/fixed_target_optimization.md)：

```bash
python3 simulation/optimize_fixed_target.py
python3 simulation/optimize_fixed_target.py --holdout
python3 simulation/optimize_fixed_target.py --holdout --delay-sensitivity
python3 simulation/optimize_fixed_target.py --images --holdout
```

原基础入口仍作为0额外补偿的基准；优化入口在开发组选择并冻结额外预测补偿，再在独立种子/相位中复核，输出 `logs/fixed-target-timing-*.json/.html`。没有改上车参数。固定补偿对不同真实出膛均值可能退步，报告保留反例；20ms等仿真标定值不能直接用作实车测量值。

统一+20ms未通过真实YOLO复核，第一阶段记录保留作反例。第二阶段按模型旋向分别标定，必须使用全新的最终留出组：

```bash
python3 simulation/validate_directional_timing.py
python3 simulation/validate_fixed_target.py --result logs/fixed-target-directional-timing.json
```

输出 `logs/fixed-target-directional-timing.json/.html`。第一轮图像组在第二阶段明确作为开发数据，最终测试换新种子与相位，不能把同一批数据一边调参一边称为独立验证。

前轮图像留出结果：原逻辑531/600（88.50%），分旋向标定552/600（92.00%），最差一轮80%→88%，两旋向均6/6完成。该标定保持原物理难度和预算，只在离线入口应用；实车仍需重新测量标定。

继续比较俯仰标定、小半径选面与正式程序已有的历史视觉残差路径：

```bash
python3 simulation/validate_selective_fire.py --screen
python3 simulation/validate_selective_fire.py --image-screen
python3 simulation/validate_selective_fire.py
python3 simulation/validate_fixed_target.py --result logs/fixed-target-selective-images.json
```

先生成角点开发组，再生成图像开发组，最终入口冻结开发组选定策略并使用新种子/相位。原有分旋向补偿不重选；新俯仰值来自历史标注落点，只用于离线场景。小半径方案因耗时增加且无额外命中收益已排除。方法与反例见优化说明第9节，结果 `logs/fixed-target-selective-*.json/.html`。

本轮新图像留出组最终为：既有分旋向573/600（95.50%），增加-0.12°俯仰标定572/600（95.33%），最差92%降至90%，均12/12完成。因此不采用新增俯仰补偿，保留上一轮方案。前轮92%与本轮95.50%来自不同数据组，不能当作优化增益。独立弹道复算、三份当前源码严格审计及54/54回归测试通过；决策见 `logs/fixed-target-selective-decision.json`，未修改上车参数。

几何偏差已单独定位，见 [约17厘米距离偏差调查](../docs/fixed_target_geometry.md)。新增 `fixed_target_geometry_probe` 与 `analyze_fixed_target_geometry.py`，在八项离线消融中复现端点收缩→PnP深度偏差→中心偏差；恢复长度的真值消融和影子模型仅用于诊断，不构成可上车的校正策略。正式参数未修改。

端点候选已完成八组几何与新12轮图像留出：模拟中心偏差16.85→2.77cm，但命中556→551/600、最差90%→88%，正式时间小组91→87/100，保持 `corner_refine_brightness_invariant_subpixel: false`。候选能力和诊断已编入正式程序，未默认启用或改实车延时/PID。运行 `validate_endpoint_deployment.py --geometry` / `--images` 可重做主对照；`check_endpoint_formal_policy.py` 核对正式时间设置，`check_endpoint_recorded.py` 做固定真实录像回归（拒绝覆盖旧输出，重跑前需明确处理原结果）。部署状态见 [部署记录](../docs/simulation_deployment.md)，优先工作见 [离线计划](../docs/offline_vision_roadmap.md)。

继续定位测距改善却少中的原因：`run_endpoint_ablation.py` 在明确重用的旧开发子集中比较有限跟随、固定相机、瞬时跟随；`fixed_target_counterfactual` 冻结原下发/误差做时刻与角度干预；`score_frozen_uncertainty.py` 做已假设随机项的评估端移除。以上均为诊断，不是新命中率验证或上车算法。复现、限制和当前审计见 [分离对照](../docs/endpoint_ablation.md)。正式端点候选仍false。

后续完整闭环的中心机会筛选入口为 `study_centered_opportunities.py --stage development|holdout|stress`。在全新开发组比较原策略、3°/6°，冻结唯一方案再检验新种子/相位和10/40ms假设出膛抖动；采用正式预测时间与历史视觉校正，不继承旧离线单向20ms。默认0保留原发射行为，非零只收窄资格，所有结果按固定50发预算计分。完整方法、反例与是否采用见 [机会筛选实验](../docs/centered_opportunities.md)，审计入口 `audit_centered_opportunities.py`。

历史非均匀间隔与横向时间余量实验：`study_recorded_cadence.py --stage development|holdout|stress`，审计 `audit_recorded_cadence.py`。只从旧观测记录提取间隔，严格累计采样时钟，忙时丢帧不推迟采集时间；处理帧计数与正式入口一致。预先固定20ms时间余量与0.2°角余量，再用全新种子/相位检验；不读取当发真实出膛延迟。完整定义、实测与假设的区别、最终结果见 [历史节奏实验](../docs/recorded_cadence.md)。

时序可观测性诊断：`fixed_target_counterfactual INPUT OUTPUT --delay-grid` 在冻结射线/序列上分离均值与随机项；`study_delay_observability.py` 构造瞬时伺服、装甲板视觉相同而出膛抖动不同的诊断孪生；`audit_delay_observability.py` 复算全部120格/40,000次评估、孪生与默认轨迹。装甲板残差/下发时刻相同仍出现98→84/100，不能用视觉一致性代替独立执行器时序观测。这些不是新命中率验证或硬件抖动测量，详见 [诊断边界](../docs/delay_observability.md)。

固定倾角的平移／yaw联合拟合已完成离线对照：`study_joint_pose.py`、`test_joint_pose_geometry.py`、`audit_joint_pose.py`。600条受控几何与3,951条旧角点重放显示，较低像素残差不能保证更准的位置，端点缩短和倾角不符时多项误差反而增大，因此不接入正式求解器。4,551条独立刚体投影复算、7项新数值／负对照测试、54项原回归通过；这轮没有发射或新命中率，详见 [联合拟合对照](../docs/joint_pose_geometry.md)。

灯条光度／外观对照新增 `fixed_target_photometry_probe`、`analyze_photometry.py` 和实拍事件分析工具 `tools/analyze_lightbar_photometry.py`。1,080张图、2,160次提取显示，旧Otsu＋亚像素候选在渐变圆头改善，在均匀平头却外扩，不能用固定像素伸长或单一纹理证明升级。2,152条有效位置独立复算、72张默认兼容、7项新测试及54项原回归通过；13条旧实拍检测仅作光度检查，没有独立准确率标签。详见 [光度与外观对照](../docs/lightbar_photometry.md)。

时序链故障矩阵入口：`study_timing_pipeline.py`、`--images`，审计 `audit_timing_pipeline.py`。仿真显式启用真实PoseHistory／发令前复查；18项关键点＋5项真实YOLO图像条件、双旋向46轮，72,201条事件与2,300条弹道独立核对。已合格后参考改变／图像超龄的8次请求均被拦截，55项回归及100条旧默认轨迹通过。动态姿态负对照说明固定内部延迟仍可能通过时效检查；这些是诊断，不是新算法收益、实测延迟或完整standard／MCU验收。详见 [时序链与边界](../docs/timing_pipeline.md)。

旧帧精确元数据准入候选：`study_frame_admission.py --stage development|holdout`，审计 `audit_frame_admission.py`。区分旧原图／帧号／时间重放与新图像配错时间；真正重放的图像留出结束时间45.488→24.488秒，但命中各97→96／100。56轮、2,800条弹道、96,746条元数据、32次拒绝和56项回归通过。252份旧日志778,364条有帧号记录未见同类异常，暂缺现场根因证据；候选不接入正式循环，心跳与完整通信仍需验收。详见 [恢复候选与优先级](../docs/frame_admission.md)。

## 历史：格式塔 drive-aim v4q 仿真归档

以下旧归档未参与本轮固定旋转靶实验；其中配置不是当前设备的运行配置。

最终候选 v4q 的完整结论见 `REPORT.md`。

- 反转：5/5 有效，150 发，120 次伤害命中，80.0%。
- 正转：5/5 有效，150 发，108 次伤害命中，72.0%。
- 合计：10/10 有效，300 发，228 次伤害命中，76.0%。

本目录保留报告、全 91 轮的 `RESULTS.csv`、聚合 `SUMMARY.csv`、唯一正式配置 `configs/standard.yaml`、最终源码 `src/gestalt.cpp`以及最终对称验证的 10 份日志。Windows 可执行文件未重复上传；报告中保留了它的 SHA-256，源码与结果足以审阅和重建。

这些结果仅说明格式塔仿真条件下的表现，不能替代真实相机、云台、主控板和弹丸闭环验证。
