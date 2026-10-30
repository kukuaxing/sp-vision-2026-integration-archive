# 当前自瞄参数索引（2026-10-18）

唯一数值来源：`configs/standard.yaml`。本页从副本的当前 YAML 生成，整理前后所有解析值一致，没有另建运行配置。配置注释中的“当前调参值”用于快速定位。

副本路径 `/home/rcsdart/rm27_1018`，继承源 `/home/rcsdart/rm27`。这是一份继承参数快照，不是新增实测或推荐参数。程序启动时读取，启动器生成缓存并可能覆盖颜色；修改正式配置需由用户重新启动现场程序后生效。

## 协议与许可

| 参数 | 当前值 | 单位／适用范围 |
|---|---|---|
| `board_protocol` | `"SENTINEL"` | 当前后端 |
| `xuc_serial_baud` | `921600` | bit/s |
| `sentinel_tx_hz` | `50` | Hz，发送上限 |
| `sentinel_valid_for_ms` | `100` | ms，引用有效期 |
| `xuc_allow_control` | `true` | 视觉控制配置许可 |
| `sentinel_allow_friction` | `true` | 摩擦轮配置许可 |
| `xuc_allow_shoot` | `true` | 射击配置许可 |
| `auto_fire` | `true` | 自动开火资格开关 |
| `sentinel_require_verified_pose` | `false` | false 为沿用既有标定，不代表认证完成 |
| `sentinel_friction_lost_timeout_ms` | `5000` | ms，正常丢实际检测后的摩擦轮保持；不是命令租约 |

## 相机与检测

| 参数 | 当前值 | 单位／适用范围 |
|---|---|---|
| `enemy_color` | `"blue"` | 敌方颜色；启动器可用保存颜色覆盖运行缓存 |
| `camera_name` | `"hikrobot"` | 相机后端 |
| `camera_width` | `1440` | px |
| `camera_height` | `1080` | px |
| `exposure_ms` | `6.0` | ms |
| `gain` | `10.02` | SDK 增益 |
| `camera_fps` | `150.0` | FPS 设定，不是实测吞吐率 |
| `yolo_name` | `"yolo26"` | 检测后端 |
| `device` | `"CPU"` | OpenVINO设备 |
| `min_confidence` | `0.1` | 普通置信度门限 |
| `yolo26_candidate_confidence` | `0.04` | 候选门限；仍受颜色/几何等规则限制 |
| `yolo26_color_confirmed_min_confidence` | `0.04` | 当前图像颜色确认候选门限 |
| `yolo26_memory_confirmed_min_confidence` | `0.06` | 近期颜色记忆确认门限 |
| `yolo26_refine_corners` | `true` | 灯条角点修正 |
| `corner_refine_brightness_invariant_subpixel` | `false` | 新端点候选已编译，命中留出未通过，保留旧路径 |
| `use_roi` | `true` | 已有自适应ROI搜索；失配回到全图 |

## 跟踪与命令平滑

| 参数 | 当前值 | 单位／适用范围 |
|---|---|---|
| `min_detect_count` | `5` | 检测确认计数 |
| `max_temp_lost_count` | `15` | 短时丢失计数 |
| `tracker_association_max_cost` | `2.0` | 关联代价门限 |
| `tracker_target_switch_min_frames` | `3` | 切换确认帧数 |
| `xuc_direct_yaw_guard` | `true` | 直测水平守卫 |
| `xuc_direct_pitch_guard` | `true` | 直测俯仰守卫 |
| `xuc_final_yaw_command_max_rate_dps` | `30.0` | deg/s，最终水平命令变化率 |
| `xuc_direct_pitch_max_rate_dps` | `45.0` | deg/s，俯仰命令变化率 |
| `xuc_direct_yaw_hold_ms` | `300.0` | ms，短失测保持 |
| `xuc_direct_pitch_hold_ms` | `180.0` | ms，短失测保持 |

## 开火门限（按代码分支适用）

| 参数 | 当前值 | 单位／适用范围 |
|---|---|---|
| `xuc_fire_guard_enabled` | `true` | 开启后使用生产代码中的对准/新鲜度等门槛 |
| `xuc_fire_yaw_tolerance_deg` | `1.5` | deg，固定靶直测水平窗口 |
| `xuc_fire_pitch_tolerance_deg` | `0.75` | deg，俯仰窗口，发送前还会复查姿态 |
| `xuc_fire_stable_ms` | `120.0` | ms，固定靶连续对准；不是全模式统一等待时间 |
| `xuc_fire_require_tracker_converged` | `true` | 要求跟踪器收敛 |
| `xuc_gyro_predicted_yaw_tolerance_deg` | `2.25` | deg，旋转预测分支的水平窗口 |
| `xuc_event_gimbal_tracking_tolerance_deg` | `3.0` | deg，高速事件分支云台跟随误差窗口 |
| `xuc_gyro_fire_min_consecutive_frames` | `2` | 预测非事件分支帧数；有效事件分支使用既有1帧规则 |
| `xuc_gyro_fire_stable_ms` | `30.0` | ms，仅非预测且单面策略分支 |
| `xuc_gyro_require_event_lock_for_fire` | `true` | 高速分支要求事件锁定 |
| `xuc_event_phase_grace_deg` | `4.0` | deg，现有相位不确定度预算附加量 |
| `xuc_event_require_face_consistency` | `false` | 未来装甲面一致性门槛开关 |

## 时延与弹道

| 参数 | 当前值 | 单位／适用范围 |
|---|---|---|
| `high_speed_delay_time` | `0.085` | s，Aimer高速预测的额外发弹延迟 |
| `low_speed_delay_time` | `0.085` | s，Aimer低速预测的额外发弹延迟 |
| `xuc_event_fire_delay_s` | `0.085` | s，事件调度的命令到出膛名义延迟，尚非本轮实测 |
| `xuc_event_fire_delay_min_ms` | `65` | ms，事件调度采用的不确定区间下界 |
| `xuc_event_fire_delay_max_ms` | `105` | ms，事件调度采用的不确定区间上界 |
| `xuc_shot_delay_adaptation_enabled` | `false` | Sentinel无拨弹反馈，此后端不启用自动学习 |
| `xuc_projectile_speed_mps` | `19.36` | m/s，沿用估计值，不是测速结果 |
| `xuc_ballistic_pitch_enabled` | `true` | 俯仰弹道补偿 |
| `xuc_ballistic_pitch_trim_deg` | `-2.32915` | deg，当前累计经验落点修正 |
| `xuc_camera_forward_of_muzzle_m` | `0.06` | m，沿用相机/枪口几何 |
| `xuc_camera_above_muzzle_m` | `0.04` | m，沿用相机/枪口几何 |

## 标定和兼容参数

`camera_matrix`、`distort_coeffs`、`R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`、`t_muzzle2gimbal`、`gun_forward_in_gimbal` 全部原样保留，数值以 YAML 为准；没有在整理中重标或从别的工程覆盖。

`rmv_*` 只适用于 RMV 调试后端。`actuation_delay_ms` 属于 Planner 路径，不能代替当前 `standard` 的 Aimer/事件延迟。已关闭功能和未选模型对应的参数保留兼容性，不把它们列为当前必然生效项。

120 ms 连续对准与 85 ms 发弹延迟是不同概念；后者用于预测/事件调度，不是在当前图像发现目标后额外等85ms。程序输出请求也不等于实际出膛或命中。

### 角点测量行为（2026-10-18）

`yolo26_refine_corners=true`时，使用RGB/BGR顺序无关的红蓝色差提取两根实际灯条端点，排除白色数字和面板的连通干扰。提取失败时，网络框/角点仍可显示，但`pose_observation_valid=false`，Solver不会把它们解释为有效位姿；已有轨迹按原丢失门槛暂时预测，当前帧不计为测量更新。未引入历史角点平移、端点缩放或经验距离补偿，也未修改关联、开火或延迟阈值。配置中的false仍是显式仅用网络角点的旧选项，本次未切换到false。
