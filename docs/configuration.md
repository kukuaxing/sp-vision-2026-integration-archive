# 配置与标定状态

只编辑 `configs/standard.yaml`。启动器读取它并生成 `logs/xuc-bringup/field_autoaim.runtime.yaml`，仅覆盖已保存的敌方颜色。不要修改这个生成文件；下次启动会重建。

颜色保存在 `~/.config/sp-vision/field_autoaim_enemy_color`。标定工具使用 `configs/calibration.yaml`，其历史外参与正式运行配置并不相同；二者用途不同，不能互相覆盖。

用户最新安装测量：机器人中心为原点，X前、Y左、Z上，右手系；光心(15,0,50) cm，机身(10,0,50) cm，炮口(10,0,45) cm。测量记录尚未应用。代码中的相机、云台、IMU坐标原点和轴定义仍需核对，再完成标定。

标定源文件在 `calibration/`；原始采集资料集中保留于小电脑 `data/`。旧实验配置和旧报告已清除，工作区不保留候选参数副本。
