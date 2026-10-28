# 蓝思取送拭子运动学示例

robot_demo.xml 是自包含场景入口（网格/纹理为仓库内相对引用），robot_trajectory.json 保存关节、底座、夹具与拭子位姿。关节限位内求解，末端误差见 JSON；底座运动是运动学回放，不是轮地动力学。示意延长夹具的 TCP 在手腕局部 -Z 方向 0.28 m。

实际夹具接口的刚体取送请运行 `python tools/run_airport_interaction.py --viewer`；整机视频见 `validation/previews/airport_operations_v030.mp4`。视频没有人物采样或仪器化学响应。

从仓库根目录运行整机回放：`python examples/airport_operations/replay_robot.py`；无窗口验证使用 `--headless`。蓝思源模型位于 assets/user_supplied/lansi_wheel_arm_robot，沿用该资产的发布说明。
