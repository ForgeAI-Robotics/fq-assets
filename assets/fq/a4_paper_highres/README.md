# 加密物理网格实验 v0.1.0 — 未通过验收

峰值边长变化 1.02037%，超过原有 1% 门槛，因此仅发布为独立实验资产，原 a4_paper v0.1.2 保持不变。碰撞路径复核通过不等于整体物理验收通过。参考运行模拟 7 秒约耗时 915 秒，不支持实时控制。

当前为 29×41 网格：1189 顶点、2240 物理三角面，是上一版面数的 4 倍。默认视频为 1920×1080。没有额外生成不参与碰撞的表面细分皱纹。仍是阶段性加密原型，不声明照片级真实感或网格收敛。

运行 `python tools/a4_paper/run_ipc_highres.py`，输出到 `outputs/a4_ipc_highres`。有 CUDA 时使用 GPU float64 Cholesky 解线性方程，否则使用 CPU；IPC 碰撞和其余部分仍主要在 CPU。默认 12.5 ms 步长，总计 7 秒。

挤压和桌面法向惩罚按顶点分摊面积加权，避免网格加密导致总外加载荷增长；塑性屈服角按网格间距缩放至 0.09 rad，这是曲率阈值的一阶近似，未实物标定。粗网格同规则对照见 ipc_area_scaled_coarse_control.json，上一版记录见 lowres_v012_validation.json。两种不同规则下的结果不能直接当作网格收敛证据。

逐顶点接触力见 ipc_contact_forces.npz（1189 顶点）；`python tools/a4_paper/export_contact_forces.py --mode a4_ipc_highres` 可重建。单位 N、世界坐标、顶点索引对应 ipc_trajectory.npz，非实测传感器力。

