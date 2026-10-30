# A4 可揉皱纸团原型：IPC 版本

当前默认是松散纸团的静态快照，平纸保存在 preview.xml/usda。运行物理请使用 `tools/a4_paper/run_ipc.py`，不是 MuJoCo / Isaac Sim 原生纸张求解。

已验证 315 顶点、560 面的 A4 纸经过球形软压缩、释放、落桌后保持团状。每次接受的 Newton 变形更新经过 IPC 连续碰撞检测，导出的完整路径另行复核。验证与材料限制见 ipc_acceptance.json、ipc_validation.json、ipc_parameters.json。更紧的压缩超过本版 1% 边长误差门槛，保留为实验记录。

安装 `pip install -r tools/a4_paper/requirements-ipc.txt`，运行 `python tools/a4_paper/run_ipc.py --nx 15 --ny 21 --steps 180 --settle 100 --radius .06 --out outputs/a4_ipc`。加 `--elastic` 关闭塑性作对照。CPU 即可运行，当前是离线原型，不声明实时机器人控制。

`crumple_playback.usdc` 保存内部连续检测路径的非均匀时间采样。不要只保留每个物理时间步的端点再做直线插值，否则可能重新引入穿插；原始端点轨迹和完整路径分别存于 ipc_trajectory.npz 与 ipc_ccd_path.npz。USD/MuJoCo 快照没有原生柔性物理或碰撞代理。

压缩装置是外部软势能场，允许有限压缩超界，不代表刚性球壳。纸层接触无摩擦、材料未标定；未实现手指揉纸、撕裂、纤维方向或湿度影响。以下为旧版压折实验的历史说明：

# A4 塑性纸张：压折实验原型

尺寸 210×297 mm，按 80 g/m² 办公纸设置目标质量 4.99 g。默认 625 顶点、1152 三角面。物理厚度假设 0.1 mm，压折仿真的数值层间距为 0.6 mm。参数没有用实纸标定。

已完成边缘运动学夹持折叠、压条压折及释放实验。紧密揉团实验存在过大的边长变形与面片穿插，尚不满足纸团验收；不能把折叠通过等同于揉纸成功。三角面穿插检查仅为离散抽样，不是连续碰撞保证。

物理求解使用 MIT 开源 kami / NVIDIA Warp，FQ 对方形网格做矩形适配。MuJoCo 仅渲染网格；preview.xml/usda 为静态外观，fold_playback.usdc 是烘焙动画，不含 PhysX 纸张物理。当前没有机器人接触力耦合。

在独立 Python 环境安装 `pip install -r tools/a4_paper/requirements.txt`，然后运行 `python tools/a4_paper/run_demo.py --mode fold --output outputs/a4_paper`。关闭塑性对照加 `--elastic`。`--mode crumple --N 33` 为尚未通过的紧密挤压实验，默认不运行。推荐有 CUDA 的 Torch 与 Warp；当前在 RTX 4080 SUPER / Torch 2.5.1+cu118 / Warp 1.16 验证。

入口工具以运动学方式控制，force 参数是上游折痕模型的输入载荷，不代表机器人力传感器读数。矩形适配尚未通过连续体材料和各向同性标定。来源、MIT 许可和源码差异均保留。
