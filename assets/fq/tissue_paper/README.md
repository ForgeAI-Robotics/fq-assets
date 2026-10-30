# 卫生纸片柔性揉团原型

100×120 mm 等效单张薄片，30 g/m²、质量 0.36 g。参数为演示假设，不对应指定品牌、层数或实测材质。相比 A4 原型降低弯曲和拉伸刚度，保留塑性与摩擦。没有撕裂、吸水、分层、穿孔撕线或手指操作。

安装 `pip install -r tools/tissue_paper/requirements.txt` 后运行 `python tools/tissue_paper/run_demo.py`。默认输出 `outputs/tissue_ipc`，7 秒仿真、12.5 ms 步长。CPU 或 CUDA 可用。当前是外部求解器，MuJoCo/USD 为快照和烘焙回放，不是原生柔性物理。

`preview.xml/usda` 是纸团，`flat.xml/usda` 是平纸；`crumple_playback.usdc` 保存连续碰撞复核过的内部路径。contact_forces.npz 按轨迹顶点顺序记录世界坐标下的自接触法向/摩擦力、桌面力，单位 N；不是实测传感器数据。

本演示门槛：峰值边长变化 <1%、未检测到路径穿插、末半秒最大顶点移动 <1 mm，最终外包围盒最长边 <90 mm、最短边 >15 mm。仅表示所给松散揉团测试通过，不代表真实卫生纸已标定或任意操作可靠。
