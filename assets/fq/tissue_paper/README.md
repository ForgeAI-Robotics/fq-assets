# 卫生纸片柔性揉团原型

100×120 mm 等效单张薄片，30 g/m²、质量 0.36 g。参数为演示假设，不对应指定品牌、层数或实测材质。相比 A4 原型降低弯曲和拉伸刚度，保留塑性与摩擦。没有撕裂、吸水、分层、穿孔撕线或手指操作。

安装 `pip install -r tools/tissue_paper/requirements.txt` 后运行 `python tools/tissue_paper/run_demo.py`。默认输出 `outputs/tissue_ipc`，7.5 秒仿真、12.5 ms 步长。CPU 或 CUDA 可用。当前是外部求解器，MuJoCo/USD 为快照和烘焙回放，不是原生柔性物理。

`preview.xml/usda` 是纸团，`flat.xml/usda` 是平纸；`crumple_playback.usdc` 保存连续碰撞复核过的内部路径。contact_forces.npz 按轨迹顶点顺序记录世界坐标下的自接触法向/摩擦力、桌面力，单位 N；不是实测传感器数据。

本演示门槛：峰值边长变化 <1%、未检测到路径穿插、末半秒最大顶点移动 <1 mm，最终外包围盒最长边 <90 mm、最短边 >15 mm。仅表示所给松散揉团测试通过，不代表真实卫生纸已标定或任意操作可靠。


v0.1.1: neutral-white fine fibers and dot embossing added. Embossing is a visual normal map, not physical microgeometry. Shell refined to 775 vertices / 1440 triangles, bending stiffness 8e-7 Nm, compression radius 27 mm; trajectory and contact forces revalidated. MuJoCo displays the albedo; USD also carries the normal map.


### Playback sampling
The 80 Hz trajectory.npz contains physical-step endpoints for analysis and force indexing. Straight interpolation at release interval 270 crosses. Use ccd_path.npz or crumple_playback.usdc for interpolation: the release export preserves all physical endpoints plus necessary intermediate updates, with every published segment checked again after float32 conversion. Original solver audit: 14807 internal update segments, no crossings. See playback_compaction.json for retained sample counts and verification; full solver outputs can be regenerated with run_demo.py. MP4 shows physical endpoint frames without geometry interpolation.
