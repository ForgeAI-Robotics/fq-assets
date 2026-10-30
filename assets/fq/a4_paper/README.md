# A4 塑性纸张：压折实验原型

尺寸 210×297 mm，按 80 g/m² 办公纸设置目标质量 4.99 g。默认 625 顶点、1152 三角面。物理厚度假设 0.1 mm，压折仿真的数值层间距为 0.6 mm。参数没有用实纸标定。

已完成边缘运动学夹持折叠、压条压折及释放实验。紧密揉团实验存在过大的边长变形与面片穿插，尚不满足纸团验收；不能把折叠通过等同于揉纸成功。三角面穿插检查仅为离散抽样，不是连续碰撞保证。

物理求解使用 MIT 开源 kami / NVIDIA Warp，FQ 对方形网格做矩形适配。MuJoCo 仅渲染网格；preview.xml/usda 为静态外观，fold_playback.usdc 是烘焙动画，不含 PhysX 纸张物理。当前没有机器人接触力耦合。

在独立 Python 环境安装 `pip install -r tools/a4_paper/requirements.txt`，然后运行 `python tools/a4_paper/run_demo.py --mode fold --output outputs/a4_paper`。关闭塑性对照加 `--elastic`。`--mode crumple --N 33` 为尚未通过的紧密挤压实验，默认不运行。推荐有 CUDA 的 Torch 与 Warp；当前在 RTX 4080 SUPER / Torch 2.5.1+cu118 / Warp 1.16 验证。

入口工具以运动学方式控制，force 参数是上游折痕模型的输入载荷，不代表机器人力传感器读数。矩形适配尚未通过连续体材料和各向同性标定。来源、MIT 许可和源码差异均保留。
