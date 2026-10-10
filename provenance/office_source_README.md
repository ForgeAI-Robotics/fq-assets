# 机器人测试区到会议室：简化场景

当前状态：按用户 2026-10-09 红框截图修正通道，添加材质与基础灯光，并在 192.168.31.69 完成 MuJoCo 场景/G1 模型加载及 Isaac Sim 4.5 场景验证。未执行 G1 行走或穿门任务。

照片版更新：根据用户补充的 5 张照片，将办公区和会议室改为灰色拼块地毯，会议室门前/出口小段走道改为灰色石纹砖；加入浅木纹门、黑框玻璃、浅色墙板分缝、白色吊顶、圆形筒灯、灯带与入口发光顶面。材质参考和生成提示词记录在 `photo_materials.json`。照片明确材质类型，但玻璃分格、地面分界、灯距和部分空间定位仍为近似。原平面图中的会议桌暂时保留，未重建照片中遮挡严重的桌椅、杂物、人物和其他机器人。

来源：用户提供的《平面布置图--调整3.14.pdf》第 1 页。使用图中重复的 1200 mm 桌宽校准比例，平面轮廓为人工近似描图，不是 CAD 精确复原。

会议室右上角单开门没有明确门宽、门高标注。按图比例估算门洞约 0.85–0.90 m，当前暂设 0.90 m；门高暂设 2.10 m，墙高暂设 2.70 m。以上不能代替现场测量的门框内净宽。门固定向会议室内打开 90°。

## 文件

- `scene.xml`：独立 MuJoCo 静态场景，含地面、隔墙、门、简化家具，不含机器人。
- `scene.usda`：同源几何导出的 OpenUSD 静态场景，米制、Z 轴向上，包含静态碰撞、材质、UV 和基础灯光，供 Isaac Sim 打开验证；不含 G1 或控制器。
- `textures/carpet_photo_v1.png`、`stone_photo_v1.png`、`wood_photo_v1.png`：使用内置 image_gen 按用户现场照片生成的贴图；不是直接测量得到的反射率或精确摄影测量。地毯暂按 1 m 的 2×2 拼块重复，石纹暂按 0.6 m 重复，木纹暂按 1 m 重复。`wall.png` 仍为简化墙板底色。旧 `floor.png` 已不再引用。
- `scene_g1.xml`：引用现有项目 `.deps/unitree_rl_gym` 中 G1 模型的组合入口。当前 Windows 工作区没有该机器人依赖，需要在已有 Linux 仿真项目中加载。
- `geometry.json`：米制坐标、起终点、建议途经点和导航矩形；`rectangles` 与当前项目的 `[cx,cy,hx,hy]` 格式一致。原点在图纸左下内墙附近，x 向右、y 向上。
- `route_plan.png`：俯视路线；`preview_3d.png`：剖切示意，墙在预览中降低以便看清布局，XML 仍保留全高。
- `validation.json`：本次验证结果。
- `remote_validation.json`、`isaac_validation.json`：远端引擎加载与碰撞测试结果。
- `isaac_render.png`：Isaac Sim 实际俯视渲染，拍摄时隐藏视觉吊顶；`isaac_entrance_render.png`：入口近景，保留吊顶。`mujoco_cutaway_render.png` 为 MuJoCo 剖切渲染，为观察布局仅在渲染时隐藏吊顶、降低墙高、淡化禁入区，保存场景的碰撞体未改变。

## 调整与使用

在 g1nav 包目录运行 `python tools/build_test_meeting_scene.py --door-width 0.90` 可重新生成 XML、USD、纹理和几何参数，需要 Pillow。随后运行 `python tools/preview_test_meeting_scene.py` 更新预览与静态检查报告（当前预览字体使用 Windows 微软雅黑）。预览为独立绘图示意，不是 MuJoCo 或 Isaac Sim 的真实渲染，不用于验证光照。

在有 MuJoCo 的环境里运行 `python -m mujoco.viewer --mjcf=simulation/test_meeting/scene.xml` 查看静态场景。

在 Isaac Sim 中通过 File > Open 打开 `scene.usda`；复制目录时需同时保留 textures 子目录。已在服务器 Isaac Sim 4.5 中加载、运行碰撞探针并输出渲染。MJCF 与 USD 的照明模型不同，光照参数为各自的起始值，不能保证两个渲染器画面一致。USD 使用贴图网格显示外观、不可见原生 Cube 作为静态碰撞体，避免细长凸网格的 GPU 烘焙回退。

基础外观以照片为准：灰色地毯、局部石纹地砖、浅木门、黑色窗框及玻璃、浅色墙板。USD 使用七处示意圆形面光源（4800 K）、入口矩形顶面光源和环境补光；MuJoCo 使用对应位置的简化光源。灯位数量与亮度不是实测还原；未模拟真实灯具光度、外窗环境或相机自动曝光。玻璃使用 alpha 透明近似，不是经验证的物理折射或玻璃 LiDAR 回波模型。接触摩擦系数暂设 0.7，尚未实测；贴图本身不会改变物理摩擦。

现有导航 runner 仍选择旧场景。本次没有替换默认场景，也没有启动远端机器人。接入时需要让 runner 加载 `scene_g1.xml` 和本目录 `geometry.json`，让地图发布端读取同一份矩形，并使用 JSON 中的 spawn/goal。该静态场景没有旧 runner 使用的 dynamic_box / dynamic_pedestrian，接入时需关闭相应动态障碍更新。

## 简化与验证范围

按用户红框截图保留机房右侧至建筑底部、右侧办公室左侧的纵向通道。约为 x=9.625–13.114 m、y=0.057–7.580 m，宽约 3.489 m；边界按临近墙线近似对齐，不是实测值。通道北端接机器人测试区，通道内原工位已移除。

通道左右两侧下方区域仍为临时禁入区，分别有碰撞盒及占用矩形，并非现场实体墙。上方会议桌、柜子和测试区隔墙保留；省略椅子、窗细节，室外门封闭。路线仅用于固定开门条件下的室内导航准备，未实现开门动作。

XML 结构检查通过；对建议路线以每段 301 个点检查与二维障碍盒的距离，加入门框后中心线最小间距约 0.427 m。门洞的原始暂设宽为 0.90 m；框体会占据部分空间，需实测净宽再调整。此检查不包含 G1 的身体包络、步态摆动或控制误差，不代表机器人通过测试。

服务器路径：`/home/fangqi/g1-navigation-sim/workspace/simulation/simulation/test_meeting/`。独立资产和测试文件已复制到此处，没有切换现有默认导航场景。

MuJoCo：照片版场景几何体加载成功，具体数量见 `remote_validation.json`；G1 组合模型加载成功（nq=19、nu=12），已执行 forward，但未执行策略行走。测试脚本创建私有 `g1_robot_resolved.xml` 和 `scene_g1_resolved.xml` 来解析服务器中的机器人网格路径，不修改机器人源文件。

两款引擎均进行半径 0.1 m 小球落体测试：通道内小球中心落在 z≈0.10 m，禁入区上方小球中心停在 z≈2.80 m，验证地面和禁入体接触；Isaac Sim 运行 240 步，MuJoCo 探针运行 1500 步。该测试不代表 G1 导航、步态或窄门通过率。

本机 Windows 的 MuJoCo/USD 库加载仍受应用控制限制；本次引擎结果来自用户授权的 Linux 服务器。
