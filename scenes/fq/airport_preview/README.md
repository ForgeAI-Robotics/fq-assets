# 机场入口及检测区域：照片复刻初版

当前默认展示 `photo_scene.xml` / `photo_scene.usda`，依据用户提供的《机场入口及检测区域.jpg》重新搭建。包含两个 6 号入口门组、玻璃幕墙和高挑顶棚、黑色隔离柱与深蓝色软带、多条相邻通道、中央白色柜台及未确认设备外形。无人物、机器人或动态演示。

照片仅提供外观和可见空间关系；所有尺寸与遮挡处连接关系均为估算。中央白色设备未标为防爆检测仪。不能依据本模型宣称现场 SOP、批次人数或真实放行口已得到确认。照片原文件不随资产发布，来源校验信息见 `photo_provenance.json`。

本变体包含静态碰撞，检查了两处入口中心通行间隙和地面/柜台落体接触；验证记录见 `photo_validation.json`。三张预览均为 Isaac Sim 原生 PathTracing 渲染，顶视图为观察布局临时隐藏顶棚，资产本体保留顶棚。

既有 `belt_interaction.xml` 及 `tools/retractable_belt.py` 仍用于旧布局，尚未迁移至照片布局。它们在资产描述中明确标为 previous_layout，不应直接使用旧坐标操作当前照片场景。伸缩带独立套件继续可用。场景目录版本保持 v0.3.0，以 `active_variant: photo_reference_v1` 标识当前初版。

以下为保留的旧布局说明：

# 机场入口检测场景 v0.3.0

## 当前伸缩带实现

入口、出口均接入 `assets/fq/retractable_belt_kit` v0.1.1。带身为 MuJoCo 原生柔性网格，支持端头抓取、抬起脱扣、受控回收停靠、重新拉出和下压扣回。场景网页仅展示静态三维布局；操作录制视频在伸缩带套件页查看。机器人、旅客和检测仪业务流程尚未集成。

- `belt_preview.xml` / `belt_preview.usda`：当前目录的静态外观入口。USD 不包含 PhysX 柔性。
- `belt_interaction.xml`：入口和出口两套原生 MuJoCo 柔性带，共 492 个顶点。
- `belt_interaction.json`：两套控制器的名称前缀、位置和运行入口。
- `tools/retractable_belt.py`：接触检查后的约束抓取、脱扣/扣回条件检查和受控收放。
- `examples/retractable_belt/run_cycle.py`：可复现的单套件完整物理循环。
- `validation/airport_flexible_belts.json`：机场双套件原生物理运行与互斥检查结果。

卷筒使用规定的螺旋导向收放边界；没有标定卷簧扭矩、卷层摩擦和材料参数。夹持及锁扣为检查条件后的显式约束。调用方负责实时障碍感知和停机策略。详细运行方式、接口及限制见套件 README。

机场布局版本保持 v0.3.0；新增套件单独版本化。以下内容仅说明保留的历史刚体示例，不能作为当前柔性带接口说明。

## 历史刚体操作示例（保留用于复现）


保留软带围合、旅客不摘包的批次等待区布局。此版本完成刚体交互原型，不含旅客接触采样或完整安检业务流程。

## 入口
- `scene.xml` / `scene.usdc`：环境，默认两条挡带关闭。
- `operable.xml`：环境 + 自由刚体拭子 + 示例夹具接口 + 可切换夹持约束。
- `operable.usda`：环境 + 独立 USD 刚体拭子。USD 开带接口见 `tools/airport_interaction.py`；USD 抓取控制器尚未提供。
- `interaction.json`：挡带名称、USD 路径、插槽尺寸及耗材位置。

## 已实现
入口/出口带子用薄盒代理收回和伸出；完全收回时隐藏并关闭碰撞，伸出时恢复。两口不能同时打开；出口需要显式放行授权，报警未解除时不能开带；调用方报告通道占用时禁止关闭，关闭途中报告占用则反向打开。占用、报警、放行授权是接口输入，尚未接感知系统或 Connection。

片状拭子是 170×30×6 mm、10 g 的独立刚体，参数为概念假设。取用前检查夹持点/朝向及两夹垫与柄部接触，然后启用刚性 weld 约束。完成抬升、转运、插槽、退出、放入单独已用托盘和解除夹持；已用标记禁止再次当作新拭子抓取。原实心探测仪拆成分段壳体，入口是宽 50 mm、高 25 mm、带后挡面的真实碰撞通道。槽口到位事件不等同于检测结果。

## 运行
安装 MuJoCo 3.3.6 和 NumPy 后，在仓库根目录运行：
```bash
python tools/run_airport_interaction.py
python tools/run_airport_interaction.py --viewer
python tools/validate_airport_operations_mujoco.py
```
第一个命令运行真实刚体步进与接口状态变化；第二个显示夹具接口演示。演示中的出口授权是显式模拟输入，不是仪器测得阴性。

蓝思整机示例在 `examples/airport_operations/`。该示例是限位内 IK 轨迹回放，包含示意工具延长段和底座横向运动；没有电机动力学、轮地牵引、导航或真实夹持力验证。刚体拭子取送验证与整机运动学展示的边界必须区分。

## 验证与边界
MuJoCo 23 项检查，Isaac/PhysX 7 项检查，报告 `validation/airport_operable_v030.json`。整机示例检查了渲染帧中的机器人/环境接触，未发现超过 3 mm 的穿入；这不是整机自碰撞、连续扫掠体或人员安全验证。质量、摩擦、工具与仪器尺寸均未标定。

软带仍无柔性、张力、回卷或摘挂锁扣动作；拭子不是软体。未实现人体/包擦拭、化学检测、报警复测、Connection 接入。资产网页仍是静态预览，交互需要运行上述 Python 控制器。

[演示视频](../../../validation/previews/airport_operations_v030.mp4) · [演示画面](../../../validation/previews/airport_operations_v030.png)
