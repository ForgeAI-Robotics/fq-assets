# 当前默认：机场黑色立柱变体

`airport_black_model.xml` 为原生柔性操作模型，`airport_black_preview.xml/usda` 为静态快照。34 cm 底座、44 mm 杆径、98 cm 带中心高度、65 mm 带宽，黑色柱体与深蓝带身。尺寸来自照片近似，卷筒/卡扣仍为概念机构。旧银色版本继续保存在 model.xml/preview.xml。

# 可操作伸缩隔离带套件 v0.1.1

包含两根立柱、卷筒外形、246 顶点的原生 MuJoCo 2D flex 带身、自由运动端头、向上脱出的接收槽、柱边停靠位和概念夹具。资产是完整套件；柱、卡扣和端头保留独立名称以便复用。

## 运行

使用 MuJoCo 3.3.6 和 NumPy，在仓库根目录执行：

```sh
python examples/retractable_belt/run_cycle.py --out /tmp/belt_cycle
```

脚本运行完整物理循环并输出 `validation.json` 和 `cycle_frames.json`。独立模型入口是 `model.xml`；机场双套件入口是 `scenes/fq/airport_preview/belt_interaction.xml`。USD 与网页三维入口只是静态外观，不包含 PhysX 柔性与控制器。网页默认播放原生仿真录像。

## 控制接口

`tools/retractable_belt.py` 的 `RetractableBelt(model, data, prefix='', origin=(0,0,0))` 提供：`begin_open`、`close_gripper`、`grasp`、`unlatch`、`feed_to`、`dock`、`release`、`begin_close`、`undock`、`latch`、`contacts`。

先授权开启并检查另一通道已扣住；夹具双侧接触端头后才允许夹持。端头抬起至少 32 mm 才允许脱扣。握持过程中用 `feed_to(stored_length, dt)` 收放带身，速度限制为 0.5 m/s；对准柱边停靠位后才能停靠并松手。关闭前传入通道占用状态，再抓取、解除停靠、拉出、下压对齐并扣回。未扣住或停靠的端头不能松手。占用、报警和放行授权由调用方提供，不是假装存在的传感器。

机场入口前缀为 `entry_`、原点 `(0,-1.8,0)`；出口为 `exit_`、原点 `(0,3.65,0)`。通过 `begin_open(..., other=另一套控制器)` 检查互斥。连续运动中的障碍检测与停机策略仍由上层实现。

`contacts()` 返回端头或柔性带身的接触位置与模型估计法向力。它是即时读取接口，保存日志需由调用方处理。完整循环日志也包含事件与采样帧。

## 模型边界与验证

柔性带的材料顶点始终保留；收回的部分由规定的螺旋导向约束卷入柱内，外部部分自由参与物理求解。这不是缩短刚性盒子或隐藏带身，但也没有模拟真实卷簧扭矩、卷层摩擦和内部自碰撞。抓取和扣合采用条件检查后的显式约束，不等同于验证真实夹爪的摩擦抓取能力。材料和机构参数未实物标定。

`validation.json` 包含完整循环、10 项拒绝测试、外部探针正负碰撞对照，以及机场双套件原生物理验证。视频中灰色装置是概念夹具，不是已接入的机器人操作员。

## 静态预览碰撞

`preview.xml` 与 `preview.usda` 的带身现包含静态碰撞近似，可在网页碰撞模式中查看。原生 `model.xml` 继续使用 2D flex 柔性碰撞，未替换为刚体。静态 USD 的碰撞不代表 PhysX 柔性行为。
