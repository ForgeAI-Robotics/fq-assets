# 睿尔曼 RealBot 整机资产接入

核查日期：2026-10-27。RealBot-01、RealBot-L2、RealBot-S2 已登记为待接入型号，尚未作为可加载机器人资产发布。

| 型号 | 整机 URDF + 网格 | 当前状态 |
| --- | --- | --- |
| RealBot-01 | 已检查的公开来源中未找到；官方 V1.2 示例包 64 个条目中没有 URDF、Xacro 或模型网格 | 等待厂家模型 |
| RealBot-L2 | 已检查的公开来源中未找到 | 等待厂家模型 |
| RealBot-S2 | 已检查的公开来源中未找到 | 等待厂家模型 |

来源与固定提交记录见 [检索证据](../provenance/realman/realbot_public_source_audit.json)，机器可读待接入清单见 [intake](../catalog/intake/realman_realbot.json)。本结论仅覆盖记录中的公开来源，不代表厂家没有内部或交付给客户的模型。

官方 rm_models 提供了 RM75、RX75 等机械臂 URDF，但单臂模型不等同于 RealBot 整机。其他旧版升降平台也不能直接命名为 L2/S2。未使用 RealHand 同名 realbot SDK 作为睿尔曼整机模型来源。

## 需要取得的厂家交付包

- 对应具体型号和版本的整机 URDF/Xacro，包含底盘、躯干、头部、左右臂及所选末端。
- 所有 STL/DAE/OBJ 网格、贴图和包路径；统一单位与坐标约定。
- 关节轴、限位、惯量、质量、碰撞几何和自碰撞排除配置。
- 在 fq-assets 中存储、分发原始及转换模型的许可。

## 收到模型后的入库门槛

1. 固定来源版本与哈希，检查许可证、关节树和网格依赖完整性。
2. 生成可移植 URDF 与 MuJoCo 表示，验证关节、尺度、惯量及碰撞；导入 Isaac Sim 验证 USD。
3. 渲染整机预览并检查初始姿态、关节活动与碰撞；通过后再加入 catalog/index.json、版本锁和网页可用资产列表。

当前 intake 记录不进入运行时目录，不宣称有可加载模型、碰撞验证或控制能力。
