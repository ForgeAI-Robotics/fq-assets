# 人物资产

首批四个 Microsoft Rocketbox 成人形象已入库，每项版本 0.1.0。来源固定为 Microsoft 官方仓库，许可为 MIT，逐项记录原始文件哈希并保留许可证。

| 外观 | 资产后缀 |
| --- | --- |
| 女性、粉色衬衫与牛仔裤 | rocketbox_female_adult_01 |
| 女性、通勤外套与牛仔裤 | rocketbox_female_adult_05 |
| 男性、条纹短袖与短裤 | rocketbox_male_adult_01 |
| 男性、红色长袖与长裤 | rocketbox_male_adult_06 |

完整 ID 前缀为 `third_party:characters/`。文件位于 `assets/third_party/characters/`。它们适合流程与人群的中等精度视觉原型，不是扫描级数字人。当前四人主要增加衣着/外貌变化，年龄与体型覆盖仍有限。

## 表示与边界

- GLB：自包含贴图，80 节点骨骼、175 面部形变目标；Y 向上。
- USD：Z 向上，UsdSkel 蒙皮与简短的前臂检查动作；面部形变暂未转换。
- MuJoCo/网页：静态 A 姿态和简化胶囊碰撞代理，不能据此认为人体接触已验证。
- 背部和左右手挂点已给出；携包姿态、肩带贴合、行走/等待/配合采样动作尚未实现。

检查覆盖 GLB 嵌入依赖和形变目标、MuJoCo 加载、USD 真实蒙皮顶点变形、Isaac 场景加载和渲染。验证报告在各资产目录。人物未自动放入机场场景，未接入 Connection。

## 后续技术选择

Rocketbox 的优势是可分发、骨骼结构可复用和运行负担较低，适合先完成多旅客业务流程。它不是最新的高保真人物方案。

MetaHuman 可作为高保真人物路线评估；实际使用需按 Epic 当前条款和导出工作流执行，未纳入本次发布。Isaac 的 Replicator Agent/Animated People 可用于人群行为与数据生成，但模型导入后仍需适配骨骼、动作、导航及携包，不能自动获得机场流程。

来源：[Microsoft Rocketbox](https://github.com/microsoft/Microsoft-Rocketbox)、[MetaHuman](https://www.metahuman.com/)、[Isaac 人物与事件数据生成](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/action_and_event_data_generation/index.html)。本机现有 Isaac 运行环境为 4.5，5.1 功能尚未在本机部署验证。
