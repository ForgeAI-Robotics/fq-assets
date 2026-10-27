# 女性旅客 · 粉色衬衫 / Rocketbox

版本 0.1.0。来源为 Microsoft Rocketbox，保留本目录 MIT 许可证。

- `character.glb`：米制、Y 向上；80 骨骼节点、175 个面部形变目标。
- `character.usdc`：米制、Z 向上；UsdSkel 骨骼及检查动作，未包含面部形变。贴图为相对路径。
- `preview.xml`：MuJoCo 静态 A 姿态，用于资产网页预览；不是关节驱动的人体模型。
- `attachments.json`：背部、左右手挂点。背包位置、肩带、握持姿态尚未适配。

碰撞仅有一个简化胶囊体，用于人群占位，不代表衣物、肢体或机器人擦拭人体的接触模型。未附带行走/排队控制器，未加入机场场景。

已检查依赖完整性、GLB 内嵌资源、MuJoCo 加载、USD 骨骼变形和 Isaac 实际渲染；详见 `validation.json`。静态网页不播放骨骼动画。
