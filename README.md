# fq-assets

FQ 机器人资产库：统一管理物体、场景和动作，以及姿态、标定与来源记录。资产采用米制、Z 轴向上；MuJoCo 四元数为 `wxyz`。

当前发布版本为 **[v0.9.7](https://github.com/ForgeAI-Robotics/fq-assets/releases/tag/v0.9.7)**。圆桌 0.1.3：桌沿规则化、1 mm 小倒角和显式法线，桌面/支柱/底座分区域 PBR；尺寸 70 × 70 × 75 cm、碰撞和物理参数不变。基准场景 0.1.22 同步外观。

## 获取与版本锁定

安装 Git LFS 后获取完整资产：

```bash
git clone https://github.com/ForgeAI-Robotics/fq-assets.git
cd fq-assets
git checkout v0.9.7
git lfs pull
python3 tools/check_integrity.py
python3 tools/catalog.py
```

GitHub 自动生成的源码压缩包不保证包含 LFS 实体。历史 v0.1.0 的专用附件不代表当前版本；请使用当前标签及 Git LFS 获取 v0.9.7。

网页展示 `main` 上的当前资产；仿真和回放使用锁定的完整提交及文件校验值。更新 `main` 不应改变已有任务或历史轨迹使用的快照。

## 目录与分类

前端仅有三个一级分类：**物体、场景、动作**。

| 目录 | 内容 |
|---|---|
| `assets/` | 可乐、圆桌、会议桌、篮子、冰箱；G1+O6/O7、独立 O6/O7 手、D435i；墙面、地面等材质/纹理 |
| `scenes/` | 固定办公基准场景、机场入口安检预览场景 |
| `actions/` | stand 初始化、安全站立及 SONIC v3/v4 站立、前后行走、侧移和转向等动作 |
| `profiles/` | O6/O7 姿态及 G1 初始化配置 |
| `calibration/` | 标定数据及坐标约定 |
| `catalog/` | 资产索引，representation 路径均相对仓库根目录 |
| `provenance/` | 来源、原始校验值及历史制作依据 |
| `validation/` | 按版本保留的验证证据 |
| `viewer/` | 本地网页服务与模型导出工具 |

机器人、灵巧手、相机、材质/纹理均作为“物体”的子类筛选。`origin` 区分 FQ 和 SIMPLE；资产曾经存放在 SIMPLE 中，不代表它是 SIMPLE 原生资产。许可证和上游归属见 [NOTICE.md](NOTICE.md)。

```bash
python3 tools/catalog.py --resolve fq:scenes/office_baseline --backend mujoco
python3 tools/catalog.py --resolve fq:articulated/haier_bcd402 --backend usd
```

## 基准场景

入口为 `scenes/fq/office_baseline/scene.xml`（MuJoCo）及 `scene.usda`（USD）。场地包含机器人测试区、走道、圆桌区和会议室，并非整栋建筑的实测模型。

- 固定场景包含圆桌、中文可乐、会议桌和冰箱；篮子独立提供，机器人由任务加载。
- 冰箱已按用户描述，从原墙角沿俯视图向圆桌方向移动约 1 m，位置为 `[15.5, 11.9, 0]` m，朝向不变。这个位移为估计值。
- 基准布局保持固定；动态物体和域随机由 SIMPLE 任务配置选择，不直接随机修改基准源文件。
- 会议室门固定打开；玻璃围护的禁入碰撞体和正式场景顶棚保留。网页去顶棚仅用于观察。
- 会议桌底部仍为保守整盒碰撞，不适合桌底穿越任务。

## 冰箱：整体弧面与分区域材质

v0.7.2 的左右上门共用一个横向弧面：左右外侧保持原位，中缝附近向外突出约 **1 cm**；上下边缘平滑收回，内侧结构与铰链保持原位。显示面板和标识随门面调整，下层抽屉未改为弧面。

分区域材质覆盖银色门板、侧板、塑料内衬、密封条和显示面板。参数见 `assets/fq/haier_bcd402/appearance.json`：USD 和前端使用显式粗糙度、金属度；MuJoCo 使用其材质模型近似。**窗边光照是环境设置，没有烘焙到材质中**，可另行重打光。

受影响的门板碰撞体替换为 **986 块小凸体**，保留原有把手凹槽；MJCF、USD 和基准场景同步。生成依据、数量和误差见 `arch_geometry.json`。整机质量保持 **91 kg**，可动部件合计 **14.3 kg**；右门轴摩擦力矩为 **0.1 N·m**，粘性阻尼为零；固定模式和兼容随机模式均使用 `[0.1, 0.1]` N·m。此值是用户选定的仿真参数，未做实机标定，也不代表开门任务已通过。

弧度、材质和部件质量分配是仿真估计，不是实机完整标定。中间竖梁仍独立、没有自动折叠联动，接触响应禁用；下冷冻区固定，门封吸力未标定。

构建入口为 `assets/fq/haier_bcd402/source/build_asset.py`，它读取基础 `geometry.json`，并通过 `author_arch.py` / `door_arch.py` 生成弧面、凸体和材质。不能仅凭基础 `geometry.json` 恢复最终形状。请在隔离工作副本中安装验证依赖后构建，再运行 `tools/sync_baseline_fridge.py` 同步基准 MJCF；不要在运行任务使用的快照上重建。

## 网页与 SIMPLE 接入

4080S 内网目录：**http://192.168.31.69:18780/**。支持属性、旋转查看、碰撞体显示、门/提手拖动、手部开合姿态及动作预览。浏览器只展示姿态，不运行接触物理。部署和版本语义见 [viewer/README.md](viewer/README.md)。

SIMPLE 已有资产/场景注册、动态物体、物理参数及版本锁定接入。基准场景由任务保持固定；随机参数与资产锁写入回放记录，回放不会自动升级到 `main`。运行时实现和命令以 SIMPLE 项目的 `FQ_INTEGRATION.md` 为准；本仓库 `integrations/` 中的旧接入记录需结合其版本阅读。

v0.7.2 在 4080S 的独立集成目录为 `/home/fangqi/code/SIMPLE-fq-arch-release`，SIMPLE 提交 `5953b863bbc37002e9c4273c0ee997124cefd0c5`，资产锁定 `d1a512cb7a6a009ad373b093f1e5f8f1eee5eb11`。旧任务继续使用原目录和锁定；网页同步新 `main` 不等于运行任务自动换版。

## 验证与边界

Python 3.10+，依赖见 `requirements-validation.txt`。基础加载检查：

```bash
python3 -m pip install -r requirements-validation.txt
python3 tools/validate.py --output /tmp/fq-assets-validation.json
```

基础加载检查不能替代针对改动的动力学验证。v0.7.2 已在隔离环境完成：

- 723 个单门/组合开门姿态，关闭 MuJoCo 父子接触过滤后无穿透。
- 凸体外表面采样最大误差约 **0.183 mm**；质量、惯量、铰链和阻力参数保持一致。
- MuJoCo 与 CPU PhysX 的力矩驱动开合；两扇门约 90°打开并回到关闭位置。
- 两引擎的固定/随机模式独立进程回放，四组最大误差均为 **0**；这是同引擎回放，不是跨引擎轨迹一致性声明。
- 前端 711 项 body-pose 变换检查，以及发布后的门控件和版本核对。

几何证据见 `validation/fridge_arch_geometry.json`，完整回放证据见上述 SIMPLE 目录的 `evidence/fridge-arch-v072/validation.json`。CPU PhysX 通过不代表 GPU PhysX、真实机器人接触任务或硬件材质已标定。历史 `validation/` 报告仅覆盖各自记录的版本与范围。

`provenance/source_files.json` 描述原始输入，`checksums.json` 校验当前提交中的文件。历史源脚本可能包含机器专用路径，不应直接当成当前构建入口。

## Baseline windows (v0.7.3)

Baseline 0.1.13 adds two 1.5 m windows on the refrigerator right exterior wall.
From the inside corner: 0.8 m wall, 1.5 m window, 0.8 m wall, 1.5 m window.
Height follows the existing 2.7 m wall. Frame width is estimated at 35 mm.
Glazing uses ideal light transmission; glass reflection/refraction is not modeled.
The closed facade collision remains unchanged. USD, MuJoCo and viewer visuals agree.
SIMPLE daylight is optional, sampled once per episode and recorded for replay.
Sunlight parameters are simulation distributions, not measured calibration.
Fridge remains 0.1.8; no mass, joint, resistance or door collision changes.

### 会议桌重打光与网页预览

会议桌沿用原始 UV、尺寸、场景位置和物理参数，仅迁移 v4 材质；来源与局限见 `assets/fq/conference_table/pbr.json`。USD 使用非自发光 UsdPreviewSurface，粗糙度贴图为 Raw；MuJoCo 使用底色近似。网页 PBR 预览使用固定曝光切换照明，会议桌和冰箱支持分区域材质，基准场景为部分材质支持。网页效果不等同 Isaac 光线追踪结果，底色仍可能残留少量原照明。

## 会议桌 351 cm 修订

长 3.51 m，宽 1.20 m、高 0.75 m 沿用；保留世界中心。两端 X 支架用 6 cm 方截面斜杆近似，端部留空 4 cm，长边留空 4.5 cm。碰撞为桌面盒与四根斜杆盒，移除原整块占位碰撞，桌下中央留空。底色、粗糙度贴图和 UV 沿用；尺寸与支架依据用户确认，非结构强度或实机动力学标定。历史尺寸保持记录，当前参数见 asset.json。

## Mesh wastebasket (0.9.1)

FQ mesh_trash_bin 0.1.0: empty real diamond lattice with PBR maps and open-top collision. User measured height 28 cm and upper diameter 26 cm (outer diameter interpretation). Bottom diameter 22 cm remains estimated and pending calibration. Fixed-bin coke drop verification passed at three positions; physical mass and tipping are not calibrated.

## Office chair — v0.9.8

Office chair 0.1.0 uses the refined V2 geometry, regional PBR and adjustable MuJoCo dynamics. See [asset documentation](assets/fq/office_chair/README.md). Dimensions and physical parameters are estimates. USD is visual only. The viewer supports armrest slides in cm.

## Baseline meeting chairs — v0.9.9

Baseline 0.1.23 includes three differently oriented, fixed office chairs along the conference table long side. See scenes/fq/office_baseline/chair_placements.json. This is a saved varied layout, not per-start randomization or a measured arrangement.

## Six meeting chairs — v0.9.10

Baseline 0.1.24 places three chairs on each long side of the conference table. Original three poses are preserved; opposite chairs follow the same fixed, varied-placement convention with armrests lowered 4 cm.

## 机场入口安检预览场景

新增 `fq:scenes/airport_preview` 0.1.1：[场景说明](scenes/fq/airport_preview/README.md)。提供 USD 与 MuJoCo 静态环境，不含人物、机器人、行李及业务流程；可用于工位布局和视觉预览；已配置 78 个静态碰撞体，并通过 MuJoCo 与 Isaac/PhysX 接触冒烟测试，尚未进行物理参数标定。
