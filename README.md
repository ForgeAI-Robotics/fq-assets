# fq-sim-assets

FQ 仿真资产库。首版 `v0.1.0` 发布 navigate 项目的办公场地主场景，以及可乐、圆桌、会议桌、篮子和冰箱。米制、Z 轴向上；MuJoCo 四元数采用 wxyz。

## 获取固定版本

```bash
git clone https://github.com/ForgeAI-Robotics/fq-sim-assets.git
cd fq-sim-assets
git checkout v0.1.0
git lfs pull
python3 tools/check_integrity.py
python3 tools/catalog.py
```

Release 附件 `fq-sim-assets-v0.1.0.tar.gz` 包含实际资产文件，不依赖 Git LFS 下载。GitHub 自动生成的源码压缩包不保证包含 LFS 实体，请优先使用这个专门附件。

4080S 工作副本：`/home/fangqi/code/fq-sim-assets`。原 navigate 与 SIMPLE 目录保持原状。

## 基准场景

入口：`scenes/fq/office_baseline/scene.xml`（MuJoCo）与 `scene.usda`（USD）。

- 场地：机器人测试区、走道、圆桌区、会议室及禁入区域；不是整栋建筑的实测模型。
- 固定门净宽约 0.80 m、会议桌宽 1.20 m；其余图纸估算与照片来源见 provenance。
- 基准包含圆桌和中文可乐，保持 navigate 的固定位置和静态碰撞语义。
- 篮子、冰箱作为独立资产提供，尚未指定它们在该场地的正式位置。
- 不包含机器人、控制器、策略权重、导航服务或随机化任务。
- 会议桌沿用 navigate 的桌面/桌腿外观，桌底仍为保守整盒碰撞；不适合桌底穿越任务。
- 会议室门固定打开。玻璃围护中的不可见禁入碰撞体有意保留。
- 顶棚保留在正式场景中；validation/previews 的场景图仅为去顶棚预览。

![主场景去顶棚预览](validation/previews/fq_scenes_office_baseline.png)

## 资产目录

| ID | 类型 | 说明 |
|---|---|---|
| `fq:scenes/office_baseline` | 场景 | 固定办公场地，无机器人 |
| `fq:objects/coke_cn` | 刚体 | 中文标签；独立入口为动态罐体 |
| `fq:objects/round_table` | 固定家具 | SimFoundry 派生圆桌 |
| `fq:objects/conference_table` | 固定家具 | 保留基准世界坐标；1.20 m 宽 |
| `fq:articulated/photo_basket` | 关节资产 | 自由篮体、两个被动提手转轴 |
| `fq:articulated/haier_bcd402` | 关节资产 | 固定柜体、左右门及独立竖梁转轴 |

`catalog/index.json` 索引各 asset.json，`assets.lock.json` 固定版本。representation 路径相对于仓库根目录。此版本不自动注册到 SIMPLE，也不改动原项目默认场景。

```bash
python3 tools/catalog.py --resolve fq:scenes/office_baseline --backend mujoco
python3 tools/catalog.py --resolve fq:articulated/photo_basket --backend usd
```

资产按需要加载；后续 SIMPLE 适配层可读取该目录注册资源，再由任务选择随机候选池。`origin` 支持 fq/simple；本版收录的均为 FQ 自建或 FQ 派生资产，不因为曾存放于 SIMPLE 就标为 SIMPLE 原生。

## 来源与能力边界

`provenance/source_files.json` 记录每个引入文件的源位置和原始 SHA-256；`checksums.json` 记录发布后文件内容。二者分别描述原始输入与发布结果。`provenance/source_scripts` 为历史制作依据，包含当时机器路径，不是可直接运行的通用构建入口。

可乐、圆桌的上游为 SimFoundry，经 FQ 尺度、碰撞和材质处理；中文标签为生成式视觉修复。篮子为用户照片建模。冰箱保留 v2 用户尺寸校准记录，但质量、惯量、摩擦和门封等仍未完整标定。竖梁没有自动折叠联动，下冷冻区固定。模型可动不等于机器人抓取/开门任务已验收。

资产许可证不因移入此库而改变，见 NOTICE.md。原始 manifest/README 内的旧版本和验证记录属于历史证据；本版入口以 asset.json 为准。

## 验证

Python 3.10+，验证依赖固定在 `requirements-validation.txt`：

```bash
python3 -m pip install -r requirements-validation.txt
python3 tools/validate.py --output /tmp/fq-assets-validation.json
# 有可用 OpenGL/EGL 环境时生成预览
MUJOCO_GL=egl python3 tools/validate.py --output /tmp/fq-assets-validation.json --render
```

MuJoCo 检查全部入口、可乐/篮子落稳、五个关节的范围采样、门净宽和固定布局。USD 检查资源闭包、单位、关节和实例位置。发布前在只挂载此资产库的独立容器中验证，运行入口不能引用源项目目录。验证结论与引擎版本见 `validation/`；USD 结构通过不等于 PhysX 接触物理已标定。

Isaac Sim 4.5.0 的独立加载检查见 `validation/isaac_load.json`：打开全部 USD 入口并启动固定基准场景时间线。此项覆盖加载与启动，不是冰箱开门、篮子抓取或完整导航策略回归。

## 后续工作

发布后讨论本地网页资产目录：读取同一资产清单，支持旋转查看、关节限位内拖动提手和冰箱门；浏览器展示和真实物理交互需分别标明。SIMPLE 场景/资产注册和受约束的域随机也是后续独立接入事项。本版不部署网页服务。

## v0.2.0 模型目录

新增两套 G1 机器人组合、四只独立手以及五项材质，详见 [版本说明](RELEASE_NOTES_v0.2.0.md)。现有 SIMPLE 运行仍使用自己的锁定快照。
