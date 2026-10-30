# 来源及使用记录

本库按用户授权发布到私有仓库。没有对所有模型声明统一的开源许可证。

- FQ 办公场景：依据用户提供平面图和照片进行重建，原工程 g1-navigation-sim/office_video_v2。
- FQ 篮子与冰箱：依据用户照片及部分用户尺寸制作；源文件和历史说明保留于对应目录。
- 可乐与圆桌：SimFoundry 来源，经 FQ 项目处理后用于 SIMPLE 和 navigate。上游精确文件路径及哈希见 provenance/simfoundry_source_manifest.json。上游模型的再分发许可证尚未在本次整理中确认；不能由 SIMPLE 代码的 MIT 许可证推导模型许可证。
- 中文可乐标签：FQ 生成式视觉修复，品牌文字/标记不表示品牌方授权或背书。
- SIMPLE 运行时和第三方控制代码未打包。原有资产构建脚本的版权/许可头原样保留。

进一步公开分发或改变访问范围前，应逐项补齐上游授权信息。此记录不影响已授权的当前私有发布。

## 机器人与灵巧手（v0.2.0）

G1+O6、G1+O7 是现有 FQ 组合适配的快照，不标为 FQ 原创硬件或 SIMPLE 原生资产。G1 本体来自 Unitree 模型；精确本体上游提交及模型许可未在本次整理确认，不由 SIMPLE 代码许可推导。LinkerHand O6/O7 的本地记录固定上游 linker-bot/linkerhand-urdf，提交 075cc7d42cc1e756bdcbece0fc069a0779fc5237；Apache-2.0 文本保留在 provenance/robots/LinkerHand-LICENSE。组合模型中 FQ 的安装、碰撞和关节参数适配保留原值。逐文件来源及哈希见 provenance/robots/source_files.json。原始 MJCF/URDF 存在 provenance/robots，下游使用 assets 中便携路径版本。

O6 安装件为 provisional，模型质量/惯量并非实测总成标定。四只手从当前组合模型提取，保留仿真耦合和驱动参数，不冒充未修改的厂商原版。当前私有发布包含模型与网页预览；未进行本次 GPU、抓取控制或硬件验收。

## D435i 相机（v0.3.0）

来自现有 FQ D435i 管线及 RealSense 官方模型；官方 D435i xacro 复用 d435 外壳网格。原 xacro 版权头和 Apache-2.0 文本保留在 provenance/cameras/realsense。确切上游提交未记录，按本地源文件 SHA256 固定，不能将本地提取时间当上游版本。仿真质量和参考内参不作为实测参数。详情见 RELEASE_NOTES_v0.3.0.md。

## Microsoft Rocketbox 人物资产

四个人物来自 Microsoft Rocketbox，固定提交 `0943055db6ec570bcef9f2c8b41c9e5467c808f9`，上游 MIT 许可逐项保留。FQ 修改了路径、坐标、格式和检查动作。此许可仅适用于这些人物及其对应来源，不改变仓库其他资产的许可状态。

## Google Scanned Objects baggage

Eight bag scans are Copyright 2020 Google LLC, CC-BY-4.0. Per-model source URLs, hashes, licenses and modifications are retained in assets/third_party/bags/. This license does not change other repository assets.

## Lansi wheel-arm robot

User-supplied model; owner confirmed public publication on 2026-10-27. No upstream license text supplied; no general downstream license is implied. See assets/user_supplied/lansi_wheel_arm_robot/provenance.json.

## Airport supporting assets 2026-10-28

OpenRobotics Fuel bags retain CC0 or CC-BY-4.0 as declared per asset. Passenger combinations retain Microsoft Rocketbox MIT and Google GSO CC-BY-4.0 licenses. FQ-authored box, sampling sheet and attachment additions do not relicense upstream components.

## A4 纸张实验原型

`tools/a4_paper/kami` 来源于 San Kala 的 kami（MIT），固定提交 9e7754bd458395e46640385da511d7c94be8cffc。许可全文保存在 tools/a4_paper/LICENSE-kami.txt；FQ 矩形适配差异、A4 实验及物理验证记录一并提供。当前只验证压折与塑性保留，不声明紧密揉团通过。

## A4 纸张 IPC 版本

当前可揉皱原型使用 FQ 编写的离散薄壳/塑性折痕模型和 IPC Toolkit 1.6.1（MIT）碰撞检测依赖；许可保存在 tools/a4_paper/LICENSE-ipc-toolkit.txt。旧 kami 压折版本继续保留，当前物理不使用其求解器。

## Office props — Poly Haven (CC0)

Nine individually adapted props originate from Poly Haven's clipboard, binder_notebook, office_notepads, vintage_stapler and stationery_supplies collections, all released under CC0. Source declarations: https://polyhaven.com/license . Per-source author metadata, download hashes, URLs and changes are preserved in each office prop's source.json/provenance.json and LICENSE.txt. Adaptations preserve metric dimensions and 2K materials; collisions, mass and friction are FQ approximations.

## Tissue paper prototype

Procedural geometry and uncalibrated shell/material integration by FQ; collision/friction uses IPC Toolkit 1.6.1 (MIT), license in tools/tissue_paper/LICENSE-ipc-toolkit.txt. No specific manufacturer's tissue product is reproduced.

## Office drinks and takeaway props

Two real scanned ceramic mugs originate from Google Scanned Objects (Google LLC), CC-BY-4.0, downloaded from the GoogleResearch account on Gazebo Fuel. Per-model attribution, download hash and modifications are in assets/office_refresh/*/provenance.json and LICENSE.txt.
WaterBottle is by Microsoft, CC0, distributed through KhronosGroup/glTF-Sample-Assets; original license declaration retained as LICENSE_source.md. Height adapted to 235 mm, not a manufacturer measurement.
The takeaway coffee, iced coffee, PET bottle and pizza box are FQ authored procedural props; they are not scans and contain no Starbucks branding. All mass/friction and collision proxies are uncalibrated.
