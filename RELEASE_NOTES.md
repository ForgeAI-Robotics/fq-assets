## v0.4.1 — old G1 O7 pose configuration

Use the user-selected old G1 left O7 open thumb_rotation protocol value 83 (previously 170). Preserve previous values and provenance. Closed and light-switch poses, robot geometry and dynamics unchanged. Readback only; hardware execution not validated. Existing SIMPLE locks remain unchanged.

## v0.4.0 — FQ Assets and pose profiles

Repository renamed to ForgeAI-Robotics/fq-assets. Added versioned O6 left/right task commands, O7 left task commands and a G1 BeyondMimic initial joint pose. Source file hashes and verification status are recorded. Right O6 values were approved by an operator using the left profile; they are not an independent right-hand calibration. No O7 right profile was inferred. G1 named-joint limits and kinematic replay pass for both robot models; physical balance and hardware validation remain pending. Loading profiles never changes the default pose or sends commands. Existing model geometry, dynamics, camera parameters and baseline placement are unchanged.

# v0.3.10

Correct left fridge pivot to eliminate cabinet collision across 0-90 degrees; preserve 91 kg mass and collision geometry. Fridge 0.1.3, baseline 0.1.6.

首个 FQ 仿真资产基准版本，在 4080S 整理和验证。

- 发布 navigate 办公主场景：测试区、走道、圆桌区和会议室，保留门洞、禁入区及固定布局。
- 收录中文可乐、圆桌、会议桌、双提手篮子和海尔冰箱；每个资产具有独立 MuJoCo/USD 入口。
- 保留篮子双提手、冰箱左右门及竖梁关节，附带尺寸校准依据和物理建模限制。
- 提供来源清单、原文件 SHA-256、版本锁、发布文件校验和、查询工具及验证脚本。
- 独立容器验证 MuJoCo 加载、可乐与篮子落稳、关节范围、0.80 m 门净宽和 USD 资源闭包。
- Isaac Sim 4.5.0 打开全部 USD 入口，并启动固定基准场景时间线；此项不代表关节接触物理或机器人任务已验收。

基准中篮子与冰箱尚未指定正式摆放位置；导航中的可乐保持固定，独立可乐资产可自由运动。SIMPLE 自动注册、域随机适配和网页浏览服务不属于本次发布。

请下载附件 `fq-sim-assets-v0.1.0.tar.gz` 获得完整实体文件，并用 `SHA256SUMS` 校验。Git 克隆后需要 `git lfs pull`。引擎运行证据与范围见仓库 `validation/`。
