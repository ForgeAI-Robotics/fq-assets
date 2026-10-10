# 照片参考篮子：Blender 与仿真资产

按用户提供的四张照片建立的可编辑模型。尺寸、重量与摩擦为初始估算，不是实测或制造数据。

本地已准备免安装 Blender：`C:/Users/admin/code/simple/.runtime/blender-4.5.4-windows-x64/blender.exe`。启动它后打开本目录的 `basket.blend` 即可查看和编辑。

## 打开与交付内容

- `basket.blend`：Blender 4.5.4 LTS 工程，包含篮身、独立双提手、塑料材质、灯光和相机。
- `previews/`：透视、正面、俯视预览。
- `basket.xml`：可复用 MuJoCo 篮子资产，包含自由篮身和两个转动关节。
- `test_mujoco.xml`：投球及落地测试场景。
- `basket.urdf` 与 `meshes/`：URDF 和对应的视觉、碰撞网格，移动时请保持目录结构。
- `basket.usd` 与 `basket.usdc`：自包含几何的 USD Physics 资产；两个文件须放在一起。3 个刚体、2 个转动关节、77 个碰撞体。USD 已完成结构与引用检查，完整视觉 USD 的 Isaac 实测状态以 `evidence/validation_status.json` 为准。
- `evidence/`：测试报告、日志、MuJoCo 实际运行视频。
- `references/`：原始参考照片。
- `physics_diagnostic/`：用于远端 Isaac 测试的轻量诊断场景。它使用与完整资产相同的碰撞、质量、惯性和提手关节配置，但用碰撞体显示外形，没有视觉孔纹，不能用来评价产品外观。

## 尺度与结构

单位为米、千克、秒、弧度。Z 轴向上，X 为篮子长边，Y 为短边，篮体原点在底部中央。篮口长 0.36 m、宽 0.24 m，篮身高 0.23 m，不含提手；底部约 0.29 × 0.18 m。参数集中在 `parameters.json`。

篮身估计质量 0.32 kg，两只提手各 0.045 kg。视觉部分保留菱形通孔、斜向凸纹、加宽篮口和实底；碰撞部分采用分段凸体保留篮内空间，不模拟细小物体穿过菱形孔。

两个关节名为 `handle_right_hinge`、`handle_left_hinge`，转轴分别为局部 -Y、+Y，限位 -0.65 至 2.82 rad。照片姿态为两个关节均 -0.65 rad。URDF 的零位为水平展开，需要导入后设置初始角度。MuJoCo 可通过 `mj_resetDataKeyframe(model, data, 0)` 载入 `photo_pose`。USD 已设置照片姿态。

篮体与直接连接的提手采用关节邻接碰撞过滤，避免连接轴处的接触冲突。提手与环境、另一只提手的接触独立处理。物理参数仍需根据实际材料、称重与应用场景标定。

## 验证说明

本地 MuJoCo 3.3.6 完成了 5 秒投球及篮体落地仿真，并分别以力矩驱动两只提手。完整报告见 `evidence/mujoco_report.json`，视频为 `evidence/mujoco_validation.mp4`。模型通过了物体留在篮内、最终速度、关节独立运动、限位和有限数值检查。

5.21 已确认有 RTX 4080 SUPER、Isaac Sim 4.5.0 镜像及含 MuJoCo 3.3.6 的现有镜像。其局域网出现严重 TCP 重传，完整 Blender 安装包传输受阻，因此本轮建模和完整视觉资产的 MuJoCo 验证在本地完成。不要将远端脚本存在视为平台测试通过；远端诊断结果及仍未完成事项统一记录在 `evidence/validation_status.json`。

## 可乐放入测试（新增）

`coke_test/comparison.html` 提供参考照片与实际落稳姿态的 Blender 渲染对照。复用了 SIMPLE o7 现有可乐资产的原始视觉网格、碰撞网格和 0.35 kg 质量，未缩放；空篮估算 0.41 kg。横放、竖放、倾斜落入三组本地 MuJoCo 测试均通过。测试场景使用 0.5 ms 步长和较硬的可乐接触参数以降低冲击瞬间重叠，详见 `coke_test/README.md`。可乐组合尚未在 Isaac 或机器人任务中验证。

## 重建

使用 Blender 4.5.4 或兼容版本：

```bash
blender --background --python build_basket.py -- /absolute/path/to/basket-asset
python export_assets.py
blender --background --python build_usd.py
python validate_mujoco.py
```

`export_assets.py` 仅依赖 Python 标准库；MuJoCo 测试使用 `mujoco==3.3.6`、`numpy`、`imageio`、`imageio-ffmpeg`。USD 生成使用 Blender 自带的 OpenUSD 25.2。`validate_isaac.py` 应在 Isaac Sim 4.5.0 的 Python 环境内运行，它会实际导入 URDF 并运行测试，不是仅检查文件存在。

Isaac 中优先加载 `basket.usd`。如果使用 URDF，采用 `validate_isaac.py` 中的导入与碰撞重建步骤：禁用从视觉网格生成碰撞，并明确写入 77 个独立碰撞体。首次诊断发现默认导入后的凸包会封住篮口，不能把未经检查的默认 URDF 导入结果当作可投放物体的篮子。

在 5.21 上可使用 `run_validation.sh` 复用已有 Docker 镜像。视频编码包从已有 MuJoCo 镜像提取到资产目录的 `runtime/`，不会修改现有业务容器。交付压缩包不包含这些运行时依赖和 Blender 安装包。

远端物理诊断最终通过了篮内投放与双提手独立运动；报告在 `physics_diagnostic/evidence/isaac_report.json`。这只证明相同物理结构的运行结果，完整视觉资产的远端同步和 Isaac 导入仍待完成。

本地已下载并校验完整 Linux 安装包：`C:/Users/admin/code/simple/.runtime/parallel-blender-4.5.4-linux-x64.tar.xz`。SHA-256 为 `2e6ef8e99fc36327270429ddc8e7bad2859dd878a5a137d2e0bf0f02f6792505`。网络恢复后，将它传到 `/home/fangqi/.local/opt/blender-4.5.4-verified.tar.xz`，再在完整资产目录运行 `bash run_build.sh`；该脚本再次校验哈希后安装、建模并验证。

参考文档：[Blender 4.5 LTS](https://www.blender.org/releases/4-5/)、[MuJoCo 碰撞计算](https://mujoco.readthedocs.io/en/latest/computation/)、[Isaac Sim 4.5 URDF 导入](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/import_urdf.html)。
