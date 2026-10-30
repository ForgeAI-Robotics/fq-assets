# 可开合笔记本电脑
来源 https://grab3d.com/models/electronics/laptop/ ，CC0；来源原文及文件哈希见 LICENSE.txt、upstream_manifest.json、provenance.json。

MuJoCo model.xml：自由底座 + 被动旋转铰链 lid_hinge。q=0 闭合，正角打开，范围 0–140°，模型默认 110°。角度单位 rad；建议 timestep <= 0.002 s。质量 1.45 kg（底座 1.10、屏幕 0.35），阻尼 0.035 Nms/rad，估计库仑阻力 0.25 Nm；均未实物标定。

USD：双刚体、RevoluteJoint、相同几何/限位、默认打开 110°，粘性阻尼按角度单位换算；未配置引擎专属库仑铰链摩擦。URDF：相同分体与关节，默认 q=0 闭合。GLB 保留上游独立部件与完整原材质，仅用于外观，不带本次物理设置。

底座和屏幕面板使用独立盒状近似碰撞；铰链桶体、按键细节不作接触面。显式启用底座/屏幕面板碰撞。原模型为风格化程序建模，不是实物扫描。

演示视频使用固定底座及临时位置伺服来展示关节；正式 model.xml 不含电机。USD 仅完成结构/引用校验，动力学验证在 MuJoCo 完成；不声称跨引擎力学一致。
