# RhinoV6 实装打印夹爪

原包完整保留在 `source/`；`model.urdf` 仅改为可移植网格路径；`model.glb` 为零位静态装配预览。

- Eight revolute joints have zero range, effort and velocity
- No linkage closure, mimic constraints, spring forces or actuators supplied
- Print tolerances and installation transform to Tianji are unverified
- GLB is a static zero-pose visual preview, not a physics articulation
- Source collision meshes retained; contact behavior has not been validated
- No claim that all STL links are individual fabrication-ready parts

## 安装结构件

见 `mounting_parts.json`，列出源 URDF 的 link、网格和连接关节。Tianji 的 `upper_adapter_flange.stl` 为左右共用法兰；`Link_Stand` 下含立柱/头部结构网格，`assembly_lower_mount` 下含 cad_11/12。空坐标系不作为实体支架。RhinoV6 的 base_link.STL 含基座，其他网格为夹爪连杆；不等于独立支架打印套件。CAD 编号件的具体制造名称未确认。

Tianji 源夹爪与 RhinoV6 几何不同。用户确认实机安装 RhinoV6。独立夹爪保留原始数据；Tianji 0.2.0 活动装配已使用左右两套 RhinoV6 和三台 D435i，源 source/ 中旧夹爪仅用于溯源。法兰安装变换仍待实测标定，未验证孔位、紧固和动态抓取。

网页使用 `preview.xml`：固定零位、关闭接触，只用于外观展示；它不是可执行动力学版本。源 URDF 保留原有碰撞定义。
