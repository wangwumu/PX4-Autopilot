# abc_vtol Gazebo 模型

尾座式三旋翼 VTOL 的 Gazebo 仿真模型。

## 机体坐标系约定

**与 PX4 一致，见 `src/modules/vtol_att_control/abc_vtol.h` 顶部「ABC-VTOL 机体坐标系约定」。**

- **MC 与 FW 使用同一机体坐标系**，不做单独「FW 坐标系」定义。
- **唯一机体坐标系**：`base_link` = PX4 FRD（Forward-Right-Down），X 前、Y 右、Z 下；Roll/Pitch/Yaw 含义统一，pitch 0° = 机头水平（水平前飞）。
- 仿真只提供该 FRD 机体状态；控制中 MC 与 FW 的欧拉角在同一坐标系下应一致（同一姿态则相同 roll/pitch/yaw）。
