# 末端装配模型

当前模型来自主目录 `robot (2)/robot.urdf` 及其三个STL；保留相对tool0的原有-90°安装旋转；按现场确认，安装平移为 `[0,0,-0.0027]` 米（沿tool0 -Z靠近法兰2.7mm）。tool0与力传感器TF未修改。

| 新版URDF | 共用模型名称 | 网格 |
| --- | --- | --- |
| base_link | terminal_base_link（空装配基准） | 无 |
| Joint_1 → Link_1 | terminal_assembly_fixed → terminal_assembly_link | Link_1.stl |
| Joint_2 → Link_2 | terminal_joint_1 → terminal_link_1（相机） | Link_2.stl |
| Joint_3 → Link_4 | terminal_joint_2 → terminal_link_2（探针） | Link_4.stl |

三条固定关节的位置和旋转直接取新版URDF。空的Link_5没有几何或后代，其Joint_4不加入六轴控制链。相机转换使用terminal_link_1，不额外叠加绕Z的180°修正。外形和碰撞使用同一套新网格，真机、仿真和规划共用此模型。

安装平移统一作用于装配本体、相机、探针的可视/碰撞几何及TF，不修改导出模型的内部关节参数。默认相机原点在tool0下为 `[-0.05748778,-0.02920351,0.0555]` 米，探针检测点为 `[0,0,0.1460]` 米。
