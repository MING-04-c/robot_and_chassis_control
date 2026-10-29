# 参照 PDF 的亮度渐变与断线处理

参考文件：`基于OpenCV在含噪亮度渐变图像中检测深灰色斜线的问题.pdf`，重点为第2、3页。
文中建议亮度处理、保边去噪、方向性连接、连通域过滤及自适应霍夫线段检测。
当前场景是蓝色胶带，本次保留蓝色识别要求，借用其亮度和几何处理思路。

## 当前流程

1. 保留原有中值滤波。用局部明度背景估计对暗处的明度门槛做补偿，增益最多2倍；
   原始色相、饱和度、蓝色优势及原始信号下限仍参与筛选，灰色线不会因此成为蓝色胶带。
2. 继续按连通域、细长度、内部实心程度和截面宽度过滤噪声。
3. 每个合格候选用实测截面中点生成中心投票图，只在图中连接同一候选内相邻采样点，
   避免稀疏采样导致斜线无法投票；这个方向性连接不更改真实掩膜或实测中线。执行 HoughLinesP。
   投票门槛随当前候选实测点数量变化，最短线段随处理图尺寸变化；只在多数点支持同一方向时
   用该线段的内点稳健拟合候选方向。弯线或证据不足时回到原有局部端点拟合。
4. 缺口两端的实测长度、宽度、间隔先满足限制，再通过“同向对齐”或“延长线在缺口内部相交”
   中的一项，保留两段候选。中间不填入真实蓝色掩膜，也不发布成实测中线。
5. 控制端也用多点中位数斜率拟合两侧方向，降低单个毛刺或反光端点对方向判断的影响。
   已批准的连接按同一条线跟随，不因缺口额外降速；只有单端或关联失败时跟随剩余可用近段。
6. 视觉不足立即发零速度；独立任务保留5秒恢复窗口，连续有效帧恢复后继续。
   通信、图像时效和底盘状态检查保持生效，单轮总超时仍为90秒。

没有直接对整张灰度图做霍夫循迹，也没有用大尺度闭运算把反光地面涂成胶带。
方向性连接表示一项推测关联，不代表白色反光区域已经被相机测量为真实线段。

## 参数与诊断

视觉配置：`src/ruier_camer_diver/config/line_tracker.yaml`。

| 参数 | 部署值 | 作用 |
|---|---|---|
| local_brightness_normalization | true | 局部明度补偿 |
| hough_join_support | true | 霍夫线段辅助方向拟合 |
| extended_join_gap_ratio | 0.30 | 长缺口候选续接上限 / 图像扫描轴长度 |
| extended_join_support_ratio | 0.10 | 两端各自最低实测跨度 |
| extended_join_angle_rad | 0.60 | 缺口内相交允许转角上限，约34度 |

霍夫辅助只在候选多数实测点支持直线时采用；`line/status` 的
`hough_supported_candidates` 表示本帧获得霍夫方向支持的合格候选数，0表示没有采用，
不等同于没有路线。`candidates[].retained` 与 `joins[].rejection` 可检查另一端为何未保留。

控制参数仍在 `src/masc_chassis_user_control/config/line_follow.yaml`。
预览 `GAP: JOINED` 标记通过两端几何验证的连接，`INPUT` 显示当前输入可行性，`LAST BLOCK` 保留上次阻塞原因。
若 INPUT 为 tracking 而 CONTROL 为 disabled，需启动任务；继续降低图像阈值不会使能任务。

## 运行

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ruier_camer_diver masc_chassis_user_control robot_task_control
source install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py line_show_window:=true
```

完成 CAN 配置后，在另一个已加载环境的终端开始任务：

```bash
ros2 service call /task_controller/start std_srvs/srv/Trigger '{}'
```

停止旧启动会话后再加载新代码，避免重复节点。上述规则为图像几何判定，未做地面距离标定，
不能据图像比例承诺实际可跨越的厘米数；本次以含噪亮度渐变、暗蓝片段、反光缺口及
背景灰线的合成图测试，实车效果仍需验证。

已确认续接路径的缺口系数默认1.0，不再叠加上游疑似遮挡减速。若前瞻点在已确认缺口内部，使用端点之间的插值目标，保留实测与插值的区别（geometry.target_gap_interpolated）。单端、错位、过大缺口及设备故障的规则不放宽；胶带几何关联不是实际障碍物检测。
