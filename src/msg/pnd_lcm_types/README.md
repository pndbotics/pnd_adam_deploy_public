# PND LCM 类型定义

这个文件夹包含了PND机器人通信所需的LCM类型定义。这些类型基于ROS消息格式转换而来。

## 文件说明

- **pnd_imu_lcmt.lcm**: IMU数据类型，包含姿态角和加速度信息
- **pnd_joint_cmd_lcmt.lcm**: 关节控制命令类型，包含位置、速度、力矩控制信息
- **pnd_robot_state_lcmt.lcm**: 机器人状态类型，包含关节实际位置、速度、力矩信息

## 编译方法

使用LCM工具编译这些类型文件:

```bash
lcm-gen --cpp-std=c++11 *.lcm  # 生成C++11代码
lcm-gen -p *.lcm  # 生成Python代码
``` 