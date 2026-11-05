# 工程结构与文件说明

> 当前代码索引。核心模块与调用关系见 [MPC+WBC 核心算法说明](README.md)，历史实测结果见 [设计指标](00_设计指标.md)。独立调参程序、验收脚本和场景已删除。

## 1. 运行入口与控制路径

main.cpp 构造 StateMachine。当前 CMake 构建仿真程序；用户输入仅保留键盘和飞智沙漠狐 USB 手柄（有线 XInput 模式，VID:PID `045e:028e`，由 Linux `xpad` 识别为 Xbox 360 手柄）。默认使用键盘；设置 `LITE3_USER_INPUT=usb` 时读取 `/dev/input/js0`。手柄按键为 A 站立、B 躺下、X 选 RL、Y 选 MPC、LB 阻尼；摇杆死区约 10%，满杆映射到对应方向的最大归一化速度命令。状态机同时注册 ONNX RL 与 MPC+WBC 两种运动控制状态，由站立后的用户输入选择。状态机通过 SimulationInterface 读取关节与 IMU，并向 MuJoCo 下发 12×5 关节命令。

控制链：键盘与 MuJoCo 状态 → 状态机 → ONNX 或 MPC+WBC → 12×5 关节命令 → MuJoCo。

MuJoCo 使用两个独立的墙钟循环：Python 仿真约每 1 ms 步进并发送状态，C++ SimulationInterface 约每 1 ms 发送最近的命令。协议不锁步、不要求观测/命令一一配对；控制器只在状态时间戳前进时更新目标。其收发方式与 /home/robot/pengyingjian/rl_deploy 对齐。项目特有的站立初始位姿、足端碰撞设置和日志仍由本项目仿真文件配置。

## 2. 目录与主要文件

| 位置 | 职责 |
| --- | --- |
| CMakeLists.txt、main.cpp | 构建入口和程序入口；生成共用状态机的 mpc_control、rl_control |
| state_machine/ | 状态切换；站立后选择 RL 或 MPC，mpc_control_state.hpp 接入 MPC+WBC |
| state_machine/parameters/ | Lite3 站姿、增益和关节相关参数 |
| interface/robot/robot_interface.h | 状态读取与 12×5 关节命令统一接口 |
| interface/robot/simulation/ | C++ UDP 接口与 Python MuJoCo 仿真 |
| interface/robot/hardware/ | 保留的 Lite3 MotionSDK 真机接口；当前主程序只构建仿真路径 |
| interface/user_command/ | 键盘与飞智沙漠狐 USB 手柄输入（有线 XInput）；默认键盘，可用 `LITE3_USER_INPUT=usb` 选择手柄 |
| onnx/run_policy/、onnx/policy/ | RL 模式使用的 ONNX 推理实现与模型 |
| mpc_wbc/00_lite3_model.hpp | Lite3 运动学、接触雅可比和全身动力学 |
| mpc_wbc/01_state_estimator.hpp | 基座速度、高度和有效性估计 |
| mpc_wbc/02_gait_trajectory.hpp | trot 相位、足端轨迹和未来接触表 |
| mpc_wbc/03_box_qp.hpp | MPC 使用的盒约束 QP 求解器 |
| mpc_wbc/04_qp_ipm.hpp | WBC 使用的等式/不等式 QP 内点法求解器 |
| mpc_wbc/05_srbd_mpc.hpp | SRBD 预测控制与基座参考 |
| mpc_wbc/06_weighted_wbc.hpp | 全身动力学与任务 QP 控制 |
| mpc_wbc/07_mpc_wbc_controller.hpp | 控制器编排、MPC 工作线程、命令和故障处理 |
| robot_models/ | Lite3 MJCF/URDF、平地和网格模型 |
| support/ | 公共类型、工具及 Eigen、ONNX Runtime、MotionSDK 等依赖 |

## 3. 文档入口

- [算法框架](02_MPC-WBC算法框架.md)：这次重建采用的分层控制和接口约定。
- [理论与旧实现参考](03_Lite3_MPC_WBC从理论到实现.md)：含另一套实现经验，不能当作本工程当前代码或验收结论。
- [重建计划与进度](04_Lite3_MPC_WBC实现步骤与里程碑.md)：当前优化顺序、阶段门槛和新实验记录。
- [旧版实现与试验记录](archive/04_旧版实现与试验记录.md)：历史日志，供追溯，不作为新节拍下的基线。
