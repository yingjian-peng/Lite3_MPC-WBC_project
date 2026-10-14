# 工程结构与文件说明

> 当前代码索引。控制效果的现状与后续验收以 [重建计划](04_Lite3_MPC_WBC实现步骤与里程碑.md) 为准；“源码存在”不表示整机控制已经调好。

## 1. 运行入口与控制路径

main.cpp 构造 StateMachine。状态机负责 Idle、StandUp、运动控制、LieDown、JointDamping 状态，并由 LITE3_CONTROL_PATH 在启动时选择 ONNX（默认）或 mpc_wbc；两条运动控制路径互斥构造。状态机使用 RobotInterface 读取关节与 IMU，并向仿真或真机下发 12×5 关节命令。

控制链：键盘/手柄与机器人状态 → 状态机 → ONNX 或 MPC+WBC → 12×5 关节命令 → MuJoCo 或 Lite3 SDK。

MuJoCo 使用两个独立的墙钟循环：Python 仿真约每 1 ms 步进并发送状态，C++ SimulationInterface 约每 1 ms 发送最近的命令。协议不锁步、不要求观测/命令一一配对；控制器只在状态时间戳前进时更新目标。其收发方式与 /home/robot/pengyingjian/rl_deploy 对齐。项目特有的站立初始位姿、足端碰撞设置和日志仍由本项目仿真文件配置。

## 2. 目录与主要文件

| 位置 | 职责 |
| --- | --- |
| CMakeLists.txt、main.cpp | x86/ARM 构建选项和程序入口；rl_deploy 是主执行文件 |
| state_machine/ | 状态切换；state_machine.hpp 选 ONNX/MPC 路径，mpc_control_state.hpp 接入 MPC+WBC |
| state_machine/parameters/ | Lite3 站姿、增益和关节相关参数 |
| interface/robot/robot_interface.h | 状态读取与 12×5 关节命令统一接口 |
| interface/robot/simulation/ | C++ UDP 接口与 Python MuJoCo 仿真 |
| interface/robot/hardware/ | Lite3 MotionSDK 真机接口；MPC+WBC 尚未完成真机验证 |
| interface/user_command/ | 键盘、Xbox、Retroid、Skydroid 输入接口 |
| onnx/run_policy/、onnx/policy/ | 原有 ONNX 控制器与模型，供独立运行和对照 |
| mpc_wbc/00_lite3_model.hpp | Lite3 运动学、接触雅可比和全身动力学 |
| mpc_wbc/01_state_estimator.hpp | 基座速度、高度和有效性估计 |
| mpc_wbc/02_gait_trajectory.hpp | trot 相位、足端轨迹和未来接触表 |
| mpc_wbc/03_srbd_mpc.hpp、box_qp.hpp | SRBD 预测控制与盒约束 QP |
| mpc_wbc/04_weighted_wbc.hpp、qp_ipm.hpp | 全身 QP 与内点法求解器 |
| mpc_wbc/05_mpc_wbc_controller.hpp | 控制器编排、MPC 工作线程、命令和故障处理 |
| mpc_wbc/apps/position_loop_sim.cpp | 当前重建的开环主线：步态、IK、关节 PD，无 MPC/WBC |
| mpc_wbc/apps/mpc_wbc_sim.cpp | 已有 MPC+WBC 仿真 harness，留待分层复验 |
| mpc_wbc/test/ | 模型对表、离线单测、MuJoCo 场景与日志分析工具 |
| robot_models/ | Lite3 MJCF/URDF、平地和网格模型 |
| support/ | 公共类型、工具及 Eigen、ONNX Runtime、MotionSDK 等依赖 |

## 3. 文档入口

- [算法框架](02_MPC-WBC算法框架.md)：这次重建采用的分层控制和接口约定。
- [理论与旧实现参考](03_Lite3_MPC_WBC从理论到实现.md)：含另一套实现经验，不能当作本工程当前代码或验收结论。
- [重建计划与进度](04_Lite3_MPC_WBC实现步骤与里程碑.md)：当前优化顺序、阶段门槛和新实验记录。
- [旧版实现与试验记录](archive/04_旧版实现与试验记录.md)：历史日志，供追溯，不作为新节拍下的基线。
