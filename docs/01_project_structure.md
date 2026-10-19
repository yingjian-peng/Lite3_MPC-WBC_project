# 工程结构与文件职责

> 职责：索引当前文件、入口和依赖关系。运行方法见根目录与 `mpc_wbc/` 的 README；任务状态、实验结论和技术公式各由对应文档维护。

## 1. 运行入口

| 入口 | 调用关系 |
| --- | --- |
| [`main.cpp`](../main.cpp) | 构造 `StateMachine`，运行 ONNX 状态机，通过统一机器人接口连接仿真或 SDK |
| [`position_loop_sim.cpp`](../mpc_wbc/apps/position_loop_sim.cpp) | 独立读取场景，调用公共基线参数、步态和 IK，发送关节 PD 命令 |
| [`run_baseline.py`](../mpc_wbc/test/run_baseline.py) | 编排 Python 仿真与开环 app，保存配置、日志及分析结果 |
| [`mpc_wbc/CMakeLists.txt`](../mpc_wbc/CMakeLists.txt) | 注册基础检查到 CTest；生成模型对表数据并运行对应检查 |

`LITE3_CONTROL_PATH=mpc_wbc` 当前没有可运行入口；状态机构造阶段会拒绝该选项。闭环模块的实现状态统一查阅实施进度。

## 2. 源码索引

| 文件或目录 | 唯一职责 |
| --- | --- |
| [`CMakeLists.txt`](../CMakeLists.txt) | 平台、仿真、测试与部署构建选项 |
| [`state_machine/`](../state_machine) | Idle、StandUp、ONNX、LieDown、Damping 状态与切换 |
| [`state_machine/parameters/`](../state_machine/parameters) | 原状态机的站姿、关节参数与增益 |
| [`robot_interface.h`](../interface/robot/robot_interface.h) | 机器人状态、完整快照与关节命令的抽象接口 |
| [`simulation_interface.hpp`](../interface/robot/simulation/simulation_interface.hpp) | C++ UDP 状态接收、快照发布、命令发送及接口生命周期 |
| [`mujoco_simulation.py`](../interface/robot/simulation/mujoco_simulation.py) | MuJoCo 步进、命令执行、状态发送与真值记录 |
| [`model_config.py`](../interface/robot/simulation/model_config.py) | 仿真与对表共用的模型 override 配置与应用 |
| [`hardware_interface.hpp`](../interface/robot/hardware/hardware_interface.hpp) | Lite3 MotionSDK 适配 |
| [`interface/user_command/`](../interface/user_command) | 键盘、Xbox、Retroid 输入适配 |
| [`onnx/`](../onnx) | 原 ONNX 策略模型与推理实现 |
| [`00_lite3_model.hpp`](../mpc_wbc/00_lite3_model.hpp) | FK/IK、雅可比、刚体动力学、质心几何与雅可比 |
| [`01_state_estimator.hpp`](../mpc_wbc/01_state_estimator.hpp) | 支撑足运动学状态估计 |
| [`02_gait_trajectory.hpp`](../mpc_wbc/02_gait_trajectory.hpp) | trot 相位、足端轨迹、分段切换及接触预测 |
| [`baseline_profile.hpp`](../mpc_wbc/baseline_profile.hpp) | 冻结基线参数、命令包络、步幅和斜坡公共定义 |
| [`closed_loop/contracts.hpp`](../mpc_wbc/closed_loop/contracts.hpp) | 坐标导数、总力矩命令及解缓存准入条件 |
| [`closed_loop/model.hpp`](../mpc_wbc/closed_loop/model.hpp) | 显式加入电枢惯量和阻尼的有效动力学 |
| [`scenario_io.hpp`](../mpc_wbc/apps/scenario_io.hpp) | 场景 CSV 解析 |
| [`mpc_wbc/test/`](../mpc_wbc/test) | 基础数值检查、独立真值生成、开环分析与调参工具 |
| [`robot_models/`](../robot_models) | Lite3 MJCF/URDF、网格和地面模型 |
| [`support/`](../support) | 共用类型、数学/遥测工具、部署脚本与外部依赖 |

## 3. 文档分工与入口

| 文档 | 回答的问题 | 内容边界 |
| --- | --- | --- |
| [00 设计指标](00_设计指标.md) | 要达到什么要求？ | 目标、包络、评价口径、验收条件；不登记参数沿革或实测达标率 |
| 本文 | 代码和资料在哪里？ | 文件职责、调用关系、导航；不展开技术推导和实施任务 |
| [04 实施步骤与里程碑](04_Lite3_MPC_WBC实现步骤与里程碑.md) | 做到哪里，接下来做什么？ | 阶段状态、任务依赖、完成条件和证据链接；不复制实验数据或操作命令 |
| [05 实验与优化日志](05_优化日志.md) | 为什么选择这些参数，有什么证据？ | 按日期记录配置、对比、成功/失败、原始证据与结论；不承担当前待办列表 |
| [09 推导与基础契约](09_MPC_WBC重新推导与基础契约.md) | 控制模型与接口如何定义？ | 数学推导、假设、坐标、任务和数据契约；不记录开发进度或测试结果 |

操作入口分别为 [根 README](../README.md)、[控制模块 README](../mpc_wbc/README.md)、[模型说明](../robot_models/README.md) 和 [场景索引](../mpc_wbc/test/scenarios/README.md)。新增内容按上述职责归位，其他文档使用链接引用，避免再维护同一份数据。
