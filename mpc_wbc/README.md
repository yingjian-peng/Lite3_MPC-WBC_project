# MPC/WBC 目录与当前重建入口

本目录保留 Lite3 的运动学、动力学、状态估计、步态、SRBD-MPC、Weighted-QP WBC 及测试工具。现阶段先调通“足端轨迹 → IK → 关节 PD”的开环位置控制，前馈力矩为零；已有优化器暂不作为行走效果已经合格的证据。当前阶段、门槛与实验记录见 [重建计划](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)，旧实验见 [归档](../docs/archive/04_旧版实现与试验记录.md)。

## 代码入口

| 文件 | 用途 |
| --- | --- |
| 00_lite3_model.hpp | 运动学、雅可比、全身动力学与质心惯量 |
| 01_state_estimator.hpp | 基座速度、高度、估计有效性 |
| 02_gait_trajectory.hpp | trot 相位、足端位置/速度目标、未来支撑表 |
| 03_srbd_mpc.hpp、box_qp.hpp | SRBD 预测控制与接触力 QP |
| 04_weighted_wbc.hpp、qp_ipm.hpp | 全身控制 QP 与求解器 |
| 05_mpc_wbc_controller.hpp | 估计、步态、MPC 线程和 WBC 的已有编排 |
| apps/position_loop_sim.cpp | 当前开环调试主程序，无 MPC/WBC 前馈 |
| apps/mpc_wbc_sim.cpp | 已有优化器整机 harness，留待逐层复验 |
| test/scenarios/、test/analyze_s9_run.py | 场景与日志分析；旧 PASS 门槛需重新审定 |

## 构建与离线检查

在工程根目录执行：

    cmake -S . -B build -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DBUILD_MPC_WBC_SIM=ON -DBUILD_MPC_WBC_TESTS=ON
    cmake --build build -j
    ctest --test-dir build --output-on-failure

CTest 检查模型对表、步态、估计、QP 和控制器离线行为。它不运行完整的实时 MuJoCo 行走场景，也不证明开环或 MPC+WBC 整机效果。

## 当前开环调试

无头采集可运行：

    mpc_wbc/test/run_s4_position_loop.sh 24

针对单个场景运行开环：

    mpc_wbc/test/run_s9_ab.sh --baseline --only axis_fwd

也可先运行 Python 仿真，再运行 build/mpc_wbc/mpc_wbc_position_loop_sim。仿真按墙钟约 1 ms 自主步进，C++ 接口独立约 1 ms 发送最近命令，与参考 /home/robot/pengyingjian/rl_deploy 的通信节拍一致。控制器按新状态时间戳更新目标；两端不锁步，重复运行应统计波动，不能期待逐位相同。

主要开环参数由 S4_KP、S4_KD、S4_SWING_HEIGHT、S4_PERIOD、S4_DUTY、S4_CMD_SCALE、S4_SWEEP_GAIN 等环境变量覆盖。当前默认值只代表代码中的旧配置，需要在新仿真节拍下重新测量，不能直接沿用旧报告里的“最优参数”。S4_TRACE 可记录每控制周期的目标命令。初始姿态与接触模式由 SIM_INIT_POSE、SIM_INIT_HEIGHT 和 SIM_LEG_COLLISION 设置；SIM_LOG、SIM_LOG_EVERY、SIM_DURATION 与 MUJOCO_VIEWER 控制日志、时长和显示。

先看站姿、关节跟踪和真实力矩，再看单腿净空、支撑滑移、三轴速度及停步。至少多次重复同一场景，保存原始 CSV、配置和代码版本。旧版锁步结果在 [历史记录](../docs/archive/04_旧版实现与试验记录.md) 中保留供追溯，但不能作为新时序的对照基线。

## 后续优化器路径

优化器源码和 mpc_wbc_mpc_wbc_sim、state_machine/mpc_control_state.hpp 仍可构建。进入这些路径前，先完成 [R0–R3 开环验收](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)；随后逐项打开高度/姿态反馈、状态估计、力前馈、MPC 和 WBC。每次只改变一个控制层，保留可回退的上一步配置，分别验证数学残差与真实足端/机身运动。
