# Lite3 模型与平地环境

```text
robot_models/
├── robots/Lite3/
│   ├── Lite3_urdf/urdf/Lite3.urdf
│   ├── Lite3_urdf/meshes/
│   ├── Lite3_mjcf/mjcf/Lite3.xml
│   ├── Lite3_mjcf/mjcf/Lite3_flat.xml
│   └── Lite3_mjcf/meshes/
└── environments/flat/mjcf/floor.xml
```

`Lite3.xml` 定义机器人本体、惯量、12 个驱动关节和 MuJoCo 执行器。`Lite3_flat.xml` 组合机器人与平地，供 `interface/robot/simulation/mujoco_simulation.py` 加载。URDF 保留用于核对关节、几何和动力学参数。

当前工程仅保留 Lite3 平地验证所需的场景。机器人与真机的关节顺序、方向、限位和惯量仍需在 MPC+WBC 实现前核验。
