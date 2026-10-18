import os
import time
import socket
import struct
import threading
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from colorama import init, Fore, Style

# Initialize colorama for colored terminal output
init(autoreset=True)

MODEL_NAME = "lite3"
XML_PATH = "../../../robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"
LOCAL_PORT = 20001
CTRL_IP = "127.0.0.1"
CTRL_PORT = 30010
USE_VIEWER = os.environ.get("MUJOCO_VIEWER", "1") != "0"
PRINT_DEBUG = os.environ.get("SIM_DEBUG", "1") != "0"
SIM_LOG = os.environ.get("SIM_LOG", "")            # 可选：把状态写为 CSV
SIM_LOG_EVERY = int(os.environ.get("SIM_LOG_EVERY", "10"))
SIM_DURATION = float(os.environ.get("SIM_DURATION", "0"))  # >0 时到点自动退出
SIM_INIT_YAW = float(os.environ.get("SIM_INIT_YAW", "0"))  # 初始航向（rad，绕世界 z）
DT = 0.001
RENDER_INTERVAL = 10
# 与 /home/robot/pengyingjian/rl_deploy 一致：MuJoCo 按墙钟独立步进，
# 控制器和仿真各自运行；每步使用最近收到的关节命令。

URDF_INIT = {
    "lite3": np.array([0, -1.35453, 2.54948] * 4, dtype=np.float32)
}

# 站立构型（与 mpc_wbc 的 `DefaultStandLegAngles` 一致）与「足端球刚好相切」的基座高度。
STAND_LEG_ANGLES = (0.0, -0.65, 1.3)
STAND_BASE_HEIGHT = 0.3485

# 初始位姿模式：`stand`（默认）= 从站立平衡位开始；`drop` = 旧的 MJCF 默认
# （基座抬到 1 m + URDF 折叠关节角，等价于「高空折叠坠落再自己撑起来」）。
# 落地本身是确定的，但**自撑起**过程对控制帧时序极敏感，会给所有行走试验注入
# 不可重复的初始条件（同一场景 0.34 m vs 0.80 m，见 docs/04 §8「S9 第三项」）。
SIM_INIT_POSE = os.environ.get("SIM_INIT_POSE", "stand").strip()
SIM_INIT_HEIGHT = float(os.environ.get("SIM_INIT_HEIGHT", str(STAND_BASE_HEIGHT)))

# S9 诊断日志用：足端 body（顺序与 mpc_wbc 的腿序 FL, FR, HL, HR 一致）
FOOT_BODY_NAMES = ("FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT")
kNumFeet = len(FOOT_BODY_NAMES)

class MuJoCoSimulation:
    def __init__(self, model_key: str = MODEL_NAME, 
                 xml_relpath: str = XML_PATH,
                 local_port: int = LOCAL_PORT, 
                 ctrl_ip: str = CTRL_IP, 
                 ctrl_port: int = CTRL_PORT):
        
        # UDP interface
        self.local_port = local_port
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("0.0.0.0", local_port))
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Load MJCF
        xml_full = str(Path(__file__).resolve().parent / xml_relpath)
        print("xml_full", xml_full)
        if not os.path.isfile(xml_full):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_full}")

        self.model = mujoco.MjModel.from_xml_path(xml_full)
        self.model.opt.timestep = DT
        self._apply_model_overrides()

        # 接触模型：**默认只让足端球与地面接触**。本 MJCF 的小腿碰撞 mesh（`*_SHANK_collision`，
        # 与 visual 同一个 shank.STL，整条小腿）在名义站姿下也压地——实测它承担约 97% 的支撑力
        # （每腿竖向 19–28 N）且 mesh 三角面接触的法向倾斜 ~20°，带来几十牛的**水平**支反力。这与
        # 真实狗（只有橡胶足端球触地）以及 MPC/WBC 假设的「点足 + 竖直法向 + 摩擦锥在足端」都不符。
        # （该 MJCF 里机器人各 geom 都是 contype=0/conaffinity=1，即本来就**没有**自碰撞，所以关掉这
        #  几个 geom 的碰撞只影响「小腿 × 地面」这一对。）
        # SIM_LEG_COLLISION=full 可恢复原来的行为（用于对照测量）。
        self.leg_collision_mode = os.environ.get("SIM_LEG_COLLISION", "foot_only").strip()
        if self.leg_collision_mode != "full":
            disabled = []
            for i in range(self.model.ngeom):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
                if "SHANK_collision" in name:
                    self.model.geom_contype[i] = 0
                    self.model.geom_conaffinity[i] = 0
                    disabled.append(name)
            print(f"[sim] 接触模型：只保留足端球（已关闭 {len(disabled)} 个小腿碰撞 geom；"
                  f"SIM_LEG_COLLISION=full 可恢复）")

        # 坡面：SIM_SLOPE_DEG 把地面绕世界 y 轴倾斜（正值 = +x 方向下坡）。
        # 默认 0 即平地，不影响既有结果。用于评估开环基线的坡度鲁棒性。
        self.slope_deg = float(os.environ.get("SIM_SLOPE_DEG", "0") or 0.0)
        if self.slope_deg != 0.0:
            half = 0.5 * np.radians(self.slope_deg)
            quat = np.array([np.cos(half), 0.0, np.sin(half), 0.0])
            tilted = []
            for i in range(self.model.ngeom):
                if self.model.geom_type[i] != mujoco.mjtGeom.mjGEOM_PLANE:
                    continue
                self.model.geom_quat[i] = quat
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom{i}"
                tilted.append(name)
            print(f"[sim] 坡面：地面绕 y 轴倾斜 {self.slope_deg:+.2f}°（正=+x 下坡），geom={tilted}")

        self.data = mujoco.MjData(self.model)

        # Robot DOF list
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..11
        self.dof_num = len(self.actuator_ids)

        # Initialize standing pose
        self._set_initial_pose(model_key)

        # Buffers
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)
        # 收到第一帧控制包之前用内部 PD 保持初始姿态（见 _apply_joint_torque）
        self.received_cmd = False
        self.qpos_hold = self.data.qpos[7:7+self.dof_num].copy().reshape(-1, 1)
        self.hold_kp = float(os.environ.get("SIM_HOLD_KP", "200"))
        self.hold_kd = float(os.environ.get("SIM_HOLD_KD", "5"))

        # 外力扰动：SIM_PUSH_FX/FY [N]、SIM_PUSH_T0 [s]、SIM_PUSH_DUR [s]，
        # 作用在机身（TORSO）质心上，用于评估推力扰动的恢复能力。默认全 0 = 关闭。
        self.push_fx = float(os.environ.get("SIM_PUSH_FX", "0") or 0.0)
        self.push_fy = float(os.environ.get("SIM_PUSH_FY", "0") or 0.0)
        self.push_t0 = float(os.environ.get("SIM_PUSH_T0", "0") or 0.0)
        self.push_dur = float(os.environ.get("SIM_PUSH_DUR", "0") or 0.0)
        self.push_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
        if self.push_body < 0:
            self.push_body = 1  # 兜底：世界体是 0，机器人基座是 1
        if self.push_dur > 0.0:
            print(f"[sim] 推力扰动：({self.push_fx:+.1f}, {self.push_fy:+.1f}) N，"
                  f"t={self.push_t0:.2f}–{self.push_t0 + self.push_dur:.2f} s，"
                  f"impulse=({self.push_fx * self.push_dur:+.2f}, "
                  f"{self.push_fy * self.push_dur:+.2f}) N·s")

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0
        self.last_print_time = 0  # Track last print time
        self._foot_body_to_leg = None  # 足端 body id → 腿序号（首次写日志时构建）

        print(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # Visualization
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._set_viewer_camera()

    def _set_viewer_camera(self):
        self.viewer.cam.distance = 3.8
        self.viewer.cam.azimuth = 135
        self.viewer.cam.elevation = -18
        self.viewer.cam.lookat[:] = np.array([0.4, 0.0, 0.45])

    def _apply_model_overrides(self):
        """仿真层叠加执行器/足垫模型（**默认按本工程标定值开启**）。

        官方 MJCF 与 URDF 均未给关节 damping/armature，足端接触为 condim=3（无扭转摩擦）；
        真机减速器有转子惯量与粘性阻尼，橡胶球足有接触斑扭转刚度。这些属于模型的简化，
        不写进 XML，用环境变量控制，便于 A/B 对照和回溯：

          SIM_JOINT_ARM     [kg*m^2]     关节电枢（转子折算）惯量，只作用于 12 个铰链关节
          SIM_JOINT_DAMP    [N*m*s/rad]  关节粘性阻尼
          SIM_FOOT_CONDIM   [3|4|6]      足端接触维度，4 启用扭转摩擦
          SIM_FOOT_TORSION  [-]          扭转摩擦系数（friction 第 2 项），需 condim>=4

        默认值 0.01 / 0.1 / 4 / 0.03 为本工程开环基线标定值；
        要回到与官方模型完全一致的行为，四个都设 0。
        """
        arm = float(os.environ.get("SIM_JOINT_ARM", "0.01") or 0.0)
        damp = float(os.environ.get("SIM_JOINT_DAMP", "0.1") or 0.0)
        condim = int(os.environ.get("SIM_FOOT_CONDIM", "4") or 0)
        torsion_raw = os.environ.get("SIM_FOOT_TORSION", "0.03").strip()
        torsion = float(torsion_raw) if torsion_raw and float(torsion_raw) > 0 else None
        if arm <= 0.0 and damp <= 0.0 and condim <= 0 and torsion is None:
            print("[sim] model override: off (与官方模型一致)")
            return
        torsion_effective = torsion if torsion is not None else 0.03

        hinges = [int(self.model.jnt_dofadr[j]) for j in range(self.model.njnt)
                  if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
        if arm > 0.0:
            self.model.dof_armature[hinges] = arm
        if damp > 0.0:
            self.model.dof_damping[hinges] = damp

        foot_ids = []
        if condim >= 3 or torsion is not None:
            for i in range(self.model.ngeom):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
                if name.endswith("FOOT_collision"):
                    foot_ids.append(i)
        if condim >= 3 and foot_ids:
            self.model.geom_condim[foot_ids] = condim
        if torsion is not None and foot_ids:
            self.model.geom_friction[foot_ids, 1] = torsion

        print(f"[sim] model override: armature={arm:g} damping={damp:g} "
              f"foot_condim={condim if condim >= 3 else 3} "
              f"foot_torsion={torsion_effective:g} "
              f"({len(foot_ids)} foot geoms)")

    def _set_initial_pose(self, key: str):
        """初始位姿。默认从**站立平衡位**开始（关节 = 站立构型，基座 = 足端刚好相切的高度），
        这样 A/B 试验的初始条件可重复；`SIM_INIT_POSE=drop` 恢复「高空折叠坠落」写法。"""
        qpos0 = self.data.qpos.copy()
        if SIM_INIT_POSE == "stand":
            qpos0[2] = SIM_INIT_HEIGHT
            qpos0[7:7+self.dof_num] = np.tile(np.asarray(STAND_LEG_ANGLES, dtype=np.float64),
                                              self.dof_num // 3)
            print(f"[sim] 初始位姿=站立（基座 z={SIM_INIT_HEIGHT:.4f} m，关节={STAND_LEG_ANGLES}）")
        else:
            qpos0[7:7+self.dof_num] = URDF_INIT[key]
            print("[sim] 初始位姿=drop（MJCF 默认：基座 1 m + URDF 折叠关节角）")
        if SIM_INIT_YAW != 0.0:  # S9：起始航向（只改基座偏航，姿态仍水平）
            qpos0[3:7] = [np.cos(0.5 * SIM_INIT_YAW), 0.0, 0.0, np.sin(0.5 * SIM_INIT_YAW)]
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)
        self.qpos_hold = self.data.qpos[7:7+self.dof_num].copy().reshape(-1, 1)

    def print_debug_info(self):
        """Consolidated function to print debug information with colors and aligned formatting."""
        # Format arrays with 2 decimal places and fixed width
        def format_array(arr):
            return "[" + ", ".join(f"{x:6.2f}" for x in arr) + "]"

        # Get current joint states for printing
        q = self.data.qpos[7:7+self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6+self.dof_num].reshape(-1, 1)
        tau = self.input_tq.flatten()
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        angvel_b = self.data.qvel[3:6]
        mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(mat, q_world.astype(np.float64))
        R = mat.reshape(3, 3)
        body_acc = self.data.sensordata[16:19]

        print(f"{Fore.CYAN}=== [Debug Info] ==={Style.RESET_ALL}")
        print(f"{Fore.GREEN}[IMU] RPY        :{Style.RESET_ALL} {format_array(rpy.flatten())}")
        print(f"{Fore.GREEN}[IMU] Omega      :{Style.RESET_ALL} {format_array(angvel_b.flatten())}")
        print(f"{Fore.GREEN}[IMU] Acc_body   :{Style.RESET_ALL} {format_array(body_acc.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Position  :{Style.RESET_ALL} {format_array(q.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Velocity  :{Style.RESET_ALL} {format_array(dq.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Torque    :{Style.RESET_ALL} {format_array(tau.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Pos:{Style.RESET_ALL} {format_array(self.pos_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Pos:{Style.RESET_ALL} {format_array(q.T.flatten())}")     
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Vel:{Style.RESET_ALL} {format_array(self.vel_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Vel:{Style.RESET_ALL} {format_array(dq.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kp Term   :{Style.RESET_ALL} {format_array(self.kp_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kd Term   :{Style.RESET_ALL} {format_array(self.kd_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] FF Tau    :{Style.RESET_ALL} {format_array(self.tau_ff.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Final Torq:{Style.RESET_ALL} {format_array(self.input_tq.T.flatten())}")
        print(f"{Fore.CYAN}==================={Style.RESET_ALL}")

    def _foot_contact_force_world(self):
        """逐足世界系接触力（累加该足 geom 参与的所有接触）+ 全部接触合力。

        返回 (f_feet(4,3), f_all(3))。腿部顺序 FL, FR, HL, HR。
        注意：Lite3 的 `*_SHANK_collision` geom 在名义站姿（h=0.33）下也会压在地面上
        （实测四条小腿各承 ~25 N，而足端只有 ~0.1 N），所以「谁在承重」不能只看足端。
        """
        f = np.zeros((kNumFeet, 3))
        f_all = np.zeros(3)
        if self._foot_body_to_leg is None:
            self._foot_body_to_leg = {}
            for leg, name in enumerate(FOOT_BODY_NAMES):
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                if body_id >= 0:
                    self._foot_body_to_leg[body_id] = leg
        res = np.zeros(6, dtype=np.float64)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            mujoco.mj_contactForce(self.model, self.data, i, res)
            # 接触坐标系的行向量即世界系基向量（第 0 行为法向）→ world = frame.T @ local
            f_world = c.frame.reshape(3, 3).T @ res[:3]
            f_all += f_world
            leg = self._foot_body_to_leg.get(self.model.geom_bodyid[c.geom1])
            if leg is None:
                leg = self._foot_body_to_leg.get(self.model.geom_bodyid[c.geom2])
            if leg is not None:
                f[leg] += f_world
        return f, f_all

    def _write_log(self, f):
        """写一行诊断日志：时间、基座位姿/速度、关节位置/速度、命令位置与 kp、力矩、足端接触与位置。"""
        base = self.data.qpos[0:7]
        v = self.data.qvel[0:3]
        w = self.data.qvel[3:6]
        q = self.data.qpos[7:7 + self.dof_num]
        dq = self.data.qvel[6:6 + self.dof_num]
        # input_tq = PD+ff 的计算值（未限幅）；actuator_force = ctrlrange 夹取后真正作用的力矩
        tau_cmd = np.asarray(self.input_tq, dtype=np.float64).flatten()
        tau_act = np.asarray(self.data.actuator_force, dtype=np.float64).flatten()
        foot_force, all_force = self._foot_contact_force_world()
        foot_pos = np.array([
            self.data.body(name).xpos for name in FOOT_BODY_NAMES
        ], dtype=np.float64)
        vals = [self.timestamp, *base, *v, *w, *q, *dq,
                *self.pos_cmd.flatten(), *self.kp_cmd.flatten(),
                *tau_cmd, *tau_act, *all_force.flatten(),
                *foot_force.flatten(), *foot_pos.flatten()]
        f.write(",".join(f"{x:.6g}" for x in vals) + "\n")
        f.flush()

    def start(self):
        # Start UDP receiver thread
        threading.Thread(target=self._udp_receiver, daemon=True).start()
        print(f"[INFO] UDP receiver on 0.0.0.0:{self.local_port}")

        log_f = None
        if SIM_LOG:
            log_f = open(SIM_LOG, "w")
            cols = (["t", "bx", "by", "bz", "qw", "qx", "qy", "qz",
                     "vx", "vy", "vz", "wx", "wy", "wz"]
                    + [f"q{i}" for i in range(self.dof_num)]
                    + [f"qd{i}" for i in range(self.dof_num)]
                    + [f"qcmd{i}" for i in range(self.dof_num)]
                    + [f"kp{i}" for i in range(self.dof_num)]
                    + [f"tau{i}" for i in range(self.dof_num)]      # PD+ff 计算值（未限幅）
                    + [f"atau{i}" for i in range(self.dof_num)]     # ctrlrange 夹取后实际力矩
                    + ["fxall", "fyall", "fzall"]                   # 全部接触合力（世界系）
                    + [f"f{x}{leg}" for leg in range(kNumFeet) for x in ("x", "y", "z")]
                    + [f"p{x}{leg}" for leg in range(kNumFeet) for x in ("x", "y", "z")])
            log_f.write(",".join(cols) + "\n")
            log_f.flush()
            print(f"[INFO] logging to {SIM_LOG}")

        # Main simulation loop
        step = 0
        last_time = time.time()
        while True:
            if time.time() - last_time >= DT:
                last_time = time.time()

                step += 1
                # 控制律

                # 外力扰动窗口（默认关闭）
                if self.push_dur > 0.0:
                    active = self.push_t0 <= self.timestamp < self.push_t0 + self.push_dur
                    self.data.xfrc_applied[self.push_body, 0] = self.push_fx if active else 0.0
                    self.data.xfrc_applied[self.push_body, 1] = self.push_fy if active else 0.0

                self._apply_joint_torque()
                # 模拟一步
                mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT

                # 采样 & 发送观测
                self._send_robot_state(step)
                if log_f is not None and step % SIM_LOG_EVERY == 0:
                    self._write_log(log_f)
                # 可视化
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

                # Print at 0.5 Hz (every 2 seconds)
                if PRINT_DEBUG:
                    current_time = time.perf_counter()
                    if current_time - self.last_print_time >= 2.0:
                        self.print_debug_info()
                        self.last_print_time = current_time

                if SIM_DURATION > 0 and self.timestamp >= SIM_DURATION:
                    print(f"[INFO] reached SIM_DURATION={SIM_DURATION}s, exiting")
                    if log_f is not None:
                        log_f.close()
                    return
                    

    def _udp_receiver(self):
        """
        12f kp | 12f pos | 12f kd | 12f vel | 12f tau = 240 bytes
        """
        fmt = f'{self.dof_num}f' * 5
        expected = struct.calcsize(fmt)
        while True:
            data, addr = self.recv_sock.recvfrom(expected)
            if len(data) < expected:
                print(f"[WARN] UDP packet size {len(data)} != {expected}")
                continue
            unpacked = struct.unpack(fmt, data)
            self.kp_cmd = np.asarray(unpacked[0:self.dof_num], dtype=np.float32).reshape(self.dof_num, 1)
            self.pos_cmd = np.asarray(unpacked[self.dof_num:self.dof_num * 2], dtype=np.float32).reshape(self.dof_num,
                                                                                                         1)
            self.kd_cmd = np.asarray(unpacked[self.dof_num * 2:self.dof_num * 3], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.vel_cmd = np.asarray(unpacked[self.dof_num * 3:self.dof_num * 4], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.tau_ff = np.asarray(unpacked[self.dof_num * 4:], dtype=np.float32).reshape(self.dof_num, 1)
            self.received_cmd = True

    def _apply_joint_torque(self):
        # Current joint states
        q = self.data.qpos[7:7+self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6+self.dof_num].reshape(-1, 1)

        if not self.received_cmd:
            # 收到第一帧控制包之前：用内部 PD 保持**初始姿态**。
            # 否则等待期间腿是软的（τ≡0）→ 机器人先塌下去，控制器一接管又把关节猛拉回目标
            # （实测把机器人弹到 0.86 m），而且等待时长还受墙钟影响 → 初始条件不可重复。
            self.input_tq = self.hold_kp * (self.qpos_hold - q) - self.hold_kd * dq
            self.data.ctrl[:] = self.input_tq.flatten()
            return

        # τ = kp*(q_d - q) + kd*(dq_d - dq) + τ_ff
        self.input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq) +
            self.tau_ff
        )
        # Write to control buffer
        self.data.ctrl[:] = self.input_tq.flatten()

    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _send_robot_state(self, step: int):
        # IMU
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        angvel_b = self.data.qvel[3:6]
        body_acc = self.data.sensordata[16:19]

        # Joints
        q = self.data.qpos[7:7+self.dof_num]
        dq = self.data.qvel[6:6+self.dof_num]
        tau = self.input_tq.flatten()

        # Pack and send
        payload = np.concatenate((
            np.array([self.timestamp], dtype=np.float64),
            np.asarray(rpy, dtype=np.float32),
            np.asarray(body_acc, dtype=np.float32),
            np.asarray(angvel_b, dtype=np.float32),
            q.astype(np.float32),
            dq.astype(np.float32),
            tau.astype(np.float32)
        ))
        fmt = "1d" + f"{len(payload)-1}f"
        try:
            self.send_sock.sendto(struct.pack(fmt, *payload), 
                                  self.ctrl_addr)
        except socket.error as ex:
            print(f"[UDP send] {ex}")


if __name__ == "__main__":
    sim = MuJoCoSimulation()
    sim.start()
