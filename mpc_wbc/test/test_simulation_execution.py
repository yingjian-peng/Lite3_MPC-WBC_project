"""Verify actual Python execution and timeout behavior independently of a controller."""
import os
from pathlib import Path
import sys
import time
import unittest

import numpy as np

os.environ["MUJOCO_VIEWER"] = "0"
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "interface/robot/simulation"))
from mujoco_simulation import MuJoCoSimulation


class ExecutionTest(unittest.TestCase):
    def setUp(self):
        self.sim = MuJoCoSimulation(local_port=0)

    def tearDown(self):
        self.sim.recv_sock.close()
        self.sim.send_sock.close()

    def test_total_torque_does_not_depend_on_state_delay(self):
        command = np.zeros((5, 12), dtype=np.float32)
        command[4] = np.linspace(-20, 20, 12)
        self.sim._latest_command = (command, time.monotonic())
        self.sim.data.qpos[7:] += 0.1
        self.sim.data.qvel[6:] = np.linspace(-3, 3, 12)
        self.sim._apply_joint_torque()
        np.testing.assert_allclose(self.sim.data.ctrl, command[4], atol=1e-6)

    def test_expired_packet_cannot_keep_old_force_or_stiffness(self):
        command = np.full((5, 12), 10.0, dtype=np.float32)
        self.sim._latest_command = (command, time.monotonic() - 1)
        self.sim.data.qvel[6:] = np.linspace(-2, 2, 12)
        self.sim._apply_joint_torque()
        self.assertTrue(self.sim.command_expired)
        np.testing.assert_allclose(self.sim.data.ctrl, -2.5 * self.sim.data.qvel[6:])
        self.assertTrue((self.sim.kp_cmd == 0).all())
        self.assertTrue((self.sim.tau_ff == 0).all())


if __name__ == "__main__":
    unittest.main()
