#!/usr/bin/env python3
"""MuJoCo-Simulation des Cart Pole als ROS2-Node.

Publiziert  : /joint_states          (sensor_msgs/JointState, Stamp = Simulationszeit)
Abonniert   : /cartpole/effort_cmd   (std_msgs/Float64, Kraft auf den Cart in N)
              /cartpole/reset        (std_msgs/Empty, setzt die Simulation zurueck)
"""
import os

import mujoco
import mujoco.viewer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64

CART_JOINT = 'slider_to_cart'
POLE_JOINT = 'cart_to_pole'


class SimulationNode(Node):
    def __init__(self):
        super().__init__('cartpole_mujoco_simulation')

        default_model = os.path.join(
            get_package_share_directory('cartpole_mujoco'), 'model', 'cartpole_primitive.xml')
        self.declare_parameter('model_file', default_model)
        self.declare_parameter('reset_angle', 0.1)   # rad, Anfangsauslenkung des Pendels
        self.declare_parameter('viewer', True)

        model_file = self.get_parameter('model_file').value
        self.reset_angle = float(self.get_parameter('reset_angle').value)

        self.model = mujoco.MjModel.from_xml_path(model_file)
        self.data = mujoco.MjData(self.model)

        self.cart_qpos, self.cart_dof = self._joint_addresses(CART_JOINT)
        self.pole_qpos, self.pole_dof = self._joint_addresses(POLE_JOINT)

        self.effort = 0.0
        self._apply_initial_state()

        self.viewer = None
        if self.get_parameter('viewer').value:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Float64, '/cartpole/effort_cmd', self._on_effort_cmd, 10)
        self.create_subscription(Empty, '/cartpole/reset', self._on_reset, 10)

        self.create_timer(self.model.opt.timestep, self._step)
        self.get_logger().info(f'Simulation bereit, Modell: {model_file}')

    def _joint_addresses(self, name):
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"Gelenk '{name}' nicht im Modell gefunden")
        return int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])

    def _apply_initial_state(self):
        self.data.qpos[self.pole_qpos] = self.reset_angle
        mujoco.mj_forward(self.model, self.data)

    def _on_effort_cmd(self, msg: Float64):
        self.effort = msg.data

    def _on_reset(self, _msg: Empty):
        mujoco.mj_resetData(self.model, self.data)
        self.effort = 0.0
        self._apply_initial_state()
        self.get_logger().info('Simulation zurueckgesetzt.')

    def _step(self):
        if self.viewer is not None and not self.viewer.is_running():
            self.get_logger().info('Viewer geschlossen, beende Node.')
            if rclpy.ok():
                rclpy.shutdown()
            return

        self.data.ctrl[0] = self.effort
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

        sim_time = float(self.data.time)
        msg = JointState()
        msg.header.stamp.sec = int(sim_time)
        msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        msg.name = [POLE_JOINT, CART_JOINT]
        msg.position = [float(self.data.qpos[self.pole_qpos]),
                        float(self.data.qpos[self.cart_qpos])]
        msg.velocity = [float(self.data.qvel[self.pole_dof]),
                        float(self.data.qvel[self.cart_dof])]
        self.joint_state_pub.publish(msg)


def main():
    rclpy.init()
    node = SimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
