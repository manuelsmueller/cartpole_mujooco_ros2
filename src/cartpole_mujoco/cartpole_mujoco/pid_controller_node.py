#!/usr/bin/env python3
"""MISO-PID-Regler fuer das Cart Pole als eigener ROS2-Node.

Abonniert : /joint_states         (sensor_msgs/JointState)
            /cartpole/pid_enable  (std_msgs/Bool, Regler an/aus)
            /cartpole/reset       (std_msgs/Empty, setzt Integralanteile zurueck)
Publiziert: /cartpole/effort_cmd  (std_msgs/Float64, immer ein echter Zahlenwert)

Der Node heisst 'cartpole_pid_controller'; die Gains lassen sich zur Laufzeit
per 'ros2 param set /cartpole_pid_controller kp 80.0' aendern.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64

CART_JOINT = 'slider_to_cart'
POLE_JOINT = 'cart_to_pole'
MAX_DT = 0.1  # s, Schutz gegen Ausreisser nach Pausen/Reset


class PidControllerNode(Node):
    def __init__(self):
        super().__init__('cartpole_pid_controller')

        # Defaults = Primitiv-Modell; ueberschrieben durch config/pid_<modell>.yaml
        self.declare_parameter('kp', 77.3)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 20.6)
        self.declare_parameter('i_clamp', 10.0)
        self.declare_parameter('kp_cart', -6.3)
        self.declare_parameter('ki_cart', -2.5)
        self.declare_parameter('kd_cart', -10.3)
        self.declare_parameter('i_clamp_cart', 3.0)
        self.declare_parameter('effort_limit', 60.0)

        self.enabled = False
        self.integral = 0.0
        self.integral_cart = 0.0
        self.last_stamp = None

        self.effort_pub = self.create_publisher(Float64, '/cartpole/effort_cmd', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(Empty, '/cartpole/reset', self._on_reset, 10)
        self.create_subscription(Bool, '/cartpole/pid_enable', self._on_enable, 10)

        self.get_logger().info('cartpole_pid_controller bereit (Regler inaktiv bis pid_enable=true).')

    def _p(self, name):
        return float(self.get_parameter(name).value)

    def _on_enable(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            self.effort_pub.publish(Float64(data=0.0))
        self.get_logger().info(f'PID enabled={self.enabled}')

    def _on_reset(self, _msg: Empty):
        self.integral = 0.0
        self.integral_cart = 0.0
        self.last_stamp = None
        self.effort_pub.publish(Float64(data=0.0))

    def _on_joint_state(self, msg: JointState):
        if POLE_JOINT not in msg.name or CART_JOINT not in msg.name:
            return

        # dt aus der Simulationszeit im Stamp, nicht aus der Wanduhr
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None or stamp <= self.last_stamp:
            dt = 0.0
        else:
            dt = min(stamp - self.last_stamp, MAX_DT)
        self.last_stamp = stamp

        if not self.enabled:
            self.effort_pub.publish(Float64(data=0.0))
            return

        ip = msg.name.index(POLE_JOINT)
        ic = msg.name.index(CART_JOINT)
        angle, angle_vel = msg.position[ip], msg.velocity[ip]
        cart_pos, cart_vel = msg.position[ic], msg.velocity[ic]

        # Winkelregelung (Sollwinkel 0)
        angle_error = 0.0 - angle
        self.integral += angle_error * dt
        lim = self._p('i_clamp')
        self.integral = max(-lim, min(lim, self.integral))
        angle_term = (self._p('kp') * angle_error
                      + self._p('ki') * self.integral
                      + self._p('kd') * (-angle_vel))

        # Cart-Regelung (Sollposition 0)
        cart_error = 0.0 - cart_pos
        self.integral_cart += cart_error * dt
        limc = self._p('i_clamp_cart')
        self.integral_cart = max(-limc, min(limc, self.integral_cart))
        cart_term = (self._p('kp_cart') * cart_error
                     + self._p('ki_cart') * self.integral_cart
                     + self._p('kd_cart') * (-cart_vel))

        effort = -angle_term + cart_term
        limit = self._p('effort_limit')
        effort = max(-limit, min(limit, effort))
        self.effort_pub.publish(Float64(data=float(effort)))


def main():
    rclpy.init()
    node = PidControllerNode()
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
