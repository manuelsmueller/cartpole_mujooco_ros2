#!/usr/bin/env python3
"""Tkinter-Steuerpanel fuer das MuJoCo Cart Pole.

Start  -> /cartpole/pid_enable = true
Reset  -> Regler aus, /cartpole/reset, Regler an
Gains  -> werden beim Start vom Regler-Node geladen und per set_parameters geschrieben
"""
import math
import tkinter as tk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

CONTROLLER = 'cartpole_pid_controller'
POLE_JOINT = 'cart_to_pole'
CART_JOINT = 'slider_to_cart'

ANGLE_GAINS = ['kp', 'ki', 'kd']
CART_GAINS = ['kp_cart', 'ki_cart', 'kd_cart']
ALL_GAINS = ANGLE_GAINS + CART_GAINS


def double_param(name: str, value: float) -> Parameter:
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    return p


class ControlPanel(Node):
    def __init__(self):
        super().__init__('cartpole_mujoco_panel')

        self.set_client = self.create_client(SetParameters, f'/{CONTROLLER}/set_parameters')
        self.get_client = self.create_client(GetParameters, f'/{CONTROLLER}/get_parameters')
        self.reset_pub = self.create_publisher(Empty, '/cartpole/reset', 10)
        self.enable_pub = self.create_publisher(Bool, '/cartpole/pid_enable', 10)

        self.latest = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self._build_gui()
        self.root.after(1500, self.on_load_gains)  # Reglerwerte nach dem Start uebernehmen

    def _on_joint_state(self, msg: JointState):
        self.latest = msg

    # ---------------------------------------------------------------- GUI
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title('Cart Pole (MuJoCo) Control Panel')
        self.root.geometry('380x560')

        tk.Button(self.root, text='Start', width=15, command=self.on_start).pack(pady=(15, 5))
        tk.Button(self.root, text='Reset', width=15, command=self.on_reset).pack(pady=5)

        self.vars = {name: tk.StringVar(value='') for name in ALL_GAINS}
        self._gain_block('Winkel-Regelung (Balance)', ANGLE_GAINS)
        self._gain_block('Cart-Regelung (Rücklauf zur Mitte)', CART_GAINS)

        row = tk.Frame(self.root)
        row.pack(pady=12)
        tk.Button(row, text='Parameter anwenden', command=self.on_apply_gains).pack(side=tk.LEFT, padx=5)
        tk.Button(row, text='Werte vom Regler laden', command=self.on_load_gains).pack(side=tk.LEFT, padx=5)

        self.status_var = tk.StringVar(value='Winkel: -- °   |   Cart: -- m')
        tk.Label(self.root, textvariable=self.status_var, font=('Arial', 12)).pack(pady=10)

        self.log_var = tk.StringVar(value='')
        tk.Label(self.root, textvariable=self.log_var, fg='grey', wraplength=340).pack(pady=5)

    def _gain_block(self, title, names):
        tk.Label(self.root, text=title, font=('Arial', 10, 'bold')).pack(pady=(15, 2))
        frame = tk.Frame(self.root)
        frame.pack()
        for row, name in enumerate(names):
            tk.Label(frame, text=name, width=8, anchor='e').grid(row=row, column=0)
            tk.Entry(frame, textvariable=self.vars[name], width=10).grid(row=row, column=1, padx=5, pady=2)

    # ------------------------------------------------------------ Aktionen
    def on_start(self):
        self.enable_pub.publish(Bool(data=True))
        self.log_var.set('PID aktiviert')

    def on_reset(self):
        self.enable_pub.publish(Bool(data=False))
        self.reset_pub.publish(Empty())
        self.enable_pub.publish(Bool(data=True))
        self.log_var.set('Reset gesendet, Regler reaktiviert.')

    def on_load_gains(self):
        if not self.get_client.wait_for_service(timeout_sec=1.0):
            self.log_var.set('Regler-Node nicht erreichbar (get_parameters).')
            return
        req = GetParameters.Request()
        req.names = ALL_GAINS
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.log_var.set('Laden der Werte fehlgeschlagen.')
            return
        for name, value in zip(ALL_GAINS, future.result().values):
            self.vars[name].set(f'{value.double_value:g}')
        self.log_var.set('Werte vom Regler geladen.')

    def on_apply_gains(self):
        if not self.set_client.wait_for_service(timeout_sec=2.0):
            self.log_var.set('set_parameters nicht erreichbar')
            return
        try:
            values = {name: float(self.vars[name].get()) for name in ALL_GAINS}
        except ValueError:
            self.log_var.set('Ungültige Zahl in einem der Felder')
            return
        req = SetParameters.Request()
        req.parameters = [double_param(n, v) for n, v in values.items()]
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.log_var.set('Gesetzt: ' + ' '.join(f'{n}={v:g}' for n, v in values.items()))

    # ------------------------------------------------------------ Hauptschleife
    def _update_status(self):
        msg = self.latest
        if msg is None:
            return
        angle = cart = '--'
        if POLE_JOINT in msg.name:
            angle = f'{math.degrees(msg.position[msg.name.index(POLE_JOINT)]):6.2f}'
        if CART_JOINT in msg.name:
            cart = f'{msg.position[msg.name.index(CART_JOINT)]:6.3f}'
        self.status_var.set(f'Winkel: {angle} °   |   Cart: {cart} m')

    def spin_gui(self):
        while rclpy.ok():
            self._update_status()
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    node = ControlPanel()
    try:
        node.spin_gui()
    except (KeyboardInterrupt, tk.TclError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
