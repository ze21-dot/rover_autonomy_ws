#!/usr/bin/env python3
"""Software differential bar: anti-phase constraint via per-joint force topics.

effort mode: virtual spring-damper on common mode e = qL + qR,
tau = -(kp*e + kd*de), same tau to both joints via gz ApplyJointForce.
v1 pending Baris diff-bar failure notes.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class DiffConstraintNode(Node):

    def __init__(self):
        super().__init__('diff_constraint')
        self.declare_parameter('left_joint', 'leftjoint')
        self.declare_parameter('right_joint', 'rightjoint')
        self.declare_parameter('kp', 40.0)
        self.declare_parameter('kd', 2.0)
        self.declare_parameter('max_effort', 30.0)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('left_cmd_topic',
                               '/model/karasimsek/joint/leftjoint/cmd_force')
        self.declare_parameter('right_cmd_topic',
                               '/model/karasimsek/joint/rightjoint/cmd_force')
        self.declare_parameter('watchdog_timeout_s', 0.25)
        self.declare_parameter('invert_right', False)  # flip if roll gets WORSE

        self.left = self.get_parameter('left_joint').value
        self.right = self.get_parameter('right_joint').value
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_eff = float(self.get_parameter('max_effort').value)
        self.timeout = float(self.get_parameter('watchdog_timeout_s').value)
        self.sign_r = -1.0 if self.get_parameter('invert_right').value else 1.0

        self.q, self.v = {}, {}
        self.last_js_t = None
        self.warned = False

        self.pub_l = self.create_publisher(
            Float64, self.get_parameter('left_cmd_topic').value, 10)
        self.pub_r = self.create_publisher(
            Float64, self.get_parameter('right_cmd_topic').value, 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_js, 50)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('rate_hz').value), self.on_timer)
        self.get_logger().info(
            f"diff_constraint v1 | kp={self.kp} kd={self.kd} "
            f"max={self.max_eff} invert_right={self.sign_r < 0}")

    def on_js(self, msg):
        for i, n in enumerate(msg.name):
            if n in (self.left, self.right):
                if i < len(msg.position):
                    self.q[n] = msg.position[i]
                if i < len(msg.velocity):
                    self.v[n] = msg.velocity[i]
        if self.left in self.q and self.right in self.q:
            self.last_js_t = self.get_clock().now()

    def on_timer(self):
        if self.last_js_t is None:
            if not self.warned:
                self.get_logger().warn(
                    f"waiting for {self.left}/{self.right} in /joint_states")
                self.warned = True
            return
        if (self.get_clock().now() - self.last_js_t).nanoseconds * 1e-9 > self.timeout:
            return
        qL, qR = self.q[self.left], self.sign_r * self.q[self.right]
        vL = self.v.get(self.left, 0.0)
        vR = self.sign_r * self.v.get(self.right, 0.0)
        e, de = qL + qR, vL + vR
        tau = max(-self.max_eff, min(self.max_eff, -(self.kp * e + self.kd * de)))
        mL, mR = Float64(), Float64()
        mL.data = tau
        mR.data = self.sign_r * tau
        self.pub_l.publish(mL)
        self.pub_r.publish(mR)


def main(args=None):
    rclpy.init(args=args)
    n = DiffConstraintNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
