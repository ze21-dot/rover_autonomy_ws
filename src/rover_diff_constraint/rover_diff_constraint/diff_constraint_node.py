#!/usr/bin/env python3
"""Software differential bar: anti-phase constraint for rocker diff joints.

Physical analogy: a real differential bar mechanically enforces
    q_left = -q_right
We emulate it in software. Two modes:

  effort   -> virtual spring-damper on the common mode e = qL + qR.
              tau = -(kp*e + kd*de).  Same tau applied to BOTH joints
              (gradient of e w.r.t. each joint is +1). Physically honest,
              lets terrain still articulate the rocker (differential mode
              qL - qR stays free). PREFERRED.

  position -> commands [d, -d] where d = lowpass((qL - qR)/2).
              Use only if your controller exposes position interface.
              Stiffer, can fight the physics engine on impacts.

Constraint v1 (2026-08-18, Beijing rover night). Pending revision once
Baris's diff-bar failure notes arrive -- see README "Known unknowns".
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class DiffConstraintNode(Node):

    def __init__(self):
        super().__init__('diff_constraint')

        self.declare_parameter('left_joint', 'CHANGE_ME_left_diff_joint')
        self.declare_parameter('right_joint', 'CHANGE_ME_right_diff_joint')
        self.declare_parameter('mode', 'effort')          # 'effort' | 'position'
        self.declare_parameter('kp', 40.0)                # N*m/rad (effort) — start soft!
        self.declare_parameter('kd', 2.0)                 # N*m*s/rad
        self.declare_parameter('max_effort', 30.0)        # AK10-9 KV100 ~ stay well under stall
        self.declare_parameter('lp_alpha', 0.15)          # position-mode lowpass, 0..1
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('cmd_topic', '/diff_effort_controller/commands')
        self.declare_parameter('watchdog_timeout_s', 0.25)

        self.left = self.get_parameter('left_joint').value
        self.right = self.get_parameter('right_joint').value
        self.mode = self.get_parameter('mode').value
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_eff = float(self.get_parameter('max_effort').value)
        self.alpha = float(self.get_parameter('lp_alpha').value)
        self.timeout = float(self.get_parameter('watchdog_timeout_s').value)

        if self.mode not in ('effort', 'position'):
            raise ValueError(f"mode must be 'effort' or 'position', got '{self.mode}'")

        self.q = {}       # name -> position
        self.v = {}       # name -> velocity
        self.last_js_t = None
        self.d_filt = 0.0  # filtered differential half-angle (position mode)
        self.warned_missing = False

        self.pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('cmd_topic').value, 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 50)

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"diff_constraint up | mode={self.mode} "
            f"joints=({self.left},{self.right}) kp={self.kp} kd={self.kd} "
            f"max_effort={self.max_eff}")

    def on_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in (self.left, self.right):
                if i < len(msg.position):
                    self.q[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.v[name] = msg.velocity[i]
        if self.left in self.q and self.right in self.q:
            self.last_js_t = self.get_clock().now()

    def on_timer(self):
        # Watchdog: no fresh state -> publish nothing (never drive blind).
        if self.last_js_t is None:
            if not self.warned_missing:
                self.get_logger().warn(
                    f"Waiting for joints '{self.left}'/'{self.right}' in "
                    f"/joint_states. Check names in config/diff_constraint.yaml")
                self.warned_missing = True
            return
        age = (self.get_clock().now() - self.last_js_t).nanoseconds * 1e-9
        if age > self.timeout:
            return

        qL, qR = self.q[self.left], self.q[self.right]
        vL = self.v.get(self.left, 0.0)
        vR = self.v.get(self.right, 0.0)

        msg = Float64MultiArray()
        if self.mode == 'effort':
            e = qL + qR            # common mode (should be 0)
            de = vL + vR
            tau = -(self.kp * e + self.kd * de)
            tau = max(-self.max_eff, min(self.max_eff, tau))
            msg.data = [tau, tau]  # same sign on both: pushes common mode to 0
        else:  # position
            d = 0.5 * (qL - qR)    # differential half-angle (terrain-driven)
            self.d_filt += self.alpha * (d - self.d_filt)
            msg.data = [self.d_filt, -self.d_filt]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffConstraintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
