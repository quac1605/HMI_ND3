#!/usr/bin/env python3
# check_velocity.py — cong cu test: so sanh van toc LENH (/cmd_vel) voi van toc DO (/encoder)
# cua tung banh, cung don vi m/s, ve lai bang 5 lan/giay. Khong thuoc node zlac_node.
#
# Chay o 1 cua so rieng (zlac_node dang chay, teleop o cua so khac):
#   python3 ~/ros2_ws/src/zlac_driver/scripts/check_velocity.py
#
# Thong so xe la tham so ROS, mac dinh PHAI KHOP voi ZlacConfig
# (include/zlac_driver/zlac_driver.hpp) — Python khong doc duoc header C++ nen phai chep:
#   --ros-args -p wheel_base:=0.40 -p wheel_radius:=0.0825 -p rpm_max:=300.0 -p max_linear_speed:=3.0

import math
import sys
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

RPM_TO_RADPS  = 2.0 * math.pi / 60.0
POS_WINDOW_S  = 1.0   # cua so tinh van toc tu position (s)
ENC_TIMEOUT_S = 0.5   # qua lau khong co /encoder thi bao
PCT_MIN_RPM   = 3     # |rpm gui| nho hon thi khong in %, chi in sai lech tuyet doi


def lround(x):
    # Giong std::lround cua C++: lam tron nua ra xa 0 (round() cua Python lam tron ve so chan)
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def nz(x):
    # Bo -0.0 (vd 0 * dir_right) va sai so float rat nho, de bang khong hien "-0.0000"
    return round(x, 6) + 0.0


class CheckVelocity(Node):
    def __init__(self):
        super().__init__('check_velocity')
        self.L       = self.declare_parameter('wheel_base', 0.40).value
        self.r       = self.declare_parameter('wheel_radius', 0.0825).value
        self.rpm_max = self.declare_parameter('rpm_max', 300.0).value
        self.v_max   = self.declare_parameter('max_linear_speed', 3.0).value

        self.cmd = None             # (v, w) lenh cuoi ma driver da nhan
        self.cmd_time = None
        self.cmd_rejected = False   # lenh moi nhat bi driver bo qua
        self.enc = None             # JointState cuoi
        self.enc_time = None
        self.pos_hist = deque()     # (t, pos_trai, pos_phai) trong POS_WINDOW_S gan nhat
        self.tty = sys.stdout.isatty()

        self.create_subscription(Twist, '/cmd_vel', self.onCmdVel_callback, 10)
        self.create_subscription(JointState, '/encoder', self.onEncoder_callback, 10)
        self.create_timer(0.2, self.onTimer_callback)

    def onCmdVel_callback(self, msg):
        # Giong ZlacDriver::send_velocity: |v| > max_linear_speed thi bo qua, banh giu lenh cu
        if abs(msg.linear.x) > self.v_max:
            self.cmd_rejected = True
            return
        self.cmd_rejected = False
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def onEncoder_callback(self, msg):
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            return
        self.enc = msg
        self.enc_time = self.get_clock().now()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pos_hist.append((t, msg.position[0], msg.position[1]))
        while t - self.pos_hist[0][0] > POS_WINDOW_S:
            self.pos_hist.popleft()

    def expected(self, v, w):
        # Giong ZlacDriver: inverse kinematics -> rpm -> clamp rpm_max -> lround.
        # dir = +-1 doi dau ca truoc lan sau khi gui nen khong doi ket qua; tinh theo chieu xe tien.
        out = []
        for v_wheel in (v - w * self.L / 2.0, v + w * self.L / 2.0):
            rpm = v_wheel / self.r * 60.0 / (2.0 * math.pi)
            sent = lround(max(-self.rpm_max, min(self.rpm_max, rpm)))
            out.append((rpm, sent, sent * RPM_TO_RADPS * self.r))
        return out

    def pos_slope(self):
        # Delta position / Delta t trong cua so, doi ra m/s. Lech cot velocity theo ti le
        # co dinh (x4, /4) -> counts_per_line sai.
        if len(self.pos_hist) < 2:
            return None
        t0, l0, r0 = self.pos_hist[0]
        t1, l1, r1 = self.pos_hist[-1]
        if t1 - t0 < 0.5 * POS_WINDOW_S:
            return None
        return ((l1 - l0) / (t1 - t0) * self.r, (r1 - r0) / (t1 - t0) * self.r)

    def onTimer_callback(self):
        now = self.get_clock().now()
        lines = [f'L = {self.L} m   r = {self.r} m   rpm_max = {self.rpm_max:g}'
                 '   (phai khop ZlacConfig trong zlac_driver.hpp)']

        if self.cmd is None:
            v, w = 0.0, 0.0
            lines.append('/cmd_vel: chua nhan lenh nao (banh dang dung)')
        else:
            v, w = self.cmd
            age = (now - self.cmd_time).nanoseconds * 1e-9
            lines.append(f'/cmd_vel: v = {v:+.3f} m/s   w = {w:+.3f} rad/s   (lenh cuoi {age:.1f} s truoc)')
        if self.cmd_rejected:
            lines.append(f'  !! lenh moi nhat co |v| > {self.v_max:g} m/s -> driver bo qua, banh giu lenh cu')

        enc_ok = (self.enc is not None and
                  (now - self.enc_time).nanoseconds * 1e-9 < ENC_TIMEOUT_S)
        slope = self.pos_slope() if enc_ok else None

        lines.append('')
        lines.append(f'{"":12}{"---------- LENH ----------":^27}{"---- DO (encoder) ----":^26}')
        lines.append(f'{"":12}{"rpm tinh -> gui":>17}{"m/s":>10}{"velocity":>12}{"tu position":>14}'
                     '   LECH (do - lenh)')
        for i, (name, (rpm, sent, exp_ms)) in enumerate(zip(('banh trai', 'banh phai'),
                                                            self.expected(v, w))):
            rpm, exp_ms = nz(rpm), nz(exp_ms)
            if enc_ok:
                meas = nz(self.enc.velocity[i] * self.r)
                err = nz(meas - exp_ms)
                vel_s = f'{meas:+12.4f}'
                pos_s = f'{nz(slope[i]):+14.4f}' if slope else f'{"--":>14}'
                err_s = f'{err:+.4f} m/s'
                if abs(sent) >= PCT_MIN_RPM:
                    err_s += f'  ({nz(err / exp_ms * 100):+.1f}%)'
            else:
                vel_s, pos_s, err_s = f'{"--":>12}', f'{"--":>14}', '--'
            lines.append(f'{name:<12}{rpm:+9.2f} -> {sent:+4d}{exp_ms:+10.4f}{vel_s}{pos_s}   {err_s}')

        if not enc_ok:
            lines.append('')
            lines.append('!! khong co /encoder (zlac_node chua chay, hoac zlac_encoder dang enabled=false)')

        text = '\n'.join(lines)
        if self.tty:
            sys.stdout.write('\033[H\033[J' + text + '\n')   # ve lai tai cho
        else:
            sys.stdout.write(text + '\n' + '-' * 40 + '\n')   # ghi ra file / pipe
        sys.stdout.flush()


def main():
    rclpy.init()
    node = CheckVelocity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
