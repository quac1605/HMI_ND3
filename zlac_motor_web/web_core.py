"""ROS-independent kinematics and single-operator dead-man controller."""
import math
import secrets
import time
from dataclasses import asdict, dataclass

JOINTS = ('left_wheel_joint', 'right_wheel_joint')


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Giá trị phải là số hữu hạn.')
    return float(value)


def lround(value):
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


@dataclass(frozen=True)
class Config:
    wheel_base: float = 0.40
    wheel_radius: float = 0.0825
    rpm_max: float = 300.0
    max_linear_speed: float = 3.0
    dir_left: float = 1.0
    dir_right: float = -1.0
    command_timeout: float = 0.5
    io_timeout_ms: int = 1000

    def __post_init__(self):
        for name, value in asdict(self).items():
            finite(value)
            if name.startswith('dir_'):
                if abs(value) != 1:
                    raise ValueError('Invalid wheel direction')
            elif value <= 0:
                raise ValueError('Invalid positive configuration')
        if self.rpm_max > 32767 or int(self.rpm_max) != self.rpm_max:
            raise ValueError('rpm_max must be an integer in 1..32767')
        if not math.isfinite(self.max_angular_speed) or self.max_angular_speed <= 0:
            raise ValueError('Invalid derived angular limit')
        if not math.isfinite(self.max_linear_speed / self.wheel_radius * 60 / (2 * math.pi)):
            raise ValueError('Invalid derived RPM')
        if not 0.1 <= self.command_timeout <= 5 or not 10 <= self.io_timeout_ms <= 10000:
            raise ValueError('Invalid watchdog/serial timeout')

    @property
    def max_angular_speed(self):
        return 2 * self.rpm_max * 2 * math.pi / 60 * self.wheel_radius / self.wheel_base

    def public(self):
        return dict(asdict(self), max_angular_speed=self.max_angular_speed)

    def validate(self, v, w):
        v, w = finite(v), finite(w)
        if abs(v) > self.max_linear_speed or abs(w) > self.max_angular_speed:
            raise ValueError('Lệnh vượt giới hạn cấu hình driver.')
        return v, w

    def expected(self, v, w):
        result = []
        for speed in (v - w * self.wheel_base / 2, v + w * self.wheel_base / 2):
            rpm = speed / self.wheel_radius * 60 / (2 * math.pi)
            sent = lround(max(-self.rpm_max, min(self.rpm_max, rpm)))
            result.append(dict(raw_rpm=rpm, sent_rpm=sent,
                               mps=sent * 2 * math.pi / 60 * self.wheel_radius,
                               limited=abs(rpm) > self.rpm_max))
        return result


def encoder_velocity(names, velocities, radius):
    # JointState array order is not part of the contract. Match each joint by name.
    if any(names.count(name) != 1 for name in JOINTS):
        raise ValueError('Thiếu hoặc trùng tên joint encoder.')
    return [finite(velocities[names.index(name)]) * radius for name in JOINTS]


class Controller:
    """Caller holds a lock. Only newly received, fresh commands cause publication.

    Stop/expiry revokes the lease. Sequence fences reject requests reordered after
    stop. Single-use, server-timed tickets reject delayed/buffered motion packets.
    A new deliberate arm is required after every stop; no automatic restart.
    """
    def __init__(self, publish, config=None, timeout=0.25, clock=time.monotonic):
        if not 0.1 <= timeout <= 1:
            raise ValueError('web timeout must be in 0.1..1 s')
        self.publish = publish
        self.config = config
        self.timeout = timeout
        self.clock = clock
        self.sessions = {}
        self.owner = None
        self.lease = None
        self.last_command = 0.0
        self.v = self.w = 0.0
        self.sent_at = None
        self.reason = 'Chưa phát lệnh'

    def session(self):
        now = self.clock()
        self.sessions = {k: s for k, s in self.sessions.items()
                         if k == self.owner or now - s['seen'] < 3600}
        if len(self.sessions) >= 32:
            raise ValueError('Quá nhiều phiên web. Khởi động lại cầu nối khi an toàn.')
        token = secrets.token_urlsafe(24)
        self.sessions[token] = dict(seq=-1, seen=now, tickets={})
        return token

    def ticket(self, token):
        s = self.sessions[token]
        now = self.clock()
        s['seen'] = now
        s['tickets'] = {k: t for k, t in s['tickets'].items() if now - t < self.timeout}
        ticket = secrets.token_urlsafe(16)
        s['tickets'][ticket] = now
        return ticket

    def authenticate(self, token):
        if token not in self.sessions:
            raise ValueError('Phiên điều khiển không hợp lệ. Tải lại trang.')

    def sequence(self, token, seq):
        self.authenticate(token)
        s = self.sessions[token]
        if type(seq) is not int or seq <= s['seq']:
            raise ValueError('Lệnh cũ đã bị bỏ qua.')
        s['seq'] = seq

    def fresh(self, token, ticket):
        issued = self.sessions[token]['tickets'].pop(ticket, None)
        if issued is None or self.clock() - issued >= self.timeout:
            raise ValueError('Lệnh đến trễ. Nhả và nhấn lại joystick.')

    def arm(self, token, seq, ticket, ready):
        self.sequence(token, seq)
        self.fresh(token, ticket)
        if not ready or self.config is None:
            raise ValueError('Chưa có cấu hình, driver và encoder mới hợp lệ.')
        if self.owner is not None:
            raise ValueError('Một phiên đang điều khiển. Nhấn Dừng để thu hồi quyền.')
        self.owner = token
        self.lease = secrets.token_urlsafe(24)
        self.last_command = self.clock()
        return self.lease

    def command(self, token, seq, ticket, lease, v, w, ready):
        self.sequence(token, seq)
        if self.owner != token or lease != self.lease:
            raise ValueError('Quyền điều khiển đã hết hạn. Nhấn lại joystick.')
        try:
            self.fresh(token, ticket)
            if self.clock() - self.last_command >= self.timeout:
                raise ValueError('Heartbeat đã hết hạn. Nhấn lại joystick.')
            if not ready:
                raise ValueError('Driver hoặc encoder không còn sẵn sàng.')
            v, w = self.config.validate(v, w)
        except (ValueError, TypeError):
            self.stop('Lệnh lỗi hoặc dữ liệu quá hạn')
            raise
        self.publish(v, w)
        self.v, self.w = v, w
        self.sent_at = time.time()
        self.last_command = self.clock()
        self.reason = 'Đã phát /cmd_vel'  # Never implies motor execution.

    def stop(self, reason):
        self.owner = self.lease = None
        self.v = self.w = 0.0
        self.reason = reason
        self.publish(0.0, 0.0)
        self.sent_at = time.time()

    def user_stop(self, token, seq):
        self.sequence(token, seq)
        self.stop('Đã phát lệnh dừng')

    def tick(self, ready):
        if self.owner is not None and (not ready or self.clock() - self.last_command >= self.timeout):
            self.stop('Dừng: mất heartbeat hoặc dữ liệu quá hạn')
