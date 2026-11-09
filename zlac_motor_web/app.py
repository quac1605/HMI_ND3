#!/usr/bin/env python3
"""Local HTTP/ROS bridge. No serial access. Demo imports no ROS and publishes no ROS topics."""
import argparse
import json
import math
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from web_core import Config, Controller, encoder_velocity


class Backend:
    def __init__(self, demo=False, web_timeout=0.25, encoder_timeout=0.5):
        self.lock = threading.RLock()
        self.demo = demo
        self.encoder_timeout = encoder_timeout
        self.encoder = None
        self.encoder_received = self.encoder_stamp = self.encoder_wall = None
        self.encoder_error = None
        self.driver = None
        self.driver_received = None
        self.driver_count = self.encoder_count = self.command_count = 0
        self.ros = None
        self.controller = Controller(self.publish, Config() if demo else None, web_timeout)
        self.demo_measurement = [0.0, 0.0]
        self.demo_dropout = False
        self.done = threading.Event()
        if not demo:
            self.setup_ros()
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()

    def setup_ros(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        rclpy.init()
        self.ros = Node('zlac_web_bridge')
        self.twist_type = Twist
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=0.2))
        self.publisher = self.ros.create_publisher(Twist, '/cmd_vel', qos)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.ros.create_subscription(JointState, '/encoder', self.on_encoder, sensor_qos)
        self.ros.create_subscription(String, '/zlac/status', self.on_status, 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.ros)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

    def publish(self, v, w):
        if self.ros:
            msg = self.twist_type()
            msg.linear.x, msg.angular.z = float(v), float(w)
            self.publisher.publish(msg)

    def on_status(self, msg):
        with self.lock:
            try:
                data = json.loads(msg.data)
                if data.get('source') != 'independent_guard':
                    raise ValueError('Expected independent guard status')
                cfg = Config(**data['config'])
                finite_pair = (float(data['v']), float(data['w']))
                if not all(math.isfinite(x) for x in finite_pair):
                    raise ValueError('Invalid status')
                if self.controller.config is not None and cfg != self.controller.config:
                    self.controller.stop('Dừng: cấu hình driver thay đổi')
                self.controller.config = cfg
                self.driver = data
                self.driver_received = time.monotonic()
            except (ValueError, TypeError, KeyError):
                self.driver = None

    def on_encoder(self, msg):
        with self.lock:
            if self.controller.config is None:
                return
            try:
                values = encoder_velocity(list(msg.name), list(msg.velocity),
                                          self.controller.config.wheel_radius)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                # Source stamp must also be recent; repeated old messages cannot refresh readiness.
                ros_now = self.ros.get_clock().now().nanoseconds * 1e-9
                if stamp <= 0 or not -0.1 <= ros_now - stamp < self.encoder_timeout:
                    raise ValueError('Timestamp encoder cũ hoặc nằm trong tương lai.')
                if self.encoder_stamp is not None and stamp <= self.encoder_stamp:
                    raise ValueError('Timestamp encoder không tăng.')
                self.encoder, self.encoder_stamp = values, stamp
                self.encoder_received, self.encoder_wall = time.monotonic(), time.time()
                self.encoder_error = None
            except (ValueError, IndexError, TypeError) as exc:
                self.encoder_error = str(exc)

    def ready(self):
        now = time.monotonic()
        return (self.controller.config is not None and self.driver is not None
                and self.driver.get('ready') is True and self.driver_received is not None
                and now - self.driver_received < 0.5 and self.encoder_received is not None
                and now - self.encoder_received < self.encoder_timeout and not self.encoder_error
                and (self.demo or (self.driver_count == 1 and self.encoder_count == 1
                                   and self.command_count == 1)))

    def loop(self):
        while not self.done.wait(0.02):
            with self.lock:
                now = time.monotonic()
                if self.demo:
                    self.driver = dict(ready=True, ack=False, reason='simulated',
                                       v=self.controller.v, w=self.controller.w)
                    self.driver_received = now
                    if not self.demo_dropout:
                        targets = self.controller.config.expected(self.controller.v, self.controller.w)
                        self.demo_measurement = [old + (wheel['mps'] - old) * 0.12
                                                 for old, wheel in zip(self.demo_measurement, targets)]
                        self.encoder = list(self.demo_measurement)
                        self.encoder_received = now
                        self.encoder_stamp = self.encoder_wall = time.time()
                else:
                    self.driver_count = self.ros.count_publishers('/zlac/status')
                    self.encoder_count = self.ros.count_publishers('/encoder')
                    self.command_count = self.ros.count_publishers('/cmd_vel')
                self.controller.tick(self.ready())

    def snapshot(self, token=None):
        c = self.controller
        age = None if self.encoder_received is None else time.monotonic() - self.encoder_received
        enc_state = ('invalid' if self.encoder_error else 'missing' if age is None
                     else 'stale' if age >= self.encoder_timeout else 'fresh')
        wheels = c.config.expected(c.v, c.w) if c.config else []
        for i, wheel in enumerate(wheels):
            wheel['measured'] = self.encoder[i] if enc_state == 'fresh' else None
            wheel['error'] = (wheel['measured'] - wheel['mps']) if wheel['measured'] is not None else None
        driver_age = None if self.driver_received is None else time.monotonic() - self.driver_received
        return dict(demo=self.demo, ready=self.ready(), config=c.config.public() if c.config else None,
                    command=dict(v=c.v, w=c.w, sent_at=c.sent_at, reason=c.reason), wheels=wheels,
                    encoder=dict(state=enc_state, age=age, stamp=self.encoder_stamp,
                                 received_at=self.encoder_wall, error=self.encoder_error),
                    driver=self.driver, driver_age=driver_age,
                    publishers=dict(driver=self.driver_count, encoder=self.encoder_count, cmd_vel=self.command_count),
                    controlling=c.owner == token if token else False, occupied=c.owner is not None,
                    ticket=c.ticket(token) if token else None, web_timeout=c.timeout,
                    encoder_timeout=self.encoder_timeout, server_time=time.time())

    def close(self):
        self.done.set()
        self.worker.join(timeout=1)
        with self.lock:
            self.controller.stop('Cầu nối đang tắt')
        if self.ros:
            self.executor.shutdown(timeout_sec=1)
            self.ros_thread.join(timeout=1)
            self.ros.destroy_node()
            import rclpy
            rclpy.try_shutdown()


def make_handler(backend, static_dir, allowed_hosts):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Never log session credentials or request bodies.

        def send(self, code, data, content_type='application/json; charset=utf-8'):
            body = json.dumps(data, allow_nan=False).encode() if isinstance(data, dict) else data
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Lease watchdog handles transport loss even if stop cannot be delivered.

        def valid_host(self):
            return self.headers.get('Host') in allowed_hosts

        def do_GET(self):
            if not self.valid_host():
                return self.send(403, dict(error='Host không được cho phép.'))
            routes = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                      '/style.css': ('style.css', 'text/css; charset=utf-8')}
            if self.path not in routes:
                return self.send(404, dict(error='Not found'))
            filename, mime = routes[self.path]
            self.send(200, (static_dir / filename).read_bytes(), mime)

        def do_POST(self):
            if not self.valid_host() or self.headers.get('Origin') != 'http://' + self.headers.get('Host', ''):
                return self.send(403, dict(error='Yêu cầu phải cùng origin với trang điều khiển.'))
            token = None
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                self.connection.settimeout(1)
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError('Invalid request')
                with backend.lock:
                    c = backend.controller
                    if self.path == '/api/session':
                        token = c.session()
                        result = dict(session=token, **backend.snapshot(token))
                    else:
                        token = data.get('session')
                        c.authenticate(token)
                        if self.path == '/api/arm':
                            lease = c.arm(token, data['seq'], data['ticket'], backend.ready())
                            result = dict(lease=lease, **backend.snapshot(token))
                        elif self.path == '/api/command':
                            c.command(token, data['seq'], data['ticket'], data['lease'],
                                      data['v'], data['w'], backend.ready())
                            result = backend.snapshot(token)
                        elif self.path == '/api/stop':
                            c.user_stop(token, data['seq'])
                            result = backend.snapshot(token)
                        elif self.path == '/api/state':
                            result = backend.snapshot(token)
                        elif self.path == '/api/demo-dropout' and backend.demo:
                            backend.demo_dropout = not backend.demo_dropout
                            result = backend.snapshot(token)
                        else:
                            return self.send(404, dict(error='Not found'))
                    self.send(200, result)
            except (ValueError, TypeError, KeyError, TimeoutError) as exc:
                with backend.lock:
                    # Malformed commands from the owner must fail closed.
                    if token is not None and token == backend.controller.owner and self.path != '/api/state':
                        backend.controller.stop('Dừng: yêu cầu không hợp lệ')
                self.send(400, dict(error=str(exc)))
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='SIMULATED DATA; no ROS or serial access')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--allowed-host', action='append', default=[], help='Additional exact HTTP Host (host:port)')
    parser.add_argument('--web-timeout', type=float, default=0.25)
    parser.add_argument('--encoder-timeout', type=float, default=0.5)
    args = parser.parse_args()
    if not 0.1 <= args.encoder_timeout <= 2:
        parser.error('--encoder-timeout must be in 0.1..2 s')
    static_dir = Path(__file__).resolve().parent / 'static'
    hosts = {f'127.0.0.1:{args.port}', f'localhost:{args.port}', *args.allowed_host}
    backend = Backend(args.demo, args.web_timeout, args.encoder_timeout)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(backend, static_dir, hosts))
    server.daemon_threads = True
    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f'http://127.0.0.1:{args.port} — ' + ('DEMO / DỮ LIỆU GIẢ LẬP' if args.demo else 'ROS /cmd_vel → /encoder'), flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        server.server_close()
        backend.close()


if __name__ == '__main__':
    main()
