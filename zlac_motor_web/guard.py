#!/usr/bin/env python3
"""Independent ROS safety relay. Original driver/encoder sources stay untouched."""
import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

from web_core import Config, encoder_velocity
from tools.build_driver import verify_manifest


def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=root/'.runtime/driver_manifest.json')
    parser.add_argument('--command-timeout',type=float,default=.5)
    parser.add_argument('--encoder-timeout',type=float,default=.5)
    args=parser.parse_args()
    if not .1 <= args.encoder_timeout <= 2: parser.error('encoder timeout must be 0.1..2 s')
    manifest=verify_manifest(args.manifest)
    cfg=Config(**manifest['config'],command_timeout=args.command_timeout)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class Guard(Node):
        def __init__(self):
            super().__init__('zlac_web_guard')
            self.output='/zlac_guard_'+secrets.token_hex(8)+'/cmd_vel'
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE,lifespan=Duration(seconds=.15))
            self.publisher=self.create_publisher(Twist,self.output,qos)
            self.status_pub=self.create_publisher(String,'/zlac/status',1)
            self.create_subscription(Twist,'/cmd_vel',self.command,1)
            self.create_subscription(JointState,'/encoder',self.encoder,
                                     QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.last_command=None; self.encoder_time=None; self.encoder_stamp=None
            self.v=self.w=0.;self.reason='waiting_for_encoder';self.active=False
            self.child=subprocess.Popen([sys.executable,str(root/'tools/driver_child.py'),str(os.getpid()),
                manifest['executable'],'--ros-args','-r','/cmd_vel:='+self.output])
            self.create_timer(.02,self.tick,clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
            self.create_timer(.1,self.status,clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
            self.get_logger().info('Original driver launched with private command topic; software stop only.')

        def ready(self):
            return (self.child.poll() is None and self.encoder_time is not None
                    and time.monotonic()-self.encoder_time < args.encoder_timeout
                    and self.publisher.get_subscription_count()==1
                    and self.count_publishers('/encoder')==1
                    and self.count_publishers('/cmd_vel')==1
                    and self.count_publishers(self.output)==1)

        def encoder(self,msg):
            try:
                encoder_velocity(list(msg.name),list(msg.velocity),cfg.wheel_radius)
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                age=self.get_clock().now().nanoseconds*1e-9-stamp
                if stamp<=0 or not -.1<=age<args.encoder_timeout or (self.encoder_stamp is not None and stamp<=self.encoder_stamp):
                    raise ValueError('stale encoder timestamp')
                self.encoder_stamp=stamp;self.encoder_time=time.monotonic()
            except (ValueError,IndexError,TypeError):
                self.encoder_time=None
                self.stop('invalid_encoder')

        def forward(self,v,w):
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w)
            self.publisher.publish(msg)
            self.v,self.w=v,w

        def stop(self,reason):
            self.active=False;self.reason=reason;self.forward(0.,0.)

        def command(self,msg):
            try:
                v,w=cfg.validate(msg.linear.x,msg.angular.z)
                if any(x!=0 for x in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                    raise ValueError('unsupported axis')
                if (v or w) and not self.ready(): raise ValueError('not ready')
            except ValueError:
                self.stop('invalid_command_or_not_ready');return
            self.last_command=time.monotonic();self.active=bool(v or w)
            self.reason='forwarded_to_driver';self.forward(v,w)

        def tick(self):
            if self.active and (self.last_command is None or time.monotonic()-self.last_command>=cfg.command_timeout):
                self.stop('watchdog_timeout')
            elif self.active and not self.ready(): self.stop('encoder_or_driver_lost')
            elif not self.active:
                # Retry zero continuously: original serial callbacks can block and have no ACK topic.
                self.forward(0.,0.)

        def status(self):
            data=dict(ready=self.ready(),ack=None,reason=self.reason,v=self.v,w=self.w,
                      config={k:v for k,v in cfg.public().items() if k!='max_angular_speed'},
                      source='independent_guard',output_topic=self.output,
                      driver_exited=self.child.poll() is not None,
                      clamped=any(x['limited'] for x in cfg.expected(self.v,self.w)))
            msg=String();msg.data=json.dumps(data,allow_nan=False);self.status_pub.publish(msg)

        def close(self):
            self.stop('guard_shutdown')
            if self.child.poll() is None:
                self.child.send_signal(signal.SIGINT)
                try:self.child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    self.get_logger().error('Driver shutdown delayed; verify wheels and hardware E-stop.')

    rclpy.init()
    node=None
    try:
        node=Guard();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node:
            # ROS may already be shut down, but SIGINT still asks the driver to stop/disable.
            try:node.close()
            except Exception:
                if node.child.poll() is None:node.child.send_signal(signal.SIGINT)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
