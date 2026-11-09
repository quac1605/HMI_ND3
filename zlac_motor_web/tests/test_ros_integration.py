"""Tests the unchanged driver via PTY + independent guard in an isolated ROS domain."""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

os.environ['ROS_DOMAIN_ID']='176'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
from fake_modbus import FakeModbus
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[1]


class Integration(unittest.TestCase):
    def wait_for(self,predicate,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.02)
            if predicate():return
        self.fail('Condition did not become true before timeout')

    def send(self,v,w=0):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.pub.publish(msg)

    def api(self,path,data=None):
        request=urllib.request.Request('http://127.0.0.1:8081/api/'+path,json.dumps(data or {}).encode(),
            {'Content-Type':'application/json','Origin':'http://127.0.0.1:8081'})
        with urllib.request.urlopen(request,timeout=1) as response:return json.load(response)

    def setUp(self):
        self.hw=FakeModbus();self.bridge=None
        self.tmp=tempfile.TemporaryDirectory(prefix='zlac-test-')
        preload=Path(self.tmp.name)/'serial_redirect.so'
        subprocess.run(['gcc','-shared','-fPIC',str(ROOT/'tests/fake_serial_redirect.c'),'-o',str(preload),'-ldl'],check=True)
        env=dict(os.environ,LD_PRELOAD=str(preload),ZLAC_TEST_PTY=self.hw.device)
        self.log=open('/tmp/zlac-integration-driver.log','w')
        self.guard=subprocess.Popen([sys.executable,str(ROOT/'guard.py'),'--command-timeout','.3'],
            env=env,stdout=self.log,stderr=self.log)
        rclpy.init();self.node=Node('zlac_test_observer');self.status=None
        self.node.create_subscription(String,'/zlac/status',lambda m:setattr(self,'status',json.loads(m.data)),1)
        self.pub=self.node.create_publisher(Twist,'/cmd_vel',1)
        self.wait_for(lambda:self.status is not None and self.status['ready'])

    def tearDown(self):
        if self.bridge:self.bridge.terminate();self.bridge.wait(timeout=4)
        if self.guard.poll() is None:self.guard.send_signal(signal.SIGINT)
        self.guard.wait(timeout=5)
        self.node.destroy_node();rclpy.try_shutdown();self.hw.close();self.log.close();self.tmp.cleanup()

    def test_guard_original_driver_and_bridge(self):
        self.assertIsNone(self.status['ack'])  # Guard never claims a hardware ACK.
        self.assertEqual(self.status['config']['io_timeout_ms'],1000)
        self.send(.05);self.wait_for(lambda:self.hw.rpm==[6,-6])
        start=time.monotonic();self.wait_for(lambda:self.hw.rpm==[0,0],1)
        self.assertLess(time.monotonic()-start,.65)
        self.wait_for(lambda:self.status['reason']=='watchdog_timeout')
        for v,w in [(float('nan'),0),(.1,float('inf')),(4,0),(0,100)]:
            self.send(.1);self.wait_for(lambda:self.hw.rpm!=[0,0])
            self.send(v,w);self.wait_for(lambda:self.hw.rpm==[0,0])
        self.node.destroy_publisher(self.pub)
        self.bridge=subprocess.Popen([sys.executable,str(ROOT/'app.py'),'--port','8081'],stdout=subprocess.DEVNULL)
        s=None
        for _ in range(50):
            try:s=self.api('session');break
            except (OSError,urllib.error.URLError):time.sleep(.1)
        self.assertIsNotNone(s);token=s['session'];seq=0
        def state():return self.api('state',dict(session=token))
        for _ in range(50):
            s=state()
            if s['ready']:break
            time.sleep(.1)
        self.assertTrue(s['ready'],s)
        self.assertEqual(s['config']['command_timeout'],.3)
        def arm():
            nonlocal seq
            seq+=1;return self.api('arm',dict(session=token,seq=seq,ticket=state()['ticket']))
        def command(s,v,w=0):
            nonlocal seq
            seq+=1;return self.api('command',dict(session=token,seq=seq,ticket=s['ticket'],lease=lease,v=v,w=w))
        a=arm();lease=a['lease'];s=command(a,.05,.3)
        self.wait_for(lambda:self.hw.rpm==[-1,-13])
        self.assertEqual([w['sent_rpm'] for w in s['wheels']],[-1,13])
        self.wait_for(lambda:self.hw.rpm==[0,0],1)
        with self.assertRaises(urllib.error.HTTPError):command(state(),.1)
        a=arm();lease=a['lease'];s=command(a,.1)
        seq+=1;self.api('stop',dict(session=token,seq=seq));self.wait_for(lambda:self.hw.rpm==[0,0])
        # Encoder loss: original read can block ~1 second; guard requests zero immediately on stale data.
        a=arm();lease=a['lease'];s=command(a,.1)
        self.hw.drop_feedback=True
        for _ in range(20):
            time.sleep(.05)
            try:s=command(s,.1)
            except urllib.error.HTTPError:break
        self.wait_for(lambda:self.hw.rpm==[0,0],4)
        self.assertFalse(state()['ready'])
        self.hw.drop_feedback=False
        for _ in range(80):
            if state()['ready']:break
            time.sleep(.05)
        a=arm();lease=a['lease'];s=command(a,.1);self.wait_for(lambda:self.hw.rpm!=[0,0])
        # SIGKILL skips all web cleanup; the separate ROS guard must send zero.
        self.bridge.kill();self.bridge.wait();self.bridge=None
        self.wait_for(lambda:self.hw.rpm==[0,0],1)
        self.wait_for(lambda:self.status['reason']=='watchdog_timeout')
        # SIGKILL on guard: Linux parent-death SIGINT asks the original child to stop/disable.
        self.pub=self.node.create_publisher(Twist,'/cmd_vel',1)
        # DDS may retain the killed bridge's publisher until discovery lease expiry.
        # Wait for the guard's one-publisher interlock before this final motion.
        def start_after_discovery():
            if self.node.count_publishers('/cmd_vel')==1 and self.status['ready']:
                self.send(.1)
            return self.hw.rpm!=[0,0]
        self.wait_for(start_after_discovery,timeout=20)
        self.guard.kill();self.guard.wait()
        self.wait_for(lambda:self.hw.rpm==[0,0],3)


if __name__=='__main__':unittest.main()
