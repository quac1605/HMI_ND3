import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web_core import Config, Controller, encoder_velocity, lround


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.published = []
        self.c = Controller(lambda v, w: self.published.append((v, w)), Config(), clock=lambda:self.now)
        self.token = self.c.session()
        self.seq = 0

    def arm(self):
        self.seq += 1
        return self.c.arm(self.token, self.seq, self.c.ticket(self.token), True)

    def command(self, lease, v=.1, w=0):
        self.seq += 1
        self.c.command(self.token, self.seq, self.c.ticket(self.token), lease, v, w, True)

    def test_round_half_away_from_zero(self):
        self.assertEqual([lround(x) for x in [-2.5,-.5,0,.5,2.5]], [-3,-1,0,1,3])

    def test_expected_matches_check_velocity(self):
        # Load the original function bodies without importing ROS.
        import ast
        tree = ast.parse((Path(__file__).resolve().parents[2]/'zlac_driver_encoder/scripts/check_velocity.py').read_text())
        functions = [n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='lround']
        cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CheckVelocity')
        expected = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='expected')
        functions.append(expected)
        namespace = dict(math=math, RPM_TO_RADPS=2*math.pi/60)
        exec(compile(ast.Module(body=functions,type_ignores=[]),'<reference>','exec'),namespace)
        ref = type('Ref',(),dict(L=.4,r=.0825,rpm_max=300))()
        for v,w in [(0,0),(.05,0),(-.05,0),(0,.3),(0,-.3),(3,3),(-3,-3),(2,12)]:
            got = Config().expected(v,w)
            for ours, reference in zip(got, namespace['expected'](ref,v,w)):
                self.assertAlmostEqual(ours['raw_rpm'],reference[0])
                self.assertEqual(ours['sent_rpm'],reference[1])
                self.assertAlmostEqual(ours['mps'],reference[2])
        self.assertTrue(Config().expected(3,3)[1]['limited'])

    def test_encoder_uses_names_not_order_and_does_not_flip_sign(self):
        self.assertEqual(encoder_velocity(['right_wheel_joint','left_wheel_joint'],[-2,1],.1),[.1,-.2])
        for names,values in [(['left_wheel_joint'],[1]),(['left_wheel_joint','right_wheel_joint'],[1,float('nan')])]:
            with self.assertRaises(ValueError): encoder_velocity(names,values,.1)

    def test_watchdog_revokes_lease_and_late_packets_cannot_restart(self):
        lease = self.arm(); self.command(lease)
        self.now += .251; self.c.tick(True)
        self.assertEqual(self.published[-1],(0,0))
        with self.assertRaises(ValueError): self.command(lease)
        self.assertIsNone(self.c.owner)
        self.command(self.arm())
        self.assertEqual(self.published[-1],(.1,0))

    def test_delayed_ticket_stops_owner(self):
        lease = self.arm(); self.command(lease)
        ticket = self.c.ticket(self.token); self.now += .251; self.seq += 1
        with self.assertRaises(ValueError):
            self.c.command(self.token,self.seq,ticket,lease,.2,0,True)
        self.assertEqual(self.published[-1],(0,0))

    def test_stop_fences_inflight_command_and_arm(self):
        lease=self.arm(); self.command(lease)
        self.c.user_stop(self.token,100)
        with self.assertRaises(ValueError): self.command(lease)
        with self.assertRaises(ValueError): self.arm()
        self.assertEqual(self.published[-1],(0,0))

    def test_second_tab_cannot_control_but_can_stop(self):
        lease=self.arm(); self.command(lease)
        other=self.c.session()
        with self.assertRaises(ValueError): self.c.arm(other,1,self.c.ticket(other),True)
        self.c.user_stop(other,2)
        self.assertEqual(self.published[-1],(0,0))
        with self.assertRaises(ValueError): self.command(lease)

    def test_invalid_inputs_stop(self):
        for v,w in [(float('nan'),0),(0,float('inf')),(3.01,0),(0,100),('1',0),(True,0)]:
            lease=self.arm()
            with self.assertRaises(ValueError): self.command(lease,v,w)
            self.assertEqual(self.published[-1],(0,0))

    def test_data_loss_stops(self):
        lease=self.arm(); self.command(lease); self.c.tick(False)
        self.assertEqual(self.published[-1],(0,0))
        with self.assertRaises(ValueError): self.c.arm(self.token,100,self.c.ticket(self.token),False)

    def test_no_idle_motion_republication(self):
        lease=self.arm(); self.command(lease)
        size=len(self.published)
        self.now += .05; self.c.tick(True)
        self.assertEqual(len(self.published),size)


if __name__ == '__main__': unittest.main()
