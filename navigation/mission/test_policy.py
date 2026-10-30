import unittest
from policy import StartGate,ReturnPolicy,vision_health


def status(count=0):
    return dict(mcu_boot=12,count_generation=4,fire_requests_sent=count,armor_last_ns=0)


class PolicyTests(unittest.TestCase):
    def test_boot_already_auto_needs_new_edge_after_readiness(self):
        g=StartGate()
        for t in [0,1,2,3]:self.assertFalse(g.update(t,True,True,True))
        self.assertFalse(g.update(4,True,False,True))
        self.assertTrue(g.update(4.1,True,True,True))
        self.assertFalse(g.update(4.2,True,True,True))

    def test_not_ready_or_stale_edge_never_starts(self):
        g=StartGate();g.update(0,True,False,True);g.update(2,True,False,True)
        self.assertFalse(g.update(3,False,True,True))
        self.assertFalse(g.update(4,True,True,True))
        self.assertFalse(g.update(5,True,False,False))
        self.assertFalse(g.update(6,True,True,True))

    def test_strict_30_and_five_seconds_with_reacquisition(self):
        p=ReturnPolicy(100,4,12)
        self.assertIsNone(p.update(10,status(130),task=True)[1])
        self.assertIsNone(p.update(20,status(131),task=True)[1])
        self.assertIsNone(p.update(25,status(131),task=True)[1])
        v=status(131);v['armor_last_ns']=int(24e9)
        self.assertIsNone(p.update(28,v,task=True)[1])
        self.assertEqual(p.update(29.01,v,task=True)[1],'armor_lost_over_5s_after_30_requests')

    def test_50_returns_immediately_even_with_armor(self):
        p=ReturnPolicy(200,4,12);v=status(250);v['armor_last_ns']=int(30e9)
        self.assertEqual(p.update(30,v,task=True),(50,'sent_50_requests'))

    def test_boot_counter_reset_and_feedback_recovery_are_faults(self):
        for key,value in [('mcu_boot',13),('count_generation',5),('fire_requests_sent',99)]:
            p=ReturnPolicy(100,4,12);v=status(100);v[key]=value
            with self.assertRaises(RuntimeError):p.update(10,v,task=True)

    def test_stale_camera_not_treated_as_armor_loss(self):
        v=dict(schema=2,count_source='host_tx',monotonic_ns=1000000000,frame_ns=1000000000,
            link_fresh=True,pose_fresh=True,fire_requests_sent=0,count_generation=0,
            mcu_reject=0,mcu_state=128)
        self.assertTrue(vision_health(v,1050000000))
        for key,value in [('frame_ns',1),('fire_requests_sent',-1),('count_source','mcu_encoder'),
                          ('link_fresh',False),('pose_fresh',False),('mcu_reject',6)]:
            bad=dict(v);bad[key]=value
            with self.assertRaises(RuntimeError):vision_health(bad,1050000000)

    def test_missing_mcu_count_and_invalid_mcu_count_do_not_gate_host_counter(self):
        v=dict(schema=2,count_source='host_tx',monotonic_ns=1000000000,frame_ns=1000000000,
            link_fresh=True,pose_fresh=True,fire_requests_sent=49,count_generation=0,
            mcu_reject=0,mcu_state=128)
        self.assertTrue(vision_health(v,1050000000))
        v.update(count_received=False,count_received_ns=0,count_flags=0,completed_steps=999999)
        self.assertTrue(vision_health(v,1050000000))


if __name__=='__main__':unittest.main()
