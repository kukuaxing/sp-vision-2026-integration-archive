from pathlib import Path
import sys,json,math,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'map_route'))
from mission import Mission
from core import State
from policy import ReturnPolicy


class WorkflowTests(unittest.TestCase):
    def simulate(self,lose):
        plan=json.loads((Path(__file__).resolve().parents[1]/'map_route/global_plan.json').read_text())
        m=Mission(plan,external_task=True);s=State(0.,.01,.02,0.,0.,0.)
        p=ReturnPolicy(100,3,5);task_start=None;stop_start=None;reason=None;used=0
        for k in range(15000):
            now=k*.02
            task=m.phase=='TASK_DWELL'
            if task and task_start is None:task_start=now
            used=min(31 if lose else 50,int((now-task_start)/.7)) if task_start is not None else 0
            # Once the decision is made, stop simulated firing as the transport does.
            if reason is not None:used=last_used
            v=dict(mcu_boot=5,count_generation=3,fire_requests_sent=100+used,
                armor_last_ns=int((task_start+31*.7 if lose and used>=31 else now)*1e9))
            _,reason=p.update(now,v,task=task)
            if reason and stop_start is None:stop_start=now
            complete=stop_start is not None and now-stop_start>=.25
            c=m.step(s,.02,.1,int(now*1e9),now,task_complete=complete)
            if reason and c['phase']=='TASK_DWELL':self.assertEqual((c['wire_vx'],c['wire_vy'],c['wire_wz']),(0.,0.,0.))
            if c['phase']=='DONE':
                self.assertTrue(m.task_reached and m.home_reached)
                self.assertLess(math.hypot(s.x,s.y),.075)
                return used,reason
            last_used=used
            tf=-c['wire_vy']*(.0885 if c['wire_vy']<=0 else .095)*1.05
            tl=c['wire_vx']*.095*.95;tw=-c['wire_wz']*.8
            f=s.forward_speed+(tf-s.forward_speed)*.2;l=s.left_speed+(tl-s.left_speed)*.2
            w=s.yaw_rate+(tw-s.yaw_rate)*.2
            s=State(s.x+(math.cos(s.yaw)*f-math.sin(s.yaw)*l)*.02,
                    s.y+(math.sin(s.yaw)*f+math.cos(s.yaw)*l)*.02,s.yaw+w*.02,f,l,w)
        self.fail('Competition mission did not finish within 300 simulated seconds')

    def test_fifty_steps_then_reverse_route_home(self):
        self.assertEqual(self.simulate(False),(50,'sent_50_requests'))

    def test_31_steps_then_five_second_armor_loss_then_home(self):
        self.assertEqual(self.simulate(True),(31,'armor_lost_over_5s_after_30_requests'))


if __name__=='__main__':unittest.main()
