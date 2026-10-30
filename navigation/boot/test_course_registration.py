import copy,math,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import course_registration as cr

def registration(origin=(2.,3.),yaw=90.):
    return dict(schema='p00_map_registration_v1',map_frame='motion_plane_candidate',registration_id='test',
        physical_p00_and_heading_confirmed=True,map_source_signature={'test':'map'},reference_map_xy_m=list(origin),
        reference_map_yaw_deg=yaw,transform_course_from_map=cr.transform_matrix(origin,yaw))
class CourseRegistrationTests(unittest.TestCase):
    def test_p00_is_origin(self):
        r=registration();p=cr.project(r,dict(xyz=[2.,3.,.4],yaw_deg=90.,stamp_ns=123))
        self.assertAlmostEqual(p['xy_m'][0],0.);self.assertAlmostEqual(p['xy_m'][1],0.)
        self.assertEqual(p['yaw_deg'],0.);self.assertEqual(p['stamp_ns'],123)
        self.assertFalse(p['vertical_reference_registered'])
    def test_forward_and_left_axes(self):
        r=registration()
        for xyz,expected in [([2.,4.,0.],[1.,0.]),([1.,3.,0.],[0.,1.]),([2.,2.,0.],[-1.,0.]),([3.,3.,0.],[0.,-1.])]:
            out=cr.project(r,dict(xyz=xyz,yaw_deg=90.,stamp_ns=1))
            for a,b in zip(out['xy_m'],expected):self.assertAlmostEqual(a,b)
    def test_arbitrary_rotation_roundtrip(self):
        r=registration((-.001045,-.105875),91.413);a=math.radians(91.413);c,s=math.cos(a),math.sin(a)
        for x,y in [(.2,0.),(0.,.2),(-.6,1.2),(3.7,-1.1)]:
            world=[r['reference_map_xy_m'][0]+c*x-s*y,r['reference_map_xy_m'][1]+s*x+c*y,0.]
            out=cr.project(r,dict(xyz=world,yaw_deg=96.413,stamp_ns=5))
            self.assertAlmostEqual(out['xy_m'][0],x);self.assertAlmostEqual(out['xy_m'][1],y)
            self.assertAlmostEqual(out['yaw_deg'],5.)
    def test_yaw_wrap(self):
        out=cr.project(registration(yaw=179.),dict(xyz=[2.,3.,0.],yaw_deg=-179.,stamp_ns=1))
        self.assertEqual(out['yaw_deg'],2.)
    def test_reject_unconfirmed_or_wrong_map(self):
        r=registration();self.assertIs(cr.validate(r,{'test':'map'}),r)
        with self.assertRaises(ValueError):cr.validate(r,{'test':'different'})
        r=copy.deepcopy(r);r['physical_p00_and_heading_confirmed']=False
        with self.assertRaises(ValueError):cr.validate(r,{'test':'map'})
    def test_reject_inconsistent_matrix(self):
        r=registration();r['transform_course_from_map'][0][2]+=.1
        with self.assertRaises(ValueError):cr.validate(r,{'test':'map'})
    def test_cloud_content_is_bound(self):
        with tempfile.TemporaryDirectory() as d:
            prior=Path(d);(prior/'clouds').mkdir();cloud=prior/'clouds/000000.pcd'
            cloud.write_bytes(b'first scan');(prior/'keyframes.txt').write_text('frame0')
            with patch.object(cr,'geometry',return_value=(prior,{'mount':'fixed'})):
                before=cr.signature(prior);cloud.write_bytes(b'changed scan');after=cr.signature(prior)
            self.assertNotEqual(before['prior_sha256'],after['prior_sha256'])
if __name__=='__main__':unittest.main()
