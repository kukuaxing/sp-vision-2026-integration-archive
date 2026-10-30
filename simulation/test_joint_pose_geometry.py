#!/usr/bin/env python3
"""Numerical and negative-control tests for the offline-only joint pose fitter."""
import math
import unittest
import cv2
import numpy as np
from joint_pose_geometry import Geometry, wrap


class JointPoseTest(unittest.TestCase):
    def setUp(self):
        self.g = Geometry(dict(camera_matrix=[1000,0,640,0,1000,512,0,0,1],
            distort_coeffs=[.01,-.005,.001,-.001,0],
            R_camera2gimbal=[0,0,1,-1,0,0,0,-1,0], t_camera2gimbal=[.02,.01,-.01]),np.eye(3))
        self.p = np.array([3., .04, .03, math.radians(22)])

    def test_independent_rigid_projection(self):
        rv=cv2.Rodrigues(self.g.world_from_camera.T @ self.g.armor_rotation(self.p[3],math.radians(15)))[0]
        tv=self.g.world_from_camera.T @ (self.p[:3]-self.g.camera_origin)
        expected=cv2.projectPoints(self.g.obj,rv,tv,self.g.k,self.g.dist)[0].reshape(4,2)
        np.testing.assert_allclose(self.g.project(self.p),expected,atol=1e-9)

    def test_roundtrip_perturbed_seed(self):
        for angle in (-50,0,50):
            truth=self.p.copy();truth[3]=math.radians(angle)
            seed=truth+np.array([.08,-.02,.01,math.radians(7)])
            result=self.g.fit(self.g.project(truth),seed)
            self.assertLess(np.linalg.norm(result.pose[:3]-truth[:3]),1e-5)
            self.assertLess(abs(wrap(result.pose[3]-truth[3])),1e-4)
            self.assertLess(result.rms_px,1e-5)

    def test_noisy_objective_never_increases(self):
        pixels=self.g.project(self.p)+np.random.default_rng(123).normal(0,.25,(4,2))
        baseline=self.g.yaw_only(pixels,self.g.ippe(pixels))
        result=self.g.fit(pixels,baseline)
        before=np.sqrt(np.mean(np.sum((self.g.project(baseline)-pixels)**2,axis=1)))
        self.assertLessEqual(result.rms_px,before+1e-10)
        self.assertTrue(np.isfinite(result.yaw_sigma_deg))

    def test_bad_observations_rejected(self):
        for pixels in (np.zeros((3,2)),np.full((4,2),np.nan),np.full((4,2),np.inf)):
            with self.assertRaises(ValueError): self.g.fit(pixels,self.p)
        with self.assertRaises(ValueError): self.g.fit(np.zeros((4,2)),np.full(4,np.nan))

    def test_behind_camera_rejected(self):
        with self.assertRaises(ValueError): self.g.project(np.array([-3.,0,0,0]))

    def test_wrong_pitch_is_not_exact_model(self):
        pixels=self.g.project(self.p,math.radians(25))
        baseline=self.g.yaw_only(pixels,self.g.ippe(pixels))
        result=self.g.fit(pixels,baseline)
        self.assertGreater(result.rms_px,.01)
        self.assertGreater(np.linalg.norm(result.pose[:3]-self.p[:3]),.001)

    def test_shortened_endpoints_remain_biased(self):
        pixels=self.g.project(self.p)
        for a,b in ((0,3),(1,2)):
            center=(pixels[a]+pixels[b])/2
            pixels[[a,b]]=center+(pixels[[a,b]]-center)*.92
        baseline=self.g.yaw_only(pixels,self.g.ippe(pixels))
        result=self.g.fit(pixels,baseline)
        # A better corner objective cannot promise unbiased depth.
        self.assertGreater(abs(result.pose[0]-self.p[0]),.02)


if __name__=='__main__': unittest.main()
