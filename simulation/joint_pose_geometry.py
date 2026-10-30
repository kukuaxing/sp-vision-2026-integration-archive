#!/usr/bin/env python3
"""File-only fixed-inclination pose experiment. Never imported by robot control.

The fitter sees four measured corners, calibration and an IPPE seed only.
Truth, target radius, center and shot outcomes are deliberately absent.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Fit:
    pose: np.ndarray
    rms_px: float
    iterations: int
    accepted: bool
    yaw_sigma_deg: float


class Geometry:
    def __init__(self, config, world_from_gimbal):
        self.k = np.asarray(config['camera_matrix'], float).reshape(3, 3)
        self.dist = np.asarray(config['distort_coeffs'], float)
        self.rcam = np.asarray(config['R_camera2gimbal'], float).reshape(3, 3)
        self.trans = np.asarray(config['t_camera2gimbal'], float)
        self.rworld = np.asarray(world_from_gimbal, float)
        self.world_from_camera = self.rworld @ self.rcam
        self.camera_origin = self.rworld @ self.trans
        self.obj = np.array([[0, .135/2, .056/2], [0, -.135/2, .056/2],
                             [0, -.135/2, -.056/2], [0, .135/2, -.056/2]])
        self.yaw_center = math.atan2(self.rworld[1, 0], self.rworld[0, 0])

    @classmethod
    def probe_fixture(cls, config):
        rcam = np.asarray(config['R_camera2gimbal'], float).reshape(3, 3)
        a = rcam[:, 2]
        b = np.array([3., 0., .03]); b /= np.linalg.norm(b)
        axis = np.cross(a, b)
        if np.linalg.norm(axis) < 1e-12:
            rotation = np.eye(3) if np.dot(a, b) > 0 else cv2.Rodrigues(np.array([np.pi, 0., 0.]))[0]
        else:
            rotation = cv2.Rodrigues(axis / np.linalg.norm(axis) * np.arccos(np.clip(np.dot(a, b), -1, 1)))[0]
        return cls(config, rotation)

    @staticmethod
    def armor_rotation(yaw, pitch):
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        return np.array([[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0, cp]])

    def project(self, pose, pitch=math.radians(15)):
        world = self.obj @ self.armor_rotation(pose[3], pitch).T + pose[:3]
        camera = (world - self.camera_origin) @ self.world_from_camera
        if not np.all(np.isfinite(camera)) or np.min(camera[:, 2]) <= .05:
            raise ValueError('invalid or behind-camera projection')
        return cv2.projectPoints(camera, np.zeros(3), np.zeros(3), self.k, self.dist)[0].reshape(4, 2)

    def ippe(self, corners):
        ok, rv, tv = cv2.solvePnP(self.obj.astype(np.float32), np.asarray(corners, np.float32),
                                 self.k, self.dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok or not np.all(np.isfinite(tv)) or tv[2, 0] <= 0:
            raise ValueError('invalid IPPE seed')
        rotation = self.world_from_camera @ cv2.Rodrigues(rv)[0]
        xyz = self.world_from_camera @ tv.ravel() + self.camera_origin
        return np.r_[xyz, math.atan2(rotation[1, 0], rotation[0, 0])]

    def yaw_only(self, corners, raw):
        # Production uses summed corner distances, then a 0.1 degree refinement.
        def cost(yaw):
            p = np.r_[raw[:3], yaw]
            return np.linalg.norm(self.project(p) - corners, axis=1).sum()
        coarse = self.yaw_center + np.radians(np.arange(-70, 71))
        best = min(coarse, key=cost)
        fine = best + np.radians(np.arange(-9, 10) * .1)
        return np.r_[raw[:3], min(fine, key=cost)]

    def jacobian(self, pose):
        columns = []
        for index in range(4):
            delta = np.zeros(4); delta[index] = 1e-5
            columns.append(((self.project(pose + delta) - self.project(pose - delta)) / 2e-5).ravel())
        return np.column_stack(columns)

    def fit(self, corners, seed):
        corners = np.asarray(corners, float)
        seed = np.asarray(seed, float)
        if corners.shape != (4, 2) or seed.shape != (4,) or not np.all(np.isfinite(corners)) or not np.all(np.isfinite(seed)):
            raise ValueError('four finite corners and a finite xyz/yaw seed required')
        def residual(p):
            return (self.project(p) - corners).ravel()
        initial = residual(seed)
        best, best_cost = seed.copy(), float(initial @ initial)
        iterations = 0
        # Multiple starts address the planar frontal ambiguity, without truth.
        for yaw_delta in (0., math.radians(-20), math.radians(20)):
            p = seed.copy(); p[3] += yaw_delta
            if abs(wrap(p[3] - self.yaw_center)) > math.radians(70):
                continue
            damping = 1e-3
            for _ in range(35):
                iterations += 1
                r = residual(p); cost = float(r @ r)
                if cost < best_cost:
                    best, best_cost = p.copy(), cost
                jac = self.jacobian(p); normal = jac.T @ jac
                try:
                    step = np.linalg.solve(normal + damping * np.diag(np.maximum(np.diag(normal), 1e-9)), -jac.T @ r)
                except np.linalg.LinAlgError:
                    break
                proposal = p + step
                valid = abs(wrap(proposal[3] - self.yaw_center)) <= math.radians(70)
                try:
                    rr = residual(proposal) if valid else None
                    new_cost = float(rr @ rr) if rr is not None else float('inf')
                except (ValueError, cv2.error):
                    new_cost = float('inf')
                if new_cost < cost:
                    p = proposal
                    if new_cost < best_cost:
                        best, best_cost = p.copy(), new_cost
                    damping = max(1e-9, damping * .3)
                    if np.linalg.norm(step) < 1e-8 or cost - new_cost < 1e-12:
                        break
                else:
                    damping *= 10
                    if damping > 1e9:
                        break
        # Only a local, assumed 0.25px iid-noise scale; NOT a bias/confidence guarantee.
        normal = self.jacobian(best).T @ self.jacobian(best)
        sigma = float('nan')
        if np.linalg.matrix_rank(normal) == 4:
            variance = np.linalg.inv(normal)[3, 3] * .25**2
            if variance >= 0:
                sigma = math.degrees(math.sqrt(variance))
        return Fit(best, math.sqrt(best_cost/4), iterations, best_cost < float(initial @ initial) - 1e-12, sigma)
