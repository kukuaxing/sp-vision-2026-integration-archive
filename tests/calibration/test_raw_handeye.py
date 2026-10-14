import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys
import itertools
import yaml
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('raw_handeye', ROOT/'scripts/calibrate_rmv_rotation.py')
cal = importlib.util.module_from_spec(spec); spec.loader.exec_module(cal)


def rot(vector):
    return cv2.Rodrigues(np.asarray(vector, dtype=float))[0]


def fixture(single_axis=False):
    x = rot([.4, -.8, 1.1]); fixed = rot([-.25, .15, .3])
    sensor = [rot([0, p, 0]) @ rot([0, 0, y]) for p in ([0] if single_axis else [-.2, -.1, 0, .1, .2]) for y in [-.2, -.1, 0, .1, .2]]
    board = [x.T @ s.T @ fixed for s in sensor]
    return x, sensor, board


class RawRotationTest(unittest.TestCase):
    def test_rendered_chessboards_full_cli(self):
        # Project checker squares independently, then exercise detection, PnP,
        # raw quaternion provenance and the production file-only CLI.
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            context = {'pose_source': cal.SOURCE, 'boot_id': 9, 'frame_epoch': 1, 'geometry_revision': 0, 'firmware_version': 42}
            for name, text in [('pose_context.json', json.dumps(context)), ('pose_source.txt', cal.SOURCE), ('dataset_kind.txt', 'image_and_pose')]:
                (folder/name).write_text(text)
            k = np.array([[1200., 0, 480], [0, 1200., 360], [0, 0, 1]])
            cfg = {'pattern_cols': 11, 'pattern_rows': 8, 'square_size_mm': 15,
                   'camera_width': 960, 'camera_height': 720, 'camera_matrix': k.ravel().tolist(), 'distort_coeffs': [0]*5}
            config = folder/'config.yaml'; config.write_text(yaml.safe_dump(cfg))
            config_before = config.read_bytes()
            x = rot([.4, -.8, 1.1])
            for index, (pitch, yaw) in enumerate(itertools.product(np.linspace(-.2, .2, 5), repeat=2), 1):
                delta = rot([pitch, 0, 0]) @ rot([0, yaw, 0])
                ws = x @ delta @ x.T; cb = delta.T
                rv = cv2.Rodrigues(cb)[0]; tv = np.array([-75., -52.5, 900.])
                image = np.full((1440, 1920), 220, np.uint8)
                for row in range(9):
                    for col in range(12):
                        square = np.array([[(col-1)*15, (row-1)*15, 0], [col*15, (row-1)*15, 0],
                                           [col*15, row*15, 0], [(col-1)*15, row*15, 0]], float)
                        corners = cv2.projectPoints(square, rv, tv, k, np.zeros(5))[0].reshape(-1, 2)*2
                        cv2.fillConvexPoly(image, np.rint(corners).astype(np.int32), 20 if (row+col) % 2 else 245, cv2.LINE_AA)
                image = cv2.resize(image, (960, 720), interpolation=cv2.INTER_AREA)
                self.assertTrue(cv2.imwrite(str(folder/f'{index}.jpg'), image))
                vector = cv2.Rodrigues(ws)[0].ravel(); angle = np.linalg.norm(vector)
                q = np.r_[np.cos(angle/2), vector*np.sin(angle/2)/angle] if angle > 1e-8 else np.array([1, 0, 0, 0])
                np.savetxt(folder/f'{index}.txt', q[None, :])
                stamp = 100000 + index * 10000 + (300000000 if index > 5 else 0)
                data = dict(context, sensor_id=1, frame_id=1, valid_flags=55, accepted_samples=50,
                            image_host_us=stamp, sample_host_us=stamp-1000, quaternion_wxyz=q.tolist(),
                            image_pose_method='nearest sample within 50 ms, inside stationary interval')
                (folder/f'{index}.pose.json').write_text(json.dumps(data))
            output = folder/'result.json'
            result = subprocess.run([sys.executable, str(ROOT/'scripts/calibrate_rmv_rotation.py'), str(folder),
                                     '-c', str(config), '-o', str(output)], capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            report = json.loads(output.read_text())
            error = np.rad2deg(np.linalg.norm(cv2.Rodrigues(x.T @ np.array(report['R_camera2imubody']))[0]))
            self.assertLess(error, .5)
            self.assertIsNone(report['translation'])
            self.assertIsNone(report['camera_support_G_transform'])
            self.assertEqual(len(report['views']), 25)
            self.assertEqual(config.read_bytes(), config_before)
            result = subprocess.run([sys.executable, str(ROOT/'scripts/calibrate_rmv_rotation.py'), str(folder),
                                     '-c', str(config), '-o', str(output), '--capture-segment=2'],
                                    capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            report = json.loads(output.read_text())
            self.assertEqual(report['input_view_count'], 25)
            self.assertEqual(len(report['views']), 20)
            self.assertEqual(len(report['unselected_views']), 5)
            self.assertEqual(report['selected_capture_segment'], 2)
            self.assertEqual(len(report['full_dataset_assessment']['images']), 25)
            self.assertEqual(len(list(folder.glob('*.jpg'))), 25)
            self.assertEqual(config.read_bytes(), config_before)
            output_before = output.read_bytes()
            result = subprocess.run([sys.executable, str(ROOT/'scripts/calibrate_rmv_rotation.py'), str(folder),
                                     '-c', str(config), '-o', str(output), '--capture-segment=3'],
                                    capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(output.read_bytes(), output_before)
            # Independent known mounting, with training image identities from
            # another fixture. Only the board reference may be estimated here.
            reference = dict(report, R_camera2imubody=x.tolist())
            reference['views'] = [{'image': 'reference.jpg', 'sha256': {'reference.jpg': '0'*64}}]
            reference_path = folder/'independent-reference.json'
            reference_path.write_text(json.dumps(reference))
            independent = folder/'independent-check.json'
            command = [sys.executable, str(ROOT/'scripts/calibrate_rmv_rotation.py'), str(folder),
                       '-c', str(config), '-o', str(independent), '--verify-candidate', str(reference_path)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            verified = json.loads(independent.read_text())
            self.assertEqual(verified['result_kind'], 'independent_fixed_rotation_validation')
            self.assertFalse(verified['independent_reference']['rotation_refitted'])
            self.assertEqual(verified['R_camera2imubody'], x.tolist())
            fixed_command = command + ['--same-fixed-board']
            result = subprocess.run(fixed_command, capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            verified = json.loads(independent.read_text())
            self.assertFalse(verified['independent_reference']['board_reference_refitted'])
            old_reference = reference['quality']['R_board_to_navigation_reference']
            reference['quality']['R_board_to_navigation_reference'] = (rot([0, 0, .1]) @ np.array(old_reference)).tolist()
            reference_path.write_text(json.dumps(reference))
            result = subprocess.run(fixed_command, capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 2)
            self.assertFalse(json.loads(independent.read_text())['passed'])
            self.assertGreater(json.loads(independent.read_text())['quality']['max_error_deg'], 1.5)
            reference['views'] = report['views']  # Reusing fit data must be rejected.
            reference_path.write_text(json.dumps(reference))
            result = subprocess.run(command, capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 2)
            self.assertIn('reuses reference', result.stderr)
            self.assertEqual(config.read_bytes(), config_before)

    def test_segment_boundaries_use_time_not_quality(self):
        views = [{'image_host_us': t} for t in [0, 1000, 122000000, 122001000]]
        self.assertEqual(cal.capture_segments(views, 120000000), [[0, 1], [2, 3]])
        for gap in [0, -1, float('nan')]:
            with self.assertRaises(ValueError): cal.capture_segments(views, gap)
        with self.assertRaises(ValueError):
            cal.capture_segments([{'image_host_us': 2}, {'image_host_us': 1}], 100)

    def test_forward_geometry_and_independent_opencv_park(self):
        x, sensor, board = fixture()
        train = [i for i in range(len(sensor)) if i % 4]
        solved, info = cal.fit_rotation([sensor[i] for i in train], [board[i] for i in train])
        self.assertLess(np.linalg.norm(solved-x), 1e-8)
        park, _ = cv2.calibrateHandEye(sensor, [np.zeros(3)]*len(sensor), board,
                                      [np.zeros(3)]*len(board), method=cv2.CALIB_HAND_EYE_PARK)
        self.assertLess(np.linalg.norm(solved-park), 1e-8)
        quality = cal.assess_rotation(sensor, board, solved, train)
        self.assertLess(quality['held_out_rms_deg'], 1e-5)
        self.assertGreater(min(info['second_to_first_axis_singular_ratios']), .05)

    def test_navigation_basis_does_not_change_mount(self):
        x, sensor, board = fixture()
        navigation = rot([.6, .3, -.9])
        solved, _ = cal.fit_rotation([navigation @ s for s in sensor], board)
        self.assertLess(np.linalg.norm(solved-x), 1e-8)

    def test_single_axis_and_invalid_rotations_rejected(self):
        _, sensor, board = fixture(True)
        with self.assertRaisesRegex(ValueError, 'Single-axis'): cal.fit_rotation(sensor, board)
        _, sensor, board = fixture(); sensor[0] = np.diag([-1, 1, 1])
        with self.assertRaisesRegex(ValueError, 'proper rotation'): cal.fit_rotation(sensor, board)
        with self.assertRaises(ValueError): cal.quaternion_rotation([0, 0, 0, 0])
        with self.assertRaises(ValueError): cal.quaternion_rotation([np.nan, 0, 0, 0])

    def test_held_out_moved_board_is_detected(self):
        _, sensor, board = fixture(); train = [i for i in range(len(sensor)) if i % 4]
        solved, _ = cal.fit_rotation([sensor[i] for i in train], [board[i] for i in train])
        board[0] = rot([0, 0, .15]) @ board[0]
        quality = cal.assess_rotation(sensor, board, solved, train)
        self.assertGreater(quality['held_out_rms_deg'], .5)
        self.assertGreater(quality['max_error_deg'], 1.5)

    def test_provenance_and_sample_association(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            context = {'pose_source': cal.SOURCE, 'boot_id': 2, 'frame_epoch': 1, 'geometry_revision': 0, 'firmware_version': 2026100501}
            data = dict(context, sensor_id=1, frame_id=1, valid_flags=55, accepted_samples=50,
                        image_host_us=100000, sample_host_us=99000, quaternion_wxyz=[1, 0, 0, 0],
                        image_pose_method='nearest sample within 50 ms, inside stationary interval')
            (folder/'1.txt').write_text('1 0 0 0')
            def write(value): (folder/'1.pose.json').write_text(json.dumps(value))
            write(data); self.assertTrue(np.allclose(cal.load_pose(folder, '1', context), np.eye(3)))
            for key, value in [('boot_id', 3), ('frame_id', 2), ('valid_flags', 54),
                               ('image_host_us', 200000), ('image_pose_method', 'synthetic')]:
                changed = dict(data); changed[key] = value; write(changed)
                with self.assertRaises(ValueError): cal.load_pose(folder, '1', context)
            write(data); (folder/'1.txt').write_text('.707106781 0 .707106781 0')
            with self.assertRaisesRegex(ValueError, 'differs'): cal.load_pose(folder, '1', context)


if __name__ == '__main__': unittest.main()
