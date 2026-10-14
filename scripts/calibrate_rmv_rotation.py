#!/usr/bin/env python3
"""File-only camera->raw small-Y sensor rotation calibration; never writes YAML.

R_navigation_sensor[i] X R_camera_board[i] is constant for a fixed board.
Solve A X = X B in SO(3), without invented IMU positions or legacy G transforms.
Translation, G axes, world alignment and control remain unverified.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

SOURCE = 'rmv_sensor1_frame1_wxyz'
CONTEXT_KEYS = ('boot_id', 'frame_epoch', 'geometry_revision', 'firmware_version')


def rotation(matrix):
    r = np.asarray(matrix, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all() or np.linalg.norm(r.T @ r - np.eye(3)) > 1e-6 or abs(np.linalg.det(r)-1) > 1e-6:
        raise ValueError('Expected a finite proper rotation matrix')
    return r


def nearest_rotation(matrix):
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .99 <= np.linalg.norm(q) <= 1.01:
        raise ValueError('Invalid real wxyz quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def fit_rotation(sensor, board):
    if len(sensor) != len(board) or len(sensor) < 5:
        raise ValueError('Too few paired rotations')
    sensor = [rotation(r) for r in sensor]
    board = [rotation(r) for r in board]
    aa, bb = [], []
    for i, j in itertools.combinations(range(len(sensor)), 2):
        a = cv2.Rodrigues(sensor[i].T @ sensor[j])[0].ravel()
        b = cv2.Rodrigues(board[i] @ board[j].T)[0].ravel()
        if min(np.linalg.norm(a), np.linalg.norm(b)) < np.deg2rad(1):
            continue
        aa.append(a); bb.append(b)
    if len(aa) < 10:
        raise ValueError('Insufficient rotation: move both small Y and P across multiple poses')
    aa, bb = np.asarray(aa), np.asarray(bb)
    spectra = [np.linalg.svd(v, compute_uv=False) for v in (aa, bb)]
    ratios = [float(s[1] / s[0]) for s in spectra]
    if min(ratios) < .05:
        raise ValueError('Single-axis/degenerate dataset: independent small-Y AND P poses required')
    x = nearest_rotation(aa.T @ bb)
    return x, {'pair_count': len(aa), 'second_to_first_axis_singular_ratios': ratios}


def assess_rotation(sensor, board, x, train, reference=None):
    orientations = [rotation(s) @ rotation(x) @ rotation(b) for s, b in zip(sensor, board)]
    reference = nearest_rotation(sum(orientations[i] for i in train)) if reference is None else rotation(reference)
    errors = np.array([np.rad2deg(np.linalg.norm(cv2.Rodrigues(reference.T @ r)[0])) for r in orientations])
    held = [i for i in range(len(errors)) if i not in train]
    if not held:
        raise ValueError('Independent held-out views are required')
    return {'per_view_error_deg': errors.tolist(), 'train_rms_deg': float(np.sqrt(np.mean(errors[train]**2))),
            'held_out_rms_deg': float(np.sqrt(np.mean(errors[held]**2))), 'max_error_deg': float(errors.max()),
            'held_out_indices': held, 'R_board_to_navigation_reference': reference.tolist()}


def load_pose(folder, stem, context):
    metadata = json.loads((folder / f'{stem}.pose.json').read_text(encoding='utf-8'))
    if metadata.get('pose_source') != SOURCE or metadata.get('sensor_id') != 1 or metadata.get('frame_id') != 1:
        raise ValueError(f'{stem}: expected raw small-Y sensor1/frame1 metadata')
    if any(metadata.get(key) != context.get(key) for key in CONTEXT_KEYS):
        raise ValueError(f'{stem}: mixed boot/epoch/geometry/firmware context')
    if metadata.get('valid_flags', 0) & 0x21 != 0x21 or metadata.get('accepted_samples', 0) <= 0:
        raise ValueError(f'{stem}: invalid quaternion/sample-time flags')
    if metadata.get('image_pose_method') != 'nearest sample within 50 ms, inside stationary interval':
        raise ValueError(f'{stem}: missing stopped image/pose association')
    if abs(metadata['image_host_us'] - metadata['sample_host_us']) > 50000:
        raise ValueError(f'{stem}: image/pose timestamps are too far apart')
    q = np.loadtxt(folder / f'{stem}.txt')
    r = quaternion_rotation(q)
    if np.linalg.norm(r - quaternion_rotation(metadata['quaternion_wxyz'])) > 1e-5:
        raise ValueError(f'{stem}: quaternion file differs from captured metadata')
    return r


def load_dataset(folder, cfg):
    if (folder / 'dataset_kind.txt').read_text(encoding='utf-8').strip() != 'image_and_pose' or (folder / 'pose_source.txt').read_text(encoding='utf-8').strip() != SOURCE:
        raise ValueError('Expected an RMV image_and_pose dataset; legacy/intrinsic datasets are not supported')
    context = json.loads((folder / 'pose_context.json').read_text(encoding='utf-8'))
    if context.get('pose_source') != SOURCE or any(key not in context for key in CONTEXT_KEYS):
        raise ValueError('Missing complete raw RMV context')
    if any(not isinstance(context[key], int) or context[key] <= 0 for key in ('boot_id', 'frame_epoch', 'firmware_version')):
        raise ValueError('Invalid boot/epoch/firmware context')
    cols, rows, square = int(cfg['pattern_cols']), int(cfg['pattern_rows']), float(cfg['square_size_mm'])
    if min(cols, rows) < 3 or not np.isfinite(square) or square <= 0:
        raise ValueError('Invalid chessboard geometry')
    k = np.asarray(cfg['camera_matrix'], dtype=float).reshape(3, 3)
    d = np.asarray(cfg['distort_coeffs'], dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    grid = np.zeros((cols*rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    images = sorted(folder.glob('*.jpg'), key=lambda p: int(p.stem))
    sensor, board, views, skipped = [], [], [], []
    for image in images:
        r = load_pose(folder, image.stem, context)  # Fail malformed provenance even if detection fails.
        img = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
        if img is None or img.shape != (cfg['camera_height'], cfg['camera_width']):
            raise ValueError(f'{image.name}: missing image or resolution mismatch')
        ok, corners = cv2.findChessboardCorners(img, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            skipped.append(image.name); continue
        corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        ok, rv, tv = cv2.solvePnP(grid, corners, k, d, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            raise ValueError(f'{image.name}: chessboard PnP failed')
        rb = cv2.Rodrigues(rv)[0]
        if np.any((grid @ rb.T + tv.ravel())[:, 2] <= 0):
            raise ValueError(f'{image.name}: chessboard behind camera')
        projected = cv2.projectPoints(grid, rv, tv, k, d)[0]
        error = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2))))
        if not np.isfinite(error) or error > 1:
            raise ValueError(f'{image.name}: PnP reprojection RMS {error:.3f}px exceeds 1px')
        sensor.append(r); board.append(rb)
        views.append({'image': image.name, 'reprojection_rms_px': error,
                      'image_host_us': json.loads(image.with_suffix('.pose.json').read_text(encoding='utf-8'))['image_host_us'],
                      'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (image, image.with_suffix('.txt'), image.with_suffix('.pose.json'))}})
    return sensor, board, views, skipped, context


def capture_segments(views, gap_us):
    if not np.isfinite(gap_us) or gap_us <= 0:
        raise ValueError('Capture gap must be finite and positive')
    segments = []
    previous = None
    for index, view in enumerate(views):
        stamp = view['image_host_us']
        if not isinstance(stamp, int) or stamp < 0 or (previous is not None and stamp < previous):
            raise ValueError('Image host timestamps must be nonnegative integers in capture order')
        if previous is None or stamp - previous > gap_us:
            segments.append([])
        segments[-1].append(index)
        previous = stamp
    return segments


def evaluate(sensor, board):
    train = [i for i in range(len(sensor)) if i % 4 != 0]
    x, excitation = fit_rotation([sensor[i] for i in train], [board[i] for i in train])
    quality = assess_rotation(sensor, board, x, train)
    passed = quality['train_rms_deg'] <= .5 and quality['held_out_rms_deg'] <= .5 and quality['max_error_deg'] <= 1.5
    return x, excitation, quality, passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_folder', type=Path)
    parser.add_argument('-c', '--config', type=Path, default=Path('configs/calibration.yaml'))
    parser.add_argument('-o', '--output', type=Path, default=Path('logs/calibration/rmv-rotation.json'))
    parser.add_argument('--capture-segment', type=int, help='Explicit 1-based contiguous capture segment; default evaluates all views')
    parser.add_argument('--max-gap-seconds', type=float, default=120, help='Segment boundary gap, default 120 s; uses timestamps, never residuals')
    parser.add_argument('--verify-candidate', type=Path, help='Validate a fixed previous rotation against independent images; never refit rotation')
    parser.add_argument('--same-fixed-board', action='store_true', help='Also freeze the previous board/navigation reference; requires board and base unchanged')
    args = parser.parse_args()
    if args.same_fixed_board and args.verify_candidate is None:
        raise ValueError('same-fixed-board requires verify-candidate')
    if args.output.suffix.lower() != '.json' or args.output.resolve() == args.config.resolve():
        raise ValueError('Output must be a JSON report, separate from the input YAML')
    if args.verify_candidate is not None:
        if args.output.resolve() == args.verify_candidate.resolve():
            raise ValueError('Independent verification cannot overwrite its reference candidate')
        if args.capture_segment is not None:
            raise ValueError('Independent verification uses the entire new dataset, not a selected segment')
    if args.output.resolve() in [args.input_folder.resolve()/name for name in ('pose_context.json', 'pose_source.txt', 'dataset_kind.txt')]:
        raise ValueError('Output cannot overwrite dataset context')
    cfg = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    sensor, board, views, skipped, context = load_dataset(args.input_folder, cfg)
    if len(views) < 15:
        raise ValueError(f'Only {len(views)} valid paired views; collect at least 15, preferably 20–30')
    segments = capture_segments(views, args.max_gap_seconds * 1000000)
    segment_info = [{'segment': n+1, 'first_image': views[ids[0]]['image'],
                     'last_image': views[ids[-1]]['image'], 'view_count': len(ids),
                     'duration_seconds': (views[ids[-1]]['image_host_us']-views[ids[0]]['image_host_us'])/1e6}
                    for n, ids in enumerate(segments)]
    full_assessment = None
    unselected = []
    input_view_count = len(views)
    if args.capture_segment is not None:
        if not 1 <= args.capture_segment <= len(segments):
            raise ValueError(f'capture-segment must be within 1..{len(segments)}')
        ids = segments[args.capture_segment-1]
        if len(ids) < 15:
            raise ValueError(f'Selected capture segment has only {len(ids)} views; at least 15 required')
        full_x, full_ex, full_q, full_passed = evaluate(sensor, board)
        full_assessment = {'passed': full_passed, 'R_camera2imubody': full_x.tolist(),
                           'excitation': full_ex, 'quality': full_q, 'images': [v['image'] for v in views]}
        unselected = [dict(v, reason='Outside explicitly selected contiguous capture segment')
                      for i, v in enumerate(views) if i not in ids]
        sensor, board, views = [sensor[i] for i in ids], [board[i] for i in ids], [views[i] for i in ids]
    reference_info = None
    if args.verify_candidate is None:
        x, excitation, quality, passed = evaluate(sensor, board)
    else:
        reference_bytes = args.verify_candidate.read_bytes()
        reference = json.loads(reference_bytes)
        if reference.get('result_kind') != 'candidate_camera_to_raw_sensor_rotation_only' or not reference.get('passed'):
            raise ValueError('Reference must be a passed raw sensor rotation candidate')
        if reference.get('context') != context or reference.get('config_sha256') != hashlib.sha256(args.config.read_bytes()).hexdigest():
            raise ValueError('Candidate context or calibration config differs; independent check cannot mix them')
        previous_images = {v['sha256'][v['image']] for v in reference['views']}
        if any(v['sha256'][v['image']] in previous_images for v in views):
            raise ValueError('Independent verification reuses reference training/held-out images')
        x = rotation(reference['R_camera2imubody'])
        train = [i for i in range(len(sensor)) if i % 4 != 0]
        # Check two-axis excitation only; discard the newly fitted rotation.
        _, excitation = fit_rotation([sensor[i] for i in train], [board[i] for i in train])
        board_reference = reference['quality']['R_board_to_navigation_reference'] if args.same_fixed_board else None
        quality = assess_rotation(sensor, board, x, train, board_reference)
        passed = quality['train_rms_deg'] <= .5 and quality['held_out_rms_deg'] <= .5 and quality['max_error_deg'] <= 1.5
        reference_info = {'path': str(args.verify_candidate.resolve()),
                          'sha256': hashlib.sha256(reference_bytes).hexdigest(), 'rotation_refitted': False,
                          'board_reference_refitted': not args.same_fixed_board}
    report = {'passed': passed, 'result_kind': 'candidate_camera_to_raw_sensor_rotation_only',
              'R_camera2imubody': x.tolist(), 'translation': None, 'camera_support_G_transform': None,
              'context': context, 'input_folder': str(args.input_folder.resolve()), 'views': views,
              'input_view_count': input_view_count, 'capture_segments': segment_info,
              'selected_capture_segment': args.capture_segment, 'max_capture_gap_seconds': args.max_gap_seconds,
              'unselected_views': unselected, 'full_dataset_assessment': full_assessment,
              'independent_reference': reference_info,
              'skipped': skipped, 'excitation': excitation, 'quality': quality,
              'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
              'quality_thresholds_deg': {'train_rms': .5, 'held_out_rms': .5, 'max_error': 1.5},
              'limits': ['No IMU translation supplied: translation is not estimated.',
                         'Raw navigation axes are not certified camera-support/world axes.',
                         'Candidate does not change YAML, firmware geometry, or control permissions.',
                         'A segment result does not certify navigation-frame stability across other time periods.',
                         'Symmetric chessboard corner flips are not auto-corrected; inconsistent rotations fail quality.']}
    if reference_info is not None:
        report['result_kind'] = 'independent_fixed_rotation_validation'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(f'Report: {args.output}; passed={passed}; held-out RMS={quality["held_out_rms_deg"]:.4f}deg')
    return 0 if passed else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, KeyError, TypeError, OSError, cv2.error, np.linalg.LinAlgError) as error:
        print(f'calibrate_rmv_rotation: {error}', file=sys.stderr)
        sys.exit(2)
