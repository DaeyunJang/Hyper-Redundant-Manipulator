#!/usr/bin/env python3
"""Assemble paper assets from frozen occlusion results; no inference or ROS.

RGB synthetic conditions are regenerated with the original seeds and checked
pixel-for-pixel against the already-saved comparison sheets. Masks, curves,
angles and FK transforms are copied/read from the original evaluation.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zlib

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

HRM_ROOT = Path(__file__).resolve().parents[1]
SEG_ROOT = HRM_ROOT.parent / 'Hyper-Redundant-Manipulator-Segmentation'
SOURCE_REL = Path('runs/occlusion_pose_20261024/pose_evaluation_short_fragments')
DEFAULT_OUTPUT = HRM_ROOT / 'docs/figures/segmentation_occlusion_20261028'
BLUE, ORANGE, GRAY = '#0072B2', '#D55E00', '#606C76'
CONDITIONS = ('clean', 'thick30', 'multi3_30')
DATASET = 'archive_45deg_1'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_csv(path, rows):
    if not rows:
        raise ValueError('Cannot write empty source data.')
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_figure(fig, stem, dpi=400, formats=('svg', 'pdf', 'png')):
    for extension in formats:
        fig.savefig(str(stem) + '.' + extension, dpi=dpi, facecolor='white')
    plt.close(fig)


def source_path(old, seg_root):
    path = Path(old)
    if path.is_file():
        return path
    anchor = '20260927_HRM_shape_estimation_apriltag/'
    if anchor in str(path):
        remapped = seg_root / 'datasets' / anchor.rstrip('/') / str(path).split(anchor, 1)[1]
        if remapped.is_file():
            return remapped
    raise FileNotFoundError(path)


def load_assets(seg_root, output):
    source = seg_root / SOURCE_REL
    protocol = json.loads((source / 'protocol.json').read_text())
    audit = json.loads((source / 'independent_audit.json').read_text())
    if audit['audit_status'] != 'PASS':
        raise ValueError('Frozen source audit did not pass.')
    for name, expected in audit['original_artifact_sha256'].items():
        if sha256(source / name) != expected:
            raise ValueError(f'Frozen source artifact changed: {name}')
    for name in ('feasibility_backends.py', 'feasibility_augmentation_v2.py'):
        if sha256(seg_root / name) != protocol['source_hashes'][name]:
            raise ValueError(f'Synthetic-image reproduction code changed: {name}')
    sys.path.insert(0, str(seg_root))
    from feasibility_backends import OpenCVBaseline
    from feasibility_augmentation_v2 import make_case_v2

    # Among the three examples selected and saved by the original evaluation,
    # maximize clean-tip transverse displacement. Selection does NOT inspect
    # occlusion error or claim to maximize curvature along the entire body.
    available = sorted((source / 'example_masks' / DATASET / 'multi3_30' / 'unet_v2').glob('*.png'))
    candidates = []
    for mask_path in available:
        npz = source / 'shape_arrays' / DATASET / 'clean' / 'unet_v2' / (mask_path.stem + '.npz')
        with np.load(npz, allow_pickle=False) as data:
            transverse = float(np.linalg.norm(data['curve_tip_xyz_m'][1:]))
        candidates.append(dict(frame_id=mask_path.stem, clean_tip_transverse_m=transverse))
    selected = max(candidates, key=lambda item: item['clean_tip_transverse_m'])
    frame = selected['frame_id']
    record = next(r for r in protocol['records'] if r['dataset'] == DATASET and r['frame_id'] == frame)
    rgb_path = source_path(record['rgb'], seg_root)
    depth_path = source_path(record['depth'], seg_root)
    rgb = cv2.cvtColor(cv2.imread(str(rgb_path)), cv2.COLOR_BGR2RGB)
    placement = OpenCVBaseline().predict_rgb(rgb)
    seed = protocol['seed'] + zlib.crc32(f"{DATASET}:{record['source_stamp_ns']}".encode())
    cases = {(c['frame_id'], c['condition']): c for c in json.loads((source / 'cases.json').read_text())
             if c['dataset'] == DATASET and c['method'] == 'unet_v2'}
    rows = {(r['frame_id'], r['condition']): r for r in csv.DictReader((source / 'per_frame.csv').open())
            if r['dataset'] == DATASET and r['method'] == 'unet_v2'}
    raw = output / 'source_data'
    raw.mkdir(parents=True, exist_ok=True)
    shutil.copy2(rgb_path, raw / 'original_rgb.png')
    shutil.copy2(depth_path, raw / ('original_aligned_depth' + depth_path.suffix))
    result = dict(source=source, record=record, frame_id=frame, cases={}, arrays={}, rgb={}, masks={}, occlusion={})
    provenance = dict(dataset=DATASET, frame_id=frame, source_stamp_ns=record['source_stamp_ns'],
                      original_record=record, resolved_rgb=str(rgb_path), resolved_depth=str(depth_path),
                      selection_rule='Largest clean-tip transverse displacement among the THREE originally saved archive mask examples; no occlusion error used.',
                      selection_candidates=candidates, reference='Same method/frame unoccluded estimate, NOT ground truth',
                      crop_k=record['crop_k'], depth_scale_m=record['depth_scale_m'],
                      source_sha256={str(p): sha256(p) for p in (rgb_path, depth_path, source/'protocol.json', source/'cases.json', source/'per_frame.csv')},
                      conditions={}, original_source_preserved=True, model_inference_performed=False)
    for condition in CONDITIONS:
        image, _, meta, occ = make_case_v2(rgb, placement, seed, condition)
        case = cases[frame, condition]
        for key, value in meta.items():
            if value != case['augmentation'][key]:
                raise ValueError(f'Case metadata differs: {condition}/{key}')
        # The first original comparison tile contains the exact RGB under its
        # 66-pixel header, with no resize or overlay.
        sheet_path = source / 'examples' / DATASET / condition / (frame + '.png')
        sheet = cv2.cvtColor(cv2.imread(str(sheet_path)), cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        if not np.array_equal(image, sheet[66:66+h, :w]):
            raise ValueError(f'Regenerated RGB differs from original evaluation sheet: {condition}')
        mask_path = source / 'example_masks' / DATASET / condition / 'unet_v2' / (frame + '.png')
        npz_path = source / rows[frame, condition]['shape_npz']
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask.shape != rgb.shape[:2] or not np.isin(mask, [0, 255]).all():
            raise ValueError('Expected frozen full-resolution binary mask.')
        with np.load(npz_path, allow_pickle=False) as z:
            arrays = {key: z[key].copy() for key in z.files}
        pixels = arrays['observed_pixels_yx']
        if np.any(occ[pixels[:, 0], pixels[:, 1]]):
            raise ValueError('Observation inside known occlusion.')
        cv2.imwrite(str(raw / f'{condition}_rgb.png'), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(raw / f'{condition}_known_occluder.png'), occ * 255)
        shutil.copy2(mask_path, raw / f'{condition}_unet_mask.png')
        shutil.copy2(npz_path, raw / f'{condition}_shape.npz')
        result['rgb'][condition], result['masks'][condition] = image, mask
        result['arrays'][condition], result['occlusion'][condition] = arrays, occ
        result['cases'][condition] = case
        provenance['conditions'][condition] = dict(
            augmentation=case['augmentation'], shape_diagnostics=case['shape_diagnostics'],
            metrics={key: float(rows[frame, condition][key]) for key in ('fk_tip_delta_mm', 'q_rmse_deg', 'fk_orientation_delta_deg')},
            original_mask=str(mask_path), original_arrays=str(npz_path),
            mask_sha256=sha256(mask_path), arrays_sha256=sha256(npz_path),
            rgb_exact_match_to_saved_sheet=True, observed_points_inside_known_occlusion=0)
    ys, xs = np.nonzero(result['masks']['clean'])
    bbox = [max(0, int(xs.min())-26), max(0, int(ys.min())-24),
            min(rgb.shape[1], int(xs.max())+27), min(rgb.shape[0], int(ys.max())+25)]
    result['viewport'] = bbox
    provenance['figure_display_viewport_xyxy'] = bbox
    provenance['viewport_note'] = 'Identical display-only zoom in A/B; exported RGB/masks/depth retain full original crop geometry.'
    result['provenance'] = provenance
    return result


def load_frozen_exports(output):
    """Read the previously exported figure data without regenerating artifacts."""
    provenance = json.loads((output / 'provenance.json').read_text())
    raw = output / 'source_data'
    result = dict(frame_id=provenance['frame_id'], cases=provenance['conditions'],
                  viewport=provenance['figure_display_viewport_xyxy'], provenance=provenance,
                  arrays={}, rgb={}, masks={})
    for condition in CONDITIONS:
        mask_path = raw / f'{condition}_unet_mask.png'
        arrays_path = raw / f'{condition}_shape.npz'
        expected = provenance['conditions'][condition]
        if sha256(mask_path) != expected['mask_sha256']:
            raise ValueError(f'Frozen exported mask changed: {condition}')
        if sha256(arrays_path) != expected['arrays_sha256']:
            raise ValueError(f'Frozen exported arrays changed: {condition}')
        result['rgb'][condition] = cv2.cvtColor(
            cv2.imread(str(raw / f'{condition}_rgb.png')), cv2.COLOR_BGR2RGB)
        result['masks'][condition] = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        with np.load(arrays_path, allow_pickle=False) as arrays:
            result['arrays'][condition] = {key: arrays[key].copy() for key in arrays.files}
    return result


def gaps(arrays):
    ids, s = arrays['observed_component_ids'], arrays['raw_parameter_s']
    if len(ids) != len(s):
        raise ValueError('Observed-component and parameter arrays disagree.')
    return [(float(s[i]), float(s[i+1])) for i in range(len(s)-1) if ids[i] != ids[i+1]]


def curve_with_gaps(ax, curve, intervals, color, lw=2.0):
    s = np.linspace(0, 1, len(curve))
    support = np.ones(len(s), dtype=bool)
    for lo, hi in intervals:
        support[(s > lo) & (s < hi)] = False
    solid = curve.copy()
    solid[~support] = np.nan
    ax.plot(*solid.T, color=color, lw=lw)
    for lo, hi in intervals:
        inside = (s >= lo-1/1000) & (s <= hi+1/1000)
        ax.plot(*curve[inside].T, color=color, lw=lw, linestyle=(0, (3, 2)))


def draw_image(ax, image, viewport, title):
    if image.ndim == 2:
        ax.imshow(image, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    else:
        ax.imshow(image, interpolation='nearest')
    x0, y0, x1, y1 = viewport
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_title(title, fontsize=10, pad=8)
    ax.set_axis_off()


def draw_a(fig, spec, data):
    sub = spec.subgridspec(1, 3, wspace=.10)
    for col, condition in enumerate(CONDITIONS):
        fraction = 100*data['cases'][condition]['augmentation']['actual_centerline_occlusion_fraction']
        title = 'Unoccluded' if condition == 'clean' else (
            f'One gap\n{fraction:.1f}%' if condition == 'thick30' else f'Three gaps\n{fraction:.1f}% total')
        ax = fig.add_subplot(sub[col])
        draw_image(ax, data['rgb'][condition], data['viewport'], title)


def draw_b(fig, spec, data):
    sub = spec.subgridspec(1, 2, wspace=.20)
    for index, (image, title) in enumerate(((data['masks']['clean'], 'Unoccluded mask'),
                                          (data['masks']['multi3_30'], 'Three gaps mask'))):
        ax = fig.add_subplot(sub[index])
        draw_image(ax, image, data['viewport'], title)


def setup_3d(ax, all_points):
    points = np.concatenate(all_points)
    low, high = points.min(axis=0), points.max(axis=0)
    span = high-low
    low -= np.maximum(span*.06, 1.5)
    high += np.maximum(span*.06, 1.5)
    for setter, limits in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), zip(low, high)):
        setter(*limits)
    ax.set_box_aspect(high-low)
    ax.view_init(elev=24, azim=-64)
    ax.set_proj_type('ortho')
    ax.set_xlabel('X (mm)', labelpad=5)
    ax.set_ylabel('Y (mm)', labelpad=5)
    ax.set_zlabel('Z (mm)', labelpad=5)
    ax.tick_params(labelsize=8, pad=1)
    ax.set_xticks([0, 20, 40, 60, 80] if high[0] > 75 else [0, 20, 40, 60])
    ax.set_yticks([0, 10, 20] if high[1] > 19 else [0, 10])
    ax.set_zticks([0, 10, 20] if high[2] > 19 else [0, 10])
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((1, 1, 1, 0))
        axis._axinfo['grid']['color'] = (.83, .85, .87, .55)
    ax.grid(True)


def draw_c(ax, data, mode='reconstruction'):
    clean, occ = data['arrays']['clean'], data['arrays']['multi3_30']
    arrays = (clean, occ)
    curves, paths = [], []
    intervals = gaps(occ)
    for condition, a, color in zip(('clean', 'multi3_30'), arrays, (BLUE, ORANGE)):
        scale = data['cases'][condition]['shape_diagnostics']['hardware_length_scale']
        curve = a['curve_xyz_m']*1000
        if mode == 'observations':
            curve = curve/scale
        curves.append(curve)
        curve_with_gaps(ax, curve, [] if condition == 'clean' else intervals, color, 2.0)
        if mode == 'reconstruction':
            chain = np.vstack((np.zeros(3), a['fk_joint_transforms'][:, :3, 3], a['fk_tip_xyz_m']))*1000
            paths.append(chain)
            ax.plot(*chain.T, color=color, lw=.95, linestyle=':', alpha=.75)
            ax.scatter(*a['fk_tip_xyz_m']*1000, s=55, marker='o' if condition=='clean' else '^',
                       facecolors='white' if condition=='clean' else color, edgecolors=color,
                       linewidths=1.5, depthshade=False, zorder=5)
    if mode == 'observations':
        observed = occ['observed_base_xyz_m']*1000
        paths.append(observed)
        ax.scatter(*observed.T, c=GRAY, s=16, depthshade=False, alpha=.9)
    setup_3d(ax, curves+paths)
    handles = [Line2D([0], [0], color=BLUE, lw=2, label='Unoccluded estimate'),
               Line2D([0], [0], color=ORANGE, lw=2, label='Occluded estimate'),
               Line2D([0], [0], color=GRAY, lw=1.5, linestyle='--', label='Curve across observation gaps')]
    if mode == 'reconstruction':
        handles += [Line2D([0], [0], color=GRAY, lw=1, linestyle=':', label='FK chain'),
                    Line2D([0], [0], color=BLUE, marker='o', mfc='white', linestyle='', label='Unoccluded FK tip'),
                    Line2D([0], [0], color=ORANGE, marker='^', linestyle='', label='Occluded FK tip')]
    else:
        handles += [Line2D([0], [0], color=GRAY, marker='o', linestyle='', label='Observed depth points (occluded)')]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(-.10, 1.04), fontsize=7.4,
              frameon=False, ncol=2, handlelength=2.1, columnspacing=1.)


def export_coordinates(data, output):
    raw = output / 'source_data'
    curve_rows, obs_rows, fk_rows, boundary_rows, angle_rows, gap_rows = [], [], [], [], [], []
    for condition in CONDITIONS:
        a = data['arrays'][condition]
        diag = data['cases'][condition]['shape_diagnostics']
        scale = diag['hardware_length_scale']
        shift = np.asarray(diag['first_point_before_production_origin_constraint_m'])
        if not np.allclose(shift, 0, atol=1e-12):
            raise ValueError('Unscaled curve display requires explicit nonzero origin shift handling.')
        for i, xyz in enumerate(a['curve_xyz_m']):
            curve_rows.append(dict(condition=condition, dense_index=i, parameter_s=i/(len(a['curve_xyz_m'])-1),
                                   x_m=xyz[0], y_m=xyz[1], z_m=xyz[2], hardware_length_scale=scale,
                                   unscaled_x_m=xyz[0]/scale, unscaled_y_m=xyz[1]/scale, unscaled_z_m=xyz[2]/scale))
        for i, (xyz, uv, comp, s) in enumerate(zip(a['observed_base_xyz_m'], a['observed_pixels_yx'],
                                                   a['observed_component_ids'], a['raw_parameter_s'])):
            obs_rows.append(dict(condition=condition, observed_index=i, component_id=int(comp), parameter_s=s,
                                 image_v=int(uv[0]), image_u=int(uv[1]), x_m=xyz[0], y_m=xyz[1], z_m=xyz[2]))
        transforms = np.concatenate((np.eye(4)[None], a['fk_joint_transforms']))
        for i, transform in enumerate(transforms):
            xyz = transform[:3, 3]
            fk_rows.append(dict(condition=condition, frame='base' if i==0 else f'joint_{i:02d}',
                                x_m=xyz[0], y_m=xyz[1], z_m=xyz[2]))
        xyz = a['fk_tip_xyz_m']
        fk_rows.append(dict(condition=condition, frame='tip', x_m=xyz[0], y_m=xyz[1], z_m=xyz[2]))
        for i, xyz in enumerate(a['boundary_xyz_m']):
            boundary_rows.append(dict(condition=condition, boundary_index=i, x_m=xyz[0], y_m=xyz[1], z_m=xyz[2]))
        delta = np.rad2deg(np.arctan2(np.sin(a['q_rad']-data['arrays']['clean']['q_rad']),
                                      np.cos(a['q_rad']-data['arrays']['clean']['q_rad'])))
        for i, (q, dq) in enumerate(zip(a['q_rad'], delta)):
            angle_rows.append(dict(condition=condition, joint_id=i+1, axis='tilt' if i%2==0 else 'pan',
                                   angle_rad=q, angle_deg=np.rad2deg(q), delta_from_clean_deg=dq))
        for i, (lo, hi) in enumerate(gaps(a)):
            gap_rows.append(dict(condition=condition, gap_index=i+1, s_start=lo, s_end=hi,
                                  meaning='Between consecutive observed skeleton components; display-only dashed interval'))
    for name, rows in (('example_curves.csv', curve_rows), ('example_observations.csv', obs_rows),
                       ('example_fk_frames.csv', fk_rows), ('example_boundaries.csv', boundary_rows),
                       ('example_joint_angles.csv', angle_rows), ('example_gap_intervals.csv', gap_rows)):
        write_csv(raw/name, rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--segmentation-root', type=Path, default=SEG_ROOT)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--composite', action='store_true', help='Also use already-generated panel_D data.')
    parser.add_argument('--png-only', action='store_true',
                        help='Refresh only B and optional composite PNGs from existing frozen exports; do not rewrite source data.')
    args = parser.parse_args()
    output = args.output.resolve()
    if output == args.segmentation_root.resolve() or output == HRM_ROOT:
        raise ValueError('Output must be a dedicated figure directory.')
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'svg.fonttype': 'none',
                         'pdf.fonttype': 42, 'axes.titleweight': 'regular', 'axes.spines.top': False,
                         'axes.spines.right': False})
    formats = ('png',) if args.png_only else ('svg', 'pdf', 'png')
    if args.png_only:
        data = load_frozen_exports(output)
        panels = (('B_visible_body_segmentation', draw_b),)
    else:
        data = load_assets(args.segmentation_root.resolve(), output)
        export_coordinates(data, output)
        panels = (('A_synthetic_occlusion', draw_a), ('B_visible_body_segmentation', draw_b))
    for panel, draw in panels:
        fig = plt.figure(figsize=(6.4, 3.8))
        spec = fig.add_gridspec(1, 1, left=.02, right=.98, top=.84, bottom=.04)
        draw(fig, spec[0], data)
        save_figure(fig, output/panel, formats=formats)
    modes = () if args.png_only else (
        ('reconstruction', 'C_3d_reconstruction'), ('observations', 'C_observed_points_unscaled'))
    for mode, name in modes:
        fig = plt.figure(figsize=(6.4, 4.6))
        ax = fig.add_axes([.02, .08, .91, .80], projection='3d')
        draw_c(ax, data, mode)
        footer = ('Length-normalized curves and FK chains; hrm_base coordinates' if mode == 'reconstruction' else
                  'Raw depth observations + fitted curves BEFORE hardware-length normalization')
        fig.text(.5, .025, footer, ha='center', fontsize=8, color=GRAY)
        save_figure(fig, output/name)
    if args.composite:
        from prepare_segmentation_panel_d import draw_panel_d
        # A-D positions follow the user's supplied screenshot. All plot elements
        # are native vector graphics; only actual camera/mask images are rasters.
        fig = plt.figure(figsize=(13.8, 9.8))
        grid = fig.add_gridspec(2, 2, left=.045, right=.97, top=.87, bottom=.10,
                               wspace=.18, hspace=.43, height_ratios=(1., 1.15))
        draw_a(fig, grid[0, 0], data)
        draw_b(fig, grid[0, 1], data)
        ax = fig.add_subplot(grid[1, 0], projection='3d')
        draw_c(ax, data)
        sub = grid[1, 1].subgridspec(1, 2, wspace=.42)
        axes = [fig.add_subplot(sub[j]) for j in range(2)]
        with (output/'panel_D/source_data/panel_D_per_frame.csv').open() as stream:
            quantitative_rows = list(csv.DictReader(stream))
        draw_panel_d(axes, quantitative_rows, title_pad=20)
        for quantitative_axis in axes:
            quantitative_axis.title.set_fontsize(10)
            for label in quantitative_axis.texts:
                if label.get_text().startswith('Mean '):
                    label.set_text(label.get_text().replace('Mean ', ''))
                    label.set_fontsize(9)
        titles = ('A  Synthetic partial occlusion', 'B  Visible-body segmentation',
                  'C  3D shape reconstruction', 'D  Quantitative evaluation')
        for index, (spec, title) in enumerate(zip((grid[0,0], grid[0,1], grid[1,0], grid[1,1]), titles)):
            box = spec.get_position(fig)
            fig.text(box.x0, box.y1+(.045 if index < 2 else .068), title, fontsize=13, weight='bold')
        fig.suptitle('Shape estimation robustness under synthetic partial occlusion',
                     fontsize=17, weight='bold', x=.045, ha='left', y=.985)
        fig.text(.045, .946, 'Paired differences from the same-frame unoccluded estimate — not ground-truth accuracy',
                 fontsize=10, color=GRAY)
        fig.text(.045, .025, 'A–C: one archived bent RGB-D frame. D: three gaps (~30%); points = frames, diamonds = means.\n'
                 'FK difference ≤1 mm: Straight 38/40 (95.00%); Bent 58/64 (90.63%).\n'
                 'C uses length-normalized curves and FK chains. Raw depth observations are supplied separately.',
                 fontsize=8.8, color=GRAY, linespacing=1.6)
        save_figure(fig, output/'figure_segmentation_ABCD', formats=formats)
    if args.png_only:
        print(json.dumps({'output': str(output), 'frame': data['frame_id'],
                          'frozen_exports_reused': True, 'png_only': True}))
        return
    data['provenance']['display_geometry_note'] = (
        'Main C: saved hardware-length-normalized curves and FK chains, not raw depth points. '
        'Supplement C_observed_points_unscaled: original observed_base_xyz_m and curve_xyz_m/hardware_length_scale. '
        'No observations were translated, rescaled or invented for display. All axes use equal physical scale. '
        'Dashed curves span s intervals between observed component endpoints, not exact physical occluder boundaries.')
    (output/'provenance.json').write_text(json.dumps(data['provenance'], indent=2)+'\n')
    print(json.dumps({'output': str(output), 'frame': data['frame_id'], 'pixel_exact_cases': len(CONDITIONS),
                      'conditions': data['provenance']['conditions'].keys()}, default=list))


if __name__ == '__main__':
    main()
