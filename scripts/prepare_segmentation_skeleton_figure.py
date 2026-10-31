#!/usr/bin/env python3
"""Show frozen visible-body skeletons without rerunning any estimation.

Only the existing source_data arrays, masks, RGB images, and provenance are
read. Binary PNGs preserve the exact saved skeleton support at native image
resolution; colored overlays enlarge markers for display only. All plots are
PNG. The main A--D figure and original source files are never written.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "docs/figures/segmentation_occlusion_20261028"
CONDITIONS = ("clean", "thick30", "multi3_30")
COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")
COMPARISON = "B_visible_body_skeleton.png"
MANIFEST = "skeleton_visualization.json"
PIXEL_CSV = "example_skeleton_pixels.csv"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def output_names():
    names = {PIXEL_CSV}
    for condition in CONDITIONS:
        names.update((f"{condition}_skeleton.png",
                      f"{condition}_skeleton_overlay.png",
                      f"{condition}_mask_skeleton_overlay.png"))
    return names


def protected_hashes(source):
    """Protect every original source-data file and existing main figure image."""
    paths = [p for p in (source / "source_data").iterdir()
             if p.is_file() and p.name not in output_names()]
    paths.append(source / "provenance.json")
    paths.extend(p for p in source.iterdir()
                 if p.is_file() and p.suffix.lower() in (".png", ".pdf", ".svg")
                 and p.name != COMPARISON)
    return {p.relative_to(source).as_posix(): sha256(p) for p in sorted(paths)}


def load_frozen(source):
    provenance = json.loads((source / "provenance.json").read_text())
    assets = {}
    for condition in CONDITIONS:
        raw = source / "source_data"
        arrays_path = raw / f"{condition}_shape.npz"
        mask_path = raw / f"{condition}_unet_mask.png"
        expected = provenance["conditions"][condition]
        if sha256(arrays_path) != expected["arrays_sha256"]:
            raise ValueError(f"Frozen arrays no longer match provenance: {condition}")
        if sha256(mask_path) != expected["mask_sha256"]:
            raise ValueError(f"Frozen mask no longer matches provenance: {condition}")
        with np.load(arrays_path, allow_pickle=False) as saved:
            skeleton = saved["skeleton"].copy()
            pixels = saved["observed_pixels_yx"].copy()
            component_ids = saved["observed_component_ids"].copy()
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        rgb_bgr = cv2.imread(str(raw / f"{condition}_rgb.png"), cv2.IMREAD_COLOR)
        if mask is None or rgb_bgr is None:
            raise ValueError(f"Missing frozen image: {condition}")
        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        if skeleton.shape != mask.shape or rgb.shape[:2] != mask.shape:
            raise ValueError(f"Frozen image geometry differs: {condition}")
        if not np.isin(skeleton, [0, 1, 255]).all():
            raise ValueError(f"Skeleton must be a binary image: {condition}")
        if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) != len(component_ids):
            raise ValueError(f"Observed arrays have inconsistent shapes: {condition}")
        if not np.issubdtype(pixels.dtype, np.integer):
            raise ValueError(f"Observed pixels must be integer coordinates: {condition}")
        if not len(pixels) or np.any(pixels < 0) or np.any(pixels >= np.array(mask.shape)):
            raise ValueError(f"Observed pixels outside image: {condition}")
        support = skeleton > 0
        skeleton_pixels = np.argwhere(support)
        if np.any(support & (mask == 0)):
            raise ValueError(f"Saved skeleton extends outside predicted mask: {condition}")
        if not np.all(support[pixels[:, 0], pixels[:, 1]]):
            raise ValueError(f"Observed pixels outside saved skeleton: {condition}")
        skeleton_set = set(map(tuple, skeleton_pixels.tolist()))
        observed_set = set(map(tuple, pixels.tolist()))
        if len(observed_set) != len(pixels):
            raise ValueError(f"Duplicate observed pixels: {condition}")
        # No component labeling, thinning, bridging, or resampling is run here.
        # Stored observations alone identify the colored path order. Ungrouped
        # skeleton pixels, if present in another frozen export, remain gray.
        starts = np.r_[0, np.flatnonzero(np.diff(component_ids) != 0) + 1]
        ends = np.r_[starts[1:], len(component_ids)]
        paths = []
        for order, (start, end) in enumerate(zip(starts, ends), start=1):
            if order > len(COLORS):
                raise ValueError("This display palette supports at most four paths.")
            paths.append(dict(order=order, component_id=int(component_ids[start]),
                              pixels=pixels[start:end], color=COLORS[order-1]))
        assets[condition] = dict(
            skeleton=support, skeleton_pixels=skeleton_pixels, pixels=pixels,
            component_ids=component_ids, rgb=rgb, mask=mask, paths=paths,
            sets_equal=skeleton_set == observed_set,
            skeleton_without_depth_count=len(skeleton_set - observed_set),
            observed_outside_skeleton_count=len(observed_set - skeleton_set),
        )
    return provenance, assets


def draw_markers(ax, asset, labels=False, native=False):
    """Plot points only: neither gaps nor adjacent samples get joining lines."""
    size = 6.0 if not native else 3.3
    skeleton_pixels = asset["skeleton_pixels"]
    ax.scatter(skeleton_pixels[:, 1], skeleton_pixels[:, 0], s=size,
               marker="o", c="#8A8A8A", linewidths=0, zorder=2)
    for path in asset["paths"]:
        points = path["pixels"]
        ax.scatter(points[:, 1], points[:, 0], s=size, c=path["color"],
                   marker="o", linewidths=0, zorder=3)
        if labels:
            center = points[len(points)//2]
            ax.text(float(center[1]) + 13, float(center[0]), str(path["order"]),
                    color=path["color"], ha="center", va="center", fontsize=10,
                    fontweight="bold", zorder=5,
                    bbox=dict(boxstyle="round,pad=0.18", facecolor="white",
                              edgecolor="none", alpha=.94))
    base = asset["pixels"][0]
    ring_size = 66 if not native else 31
    ax.scatter([base[1]], [base[0]], s=ring_size, facecolors="none",
               edgecolors="black", linewidths=2.3 if not native else 1.8, zorder=6)
    ax.scatter([base[1]], [base[0]], s=ring_size, facecolors="none",
               edgecolors="#F0E442", linewidths=1.2 if not native else .9, zorder=7)


def draw_image(ax, asset, background, viewport, labels=False):
    image = asset[background]
    ax.imshow(image, cmap="gray" if background == "mask" else None,
              vmin=0, vmax=255, interpolation="nearest", origin="upper")
    draw_markers(ax, asset, labels=labels)
    x0, y0, x1, y1 = viewport
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_axis_off()


def save_native_overlay(path, asset, background):
    """A full-resolution display overlay, distinct from exact binary output."""
    height, width = asset["skeleton"].shape
    dpi = 100
    fig = plt.figure(figsize=(width/dpi, height/dpi), dpi=dpi, frameon=False)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.imshow(asset[background], cmap="gray" if background == "mask" else None,
              vmin=0, vmax=255, interpolation="nearest", origin="upper")
    draw_markers(ax, asset, native=True)
    ax.set_xlim(-.5, width-.5)
    ax.set_ylim(height-.5, -.5)
    ax.set_axis_off()
    fig.savefig(path, dpi=dpi, pad_inches=0)
    plt.close(fig)
    if cv2.imread(str(path), cv2.IMREAD_UNCHANGED).shape[:2] != (height, width):
        raise ValueError(f"Overlay dimensions changed: {path}")


def comparison_figure(source, provenance, assets):
    fig = plt.figure(figsize=(7.8, 10.2), facecolor="white")
    fig.suptitle("Visible-body skeletons from the saved prediction", y=.985,
                 fontsize=15, fontweight="bold")
    fig.text(.5, .955, "Representative frame 822  |  Colored dots are saved observed pixels",
             ha="center", fontsize=9.5, color="#404040")
    grid = fig.add_gridspec(2, 2, left=.07, right=.93, top=.895, bottom=.19,
                           hspace=.23, wspace=.27)
    for col, condition in enumerate(("clean", "multi3_30")):
        asset = assets[condition]
        count = len(asset["skeleton_pixels"])
        observations = len(asset["pixels"])
        label = "Unoccluded" if condition == "clean" else "Three gaps"
        for row, background in enumerate(("mask", "rgb")):
            ax = fig.add_subplot(grid[row, col])
            draw_image(ax, asset, background, provenance["figure_display_viewport_xyxy"],
                       labels=(row == 0))
            title = f"{label}: {'mask' if background == 'mask' else 'RGB'} + skeleton"
            if row == 0:
                title += f"\n{count} skeleton pixels / {observations} valid-depth observations"
            ax.set_title(title, fontsize=10, pad=9, linespacing=1.6)
    handles = [Line2D([], [], marker="o", color=color, linestyle="None", markersize=6,
                      label=f"Path {i}") for i, color in enumerate(COLORS, start=1)]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .127),
               ncol=4, frameon=False, fontsize=10, columnspacing=1.7)
    fig.text(.5, .124, "Paths ordered base to tip; component IDs are kept in the source data.",
             ha="center", fontsize=9, va="top")
    fig.text(.5, .100, "Three gaps: 2, 8, 3, 18 pixels in Paths 1–4 (raw IDs 4, 3, 2, 1).",
             ha="center", fontsize=9, va="top")
    fig.text(.5, .076, "Yellow ring: base-side first observation (v = 295, u = 189; zero-based).",
             ha="center", fontsize=9, va="top")
    fig.text(.5, .050, "Dots are enlarged for visibility. No pixels or lines are inserted across gaps.\n"
             "Skeleton support equals valid-depth observation support in these saved examples.",
             ha="center", fontsize=9, va="top", linespacing=1.5, color="#404040")
    fig.savefig(source / COMPARISON, dpi=300, facecolor="white")
    plt.close(fig)


def build(source):
    before = protected_hashes(source)
    provenance, assets = load_frozen(source)
    raw = source / "source_data"
    rows = []
    condition_metadata = {}
    for condition, asset in assets.items():
        binary = asset["skeleton"].astype(np.uint8) * 255
        binary_path = raw / f"{condition}_skeleton.png"
        if not cv2.imwrite(str(binary_path), binary):
            raise OSError(f"Could not write {binary_path}")
        exported = cv2.imread(str(binary_path), cv2.IMREAD_GRAYSCALE)
        np.testing.assert_array_equal(exported, binary)
        for v, u in asset["skeleton_pixels"]:
            rows.append(dict(condition=condition, image_v=int(v), image_u=int(u)))
        save_native_overlay(raw / f"{condition}_skeleton_overlay.png", asset, "rgb")
        save_native_overlay(raw / f"{condition}_mask_skeleton_overlay.png", asset, "mask")
        condition_metadata[condition] = dict(
            source_array=f"source_data/{condition}_shape.npz",
            saved_skeleton_pixels=len(asset["skeleton_pixels"]),
            valid_depth_observed_points=len(asset["pixels"]),
            saved_skeleton_set_equals_observed_pixel_set=asset["sets_equal"],
            saved_skeleton_pixels_without_valid_depth=asset["skeleton_without_depth_count"],
            observed_pixels_outside_saved_skeleton=asset["observed_outside_skeleton_count"],
            first_observed_pixel_yx=asset["pixels"][0].tolist(),
            last_observed_pixel_yx=asset["pixels"][-1].tolist(),
            paths_base_to_tip=[dict(
                path_order=path["order"], raw_component_id=path["component_id"],
                observed_pixel_count=len(path["pixels"]), color=path["color"],
                first_observed_pixel_yx=path["pixels"][0].tolist(),
                last_observed_pixel_yx=path["pixels"][-1].tolist(),
            ) for path in asset["paths"]],
        )
    with (raw / PIXEL_CSV).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("condition", "image_v", "image_u"))
        writer.writeheader()
        writer.writerows(rows)
    comparison_figure(source, provenance, assets)
    after = protected_hashes(source)
    if before != after:
        changed = sorted(name for name in set(before) | set(after)
                         if before.get(name) != after.get(name))
        raise ValueError(f"Protected input or main-figure files changed: {changed}")
    outputs = [source / COMPARISON] + [raw / name for name in sorted(output_names())]
    metadata = dict(
        schema_version=1,
        dataset=provenance["dataset"], frame_id=provenance["frame_id"],
        source_mode="Frozen exported arrays and images only",
        input_sha256=before,
        input_hashes_unchanged_after_export=True,
        original_source_data_and_main_figures_untouched=True,
        model_inference_performed=False,
        skeletonization_performed=False,
        shape_estimation_performed=False,
        hidden_skeleton_pixels_inserted=0,
        cross_gap_lines_drawn=0,
        image_shape_hw=list(assets["clean"]["skeleton"].shape),
        pixel_coordinates="zero-based crop-local (image_v=row, image_u=column)",
        pixel_csv_order="condition order clean/thick30/multi3_30; then row-major skeleton support",
        pixel_csv_rows=len(rows),
        pixel_csv=f"source_data/{PIXEL_CSV}",
        skeleton_definition="Retained per-component longest weighted 8-connected paths saved as NPZ skeleton; not the unpruned thinning result",
        observation_definition="Saved ordered skeleton pixels that passed mask-aware depth validation",
        ordering="Stored observed_pixels_yx order; paths split whenever stored observed_component_ids changes",
        base_marker="First valid observed pixel only; not the lowest point of the entire mask or a calibrated physical base",
        binary_png_encoding="uint8 values 0 and 255; exact original one-pixel skeleton support; full source image dimensions",
        overlay_encoding="Full-size RGB/mask PNGs with enlarged point markers and a hollow base-side observation marker; display assets, not binary data",
        figure_display_viewport_xyxy=provenance["figure_display_viewport_xyxy"],
        figure_layout="2x2: clean mask, three-gap mask; clean RGB, three-gap RGB",
        plot_format="PNG only",
        conditions=condition_metadata,
        output_sha256={p.relative_to(source).as_posix(): sha256(p) for p in outputs},
    )
    (source / MANIFEST).write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(dict(comparison=str(source / COMPARISON),
                          skeleton_pixel_csv_rows=len(rows),
                          counts={c: len(a["skeleton_pixels"]) for c, a in assets.items()},
                          input_hashes_unchanged=True), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE,
                        help="Existing frozen segmentation figure directory")
    args = parser.parse_args()
    build(args.source.resolve())


if __name__ == "__main__":
    main()
