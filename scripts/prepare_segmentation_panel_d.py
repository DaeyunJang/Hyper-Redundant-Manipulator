#!/usr/bin/env python3
"""Export panel D from frozen paired pose artifacts; never run model inference.

Metrics compare U-Net-v2 under multi3_30 synthetic occlusion to that method's
unoccluded estimate on the same source frame. They are sensitivity measurements,
not errors against ground truth. Run from any directory with Python + NumPy and
Matplotlib; source and destination have repository-relative defaults.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    ROOT.parent / "Hyper-Redundant-Manipulator-Segmentation"
    / "runs/occlusion_pose_20261024/pose_evaluation_short_fragments"
)
DEFAULT_OUTPUT = ROOT / "docs/figures/segmentation_occlusion_20261028/panel_D"
DATASETS = {
    "live_black": {"label": "Straight", "n": 40, "color": "#0072B2"},
    "archive_45deg_1": {"label": "Bent", "n": 64, "color": "#D55E00"},
}
METHOD, CONDITION = "unet_v2", "multi3_30"
METRICS = (
    "fk_tip_delta_mm", "q_rmse_deg", "q_p95_abs_deg", "q_max_abs_deg",
    "fk_orientation_delta_deg", "curve_tip_delta_mm",
)
PAIRED_KEYS = (
    "q_rad", "fk_tip_xyz_m", "fk_tip_rotation", "curve_tip_xyz_m",
    "curve_xyz_m", "base_origin_camera_xyz",
)
NOTES = [
    "Reference: the same method's unoccluded estimate on the same source frame; not ground truth.",
    "Straight is live_black (40 frames); bent is archive_45deg_1 (64 frames). These frames are not independent experimental repetitions.",
    "Each plotted angle value is sqrt(mean_j(wrapped_delta_q_deg^2)) over 18 joints. Its annotated mean averages the per-frame RMSEs.",
    "Pooled q18 RMSE is sqrt(mean_t,j(wrapped_delta_q_deg^2)); it differs from the mean of per-frame RMSEs and is separately exported.",
    "All 104 requested frame pairs are valid and shown. Boxplots use Q1/Q3, median, and 1.5-IQR whiskers; scatter includes every frame and boxplot fliers are disabled to prevent duplication.",
    "The 1 mm line is a descriptive sensitivity reference, not a measured accuracy specification or acceptance threshold.",
    "Synthetic occlusion uses known missing-depth pixels and each frame's clean base calibration. It does not validate rejection of a real occluder's valid depth.",
    "The short-fragment frontend was introduced after inspecting minimum8 failures; report this as exploratory sensitivity analysis.",
]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def recompute(arrays: dict) -> tuple[dict, np.ndarray]:
    current, reference = arrays["q_rad"], arrays["clean_q_rad"]
    if current.shape != (18,) or reference.shape != (18,):
        raise ValueError("Expected exactly 18 joint angles in each paired array.")
    delta = np.rad2deg(np.arctan2(np.sin(current - reference), np.cos(current - reference)))
    rotation, clean_rotation = arrays["fk_tip_rotation"], arrays["clean_fk_tip_rotation"]
    for matrix in (rotation, clean_rotation):
        if matrix.shape != (3, 3):
            raise ValueError("Expected 3 by 3 rotation matrix.")
        np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-8)
        np.testing.assert_allclose(np.linalg.det(matrix), 1.0, atol=1e-8)
    orientation = 0.0 if np.allclose(rotation, clean_rotation, rtol=0, atol=1e-12) else float(
        np.rad2deg(np.arccos(np.clip((np.trace(clean_rotation.T @ rotation) - 1) / 2, -1, 1)))
    )
    values = {
        "fk_tip_delta_mm": float(np.linalg.norm(arrays["fk_tip_xyz_m"] - arrays["clean_fk_tip_xyz_m"]) * 1000),
        "q_rmse_deg": float(np.sqrt(np.mean(delta ** 2))),
        "q_p95_abs_deg": float(np.percentile(np.abs(delta), 95)),
        "q_max_abs_deg": float(np.max(np.abs(delta))),
        "fk_orientation_delta_deg": orientation,
        "curve_tip_delta_mm": float(np.linalg.norm(arrays["curve_tip_xyz_m"] - arrays["clean_curve_tip_xyz_m"]) * 1000),
    }
    if not np.isfinite(list(values.values())).all() or not np.isfinite(delta).all():
        raise ValueError("Nonfinite paired metrics.")
    return values, delta


def collect(source: Path) -> tuple[list, list, list, dict, dict]:
    sources = {name: digest(source / name) for name in (
        "per_frame.csv", "summary.csv", "per_joint_delta.csv", "protocol.json", "independent_audit.json"
    )}
    protocol = json.loads((source / "protocol.json").read_text())
    records = {(r["dataset"], r["frame_id"]): r for r in protocol["records"]}
    original = [r for r in read_csv(source / "per_frame.csv") if (
        r["dataset"] in DATASETS and r["method"] == METHOD and r["condition"] == CONDITION
    )]
    original.sort(key=lambda r: (list(DATASETS).index(r["dataset"]), int(r["source_stamp_ns"])))
    if len(original) != 104 or len({(r["dataset"], r["frame_id"]) for r in original}) != 104:
        raise ValueError("Expected 104 unique frame pairs.")
    joint_original = {
        (r["dataset"], r["frame_id"], int(r["joint_id"])): r
        for r in read_csv(source / "per_joint_delta.csv")
        if r["dataset"] in DATASETS and r["method"] == METHOD and r["condition"] == CONDITION
    }
    rows, joint_rows = [], []
    max_difference = {metric: 0.0 for metric in METRICS}
    max_joint_difference = 0.0
    for original_row in original:
        dataset, frame_id = original_row["dataset"], original_row["frame_id"]
        if any(original_row[field] != "True" for field in (
            "generation_success", "shape_success", "clean_shape_success", "paired_valid"
        )):
            raise ValueError(f"Unexpected failed requested pair: {dataset}/{frame_id}")
        record = records[dataset, frame_id]
        stamp = int(original_row["source_stamp_ns"])
        if stamp != int(record["source_stamp_ns"]):
            raise ValueError(f"Source timestamp mismatch: {frame_id}")
        relative_path = Path(original_row["shape_npz"])
        expected_path = Path("shape_arrays") / dataset / CONDITION / METHOD / f"{frame_id}.npz"
        if relative_path != expected_path:
            raise ValueError(f"Unexpected paired NPZ path: {relative_path}")
        clean_path = Path("shape_arrays") / dataset / "clean" / METHOD / f"{frame_id}.npz"
        for path in (relative_path, clean_path):
            sources[str(path)] = digest(source / path)
        with np.load(source / relative_path, allow_pickle=False) as npz:
            arrays = {key: npz[key] for key in npz.files}
        with np.load(source / clean_path, allow_pickle=False) as clean:
            for key in PAIRED_KEYS:
                np.testing.assert_array_equal(arrays[f"clean_{key}"], clean[key])
        np.testing.assert_array_equal(arrays["base_origin_camera_xyz"], arrays["clean_base_origin_camera_xyz"])
        values, delta = recompute(arrays)
        for metric, value in values.items():
            difference = abs(value - float(original_row[metric]))
            max_difference[metric] = max(max_difference[metric], difference)
            np.testing.assert_allclose(value, float(original_row[metric]), rtol=1e-10, atol=1e-10)
        row = {
            "dataset": dataset, "posture": DATASETS[dataset]["label"],
            "method": METHOD, "condition": CONDITION, "frame_id": frame_id,
            "source_stamp_ns": stamp, "paired_valid": True,
            "source_rgb": record["rgb"], "source_depth": record["depth"],
            "source_npz": str(relative_path), "source_npz_sha256": sources[str(relative_path)],
            "unoccluded_reference_npz": str(clean_path),
            "unoccluded_reference_npz_sha256": sources[str(clean_path)],
            "reference_definition": "same method, same source frame, unoccluded estimate",
            **values, "fk_tip_delta_le_1mm": values["fk_tip_delta_mm"] <= 1.0,
            "placement_reference_actual_occlusion": float(original_row["placement_reference_actual_occlusion"]),
        }
        for state, prefix in (("occluded", ""), ("unoccluded", "clean_")):
            for index, axis in enumerate("xyz"):
                row[f"{state}_fk_tip_{axis}_m"] = float(arrays[f"{prefix}fk_tip_xyz_m"][index])
            for index in range(18):
                row[f"{state}_q{index + 1:02d}_rad"] = float(arrays[f"{prefix}q_rad"][index])
        for index, value in enumerate(delta, start=1):
            row[f"q{index:02d}_wrapped_delta_deg"] = float(value)
            original_joint = joint_original[dataset, frame_id, index]
            joint_difference = abs(float(value) - float(original_joint["wrapped_delta_deg"]))
            max_joint_difference = max(max_joint_difference, joint_difference)
            np.testing.assert_allclose(value, float(original_joint["wrapped_delta_deg"]), rtol=1e-10, atol=1e-10)
            joint_rows.append({
                "dataset": dataset, "posture": DATASETS[dataset]["label"],
                "method": METHOD, "condition": CONDITION,
                "frame_id": frame_id, "source_stamp_ns": stamp,
                "joint_id": index, "joint_axis_name": original_joint["joint_axis_name"],
                "unoccluded_q_rad": float(arrays["clean_q_rad"][index - 1]),
                "occluded_q_rad": float(arrays["q_rad"][index - 1]),
                "wrapped_delta_deg": float(value),
            })
        rows.append(row)
    summary, per_joint = [], []
    original_summary = {(r["dataset"], r["method"], r["condition"]): r for r in read_csv(source / "summary.csv")}
    for dataset, config in DATASETS.items():
        selected = [row for row in rows if row["dataset"] == dataset]
        if len(selected) != config["n"]:
            raise ValueError(f"Wrong frame count for {dataset}.")
        count_le_1 = sum(row["fk_tip_delta_le_1mm"] for row in selected)
        entry = {
            "dataset": dataset, "posture": config["label"], "method": METHOD, "condition": CONDITION,
            "requested_frames": len(selected), "paired_valid_frames": len(selected), "invalid_frames": 0,
            "fk_tip_delta_le_1mm_count": count_le_1,
            "fk_tip_delta_le_1mm_percent": 100 * count_le_1 / len(selected),
        }
        for metric in METRICS:
            values = np.array([row[metric] for row in selected])
            for stat, function in (
                ("mean", np.mean), ("median", np.median), ("p95", lambda a: np.percentile(a, 95)),
                ("max", np.max), ("min", np.min), ("q1", lambda a: np.percentile(a, 25)),
                ("q3", lambda a: np.percentile(a, 75)),
            ):
                key = f"{metric}_{stat}"
                entry[key] = float(function(values))
                if stat in ("mean", "median", "p95", "max"):
                    np.testing.assert_allclose(entry[key], float(original_summary[dataset, METHOD, CONDITION][key]), rtol=1e-10, atol=1e-10)
        deltas = np.array([[row[f"q{j:02d}_wrapped_delta_deg"] for j in range(1, 19)] for row in selected])
        entry["q18_pooled_rmse_deg"] = float(np.sqrt(np.mean(deltas ** 2)))
        entry["mean_frame_q18_rmse_deg"] = entry["q_rmse_deg_mean"]
        summary.append(entry)
        for index in range(18):
            values = deltas[:, index]
            per_joint.append({
                "dataset": dataset, "posture": config["label"], "method": METHOD,
                "condition": CONDITION, "n_frames": len(selected), "joint_id": index + 1,
                "joint_axis_name": joint_original[dataset, selected[0]["frame_id"], index + 1]["joint_axis_name"],
                "rmse_deg": float(np.sqrt(np.mean(values ** 2))),
                "mae_deg": float(np.mean(np.abs(values))),
                "signed_mean_delta_deg": float(np.mean(values)),
                "max_abs_delta_deg": float(np.max(np.abs(values))),
            })
    expected = ((0.413, 1.265, 95.0), (0.554, 1.477, 90.625))
    for row, (tip, angle, fraction) in zip(summary, expected):
        if round(row["fk_tip_delta_mm_mean"], 3) != tip or round(row["q_rmse_deg_mean"], 3) != angle:
            raise ValueError("Recomputed means disagree with requested figure annotations.")
        np.testing.assert_allclose(row["fk_tip_delta_le_1mm_percent"], fraction, rtol=0, atol=1e-12)
    checks = {
        "status": "PASS", "requested_frames": 104, "valid_paired_frames": 104,
        "recomputed_metric_values": 104 * len(METRICS),
        "recomputed_joint_deltas": len(joint_rows), "separate_clean_npz_pairs_checked": 104,
        "max_absolute_difference_from_original_per_frame_csv": max_difference,
        "max_absolute_joint_delta_difference_from_original_csv_deg": max_joint_difference,
        "source_summary_statistics_match": True, "requested_rounded_annotations_match": True,
        "source_timestamp_and_frame_identity_match": True, "fixed_clean_base_matches": True,
    }
    return rows, summary, per_joint, {"joint_rows": joint_rows, "sources": sources}, checks


def configure_style() -> None:
    matplotlib.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 8.5, "axes.titlesize": 10,
        "axes.labelsize": 9, "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.7, "xtick.labelsize": 8.5, "ytick.labelsize": 8,
        "svg.fonttype": "none", "pdf.fonttype": 42, "ps.fonttype": 42,
        "savefig.facecolor": "white", "figure.facecolor": "white",
    })


def save_formats(fig, directory: Path, stem: str) -> list[Path]:
    paths = []
    for suffix in ("svg", "pdf", "png"):
        path = directory / f"{stem}.{suffix}"
        fig.savefig(path, dpi=600, facecolor="white")
        paths.append(path)
    plt.close(fig)
    return paths


def draw_panel_d(axes, rows: list, *, title_pad: float = 27) -> None:
    """Draw vector chart content into two existing axes for manuscript assembly.

    Rows may be either collect(source)[0] or csv.DictReader output from the
    exported per-frame CSV. The caller owns figure sizing and explanatory text.
    """
    settings = (
        ("fk_tip_delta_mm", "FK tip-position difference", "Position difference (mm)", 3.0, "mm"),
        ("q_rmse_deg", "18-joint angle difference", "Per-frame joint RMSE (°)", 5.9, "°"),
    )
    for ax, (metric, title, ylabel, upper, unit) in zip(axes, settings):
        ax.set_title(title, pad=title_pad, fontweight="bold")
        for position, (dataset, config) in enumerate(DATASETS.items(), start=1):
            values = np.array([float(row[metric]) for row in rows if row["dataset"] == dataset])
            color = config["color"]
            box = ax.boxplot([values], positions=[position], widths=0.48, patch_artist=True,
                             showfliers=False, whis=1.5, medianprops={"color": "#222222", "linewidth": 1.2},
                             whiskerprops={"color": color, "linewidth": 1.1}, capprops={"color": color, "linewidth": 1.1})
            box["boxes"][0].set(facecolor=color, edgecolor=color, linewidth=1.2, alpha=0.17)
            jitter = np.random.default_rng(104 + position).uniform(-0.16, 0.16, len(values))
            ax.scatter(position + jitter, values, s=13, color=color, alpha=0.78,
                       linewidths=0.25, edgecolors="white", zorder=3)
            mean = float(np.mean(values))
            ax.scatter(position, mean, marker="D", s=29, facecolors="white", edgecolors="#222222",
                       linewidths=0.9, zorder=4)
            ax.text(position, 1.035, f"Mean {mean:.3f}{unit if unit == '°' else ' ' + unit}",
                    transform=ax.get_xaxis_transform(), ha="center", va="bottom", color=color,
                    fontsize=9, fontweight="bold")
        ax.set_xlim(0.45, 2.55)
        ax.set_ylim(-upper * 0.015, upper)
        ax.set_xticks([1, 2], [f"{c['label']}\n(n = {c['n']})" for c in DATASETS.values()])
        ax.set_ylabel(ylabel)
        ax.set_yticks(np.arange(0, upper + 0.001, 1))
        ax.yaxis.grid(True, color="#e5e7eb", linewidth=0.55, zorder=0)
        ax.tick_params(axis="x", length=0)
        ax.tick_params(axis="y", width=0.6, length=3)
    axes[0].axhline(1.0, color="#6b7280", linestyle=(0, (4, 3)), linewidth=0.85, zorder=2)
    axes[0].text(0.98, 1.0, "1 mm", transform=axes[0].get_yaxis_transform(),
                 ha="right", va="bottom", color="#5a6068", fontsize=7.5,
                 bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.5})


def plot_panel(rows: list, summary: list, output: Path) -> list[Path]:
    configure_style()
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.8))
    fig.subplots_adjust(left=0.085, right=0.985, top=0.80, bottom=0.245, wspace=0.36)
    fig.suptitle("Three synthetic occluders (~30% total) · U-Net-v2", y=0.985, fontsize=11, fontweight="bold")
    draw_panel_d(axes, rows)
    fig.text(0.085, 0.127, "≤1 mm: Straight 38/40 (95%); Bent 58/64 (90.625%)", fontsize=8.4)
    fig.text(0.085, 0.073, "Points: frames; boxes: median / IQR; diamonds: means. All frame values shown.", fontsize=7.6, color="#444444")
    fig.text(0.085, 0.027, "Reference: same method, same frame, unoccluded estimate (not ground truth).", fontsize=7.6, color="#444444")
    return save_formats(fig, output, "panel_D_quantitative")


def plot_per_joint(per_joint: list, output: Path) -> list[Path]:
    configure_style()
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.1))
    fig.subplots_adjust(left=0.08, right=0.985, top=0.79, bottom=0.25, wspace=0.28)
    for ax, metric, title in zip(axes, ("rmse_deg", "mae_deg"), ("Per-joint RMSE", "Per-joint MAE")):
        for dataset, config in DATASETS.items():
            selected = [row for row in per_joint if row["dataset"] == dataset]
            ax.plot([r["joint_id"] for r in selected], [r[metric] for r in selected],
                    color=config["color"], marker="o", markersize=3, linewidth=1.1,
                    label=f"{config['label']} (n = {config['n']})")
        ax.set(title=title, xlabel="Joint index", ylabel="Angle difference (°)", xlim=(0.5, 18.5), ylim=(0, None))
        ax.set_xticks([1, 3, 6, 9, 12, 15, 18])
        ax.grid(axis="y", color="#e5e7eb", linewidth=0.6)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.text(0.08, 0.075, "U-Net-v2 · multi3_30 · 18 wrapped joint-angle differences, aggregated across frames.", fontsize=7.7)
    fig.text(0.08, 0.025, "Reference: same method, same frame, unoccluded estimate (not ground truth).", fontsize=7.7)
    return save_formats(fig, output, "supplement_per_joint")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if output == source or source in output.parents:
        raise ValueError("Output must not modify the frozen source tree.")
    rows, summary, per_joint, auxiliary, checks = collect(source)
    output.mkdir(parents=True, exist_ok=True)
    data_dir = output / "source_data"
    data_dir.mkdir(exist_ok=True)
    data_paths = []
    for name, entries in (
        ("panel_D_per_frame.csv", rows), ("panel_D_summary.csv", summary),
        ("panel_D_per_joint_summary.csv", per_joint),
        ("panel_D_per_frame_joint_deltas.csv", auxiliary["joint_rows"]),
    ):
        path = data_dir / name
        write_csv(path, entries)
        data_paths.append(path)
    plot_paths = plot_panel(rows, summary, output) + plot_per_joint(per_joint, output)
    for name, before in auxiliary["sources"].items():
        if digest(source / name) != before:
            raise ValueError(f"Frozen source changed during export: {name}")
    checks["source_hashes_unchanged_after_export"] = True
    checks["source_files_verified"] = len(auxiliary["sources"])
    manifest = {
        "generator": str(Path(__file__).resolve()), "source_root": str(source),
        "output_root": str(output), "method": METHOD, "condition": CONDITION,
        "figure_size_inches": [7.2, 3.8], "png_dpi": 600,
        "svg_text_editable": True, "pdf_fonts_embedded_type": 42,
        "color_palette": {k: v["color"] for k, v in DATASETS.items()},
        "reference": "Same method's unoccluded estimate on the same source frame",
        "notes": NOTES, "summary": summary, "validation": checks,
        "source_sha256": auxiliary["sources"],
        "files": [{"path": str(path), "relative_path": str(path.relative_to(output)),
                   "sha256": digest(path), "bytes": path.stat().st_size} for path in plot_paths + data_paths],
        "metric_formulas": {
            "wrapped_delta_deg": "rad2deg(atan2(sin(q_occluded - q_unoccluded), cos(q_occluded - q_unoccluded)))",
            "fk_tip_delta_mm": "1000 * norm(p_FK_occluded - p_FK_unoccluded)",
            "q_rmse_deg": "sqrt(mean_j(wrapped_delta_deg ** 2)) for 18 joints, separately for each frame",
            "mean_frame_q18_rmse_deg": "mean_t(sqrt(mean_j(wrapped_delta_deg ** 2)))",
            "q18_pooled_rmse_deg": "sqrt(mean_t,j(wrapped_delta_deg ** 2))",
            "per_joint_rmse_deg": "sqrt(mean_t(wrapped_delta_deg ** 2)) for each joint separately",
            "per_joint_mae_deg": "mean_t(abs(wrapped_delta_deg)) for each joint separately",
            "fk_orientation_delta_deg": "rad2deg(acos(clip((trace(R_unoccluded.T @ R_occluded)-1)/2, -1, 1)))",
            "curve_tip_delta_mm": "1000 * norm(p_curve_occluded - p_curve_unoccluded)",
            "p95": "numpy.percentile(values, 95), default linear interpolation",
        },
    }
    manifest_path = output / "panel_D_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), "validation": checks,
                      "means": [{k: row[k] for k in ("dataset", "fk_tip_delta_mm_mean", "mean_frame_q18_rmse_deg", "q18_pooled_rmse_deg", "fk_tip_delta_le_1mm_percent")} for row in summary]}, indent=2))


if __name__ == "__main__":
    main()
