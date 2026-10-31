#!/usr/bin/env python3
"""Bundle frozen figure data for portable MATLAB plots; no ROS/model inference.

Requires openpyxl and NumPy only while building the bundle. End users need only
MATLAB and the bundled .m, .xlsx and .png files. Source CSV files are never edited.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

import numpy as np
from openpyxl import Workbook, load_workbook
from openpyxl.styles import Font, PatternFill
from openpyxl.utils import get_column_letter

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "docs/figures/segmentation_occlusion_20261028"
DEFAULT_DEST = ROOT / "docs/figures/segmentation_occlusion_matlab_20261028"
SHEETS = {
    "curves": "source_data/example_curves.csv",
    "observations": "source_data/example_observations.csv",
    "skeleton_pixels": "source_data/example_skeleton_pixels.csv",
    "fk_frames": "source_data/example_fk_frames.csv",
    "boundaries": "source_data/example_boundaries.csv",
    "example_angles": "source_data/example_joint_angles.csv",
    "gap_intervals": "source_data/example_gap_intervals.csv",
    "per_frame": "panel_D/source_data/panel_D_per_frame.csv",
    "summary": "panel_D/source_data/panel_D_summary.csv",
    "per_joint_summary": "panel_D/source_data/panel_D_per_joint_summary.csv",
    "joint_deltas": "panel_D/source_data/panel_D_per_frame_joint_deltas.csv",
}
TEXT_COLUMNS = {
    "dataset", "posture", "method", "condition", "frame", "frame_id", "axis",
    "joint_axis_name", "meaning", "reference_definition", "source_stamp_ns",
}
EXPECTED_ROWS = {
    "curves": 3003, "observations": 245, "skeleton_pixels": 245, "fk_frames": 60, "boundaries": 60,
    "example_angles": 54, "gap_intervals": 4, "per_frame": 104, "summary": 2,
    "per_joint_summary": 36, "joint_deltas": 1872,
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def is_text(column):
    return column in TEXT_COLUMNS or column.startswith(("source_", "unoccluded_reference_"))


def convert(column, value):
    if is_text(column):
        return value
    if value == "":
        return None
    if value in ("True", "False"):
        return value == "True"
    try:
        number = float(value)
    except ValueError:
        return value
    if not np.isfinite(number):
        raise ValueError(f"Non-finite value in {column}: {value}")
    return number


def style_sheet(sheet):
    sheet.freeze_panes = "A2"
    sheet.auto_filter.ref = sheet.dimensions
    for cell in sheet[1]:
        cell.font = Font(bold=True, color="FFFFFF")
        cell.fill = PatternFill("solid", fgColor="204967")
    for cells in sheet.iter_rows(min_row=2):
        for cell in cells:
            if isinstance(cell.value, str):
                cell.number_format = "@"
            elif isinstance(cell.value, (int, float)) and not isinstance(cell.value, bool):
                cell.number_format = "0.000000000"
    for index, cell in enumerate(sheet[1], 1):
        sheet.column_dimensions[get_column_letter(index)].width = min(42, max(16, len(str(cell.value)) + 2))
    sheet.sheet_view.zoomScale = 85


def check_metrics(tables):
    rows = tables["per_frame"]
    results = []
    for dataset, count in (("live_black", 40), ("archive_45deg_1", 64)):
        group = [row for row in rows if row["dataset"] == dataset]
        assert len(group) == count
        current = np.array([[row[f"occluded_q{i:02d}_rad"] for i in range(1, 19)] for row in group])
        reference = np.array([[row[f"unoccluded_q{i:02d}_rad"] for i in range(1, 19)] for row in group])
        delta = np.rad2deg(np.arctan2(np.sin(current - reference), np.cos(current - reference)))
        exported_delta = np.array([[row[f"q{i:02d}_wrapped_delta_deg"] for i in range(1, 19)] for row in group])
        np.testing.assert_allclose(delta, exported_delta, rtol=1e-12, atol=1e-12)
        q_rmse = np.sqrt(np.mean(delta ** 2, axis=1))
        np.testing.assert_allclose(q_rmse, [row["q_rmse_deg"] for row in group], rtol=1e-12, atol=1e-12)
        current_tip = np.array([[row[f"occluded_fk_tip_{axis}_m"] for axis in "xyz"] for row in group])
        reference_tip = np.array([[row[f"unoccluded_fk_tip_{axis}_m"] for axis in "xyz"] for row in group])
        position_delta = 1000 * np.linalg.norm(current_tip - reference_tip, axis=1)
        np.testing.assert_allclose(position_delta, [row["fk_tip_delta_mm"] for row in group], rtol=1e-12, atol=1e-12)
        summary = next(row for row in tables["summary"] if row["dataset"] == dataset)
        np.testing.assert_allclose(np.mean(q_rmse), summary["q_rmse_deg_mean"], rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(np.mean(position_delta), summary["fk_tip_delta_mm_mean"], rtol=1e-12, atol=1e-12)
        assert np.count_nonzero(position_delta <= 1) == summary["fk_tip_delta_le_1mm_count"]
        for joint in range(1, 19):
            stat = next(row for row in tables["per_joint_summary"] if row["dataset"] == dataset and row["joint_id"] == joint)
            assert stat["joint_axis_name"] == ("tilt" if joint % 2 else "pan")
            np.testing.assert_allclose(np.sqrt(np.mean(delta[:, joint - 1] ** 2)), stat["rmse_deg"], rtol=1e-12, atol=1e-12)
            np.testing.assert_allclose(np.mean(abs(delta[:, joint - 1])), stat["mae_deg"], rtol=1e-12, atol=1e-12)
        results.append({"dataset": dataset, "n": count, "mean_tip_difference_mm": float(np.mean(position_delta)),
                        "mean_frame_q18_rmse_deg": float(np.mean(q_rmse)),
                        "tip_difference_le_1mm_count": int(np.count_nonzero(position_delta <= 1))})
    return results


def check_skeleton_pixels(source, tables):
    """Confirm the extra sheet contains exact frozen skeleton pixels, not bridges."""
    counts = {}
    for condition, expected in (("clean", 146), ("thick30", 68), ("multi3_30", 31)):
        with np.load(source / "source_data" / f"{condition}_shape.npz", allow_pickle=False) as arrays:
            original = np.argwhere(arrays["skeleton"] > 0)
            exported = np.array([[row["image_v"], row["image_u"]]
                                 for row in tables["skeleton_pixels"] if row["condition"] == condition])
            np.testing.assert_array_equal(exported, original)
            assert len(original) == expected
            observed = arrays["observed_pixels_yx"]
            assert {tuple(row) for row in original} == {tuple(row) for row in observed}
        counts[condition] = expected
    return counts


def build(source, dest):
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "data").mkdir(exist_ok=True)
    (dest / "images").mkdir(exist_ok=True)
    workbook = Workbook()
    workbook.remove(workbook.active)
    source_hashes, tables, headers = {}, {}, {}
    for name, filename in SHEETS.items():
        path = source / filename
        source_hashes[filename] = sha256(path)
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            columns = reader.fieldnames
            rows = [{key: convert(key, value) for key, value in row.items()} for row in reader]
        assert len(rows) == EXPECTED_ROWS[name], (name, len(rows))
        headers[name], tables[name] = columns, rows
        sheet = workbook.create_sheet(name)
        sheet.append(columns)
        for row in rows:
            sheet.append([row[column] for column in columns])
        style_sheet(sheet)

    provenance = json.loads((source / "provenance.json").read_text())
    viewport = provenance["figure_display_viewport_xyxy"]
    metadata = {
        **dict(zip(("viewport_x0", "viewport_y0", "viewport_x1", "viewport_y1"), viewport)),
        "viewport_convention": "0-based pixel coordinates; upper endpoints exclusive",
        "selected_frame": provenance["frame_id"],
        "source_stamp_ns": str(provenance["source_stamp_ns"]),
        "clean_occlusion_percent": 0,
        "single_occlusion_percent": 100 * provenance["conditions"]["thick30"]["augmentation"]["actual_centerline_occlusion_fraction"],
        "multiple_occlusion_percent": 100 * provenance["conditions"]["multi3_30"]["augmentation"]["actual_centerline_occlusion_fraction"],
        "reference": provenance["reference"],
        "selection_rule": provenance["selection_rule"],
        "D_condition": "multi3_30",
        "D_method": "unet_v2",
        "coordinate_frame": "hrm_base",
        "coordinate_unit": "m (plot x 1000 = mm)",
        "joint_order": "odd=tilt; even=pan; 1-based 1..18",
        "curve_normalization": "main C is hardware-length normalized; C_raw is NOT scaled",
        "source_stamp_storage": "TEXT: do not convert nanosecond timestamps to Excel numbers",
        "n_definition": "frames from two sequences; not independent experimental repetitions",
        "known_depth_policy": "exclude known synthetic occluder pixels; not real-occluder rejection validation",
        "base_origin_policy": "same frame's clean base origin also used for occluded estimate",
        "temporal_filters": "off",
        "skeleton_definition": "Exact saved retained-path pixels; NOT synthetic gap-filling points",
        "skeleton_csv_order": "row-major image order; use observations.observed_index for base-to-tip order",
        "skeleton_path_labels": "P1..P4 denote visible path traversal order, NOT anatomical segment/joint IDs",
    }
    sheet = workbook.create_sheet("metadata")
    sheet.append(["key", "value"])
    for key, value in metadata.items():
        sheet.append([key, str(value)])
    style_sheet(sheet)
    dictionary = workbook.create_sheet("data_dictionary")
    dictionary.append(["sheet", "content", "units_and_notes"])
    descriptions = {
        "curves": ("3 conditions x 1001 fitted samples", "xyz in m; unscaled_xyz in m before hardware-length normalization"),
        "observations": ("Actual visible depth points and original pixel indices", "xyz in m; image_u/v in full crop pixels, 0-based"),
        "skeleton_pixels": ("Exact frozen retained skeleton pixels for 3 conditions", "image_v/u 0-based; row-major, NOT path traversal order; no inserted hidden pixels"),
        "fk_frames": ("3 conditions x (base + 18 joints + tip)", "xyz in m; row order follows kinematic chain"),
        "boundaries": ("20 curve boundary points per condition", "xyz in m"),
        "example_angles": ("18 joint angles in each of 3 conditions", "rad and deg; signed wrapped change vs clean"),
        "gap_intervals": ("Curve parameters between disconnected observed fragments", "s; NOT exact ground-truth occluder boundaries"),
        "per_frame": ("104 paired samples with angles, FK points and metrics", "q in rad; deltas in deg; FK coordinates m; position difference mm"),
        "summary": ("Straight/Bent aggregates for multi3_30", "mean frame RMSE differs from pooled RMSE; not independent n trials"),
        "per_joint_summary": ("18 joints x 2 cohorts", "rmse/mae/signed_mean/max_abs in deg"),
        "joint_deltas": ("104 frames x 18 joints", "signed wrapped differences in deg"),
        "metadata": ("All cells stored as text", "viewport 0-based; source_stamp_ns MUST stay text"),
    }
    for name, (description, units) in descriptions.items():
        dictionary.append([name, description, units])
    style_sheet(dictionary)
    xlsx = dest / "data/segmentation_figure_data.xlsx"
    workbook.save(xlsx)

    # Validate values actually read from Excel, not just the source arrays.
    read_back = load_workbook(xlsx, read_only=True, data_only=True)
    loaded_tables = {}
    largest_roundtrip_error = 0.0
    for name, rows in tables.items():
        sheet = read_back[name]
        cells = list(sheet.iter_rows(values_only=True))
        assert list(cells[0]) == headers[name] and len(cells) - 1 == len(rows)
        loaded_tables[name] = [dict(zip(headers[name], row)) for row in cells[1:]]
        for expected, actual in zip(rows, loaded_tables[name]):
            for column, value in expected.items():
                read_value = actual[column]
                if isinstance(value, (str, bool)) or value is None:
                    assert read_value == value, (name, column, value, read_value)
                else:
                    error = abs(value - read_value)
                    largest_roundtrip_error = max(largest_roundtrip_error, error)
                    assert np.isclose(value, read_value, rtol=1e-14, atol=1e-12), (name, column, error)
    stamp_rows = [row for row in read_back["metadata"].iter_rows(min_row=2) if row[0].value == "source_stamp_ns"]
    assert stamp_rows[0][1].data_type == "s"
    assert stamp_rows[0][1].value == str(provenance["source_stamp_ns"])
    metrics = check_metrics(loaded_tables)
    skeleton_counts = check_skeleton_pixels(source, loaded_tables)
    read_back.close()

    copied = []
    for condition in ("clean", "thick30", "multi3_30"):
        for suffix in ("rgb", "unet_mask", "known_occluder", "skeleton", "skeleton_overlay", "mask_skeleton_overlay"):
            source_file = source / "source_data" / f"{condition}_{suffix}.png"
            target_file = dest / "images" / source_file.name
            shutil.copy2(source_file, target_file)
            assert sha256(target_file) == sha256(source_file)
            source_hashes[str(source_file.relative_to(source))] = sha256(source_file)
            copied.append(str(target_file.relative_to(dest)))
    shutil.copy2(source / "figure_segmentation_ABCD.png", dest / "preview_reference.png")
    shutil.copy2(source / "B_visible_body_skeleton.png", dest / "preview_skeleton.png")
    shutil.copy2(source / "skeleton_visualization.json", dest / "data/skeleton_visualization.json")
    shutil.copy2(source / "caption.tex", dest / "caption.tex")
    shutil.copy2(source / "provenance.json", dest / "data/source_provenance.json")
    for relative, expected_hash in source_hashes.items():
        assert sha256(source / relative) == expected_hash, f"Source unexpectedly changed: {relative}"
    report = {
        "description": "Excel/image export verification; MATLAB rendering NOT executed in this environment",
        "workbook": "data/segmentation_figure_data.xlsx", "workbook_sha256": sha256(xlsx),
        "sheets_rows": EXPECTED_ROWS, "copied_images": copied,
        "numeric_roundtrip_max_abs_error": largest_roundtrip_error,
        "timestamp_text_preserved": True, "metrics_recomputed_from_excel": metrics,
        "frozen_skeleton_pixel_counts": skeleton_counts,
        "sources_unchanged": True, "source_file_sha256": source_hashes,
        "preview_note": "preview_reference.png and preview_skeleton.png are Python figures, NOT MATLAB renders",
    }
    (dest / "data/export_validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"folder": str(dest), "workbook": str(xlsx), "rows": EXPECTED_ROWS,
                      "metrics": metrics, "numeric_roundtrip_max_abs_error": largest_roundtrip_error}, indent=2))


def archive_bundle(dest):
    """Zip the copyable package without generated MATLAB outputs/caches."""
    required = ["README_KO.md", "make_segmentation_figures.m"]
    assert all((dest / name).is_file() for name in required), "MATLAB code or README is missing"
    assert sorted(path.name for path in dest.glob("*.m")) == ["make_segmentation_figures.m"], \
        "The portable bundle must contain exactly one MATLAB source file"
    files = sorted(path for path in dest.rglob("*") if path.is_file() and not any(
        part in ("output", "__pycache__", ".DS_Store") or part.startswith("output_")
        for part in path.relative_to(dest).parts))
    archive = dest.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as stream:
        for path in files:
            stream.write(path, str(Path(dest.name) / path.relative_to(dest)))
    with zipfile.ZipFile(archive) as stream:
        assert stream.testzip() is None
        assert len(stream.namelist()) == len(files)
        for path in files:
            entry = str(Path(dest.name) / path.relative_to(dest))
            assert hashlib.sha256(stream.read(entry)).hexdigest() == sha256(path)
    print(f"ZIP verified: {archive} ({archive.stat().st_size:,} bytes; {len(files)} files)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_DEST)
    parser.add_argument("--archive", action="store_true", help="Also ZIP the complete MATLAB folder beside it")
    args = parser.parse_args()
    build(args.source.resolve(), args.output.resolve())
    if args.archive:
        archive_bundle(args.output.resolve())
