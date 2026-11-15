#!/usr/bin/env python3
"""
Generate the detector experiment accuracy report.

Reads results.csv and errors.csv written by exp_detector, and writes REPORT.rst
and figures to the report directory.

Usage: make_report.py <exp_detector_output_dir> <config.yaml> <report_dir>
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

# Fixed colour per target (first six slots of the reference categorical
# palette, in order). Colour follows the target, never its rank.
TARGETS = [
    ("aprilgrid_kalibr_kaess", "AprilGrid kalibr (kaess)", "#2a78d6"),
    ("aprilgrid_plain_kaess", "AprilGrid plain (kaess)", "#eb6834"),
    ("aprilgrid_kalibr_olsen", "AprilGrid kalibr (olsen)", "#1baf7a"),
    ("aprilgrid_plain_olsen", "AprilGrid plain (olsen)", "#eda100"),
    ("checkerboard", "Checkerboard", "#e87ba4"),
    ("charuco", "ChArUco", "#008300"),
]
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"

TILT_EDGES = [0, 15, 30, 45, 60, 75]
DIST_EDGES = [0.8, 1.2, 1.6, 2.0, 2.5]


def style_axes(ax, xlabel, ylabel, title):
  ax.set_facecolor(SURFACE)
  ax.set_title(title, loc="left", fontsize=11, color=INK, fontweight="bold")
  ax.set_xlabel(xlabel, color=INK2, fontsize=9)
  ax.set_ylabel(ylabel, color=INK2, fontsize=9)
  ax.tick_params(colors=INK2, labelsize=8)
  ax.grid(True, color=GRID, linewidth=0.6)
  ax.set_axisbelow(True)
  for side in ("top", "right"):
    ax.spines[side].set_visible(False)
  for side in ("left", "bottom"):
    ax.spines[side].set_color(GRID)


def legend_below(fig, ax):
  handles, labels = ax.get_legend_handles_labels()
  fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False,
             fontsize=8, labelcolor=INK2)


def save(fig, path):
  fig.patch.set_facecolor(SURFACE)
  fig.savefig(path, dpi=300, bbox_inches="tight", facecolor=SURFACE)
  plt.close(fig)


def corner_stats(errors):
  """Accuracy statistics from per-corner errors (px)"""
  e = errors[["err_x_px", "err_y_px"]].to_numpy()
  mag = np.linalg.norm(e, axis=1)
  bias = e.mean(axis=0)
  centered = e - bias
  return {
      "n": len(e),
      "rmse": np.sqrt((mag**2).mean()),
      "bias": np.linalg.norm(bias),
      "precision": np.sqrt((np.linalg.norm(centered, axis=1)**2).mean()),
      "p50": np.percentile(mag, 50),
      "p95": np.percentile(mag, 95),
      "p99": np.percentile(mag, 99),
  }


def fig_error_cdf(errors, path):
  fig, ax = plt.subplots(figsize=(7.5, 4.2))
  for key, label, color in TARGETS:
    e = errors[errors.target == key]
    if e.empty:
      continue
    mag = np.sort(np.linalg.norm(e[["err_x_px", "err_y_px"]], axis=1))
    cdf = np.arange(1, len(mag) + 1) / len(mag)
    ax.plot(mag, cdf, color=color, linewidth=2, label=label)
  ax.set_xscale("log")
  ax.set_xlim(0.01, 5)
  ax.set_ylim(0, 1.0)
  style_axes(ax, "Corner error [px, log scale]", "Fraction of corners",
             "Corner error distribution (cumulative)")
  fig.subplots_adjust(bottom=0.28)
  legend_below(fig, ax)
  save(fig, path)


def binned_rmse(errors, col, edges):
  centers, out = [], {}
  for key, _, _ in TARGETS:
    e = errors[errors.target == key]
    vals = []
    for lo, hi in zip(edges[:-1], edges[1:]):
      b = e[(e[col] >= lo) & (e[col] < hi)]
      mag2 = (b.err_x_px**2 + b.err_y_px**2)
      vals.append(np.sqrt(mag2.mean()) if len(b) >= 30 else np.nan)
    out[key] = vals
  centers = [(lo + hi) / 2 for lo, hi in zip(edges[:-1], edges[1:])]
  return centers, out


def fig_error_vs(errors, col, edges, xlabel, title, path):
  centers, out = binned_rmse(errors, col, edges)
  fig, ax = plt.subplots(figsize=(7.5, 4.2))
  for key, label, color in TARGETS:
    ax.plot(centers, out[key], color=color, linewidth=2, marker="o",
            markersize=5, markeredgecolor=SURFACE, markeredgewidth=1.5,
            label=label)
  ax.set_yscale("log")
  style_axes(ax, xlabel, "RMSE [px, log scale]", title)
  fig.subplots_adjust(bottom=0.28)
  legend_below(fig, ax)
  save(fig, path)


def fig_recall_vs_tilt(results, path):
  fig, ax = plt.subplots(figsize=(7.5, 4.2))
  centers = [(lo + hi) / 2 for lo, hi in zip(TILT_EDGES[:-1], TILT_EDGES[1:])]
  for key, label, color in TARGETS:
    r = results[(results.target == key) & (results.num_visible > 0)]
    vals = []
    for lo, hi in zip(TILT_EDGES[:-1], TILT_EDGES[1:]):
      b = r[(r.tilt_deg >= lo) & (r.tilt_deg < hi)]
      inliers = b.num_detected - b.num_outliers
      vals.append(100.0 * (inliers / b.num_visible).mean() if len(b) else np.nan)
    ax.plot(centers, vals, color=color, linewidth=2, marker="o", markersize=5,
            markeredgecolor=SURFACE, markeredgewidth=1.5, label=label)
  ax.set_ylim(0, 100)
  style_axes(ax, "Tilt: angle between optical axis and board normal [deg]",
             "Corner recall [%]", "Visible corners detected, by viewing angle")
  fig.subplots_adjust(bottom=0.28)
  legend_below(fig, ax)
  save(fig, path)


def fig_time(results, path):
  fig, ax = plt.subplots(figsize=(7.5, 3.6))
  labels, med, p95, colors = [], [], [], []
  for key, label, color in TARGETS:
    r = results[(results.target == key) & (results.num_detected > 0)]
    labels.append(label)
    med.append(r.detect_ms.median())
    p95.append(r.detect_ms.quantile(0.95))
    colors.append(color)
  y = np.arange(len(labels))[::-1]
  ax.barh(y, med, color=colors, height=0.55)
  for yi, m, p in zip(y, med, p95):
    ax.plot([m, p], [yi, yi], color=INK2, linewidth=1)
    ax.plot([p, p], [yi - 0.12, yi + 0.12], color=INK2, linewidth=1)
    ax.text(p + 1.5, yi, f"{m:.1f} ms (p95 {p:.1f})", va="center", fontsize=8,
            color=INK)
  ax.set_yticks(y)
  ax.set_yticklabels(labels, fontsize=8, color=INK2)
  ax.set_xlim(0, max(p95) * 1.45)
  style_axes(ax, "Detection time per image [ms]", "",
             "Detection time (frames with a detection)")
  ax.grid(False, axis="y")
  save(fig, path)


def number(cell):
  """Value of a table cell, None if it is not a number"""
  try:
    v = float(str(cell).replace(",", ""))
  except ValueError:
    return None
  return v if np.isfinite(v) else None


def bold_best(rows, col, best):
  """Bold the best value ("min" or "max") of a column, ties included"""
  values = [number(r[col]) for r in rows]
  numbers = [v for v in values if v is not None]
  if not numbers:
    return
  target = min(numbers) if best == "min" else max(numbers)
  for row, v in zip(rows, values):
    if v is not None and v == target:
      row[col] = f"**{row[col]}**"


def binned_table(errors, col, edges, unit):
  _, out = binned_rmse(errors, col, edges)
  header = ["Target"] + [f"{lo:g}-{hi:g} {unit}"
                         for lo, hi in zip(edges[:-1], edges[1:])]
  rows = []
  for key, label, _ in TARGETS:
    rows.append([label] + ["-" if np.isnan(v) else f"{v:.3f}" for v in out[key]])
  for c in range(1, len(header)):
    bold_best(rows, c, "min")
  return rst_table(header, rows)


def rst_table(header, rows, widths=None):
  out = [".. list-table::", "   :header-rows: 1", "   :align: center",
         "   :width: 80%"]
  if widths:
    out.append("   :widths: " + " ".join(str(w) for w in widths))
  out.append("")
  for row in [header] + rows:
    for j, cell in enumerate(row):
      out.append(("   * - " if j == 0 else "     - ") + str(cell))
  return "\n".join(out)


def figure(path, caption):
  return "\n".join([f".. figure:: {path}", "   :width: 80%", "   :align: center",
                    "", f"   {caption}", ""])


def refine_text(cfg):
  r = cfg["refine_corners"]
  on = [n for n, v in (("AprilGrid", r["aprilgrid"]),
                       ("checkerboard", r["checkerboard"]),
                       ("ChArUco", r["charuco"])) if v]
  return "cv::cornerSubPix for " + ", ".join(on) if on else "off"


def main():
  out_dir, config_file, report_dir = map(Path, sys.argv[1:4])
  report_dir.mkdir(parents=True, exist_ok=True)
  results = pd.read_csv(out_dir / "results.csv")
  errors = pd.read_csv(out_dir / "errors.csv")
  cfg = yaml.safe_load(open(config_file))

  fig_error_cdf(errors, report_dir / "error_cdf.png")
  fig_error_vs(errors, "dist_m", DIST_EDGES, "Distance to board center [m]",
               "Accuracy by distance", report_dir / "rmse_vs_distance.png")
  fig_error_vs(errors, "tilt_deg", TILT_EDGES,
               "Tilt: angle between optical axis and board normal [deg]",
               "Accuracy by viewing angle", report_dir / "rmse_vs_tilt.png")
  fig_recall_vs_tilt(results, report_dir / "recall_vs_tilt.png")
  fig_time(results, report_dir / "detect_time.png")

  # Statistics per target
  stats = {}
  acc_rows, det_rows, time_rows = [], [], []
  for key, label, _ in TARGETS:
    e = errors[errors.target == key]
    r = results[results.target == key]
    s = corner_stats(e)
    acc_rows.append([label, f"{s['n']:,}", f"{s['rmse']:.3f}",
                     f"{s['bias']:.3f}", f"{s['precision']:.3f}",
                     f"{s['p50']:.3f}", f"{s['p95']:.3f}", f"{s['p99']:.3f}"])

    visible = r[r.num_visible > 0]
    inl = visible.num_detected - visible.num_outliers
    detected = 100.0 * (inl > 0).mean()
    recall = 100.0 * (inl / visible.num_visible).mean()
    outl_pct = 100.0 * r.num_outliers.sum() / max(r.num_detected.sum(), 1)
    det_rows.append([label, f"{len(visible)}", f"{detected:.1f}",
                     f"{recall:.1f}", f"{outl_pct:.1f}"])

    t = r[r.num_detected > 0].detect_ms
    time_rows.append([label, f"{t.mean():.1f}", f"{t.median():.1f}",
                      f"{t.quantile(0.95):.1f}", f"{r.detect_ms.mean():.1f}"])
    stats[key] = dict(label=label, rmse=s["rmse"], bias=s["bias"], p95=s["p95"],
                      detected=detected, recall=recall, outliers=outl_pct,
                      time=t.median())

  # Quick summary: the main numbers of every target, and what stands out
  summary_rows = [[v["label"], f"{v['rmse']:.3f}", f"{v['detected']:.1f}",
                   f"{v['recall']:.1f}", f"{v['time']:.1f}"]
                  for v in stats.values()]
  bold_best(summary_rows, 1, "min")  # RMSE
  bold_best(summary_rows, 2, "max")  # Detected
  bold_best(summary_rows, 3, "max")  # Corner recall
  bold_best(summary_rows, 4, "min")  # Time

  def best(metric, mode):
    key = (min if mode == "min" else max)(stats, key=lambda k: stats[k][metric])
    return stats[key]

  accurate = best("rmse", "min")
  reliable = best("recall", "max")
  fast = best("time", "min")
  slow = best("time", "max")
  worst_bias = best("bias", "max")
  kalibr_olsen = stats["aprilgrid_kalibr_olsen"]
  plain_olsen = stats["aprilgrid_plain_olsen"]
  kalibr_kaess = stats["aprilgrid_kalibr_kaess"]
  plain_kaess = stats["aprilgrid_plain_kaess"]
  summary_bullets = [
      f"**Most accurate:** {accurate['label']}, {accurate['rmse']:.3f} px rms "
      f"(95 % of its corners within {accurate['p95']:.2f} px).",
      f"**Most corners found:** {reliable['label']} finds {reliable['recall']:.1f} % "
      f"of the visible corners, and detects the board in "
      f"{reliable['detected']:.1f} % of the poses.",
      f"**Fastest and slowest:** {fast['label']} at {fast['time']:.1f} ms "
      f"(median), {slow['label']} at {slow['time']:.1f} ms.",
      f"**Bias:** no target has a corner bias above {worst_bias['bias']:.3f} px "
      f"(largest: {worst_bias['label']}), so the errors are random scatter.",
      f"**Black squares between the tags:** they cost olsen most of its corners "
      f"({kalibr_olsen['recall']:.1f} % recall on the kalibr style grid, "
      f"{plain_olsen['recall']:.1f} % on the plain grid), while kaess is "
      f"{kalibr_kaess['recall']:.1f} % and {plain_kaess['recall']:.1f} %.",
  ]

  # Bold the best result of every column
  for c in range(2, 8):  # RMSE, bias, precision, median, p95, p99
    bold_best(acc_rows, c, "min")
  bold_best(det_rows, 2, "max")  # Detected
  bold_best(det_rows, 3, "max")  # Corner recall
  bold_best(det_rows, 4, "min")  # Outliers
  for c in range(1, 5):
    bold_best(time_rows, c, "min")

  n_poses = cfg["num_poses"]
  title = "Detector accuracy report"
  R = f"""{"=" * 72}
{title}
{"=" * 72}

Synthetic experiment: a camera at {n_poses} random poses in front of each
calibration target, rendered by ray casting every pixel onto the board plane so
the true corner locations are known exactly. Every target is viewed from the
same poses.

In the tables the best result of each column is in **bold**: the lowest error or
time and the highest detection rate or recall.

Setup
=====

{rst_table(["Setting", "Value"], [
    ["Camera", f"752x480, {cfg['camera']['fov']:.0f} deg horizontal FOV, distortion: {cfg['camera']['distort']}"],
    ["Distance to board", f"{cfg['poses']['min_dist']} - {cfg['poses']['max_dist']} m"],
    ["Viewing angle (tilt)", f"0 - {cfg['poses']['max_tilt']:.0f} deg, random roll and azimuth"],
    ["Image", f"blur sigma {cfg['render']['blur_sigma']} px, noise sigma {cfg['render']['noise_sigma']} levels, {cfg['render']['supersample']}x supersampled"],
    ["Outlier threshold", f"corner error > {cfg['evaluation']['outlier_px']} px"],
    ["Corner refinement", refine_text(cfg)],
    ["Seed", f"{cfg['seed']}"]], widths=[30, 70])}

Targets: AprilGrid 6x6 (tag 80 mm, spacing 25 %) with black squares between
tags (kalibr style) and without, each with the ``kaess`` (ethz_apriltag,
AprilTag 1) and ``olsen`` (apriltag, AprilTag 3) detectors; a 9x8 checkerboard
(``findChessboardCornersSB``); a 9x8 ChArUco board (OpenCV ``CharucoDetector``).
Squares are 70 mm. The corners are detected with the detectors of
``lib/calib``.

Results
=======

Summary
-------

{rst_table(["Target", "RMSE [px]", "Detected [%]", "Corner recall [%]", "Median time [ms]"], summary_rows)}

""" + "\n\n".join("- " + b for b in summary_bullets) + f"""

Accuracy
--------

Corner error is the distance in pixels between the detected corner and the true
projection, over every detected corner whose error is within the outlier
threshold.

{rst_table(["Target", "Corners", "RMSE [px]", "Bias [px]", "Precision [px]", "Median [px]", "p95 [px]", "p99 [px]"], acc_rows)}

- **RMSE** is the overall error. It splits into **bias** (the constant offset
  of the mean error vector) and **precision** (the random scatter around that
  offset): RMSE\ :sup:`2` = bias\ :sup:`2` + precision\ :sup:`2`.
- The median, p95 and p99 columns are the corner error magnitude percentiles.

{figure("error_cdf.png", "Corner error distribution (cumulative).")}
{figure("rmse_vs_distance.png", "Accuracy by distance to the board center.")}
RMSE [px] by distance to the board center:

{binned_table(errors, "dist_m", DIST_EDGES, "m")}

{figure("rmse_vs_tilt.png", "Accuracy by viewing angle (tilt).")}
RMSE [px] by viewing angle (tilt):

{binned_table(errors, "tilt_deg", TILT_EDGES, "deg")}

A "-" means fewer than 30 detected corners in that bin, so it is also left out
of the lines.

Detection
---------

{rst_table(["Target", "Poses", "Detected [%]", "Corner recall [%]", "Outliers [%]"], det_rows)}

- **Detected** is the share of poses with at least one correct corner.
- **Corner recall** is the average share of the visible corners that were
  detected correctly. Checkerboard and ChArUco only report a corner when the
  pattern is found.
- **Outliers** are detected corners with an error above the threshold, as a
  share of all detections.

{figure("recall_vs_tilt.png", "Corner recall by viewing angle.")}
Detection time
--------------

Time of one ``detect()`` call on one 752x480 image, measured on the machine that
ran the experiment (single run, wall clock). The ``kaess`` detector is
single-threaded; ``olsen`` uses 2 threads and OpenCV may parallelise internally.

{rst_table(["Target", "Mean [ms]", "Median [ms]", "p95 [ms]", "Mean incl. misses [ms]"], time_rows)}

The first three columns cover images where the detector returned corners, and
the last one all images, since a miss can be much faster or slower than a hit.

{figure("detect_time.png", "Detection time per image (median, with the 95th percentile).")}
Notes
=====

- Colours follow the target and are the first six slots of the reference
  categorical palette in order. The palette validator could not be run on the
  machine that made this report (the installed Node is too old), so colour
  separation was not re-checked. Every line is labelled in a legend and the
  numbers behind each chart are in the tables.
- This is a simulation. It has no motion blur, lighting, lens flare or print
  defects, so absolute numbers are best read as relative comparisons.
"""
  (report_dir / "REPORT.rst").write_text(R)
  print(f"Wrote {report_dir / 'REPORT.rst'}")


if __name__ == "__main__":
  main()
