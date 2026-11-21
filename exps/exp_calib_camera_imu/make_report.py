#!/usr/bin/env python3
"""
Generate the camera-IMU time delay benchmark report (reStructuredText).

Reads results.csv written by exp_calib_camera_imu and writes REPORT.rst and its
figure to the report directory.

Usage: make_report.py <exp_calib_camera_imu_output_dir> <config.yaml> <report_dir>
"""
import sys
import textwrap
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

# Time delay methods: key, label, description, colour, line style, marker
METHODS = {
    "pixel_velocity": (
        "PIXEL_VELOCITY",
        "Corrects each pixel with its velocity, the difference of the pixel in "
        "this view and in the previous one.",
        "#2a78d6", "-", "o",
    ),
    "pose_interp": (
        "POSE_INTERP",
        "Extrapolates the pose of the view to the camera time from the "
        "poses of this view and of the previous one.",
        "#eb6834", "--", "s",
    ),
    "imu": (
        "IMU",
        "The IMU error pre-integrates the IMU measurements in the window of "
        "the two views, shifted by the time delay. The camera errors do not use "
        "the time delay.",
        "#1baf7a", "-.", "^",
    ),
}
FALLBACK_STYLE = ("#1baf7a", "-.", "^")
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"


def label(method):
  return METHODS.get(method, (method,))[0]


def style(method, index):
  if method in METHODS:
    return METHODS[method][2:]
  return FALLBACK_STYLE


def style_axes(ax, xlabel, ylabel, title):
  ax.set_facecolor(SURFACE)
  ax.set_title(title, loc="left", fontsize=9, color=INK, fontweight="bold")
  ax.set_xlabel(xlabel, color=INK2, fontsize=8)
  ax.set_ylabel(ylabel, color=INK2, fontsize=8)
  ax.tick_params(colors=INK2, labelsize=8)
  ax.grid(True, color=GRID, linewidth=0.6)
  ax.set_axisbelow(True)
  for side in ("top", "right"):
    ax.spines[side].set_visible(False)
  for side in ("left", "bottom"):
    ax.spines[side].set_color(GRID)


WIDTH = 80  # Width of the report, in columns


def para(text, first="", rest=""):
  """A paragraph wrapped to the width of the report"""
  return textwrap.fill(text, width=WIDTH, initial_indent=first,
                       subsequent_indent=rest, break_long_words=False)


def wrap_cell(text, width):
  lines = []
  for line in str(text).split("\n"):
    lines += textwrap.wrap(line, width=width, break_long_words=False) or [""]
  return lines


def rst_table(header, rows):
  """
  A reStructuredText grid table that is at most `WIDTH` columns wide, cells
  wrap to make it fit
  """
  if len(rows) == 0:
    return "*No results.*"
  n = len(header)
  cells = [[str(c) for c in header]] + [[str(c) for c in r] for r in rows]
  min_w = [max(max((len(w) for w in row[i].split()), default=0) for row in cells)
           for i in range(n)]
  w = [max(len(row[i]) for row in cells) for i in range(n)]
  while sum(w) + 3 * n + 1 > WIDTH:
    i = max(range(n), key=lambda i: w[i] - min_w[i])
    if w[i] <= min_w[i]:
      raise ValueError("table does not fit in %d columns: %s" % (WIDTH, header))
    w[i] -= 1

  line = "+" + "+".join("-" * (x + 2) for x in w) + "+"
  head = "+" + "+".join("=" * (x + 2) for x in w) + "+"

  def fmt(row):
    wrapped = [wrap_cell(c, w[i]) for i, c in enumerate(row)]
    height = max(len(c) for c in wrapped)
    out = []
    for k in range(height):
      out.append("| " + " | ".join(
          (c[k] if k < len(c) else "").ljust(w[i])
          for i, c in enumerate(wrapped)) + " |")
    return out

  out = [line] + fmt(cells[0]) + [head]
  for row in cells[1:]:
    out += fmt(row)
    out.append(line)
  return "\n".join(out)


def summarize(df, tolerance):
  """Statistics of the time delay error per noise level, method and delay"""
  g = df.groupby(["noise", "method", "true_delay_ms"], sort=False)
  return g.agg(
      runs=("error_ms", "size"),
      mean=("error_ms", "mean"),
      std=("error_ms", lambda x: x.std(ddof=0)),
      max_abs=("error_ms", lambda x: x.abs().max()),
      failed=("error_ms", lambda x: int((x.abs() > tolerance).sum())),
      iterations=("iterations", "mean"),
      solve_s=("solve_s", "mean"),
  ).reset_index()


def plot_errors(df, noise_levels, methods, tolerance, path):
  fig, axes = plt.subplots(1, len(noise_levels), figsize=(3.6 * len(noise_levels), 3.1),
                           sharey=True, squeeze=False)
  fig.patch.set_facecolor("white")
  delays = sorted(df["true_delay_ms"].unique())
  span = (max(delays) - min(delays)) if len(delays) > 1 else 10.0
  offset = 0.02 * span
  for ax, noise in zip(axes[0], noise_levels):
    style_axes(ax, "True time delay [ms]", "Time delay error [ms]", noise)
    ax.axhspan(-tolerance, tolerance, color="#1baf7a", alpha=0.08, linewidth=0)
    ax.axhline(0.0, color=GRID, linewidth=1.0)
    for i, method in enumerate(methods):
      colour, line, marker = style(method, i)
      d = df[(df["noise"] == noise) & (df["method"] == method)]
      shift = (i - (len(methods) - 1) / 2.0) * offset
      xs = d["true_delay_ms"] + shift
      ax.scatter(xs, d["error_ms"], s=9, color=colour, alpha=0.35, linewidths=0)
      m = d.groupby("true_delay_ms")["error_ms"].agg(["mean", "std"]).reset_index()
      ax.errorbar(m["true_delay_ms"] + shift, m["mean"], yerr=m["std"].fillna(0.0),
                  color=colour, linestyle=line, marker=marker, markersize=4,
                  linewidth=1.2, capsize=2, label=label(method))
    ax.set_xticks(delays)
  axes[0][0].legend(fontsize=7, frameon=False, loc="best")
  # Errors that fail are far larger than the tolerance, keep both visible
  ymax = max(float(df["error_ms"].abs().max()), 2.0 * tolerance)
  axes[0][0].set_ylim(-1.1 * ymax, 1.1 * ymax)
  fig.tight_layout()
  fig.savefig(path, dpi=150, facecolor="white")
  plt.close(fig)


def main():
  if len(sys.argv) != 4:
    print(__doc__)
    return 1
  out_dir, config_file, report_dir = (Path(a) for a in sys.argv[1:])
  report_dir.mkdir(parents=True, exist_ok=True)
  cfg = yaml.safe_load(open(config_file))
  df = pd.read_csv(out_dir / "results.csv")

  tolerance = cfg["tolerance_ms"]
  noise_levels = list(cfg["noise_levels"].keys())
  methods = list(cfg["td_methods"])
  stats = summarize(df, tolerance)
  plot_errors(df, noise_levels, methods, tolerance, report_dir / "time_delay_error.png")

  r = []
  r.append("Camera-IMU time delay benchmark")
  r.append("===============================\n")
  r.append(para(
      "How well does ``CalibCameraImu`` recover the time delay between a "
      "camera and an IMU, and the extrinsic between them, with each of its "
      "time delay methods? The data is simulated, so the ground truth is "
      "known. The time stamp of an IMU measurement is its physical time plus "
      "the true delay.") + "\n")

  # ---- Setup
  r.append("Setup")
  r.append("-----\n")
  nl = cfg["noise_levels"]
  setup = [
      ("Dataset", "camera at %g Hz and an IMU at 1 kHz, moving on a ``%s`` "
                  "trajectory of radius %g m for %g s, in front of an "
                  "AprilGrid"
                  % (cfg["camera_rate"], cfg["trajectory"]["type"],
                     cfg["trajectory"]["radius"], cfg["trajectory"]["duration"])),
      ("True delays", ", ".join("%+g ms" % d for d in cfg["true_delays_ms"])),
      ("Initial extrinsic", "ground truth moved by %s m and rotated by %s deg "
                            "(roll, pitch, yaw)"
                            % (cfg["init_extrinsic"]["position"],
                               cfg["init_extrinsic"]["rotation_deg"])),
      ("Datasets", "%d per setting with noise, 1 without (the data is the "
                   "same for every seed)" % cfg["num_seeds"]),
      ("Solver", "at most %d iterations" % cfg["max_iters"]),
      ("Failure", "a time delay error above %g ms" % tolerance),
  ]
  r.append(rst_table(["Setting", "Value"], setup))
  r.append("")
  noise_rows = []
  for name in noise_levels:
    n = nl[name]
    noise_rows.append((name, "%g px" % n["pixel"],
                       ("%g x the noise density" % n["imu"]) if n["imu"] > 0
                       else "none"))
  r.append("Noise levels:\n")
  r.append(rst_table(["Level", "Corner noise (std)", "IMU noise"], noise_rows))
  r.append("")
  r.append("Time delay methods:\n")
  r.append(rst_table(["Method", "How the time delay enters"],
                     [(label(m), METHODS[m][1] if m in METHODS else "")
                      for m in methods]))
  r.append("")

  # ---- Results
  r.append("Results")
  r.append("-------\n")
  r.append(para(
      "Time delay error (estimate minus truth) against the true delay. Dots "
      "are single datasets, lines are the mean and standard deviation, the "
      "shaded band is the failure tolerance.") + "\n")
  r.append(".. image:: time_delay_error.png")
  r.append("   :width: 100%")
  r.append("   :alt: Time delay error against true delay, by noise level\n")

  # Overall
  r.append("Overall, over all true delays:\n")
  rows = []
  cost_rows = []
  for noise in noise_levels:
    for method in methods:
      d = df[(df["noise"] == noise) & (df["method"] == method)]
      e = d["error_ms"]
      rows.append((noise, label(method), len(d),
                   "%.2f" % np.sqrt((e ** 2).mean()), "%+.2f" % e.mean(),
                   "%.2f" % e.abs().max(),
                   "%d / %d" % ((e.abs() > tolerance).sum(), len(d))))
      cost_rows.append((noise, label(method), "%.0f" % d["iterations"].mean(),
                        "%.1f" % d["solve_s"].mean()))
  r.append(rst_table(["Noise", "Method", "Runs", "RMS [ms]", "Mean [ms]",
                      "Max [ms]", "Failed"], rows))
  r.append("")
  r.append("Solver iterations and time, on average:\n")
  r.append(rst_table(["Noise", "Method", "Iterations", "Solve [s]"], cost_rows))
  r.append("")

  # Per delay
  r.append("Per true delay:\n")
  for noise in noise_levels:
    r.append(noise)
    r.append("~" * len(noise) + "\n")
    rows = []
    for method in methods:
      for _, s in stats[(stats["noise"] == noise) & (stats["method"] == method)].iterrows():
        rows.append((label(method), "%+g" % s["true_delay_ms"], int(s["runs"]),
                     "%+.2f" % s["mean"], "%.2f" % s["std"], "%.2f" % s["max_abs"],
                     "%d / %d" % (s["failed"], s["runs"]),
                     "%.0f" % s["iterations"]))
    r.append(rst_table(["Method", "True [ms]", "Runs", "Mean error [ms]",
                        "Std [ms]", "Max error [ms]", "Failed", "Iter."], rows))
    r.append("")

  # Extrinsic
  r.append("Camera-IMU extrinsic")
  r.append("--------------------\n")
  r.append(para(
      "Error of the estimated extrinsic, mean over the runs. The extrinsic "
      "is estimated together with the time delay.") + "\n")
  rows = []
  for noise in noise_levels:
    for method in methods:
      d = df[(df["noise"] == noise) & (df["method"] == method)]
      rows.append((noise, label(method), "%.1f" % d["ext_pos_mm"].mean(),
                   "%.3f" % d["ext_rot_deg"].mean()))
  r.append(rst_table(["Noise", "Method", "Position [mm]", "Rotation [deg]"], rows))
  r.append("")

  # Notes
  r.append("Notes")
  r.append("-----\n")
  notes = [
      "``PIXEL_VELOCITY`` takes the pixel velocity from the pixels of two "
      "views. With noisy pixels the correction of a pixel is a blend of its "
      "own noise and the noise of the previous view, which is the smallest for "
      "a delay of half the time between views, so least squares is pulled "
      "towards that delay, whatever the true one is. Even without noise a "
      "difference of two views is only a first order velocity.",
      "``POSE_INTERP`` extrapolates from the estimated poses, not from the "
      "pixels, so the noise of the pixels does not enter the correction.",
      "``IMU`` integrates the IMU measurements that are in the shifted "
      "window, so the noise in them is the noise its covariance expects, and "
      "the time delay is not pulled anywhere by it. It needs IMU measurements "
      "before and after the window, as much as the largest delay.",
      "The noise densities of the simulated IMU (0.08 m/s^2/sqrt(Hz), 0.004 "
      "rad/s/sqrt(Hz)) are 20 - 40 times those of a typical MEMS IMU, the "
      "``heavy`` level uses them as they are.",
  ]
  for note in notes:
    r.append(para(note, first="- ", rest="  "))
  r.append("")

  text = "\n".join(r)
  too_long = [l for l in text.splitlines() if len(l) > WIDTH]
  if too_long:
    raise ValueError("%d lines are wider than %d columns, first: %s"
                     % (len(too_long), WIDTH, too_long[0]))
  (report_dir / "REPORT.rst").write_text(text)
  print("Wrote", report_dir / "REPORT.rst")
  return 0


if __name__ == "__main__":
  sys.exit(main())
