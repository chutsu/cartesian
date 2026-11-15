#!/usr/bin/env python3
"""
Generate the camera calibration experiment report (reStructuredText).

Reads results.csv and corners.csv written by exp_calib_camera, along with the
sample renders, and writes REPORT.rst and its figures to the report directory.

Usage: make_report.py <exp_calib_camera_output_dir> <config.yaml> <report_dir>
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from matplotlib.lines import Line2D

# Calibration targets. Series are coloured by target (the first four slots of
# the reference categorical palette, in order); every target also has its own
# marker and line style, so they can be told apart without colour.
TARGETS = [
    # key, label, colour, line style, marker
    ("aprilgrid_kaess", "AprilGrid (kaess)", "#2a78d6", "-", "o"),
    ("aprilgrid_olsen", "AprilGrid (olsen)", "#eb6834", "--", "s"),
    ("checkerboard", "Checkerboard", "#1baf7a", "-.", "^"),
    ("charuco", "ChArUco", "#eda100", ":", "D"),
]
KEYS = [t[0] for t in TARGETS]
LABEL = {t[0]: t[1] for t in TARGETS}
LEVELS = ["none", "mild", "strong"]
MODELS = ["BrownConrady4", "KannalaBrandt4"]
MODEL_TITLE = {
    "BrownConrady4": "Pinhole + radial-tangential (BrownConrady4)",
    "KannalaBrandt4": "Equidistant fisheye (KannalaBrandt4)",
}
MODEL_SHORT = {"BrownConrady4": "Pinhole", "KannalaBrandt4": "Fisheye"}
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"

# A calibration "recovers" the camera when the projection of the estimate is
# within this many pixels (rms over the image) of the ground truth
GOOD_PX = 0.5
CLIP_PX = 100.0  # Errors above this are drawn as "diverged"


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


def save(fig, path):
  fig.patch.set_facecolor(SURFACE)
  fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
  plt.close(fig)


def legend_handles():
  return [Line2D([0], [0], color=color, lw=2, linestyle=ls, marker=mk,
                 markersize=5, label=label)
          for _, label, color, ls, mk in TARGETS]


def read_image(path):
  return mpimg.imread(path) if Path(path).exists() else None


def is_skipped(df):
  return df.note.fillna("").str.startswith("skipped")


# ------------------------------------------------------------------ montages


def fig_targets(samples, path):
  names = [("aprilgrid_kalibr_kaess", "AprilGrid, kalibr style (kaess tags)"),
           ("aprilgrid_plain_kaess", "AprilGrid, plain (kaess tags)"),
           ("checkerboard", "Checkerboard"), ("charuco", "ChArUco")]
  fig, axes = plt.subplots(1, 4, figsize=(12, 3.4))
  for ax, (key, title) in zip(axes, names):
    img = read_image(samples / "targets" / f"{key}.png")
    if img is not None:
      ax.imshow(img, cmap="gray", vmin=0, vmax=1)
    ax.set_title(title, fontsize=8, color=INK)
    ax.axis("off")
  save(fig, path)


def fig_camera_views(samples, model, fovs, path):
  fig, axes = plt.subplots(len(LEVELS), len(fovs),
                           figsize=(3.2 * len(fovs), 2.2 * len(LEVELS)))
  for i, level in enumerate(LEVELS):
    for j, fov in enumerate(fovs):
      ax = axes[i][j]
      ax.axis("off")
      img = read_image(samples / "cameras" / f"{model}_{fov}_{level}.png")
      if img is None:
        ax.text(0.5, 0.5, "not representable", ha="center", va="center",
                fontsize=8, color=INK2, transform=ax.transAxes)
      else:
        ax.imshow(img, cmap="gray", vmin=0, vmax=1)
      ax.set_title(f"{fov} deg, {level}", fontsize=8, color=INK)
  save(fig, path)


def fig_views(samples, case, path, n=3):
  fig, axes = plt.subplots(len(TARGETS), n, figsize=(3.6 * n, 2.4 * len(TARGETS)))
  for i, (key, label, *_) in enumerate(TARGETS):
    for k in range(n):
      ax = axes[i][k]
      ax.axis("off")
      img = read_image(samples / "views" / f"{case}_{key}" / f"view_{k:02d}.png")
      if img is not None:
        ax.imshow(img[..., ::-1])
      ax.set_title(f"{label}, view {k}", fontsize=8, color=INK)
  save(fig, path)


def fig_coverage(corners, cases, resolution, path):
  fig, axes = plt.subplots(len(TARGETS), len(cases),
                           figsize=(2.6 * len(cases), 1.9 * len(TARGETS)))
  for i, (key, label, *_) in enumerate(TARGETS):
    for j, (name, title) in enumerate(cases):
      ax = axes[i][j]
      c = corners[(corners.case == name) & (corners.target == key)]
      ax.set_xlim(0, resolution[0])
      ax.set_ylim(resolution[1], 0)
      ax.set_aspect("equal")
      ax.tick_params(colors=INK2, labelsize=6)
      if i == 0:
        ax.set_title(title, fontsize=8, color=INK)
      if j == 0:
        ax.set_ylabel(label, fontsize=7, color=INK2)
      if c.empty:
        continue
      ax.hist2d(c.u, c.v, bins=[32, 20],
                range=[[0, resolution[0]], [0, resolution[1]]], cmap="Blues")
  save(fig, path)


# --------------------------------------------------------------------- plots


def fig_metric(df, init, column, ylabel, fovs, path):
  fig, axes = plt.subplots(len(MODELS), len(LEVELS), figsize=(12, 6.4),
                           sharey=True)
  for r, model in enumerate(MODELS):
    for c, level in enumerate(LEVELS):
      ax = axes[r][c]
      for t, (key, _, color, ls, mk) in enumerate(TARGETS):
        d = df[(df.model == model) & (df.level == level) & (df.target == key) &
               (df.init == init) & ~is_skipped(df)].sort_values("fov")
        xs = d.fov.to_numpy()
        ys = d[column].where(d.ok == 1).to_numpy(dtype=float)
        good = np.isfinite(ys) & (ys <= CLIP_PX)
        ax.plot(xs[good], ys[good], color=color, linestyle=ls, marker=mk,
                markersize=5, linewidth=1.6, markeredgecolor=SURFACE,
                markeredgewidth=0.8)
        bad = ~good
        if bad.any():
          # Diverged or failed: mark at the top, offset so crosses do not overlap
          ax.plot(xs[bad] + 2.0 * (t - 1.5), [CLIP_PX] * int(bad.sum()),
                  linestyle="", marker="x", color=color, markersize=6,
                  markeredgewidth=1.6, clip_on=False)
      ax.set_yscale("log")
      ax.set_ylim(0.03, CLIP_PX * 1.6)
      ax.set_xticks(fovs)
      style_axes(ax, "Horizontal field of view [deg]" if r == len(MODELS) - 1 else "",
                 ylabel if c == 0 else "",
                 f"{MODEL_SHORT[model]}, {level} distortion")
  fig.subplots_adjust(bottom=0.2, hspace=0.3, wspace=0.08)
  fig.legend(handles=legend_handles(), loc="lower center", ncol=4, frameon=False,
             fontsize=8, labelcolor=INK2)
  save(fig, path)


# -------------------------------------------------------------------- tables


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


def bold_best(cells, values, mode="min"):
  """Bold the best of several cells, ties included, none if no value counts"""
  numbers = [v for v in values if v is not None]
  if not numbers:
    return list(cells)
  best = min(numbers) if mode == "min" else max(numbers)
  return [f"**{c}**" if v is not None and v == best else c
          for c, v in zip(cells, values)]


def px_value(v, ok):
  """Projection error to compare, None if the calibration failed or diverged"""
  if not ok or not np.isfinite(v) or v > CLIP_PX:
    return None
  return round(float(v), 2)


def fmt_px(v, ok):
  if not ok or not np.isfinite(v):
    return "failed"
  if v > CLIP_PX:
    return f">{CLIP_PX:.0f}"
  return f"{v:.2f}"


def row_of(df, model, level, fov, init, key):
  """The result row of a camera, target and initial guess (there is always one,
  also for cameras that were skipped)"""
  sub = df[(df.model == model) & (df.level == level) & (df.fov == fov) &
           (df.init == init) & (df.target == key)]
  assert len(sub) == 1, (model, level, fov, init, key)
  return sub.iloc[0]


def summary_table(df, init, column, fovs):
  """Projection error of every target for every camera, best in bold"""
  rows = []
  for model in MODELS:
    for level in LEVELS:
      for fov in fovs:
        cells = [model, level, str(fov)]
        rs = [row_of(df, model, level, fov, init, k) for k in KEYS]
        if all(str(r.note).startswith("skipped") for r in rs):
          rows.append(cells + ["n/a"] * len(KEYS))
          continue
        values = [px_value(r[column], r.ok) for r in rs]
        texts = [fmt_px(r[column], r.ok) for r in rs]
        rows.append(cells + bold_best(texts, values))
  header = ["Model", "Distortion", "FOV [deg]"] + [LABEL[k] for k in KEYS]
  return rst_table(header, rows)


def converged(d):
  return d[(d.ok == 1) & (d.model_rmse_px < CLIP_PX)]


def param_table(df, inits):
  """Median parameter errors over the converged cameras, best in bold"""
  d = df[~is_skipped(df)].copy()
  d["fx_err"] = (100.0 * (d.fx_est - d.fx_gt) / d.fx_gt).abs()
  d["cx_err"] = (d.cx_est - d.cx_gt).abs()
  d["cy_err"] = (d.cy_est - d.cy_gt).abs()
  rows = []
  for init in inits:
    for model in MODELS:
      stats = []
      for key in KEYS:
        sub = d[(d.init == init) & (d.model == model) & (d.target == key)]
        ok = converged(sub)
        if ok.empty:
          stats.append(None)
        else:
          stats.append([ok.fx_err.median(), ok.cx_err.median(),
                        ok.cy_err.median(), ok.reproj_rmse_px.median(),
                        len(ok), len(sub)])
      columns = []
      for m, fmt in enumerate(["{:.2f}", "{:.2f}", "{:.2f}", "{:.3f}"]):
        values = [None if s is None else round(s[m], 3 if m == 3 else 2)
                  for s in stats]
        texts = ["-" if s is None else fmt.format(s[m]) for s in stats]
        columns.append(bold_best(texts, values))
      counts = [0 if s is None else s[4] for s in stats]
      converged_cells = bold_best(
          ["-" if s is None else f"{s[4]} / {s[5]}" for s in stats], counts, "max")
      for t, key in enumerate(KEYS):
        rows.append([init, model, LABEL[key]] + [c[t] for c in columns] +
                    [converged_cells[t]])
  header = ["Initial guess", "Model", "Target", "fx error [%]", "cx error [px]",
            "cy error [px]", "Reproj. rmse [px]", "Converged"]
  return rst_table(header, rows)


# ------------------------------------------------------------------- summary


def refine_text(cfg):
  r = cfg["refine_corners"]
  on = [n for n, v in (("AprilGrid", r["aprilgrid"]),
                       ("checkerboard", r["checkerboard"]),
                       ("ChArUco", r["charuco"])) if v]
  return "cv::cornerSubPix for " + ", ".join(on) if on else "off"


def quick_summary(df, init="default"):
  """Table and bullets with the main results of every target"""
  d = df[~is_skipped(df) & (df.init == init)].copy()
  d["pp_err"] = np.hypot(d.cx_est - d.cx_gt, d.cy_est - d.cy_gt)

  # The target with the lowest projection error, for every camera (ties count
  # for every tied target)
  best_for = {k: 0 for k in KEYS}
  n_cameras = 0
  for (model, level, fov), cam in d.groupby(["model", "level", "fov"]):
    errors = {r.target: px_value(r.model_rmse_px, r.ok) for r in cam.itertuples()}
    valid = {k: v for k, v in errors.items() if v is not None}
    n_cameras += 1
    if valid:
      lowest = min(valid.values())
      for k, v in valid.items():
        if v == lowest:
          best_for[k] += 1

  stats = {}
  for key in KEYS:
    t = d[d.target == key]
    ok = converged(t)
    fisheye = ok[ok.model == "KannalaBrandt4"].model_rmse_px
    pinhole = ok[(ok.model == "BrownConrady4") & (ok.fov < 160)].model_rmse_px
    stats[key] = dict(
        fisheye=fisheye.median() if len(fisheye) else None,
        pinhole=pinhole.median() if len(pinhole) else None,
        recovered=int(((t.ok == 1) & (t.model_rmse_px <= GOOD_PX)).sum()),
        total=len(t),
        best_for=best_for[key],
        pp=ok.pp_err.median() if len(ok) else None)

  def text(v, fmt="{:.2f}"):
    return "-" if v is None or not np.isfinite(v) else fmt.format(v)

  rows = []
  for key in KEYS:
    v = stats[key]
    rows.append([LABEL[key], text(v["fisheye"]), text(v["pinhole"]),
                 f"{v['recovered']} / {v['total']}", str(v["best_for"]),
                 text(v["pp"])])

  def mark(col, values, mode):
    cells = bold_best([r[col] for r in rows], values, mode)
    for r, c in zip(rows, cells):
      r[col] = c

  def clean(v):
    return None if v is None or not np.isfinite(v) else round(float(v), 2)

  mark(1, [clean(stats[k]["fisheye"]) for k in KEYS], "min")
  mark(2, [clean(stats[k]["pinhole"]) for k in KEYS], "min")
  mark(3, [stats[k]["recovered"] for k in KEYS], "max")
  mark(4, [stats[k]["best_for"] for k in KEYS], "max")
  mark(5, [clean(stats[k]["pp"]) for k in KEYS], "min")

  header = ["Target", "Fisheye: median error [px]",
            "Pinhole 90-120 deg: median error [px]",
            f"Cameras recovered (<= {GOOD_PX} px)", "Lowest error for cameras",
            "Principal point: median error [px]"]
  table = rst_table(header, rows)

  top = max(KEYS, key=lambda k: stats[k]["best_for"])
  recovered = stats[top]["recovered"]
  bullets = [f"**Best:** {LABEL[top]}, lowest error for {stats[top]['best_for']} of "
             f"{n_cameras} cameras, {recovered} of {stats[top]['total']} "
             f"recovered."]

  fisheye = [stats[k]["fisheye"] for k in KEYS if stats[k]["fisheye"] is not None]
  all_fisheye = d[d.model == "KannalaBrandt4"]
  if fisheye:
    status = ("every fisheye camera converges" if
              len(converged(all_fisheye)) == len(all_fisheye) else
              "not every fisheye camera converges")
    bullets.append(f"**Fisheye:** {status}, median error "
                   f"{min(fisheye):.2f} - {max(fisheye):.2f} px, up to 180 degrees.")

  # Pinhole at 160 degrees, from the default and a near-truth initial guess
  p160 = df[~is_skipped(df) & (df.model == "BrownConrady4") & (df.fov == 160)]
  if len(p160):
    n_default = len(converged(p160[p160.init == "default"]))
    n_near = len(converged(p160[p160.init == "near_truth"]))
    n_total = len(p160[p160.init == "default"])
    text = (f"**Pinhole at 160 degrees:** {n_default} of {n_total} calibrations "
            f"converge from the default initial guess, {n_near} from a guess "
            f"near the truth")
    bullets.append(text + (", so most failures are the initialisation, not the "
                           "model." if n_near > n_default else "."))

  # A low reprojection error does not mean the camera is right
  wrong = converged(d)
  wrong = wrong[(wrong.reproj_rmse_px < 1.0) & (wrong.model_rmse_px > 5.0)]
  for _, w in wrong.sort_values("model_rmse_px").tail(1).iterrows():
    bullets.append(f"**Reprojection error is not accuracy:** {w.model} at "
                   f"{int(w.fov)} degrees ({LABEL[w.target]}) reports "
                   f"{w.reproj_rmse_px:.2f} px but is off by "
                   f"{w.model_rmse_px:.1f} px.")
  return table, bullets


# ---------------------------------------------------------------------- main


def main():
  out_dir, config_file, report_dir = map(Path, sys.argv[1:4])
  report_dir.mkdir(parents=True, exist_ok=True)
  samples = out_dir / "samples"
  df = pd.read_csv(out_dir / "results.csv")
  corners = pd.read_csv(out_dir / "corners.csv")
  cfg = yaml.safe_load(open(config_file))
  fovs = cfg["fovs"]
  inits = cfg["initial_guesses"]
  res = cfg["camera"]["resolution"]
  assert set(cfg["targets"]) == set(KEYS), "report expects the default targets"

  # Figures
  fig_targets(samples, report_dir / "targets.png")
  for model in MODELS:
    fig_camera_views(samples, model, fovs,
                     report_dir / f"camera_views_{model}.png")
  fig_views(samples, "KannalaBrandt4_120_strong",
            report_dir / "calibration_views.png")
  cov_cases = [(f"BrownConrady4_{f}_mild", f"pinhole {f} deg")
               for f in fovs if f in (90, 120)]
  cov_cases += [(f"KannalaBrandt4_{f}_mild", f"fisheye {f} deg") for f in fovs]
  fig_coverage(corners, cov_cases, res, report_dir / "coverage.png")
  for init in inits:
    fig_metric(df, init, "model_rmse_px",
               "Projection error [px rms, log]", fovs,
               report_dir / f"model_error_{init}.png")

  r_models = cfg["models"]
  kb4 = r_models["KannalaBrandt4"]
  R = []
  R.append("=" * 72)
  R.append("Camera calibration accuracy report")
  R.append("=" * 72)
  R.append("")
  R.append(f"""Corners of an AprilGrid, a checkerboard and a ChArUco board are detected in
simulated images of cameras with horizontal fields of view of
{", ".join(str(f) for f in fovs)} degrees and different amounts of distortion,
and calibrated with ``CalibCamera``. The result is compared with the ground
truth camera.
""")

  R.append("Setup")
  R.append("=" * 5)
  R.append("")
  R.append(rst_table(["Setting", "Value"], [
      ["Resolution", f"{res[0]}x{res[1]}"],
      ["Principal point offset", f"({cfg['camera']['cx_offset']:+.1f}, "
                                 f"{cfg['camera']['cy_offset']:+.1f}) px"],
      ["fy / fx", f"{cfg['camera']['fy_ratio']}"],
      ["Views per camera and target", f"{cfg['num_views']}, random poses"],
      ["Distance to board", ", ".join(
          f"{k} deg: {v[0]}-{v[1]} m" for k, v in cfg["poses"]["dist_range"].items())],
      ["Tilt", f"0 - {cfg['poses']['max_tilt']:.0f} deg"],
      ["Image", f"blur sigma {cfg['render']['blur_sigma']} px, noise sigma "
                f"{cfg['render']['noise_sigma']} levels"],
      ["Corner refinement", refine_text(cfg)],
      ["Corners dropped beyond", f"{cfg['poses']['max_theta']:.0f} deg off the axis"],
      ["Seed", f"{cfg['seed']}"]], widths=[40, 60]))
  R.append("")
  R.append(f"""Cameras, with three distortion levels each (none, mild, strong). The pinhole
camera does not exist at 180 degrees.

- **BrownConrady4**: pinhole with radial-tangential distortion, -3 % (mild) and
  -12 % (strong) radial distortion at the horizontal image edge.
- **KannalaBrandt4**: equidistant fisheye, ``k1..k4`` = {kb4["mild"]["k"]}
  (mild) and {kb4["strong"]["k"]} (strong).

Targets, all with the detectors of ``lib/calib``:

- **AprilGrid**: 6x6 tags of 80 mm, detected with ``kaess`` and ``olsen``.
- **Checkerboard**: 9x8 squares of 70 mm. Its detector needs the whole board in
  the image, so only poses where the whole board is visible are used.
- **ChArUco**: 9x8 squares of 70 mm, 56 mm markers. It can be partly visible.

Both camera models project through the plane ``z = 1``, so corners 90 degrees or
more off the optical axis cannot be used.
""")
  R.append(figure("targets.png", "The calibration targets."))

  R.append("Camera views")
  R.append("=" * 12)
  R.append("")
  R.append("A view of the board, looking straight at it, for each camera.")
  R.append("")
  for model in MODELS:
    R.append(figure(f"camera_views_{model}.png",
                    f"{MODEL_TITLE[model]}: rows are distortion levels, columns "
                    "are fields of view."))

  R.append("Calibration data")
  R.append("=" * 16)
  R.append("")
  R.append("Detected corners (green) in three views of a 120 degree fisheye camera "
           "with strong distortion, and over all views (mild distortion).")
  R.append("")
  R.append(figure("calibration_views.png", "Three calibration views per target."))
  R.append(figure("coverage.png", "Detected corners over the image, all views."))

  R.append("Results")
  R.append("=======")
  R.append("")
  R.append(f"""The **projection error** compares the estimated and the ground truth camera:
rays over the image are projected with both and the pixel difference is
measured (rms and max). A camera is *recovered* when the rms is at most
{GOOD_PX} px. The best result of each row or column is in **bold**.
""")
  table, bullets = quick_summary(df, inits[0])
  R.append("Quick summary")
  R.append("-" * 13)
  R.append("")
  R.append("Default initial guess. Errors are medians over the cameras that "
           "converged.")
  R.append("")
  R.append(table)
  R.append("")
  for b in bullets:
    R.append("- " + b)
    R.append("")

  R.append("Parameter errors")
  R.append("-" * 16)
  R.append("")
  R.append("Median over the cameras that converged. ``fx`` is a relative error, "
           "the principal point ``cx, cy`` an absolute one.")
  R.append("")
  R.append(param_table(df, inits))
  R.append("")

  names = {"default": "Default initial guess",
           "near_truth": "Initial guess near the ground truth"}
  for init in inits:
    R.append(names[init])
    R.append("-" * len(names[init]))
    R.append("")
    R.append(figure(f"model_error_{init}.png",
                    "Projection error (rms) by field of view. Crosses mark "
                    "calibrations that diverged or failed."))
    R.append(summary_table(df, init, "model_rmse_px", fovs))
    R.append("")

  (report_dir / "REPORT.rst").write_text("\n".join(R))
  print(f"Wrote {report_dir / 'REPORT.rst'}")


if __name__ == "__main__":
  main()
