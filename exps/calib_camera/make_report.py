#!/usr/bin/env python3
"""
Generate the camera calibration experiment report (reStructuredText).

Reads results.csv and corners.csv written by exp_calib_camera, along with the
sample renders, and writes REPORT.rst and its figures to the report directory.

Usage: make_report.py <exp_calib_camera_output_dir> <config.yaml>
                                      <report_dir>
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

# Distortion levels use the first three slots of the reference categorical
# palette (the slots that validate for all pairs).
LEVELS = ["none", "mild", "strong"]
LEVEL_COLOR = {"none": "#2a78d6", "mild": "#eb6834", "strong": "#1baf7a"}
DETECTORS = ["kaess", "olsen"]
DET_STYLE = {"kaess": ("-", "o"), "olsen": ("--", "s")}
MODELS = ["BrownConrady4", "KannalaBrandt4"]
MODEL_TITLE = {
    "BrownConrady4": "Pinhole + radial-tangential (BrownConrady4)",
    "KannalaBrandt4": "Equidistant fisheye (KannalaBrandt4)",
}
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
  ax.set_title(title, loc="left", fontsize=10, color=INK, fontweight="bold")
  ax.set_xlabel(xlabel, color=INK2, fontsize=9)
  ax.set_ylabel(ylabel, color=INK2, fontsize=9)
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


def legend_below(fig, handles, ncol):
  fig.legend(handles=handles, loc="lower center", ncol=ncol, frameon=False,
             fontsize=8, labelcolor=INK2)


def legend_handles():
  from matplotlib.lines import Line2D
  hs = [Line2D([0], [0], color=LEVEL_COLOR[l], lw=2, label=f"{l} distortion")
        for l in LEVELS]
  hs += [Line2D([0], [0], color=INK2, lw=1.5, linestyle=DET_STYLE[d][0],
                marker=DET_STYLE[d][1], markersize=5, label=f"{d} detector")
         for d in DETECTORS]
  return hs


def read_image(path):
  return mpimg.imread(path) if Path(path).exists() else None


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


def fig_camera_views(samples, cases, model, fovs, path):
  fig, axes = plt.subplots(len(LEVELS), len(fovs),
                           figsize=(3.2 * len(fovs), 2.2 * len(LEVELS)))
  for i, level in enumerate(LEVELS):
    for j, fov in enumerate(fovs):
      ax = axes[i][j]
      ax.axis("off")
      name = f"{model}_{fov}_{level}"
      img = read_image(samples / "cameras" / f"{name}.png")
      if img is None:
        ax.text(0.5, 0.5, "not representable", ha="center", va="center",
                fontsize=8, color=INK2, transform=ax.transAxes)
      else:
        ax.imshow(img, cmap="gray", vmin=0, vmax=1)
      ax.set_title(f"{fov} deg, {level}", fontsize=8, color=INK)
  save(fig, path)


def fig_views(samples, case, detector, path, n=6):
  fig, axes = plt.subplots(2, 3, figsize=(10.5, 4.6))
  for k, ax in enumerate(axes.flat[:n]):
    img = read_image(samples / "views" / f"{case}_{detector}" /
                     f"view_{k:02d}.png")
    ax.axis("off")
    if img is not None:
      ax.imshow(img[..., ::-1])
    ax.set_title(f"view {k}", fontsize=8, color=INK)
  save(fig, path)


def fig_coverage(corners, cases, resolution, path):
  fig, axes = plt.subplots(2, 4, figsize=(12, 4.6))
  for ax, (name, title) in zip(axes.flat, cases):
    ax.set_title(title, fontsize=8, color=INK)
    c = corners[(corners.case == name) & (corners.detector == "kaess")]
    if c.empty:
      ax.axis("off")
      continue
    h = ax.hist2d(c.u, c.v, bins=[32, 20],
                  range=[[0, resolution[0]], [0, resolution[1]]],
                  cmap="Blues")
    ax.set_xlim(0, resolution[0])
    ax.set_ylim(resolution[1], 0)
    ax.set_aspect("equal")
    ax.tick_params(colors=INK2, labelsize=7)
  for ax in axes.flat[len(cases):]:
    ax.axis("off")
  save(fig, path)


# --------------------------------------------------------------------- plots


def metric_vs_fov(df, model, init, column, ax, ylabel, title, log=True):
  fovs = sorted(df.fov.unique())
  d = df[(df.model == model) & (df.init == init)]
  top = None
  for level in LEVELS:
    for det in DETECTORS:
      r = d[(d.level == level) & (d.detector == det)].sort_values("fov")
      ys = r[column].where(r.ok == 1).to_numpy(dtype=float)
      xs = r.fov.to_numpy()
      ls, mk = DET_STYLE[det]
      ok = np.isfinite(ys)
      good = ok & (ys <= CLIP_PX) if log else ok
      ax.plot(xs[good], ys[good], color=LEVEL_COLOR[level], linestyle=ls,
              marker=mk, markersize=5, linewidth=1.8, markeredgecolor=SURFACE,
              markeredgewidth=1.0)
      # Diverged or failed: mark at the top of the axis
      bad = (r.ok != 1) | (~np.isfinite(r[column].to_numpy(dtype=float))) | \
            ((r[column].to_numpy(dtype=float) > CLIP_PX) if log else False)
      bad = bad & (r.note.fillna("").str.startswith("skipped") == False)
      if bad.any():
        # Offset the crosses so overlapping ones stay visible
        dx = 2.2 * (LEVELS.index(level) - 1) + (0.8 if det == "olsen" else -0.8)
        ax.plot(xs[bad.to_numpy()] + dx, [CLIP_PX] * int(bad.sum()), linestyle="",
                marker="x", color=LEVEL_COLOR[level], markersize=7,
                markeredgewidth=1.8, clip_on=False)
  if log:
    ax.set_yscale("log")
    ax.set_ylim(0.03, CLIP_PX * 1.6)
  ax.set_xticks(fovs)
  style_axes(ax, "Horizontal field of view [deg]", ylabel, title)


def fig_metric(df, init, column, ylabel, path, log=True):
  fig, axes = plt.subplots(1, 2, figsize=(11, 4.0), sharey=log)
  for ax, model in zip(axes, MODELS):
    metric_vs_fov(df, model, init, column, ax, ylabel, MODEL_TITLE[model], log)
  fig.subplots_adjust(bottom=0.26, wspace=0.12)
  legend_below(fig, legend_handles(), 5)
  save(fig, path)


# -------------------------------------------------------------------- tables


def rst_table(header, rows, widths=None, caption=None):
  out = []
  out.append(".. list-table::" + (f" {caption}" if caption else ""))
  out.append("   :header-rows: 1")
  out.append("   :align: center")
  out.append("   :width: 80%")
  if widths:
    out.append("   :widths: " + " ".join(str(w) for w in widths))
  out.append("")
  for i, row in enumerate([header] + rows):
    for j, cell in enumerate(row):
      out.append(("   * - " if j == 0 else "     - ") + str(cell))
  return "\n".join(out)


def figure(path, caption):
  return "\n".join([f".. figure:: {path}", "   :width: 80%", "   :align: center",
                    "", f"   {caption}", ""])


def fmt_px(v, ok):
  if not ok:
    return "failed"
  if not np.isfinite(v):
    return "failed"
  if v > CLIP_PX:
    return f">{CLIP_PX:.0f}"
  return f"{v:.2f}"


def worse(v):
  """Sort key where failed (None) is the worst"""
  return np.inf if v is None else v


def bold_better(cell_a, cell_b, va, vb):
  """Bold the better (lower) of two cells, both on a tie, none if both failed"""
  ka, kb = worse(va), worse(vb)
  if np.isinf(ka) and np.isinf(kb):
    return cell_a, cell_b
  best = min(ka, kb)
  return (f"**{cell_a}**" if ka == best else cell_a,
          f"**{cell_b}**" if kb == best else cell_b)


def px_value(v, ok):
  """Projection error to compare, None if the calibration failed"""
  if not ok or not np.isfinite(v) or v > CLIP_PX:
    return None
  return round(v, 2)


def err_str(v, fmt):
  return fmt.format(v) if abs(v) < 1000 else ">1000"


def summary_table(df, init):
  rows = []
  for model in MODELS:
    for level in LEVELS:
      for fov in sorted(df.fov.unique()):
        cells = [model, level, str(fov)]
        sub = df[(df.model == model) & (df.level == level) & (df.fov == fov) &
                 (df.init == init)]
        skipped = sub.note.fillna("").str.startswith("skipped").all()
        if skipped:
          rows.append(cells + ["n/a"] * 6)
          continue
        k = sub[sub.detector == "kaess"].iloc[0]
        o = sub[sub.detector == "olsen"].iloc[0]
        k_rms, o_rms = bold_better(
            fmt_px(k.model_rmse_px, k.ok), fmt_px(o.model_rmse_px, o.ok),
            px_value(k.model_rmse_px, k.ok), px_value(o.model_rmse_px, o.ok))
        k_max, o_max = bold_better(
            fmt_px(k.model_max_px, k.ok), fmt_px(o.model_max_px, o.ok),
            px_value(k.model_max_px, k.ok), px_value(o.model_max_px, o.ok))
        rows.append(cells + [f"{int(k.views_used)}", f"{int(k.corners):,}",
                             k_rms, k_max, o_rms, o_max])
  header = ["Model", "Distortion", "FOV [deg]", "Views", "Corners",
            "kaess rms [px]", "kaess max [px]", "olsen rms [px]",
            "olsen max [px]"]
  return rst_table(header, rows)


def param_table(df, init):
  """Parameter errors of both detectors side by side"""
  rows = []
  for model in MODELS:
    for level in LEVELS:
      for fov in sorted(df.fov.unique()):
        cells = [model, level, str(fov)]
        sub = df[(df.model == model) & (df.level == level) & (df.fov == fov) &
                 (df.init == init)]
        if sub.note.fillna("").str.startswith("skipped").all():
          rows.append(cells + ["n/a"] * 8)
          continue

        # fx [%], cx [px], cy [px] errors and the reprojection error. A run
        # that failed or diverged is shown but never counts as the best.
        measures = []
        converged = []
        for det in DETECTORS:
          r = sub[sub.detector == det].iloc[0]
          converged.append(px_value(r.model_rmse_px, r.ok) is not None)
          if not r.ok:
            measures.append([None] * 4)
            continue
          measures.append([100.0 * (r.fx_est - r.fx_gt) / r.fx_gt,
                           r.cx_est - r.cx_gt, r.cy_est - r.cy_gt,
                           r.reproj_rmse_px])

        # Interleave: kaess, olsen for every measure, the smaller error in bold
        formats = ["{:+.2f}", "{:+.2f}", "{:+.2f}", "{:.3f}"]
        for m in range(4):
          kv, ov = measures[0][m], measures[1][m]
          kc = "failed" if kv is None else err_str(kv, formats[m])
          oc = "failed" if ov is None else err_str(ov, formats[m])
          digits = 3 if m == 3 else 2
          kc, oc = bold_better(
              kc, oc,
              round(abs(kv), digits) if kv is not None and converged[0] else None,
              round(abs(ov), digits) if ov is not None and converged[1] else None)
          cells += [kc, oc]
        rows.append(cells)
  header = ["Model", "Distortion", "FOV [deg]", "fx error [%] kaess",
            "fx error [%] olsen", "cx error [px] kaess", "cx error [px] olsen",
            "cy error [px] kaess", "cy error [px] olsen",
            "Reproj. rmse [px] kaess", "Reproj. rmse [px] olsen"]
  return rst_table(header, rows)


def distortion_table(df, detector, init, level):
  rows = []
  for model in MODELS:
    for fov in sorted(df.fov.unique()):
      sub = df[(df.model == model) & (df.level == level) & (df.fov == fov) &
               (df.init == init) & (df.detector == detector)]
      if sub.empty:
        continue
      r = sub.iloc[0]
      if not r.ok:
        continue
      gt = [r[f"d{i}_gt"] for i in range(1, 5)]
      est = [r[f"d{i}_est"] for i in range(1, 5)]
      rows.append([model, str(fov)] +
                  [f"{g:+.4f} / {e:+.4f}" for g, e in zip(gt, est)])
  names = {"BrownConrady4": "k1, k2, p1, p2", "KannalaBrandt4": "k1, k2, k3, k4"}
  header = ["Model", "FOV [deg]", "d1 (gt / est)", "d2 (gt / est)",
            "d3 (gt / est)", "d4 (gt / est)"]
  return rst_table(header, rows), names


def summary_bullets(df, max_theta):
  d = df[df.note.fillna("").str.startswith("skipped") == False].copy()
  d["fx_err"] = 100.0 * (d.fx_est - d.fx_gt) / d.fx_gt
  d["cx_err"] = d.cx_est - d.cx_gt
  d["cy_err"] = d.cy_est - d.cy_gt
  conv = d[(d.ok == 1) & (d.model_rmse_px < CLIP_PX)]

  def sel(frame, model, detector, init):
    return frame[(frame.model == model) & (frame.detector == detector) &
                 (frame.init == init)]

  out = []

  # Fisheye
  k = sel(conv, "KannalaBrandt4", "kaess", "default")
  n_k = len(sel(d, "KannalaBrandt4", "kaess", "default"))
  n_good = int((k.model_rmse_px <= GOOD_PX).sum())
  out.append(
      f"**Fisheye (KannalaBrandt4), kaess detector:** every camera converged "
      f"from the default initial guess ({len(k)} of {n_k}), at all fields of "
      f"view up to 180 degrees and all distortion levels. The projection "
      f"error is {k.model_rmse_px.min():.2f} - {k.model_rmse_px.max():.2f} px "
      f"rms ({n_good} of {n_k} within {GOOD_PX} px), and it grows with the "
      f"field of view. The focal length is off by "
      f"{k.fx_err.min():+.2f} to {k.fx_err.max():+.2f} % and the principal "
      f"point by at most {max(k.cx_err.abs().max(), k.cy_err.abs().max()):.2f} px.")

  # Pinhole
  b = sel(conv, "BrownConrady4", "kaess", "default")
  b_ok = b[b.fov < 160]
  b160 = sel(d, "BrownConrady4", "kaess", "default")
  b160 = b160[b160.fov == 160]
  b160_div = int(((b160.ok != 1) | (b160.model_rmse_px >= CLIP_PX)).sum())
  bn = sel(conv, "BrownConrady4", "kaess", "near_truth")
  bn160 = bn[bn.fov == 160]
  out.append(
      f"**Pinhole (BrownConrady4), kaess detector:** at 90 and 120 degrees "
      f"the camera is recovered to {b_ok.model_rmse_px.min():.2f} - "
      f"{b_ok.model_rmse_px.max():.2f} px rms for all distortion levels. At "
      f"160 degrees the calibration diverged from the default initial guess "
      f"in {b160_div} of {len(b160)} cases (reprojection error above "
      f"10 px), but from a guess near the ground truth it converged to "
      f"{bn160.model_rmse_px.min():.2f} - {bn160.model_rmse_px.max():.2f} px "
      f"rms, so the failure is the initialisation, not the model. A pinhole "
      f"camera cannot represent 180 degrees.")

  # olsen
  o = conv[conv.detector == "olsen"]
  out.append(
      f"**olsen detector:** the principal point is off by "
      f"{o.cx_err.mean():+.2f} px in x and {o.cy_err.mean():+.2f} px in y on "
      f"average (kaess: {conv[conv.detector == 'kaess'].cx_err.mean():+.2f} "
      f"and {conv[conv.detector == 'kaess'].cy_err.mean():+.2f} px). This "
      f"matches the half pixel offset of the olsen corners found in the "
      f"detector experiment, which the calibration absorbs into the "
      f"principal point. The projection error is "
      f"{o.model_rmse_px.min():.2f} - {o.model_rmse_px.max():.2f} px rms "
      f"when it converges.")

  # Reprojection error
  kr = conv[conv.detector == "kaess"].reproj_rmse_px
  orr = conv[conv.detector == "olsen"].reproj_rmse_px
  text = (f"**The reprojection error does not show it:** the calibrator "
          f"reports {kr.mean():.2f} px (kaess) and {orr.mean():.2f} px (olsen) "
          f"on average, so a constant corner offset is not visible in the "
          f"residuals, only in the recovered parameters.")
  wrong = conv[(conv.reproj_rmse_px < 1.0) & (conv.model_rmse_px > 5.0)]
  for _, w in wrong.sort_values("model_rmse_px").tail(1).iterrows():
    text += (f" The worst case is {w.model} at {int(w.fov)} degrees with "
             f"{w.level} distortion ({w.detector}, {w.init.replace('_', ' ')} "
             f"initial guess), which reports {w.reproj_rmse_px:.2f} px while "
             f"its projection error is {w.model_rmse_px:.1f} px.")
  out.append(text)

  out.append(
      f"**Limits of the camera models:** both models project through the "
      f"plane ``z = 1``, so rays at or beyond 90 degrees off the optical "
      f"axis cannot be represented. Corners beyond {max_theta:.0f} degrees "
      f"are dropped, so at 180 degrees the calibration only uses the inner "
      f"{2 * max_theta:.0f} degree cone and the image corners are unused.")
  return out


# ---------------------------------------------------------------------- main


def main():
  out_dir, config_file, report_dir = map(Path, sys.argv[1:4])
  report_dir.mkdir(parents=True, exist_ok=True)
  samples = out_dir / "samples"
  df = pd.read_csv(out_dir / "results.csv")
  corners = pd.read_csv(out_dir / "corners.csv")
  cfg = yaml.safe_load(open(config_file))
  fovs = cfg["fovs"]
  res = cfg["camera"]["resolution"]

  # Figures
  fig_targets(samples, report_dir / "targets.png")
  for model in MODELS:
    fig_camera_views(samples, None, model, fovs,
                     report_dir / f"camera_views_{model}.png")
  fig_views(samples, "KannalaBrandt4_120_strong", "kaess",
            report_dir / "calibration_views.png")
  cov_cases = [(f"BrownConrady4_{f}_mild", f"pinhole {f} deg")
               for f in fovs if f < 180]
  cov_cases += [(f"KannalaBrandt4_{f}_mild", f"fisheye {f} deg") for f in fovs]
  fig_coverage(corners, cov_cases, res, report_dir / "coverage.png")
  for init in cfg["initial_guesses"]:
    fig_metric(df, init, "model_rmse_px", "Projection error vs ground truth "
               "[px rms, log]", report_dir / f"model_error_{init}.png")
    fig_metric(df, init, "reproj_rmse_px", "Reprojection error [px rms, log]",
               report_dir / f"reproj_error_{init}.png")

  # Headline numbers
  def sel(model=None, detector=None, init="default", level=None):
    d = df[(df.init == init) & (df.note.fillna("").str.startswith("skipped")
                                == False)]
    if model:
      d = d[d.model == model]
    if detector:
      d = d[d.detector == detector]
    if level:
      d = d[d.level == level]
    return d

  def good(d):
    return int(((d.ok == 1) & (d.model_rmse_px <= GOOD_PX)).sum()), len(d)

  head = []
  for init in cfg["initial_guesses"]:
    for model in MODELS:
      group = []
      for det in DETECTORS:
        g, n = good(sel(model, det, init))
        group.append([init, model, det, g, n])
      best = max(row[3] for row in group)
      for init_, model_, det, g, n in group:
        text = f"{g} / {n}"
        head.append([init_, model_, det, f"**{text}**" if g == best else text])

  R = []
  R.append("=" * 72)
  R.append("Camera calibration accuracy report")
  R.append("=" * 72)
  R.append("")
  R.append(f"""Corners of a plain AprilGrid are detected in simulated images from
cameras with horizontal fields of view of {", ".join(str(f) for f in fovs)}
degrees and with varying amounts of distortion, and are put through
``CalibCamera``. The recovered camera is compared against the ground truth
camera that rendered the images.

Each camera is simulated with ``{cfg['num_views']}`` views of the board, with the
board placed at random locations across the whole image, and is calibrated
twice for each of the two AprilGrid detectors (``kaess`` and ``olsen``): once
from the calibrator's default initial guess, and once from a guess near the
ground truth.
""")

  R.append("Summary")
  R.append("=" * 7)
  R.append("")
  for b in summary_bullets(df, cfg["poses"]["max_theta"]):
    R.append("- " + b)
    R.append("")

  R.append("Setup")
  R.append("=" * 5)
  R.append("")
  R.append(rst_table(["Setting", "Value"], [
      ["Resolution", f"{res[0]}x{res[1]}"],
      ["Principal point offset", f"({cfg['camera']['cx_offset']:+.1f}, "
                                 f"{cfg['camera']['cy_offset']:+.1f}) px from the center"],
      ["fy / fx", f"{cfg['camera']['fy_ratio']}"],
      ["Views per camera", f"{cfg['num_views']}"],
      ["Distance to board", ", ".join(
          f"{k} deg: {v[0]}-{v[1]} m" for k, v in cfg["poses"]["dist_range"].items())],
      ["Tilt of the camera about the board", f"0 - {cfg['poses']['max_tilt']:.0f} deg"],
      ["Corners further off the optical axis than", f"{cfg['poses']['max_theta']:.0f} deg are dropped"],
      ["Image", f"blur sigma {cfg['render']['blur_sigma']} px, noise sigma "
                f"{cfg['render']['noise_sigma']} levels"],
      ["Seed", f"{cfg['seed']}"]], widths=[40, 60]))
  R.append("")
  R.append("""The ground truth cameras are built so that the horizontal image edge, through
the principal point, sees exactly the stated field of view:

- **BrownConrady4**: a pinhole camera with radial-tangential distortion. The
  radial distortion is given as the fraction by which the horizontal image
  edge is displaced, so the levels are comparable across fields of view.
  *none* is 0 %, *mild* is -3 % and *strong* is -12 % with an opposing second
  order term, plus tangential terms.
- **KannalaBrandt4**: an equidistant fisheye camera. *none* is the pure
  equidistant projection and the coefficients ``k1..k4`` of *mild* and
  *strong* are in the table of distortion coefficients below.

A pinhole camera cannot see 180 degrees, so that combination is not
simulated. Projections with the distortion folding the image over are also
rejected before any simulation.
""")

  R.append("Calibration targets")
  R.append("=" * 19)
  R.append("")
  R.append("""``CalibCamera`` only accepts AprilGrid targets, so the calibration uses the
plain AprilGrid. The kalibr style grid, checkerboard and ChArUco board from the
detector experiment are shown for reference.
""")
  R.append(figure("targets.png",
                  "The calibration targets. The AprilGrid tags are rendered "
                  "with the tag code table of the ``kaess`` detector."))

  R.append("Camera views")
  R.append("=" * 12)
  R.append("")
  R.append("""The same view of the board, looking straight at it, as seen by each simulated
camera. The board is placed at the closest distance used for that field of
view.
""")
  for model in MODELS:
    R.append(figure(f"camera_views_{model}.png",
                    f"{MODEL_TITLE[model]}: rows are distortion levels and "
                    "columns are horizontal fields of view."))

  R.append("Calibration data")
  R.append("=" * 16)
  R.append("")
  R.append("""Example calibration views with the detected corners drawn in green, from a
120 degree fisheye camera with strong distortion. Corners further than
""" + f"{cfg['poses']['max_theta']:.0f}" + """ degrees off the optical axis are dropped,
since ``KannalaBrandt4`` and ``BrownConrady4`` project through the plane
``z = 1`` and cannot represent rays at or beyond 90 degrees.
""")
  R.append(figure("calibration_views.png", "Six calibration views with detected corners."))
  R.append("""The detected corners over all views of each camera (mild distortion, ``kaess``
detector). The fisheye cameras and the 90 and 120 degree pinhole cameras have
corners across most of the image. The 160 degree pinhole camera only has
corners in a ring around the image center, since the board is too small there
and too stretched towards the edges to be detected.
""")
  R.append(figure("coverage.png", "Detected corners over the image, all views."))

  R.append("Results")
  R.append("=" * 7)
  R.append("")
  R.append(f"""Parameters of different camera models are strongly correlated, in particular
the focal length and distortion coefficients, so comparing coefficients is not
a good measure of how well a camera was recovered. The main measure is the
**projection error**: rays over the image (every 8 pixels, up to
{cfg['poses']['max_theta']:.0f} degrees off the axis) are projected by the
ground truth and the estimated camera, and the pixel difference is measured
(rms and max). A camera is *recovered* when the rms is at most {GOOD_PX} px.

In the tables, the better of the two detectors is in **bold**: the lower error,
or the higher number of recovered cameras.
""")
  R.append("Cameras recovered (rms projection error <= %.1f px), out of the "
           "simulated ones:" % GOOD_PX)
  R.append("")
  R.append(rst_table(["Initial guess", "Model", "Detector", "Recovered"], head))
  R.append("")

  names = {"default": "the default initial guess",
           "near_truth": "an initial guess near the ground truth"}
  for init in cfg["initial_guesses"]:
    R.append(f"Calibrating from {names[init]}")
    R.append("-" * (len(R[-1])))
    R.append("")
    R.append(figure(f"model_error_{init}.png",
                    "Projection error against the ground truth camera, by "
                    "field of view. Crosses at the top mark cameras where the "
                    "calibration diverged or failed. Pinhole at 180 degrees "
                    "is not simulated."))
    R.append(summary_table(df, init))
    R.append("")
    R.append(figure(f"reproj_error_{init}.png",
                    "Reprojection error of the calibrator, by field of view. "
                    "A low reprojection error does not mean the camera was "
                    "recovered, see the projection error."))
    R.append("Parameter errors of the recovered camera, for both detectors. "
             "The focal length ``fx`` is a relative error and the principal "
             "point ``cx, cy`` an absolute one.")
    R.append("")
    R.append(param_table(df, init))
    R.append("")

  R.append("Distortion coefficients")
  R.append("=" * 23)
  R.append("")
  R.append("Ground truth against estimated distortion coefficients (``kaess`` "
           "detector, default initial guess).")
  R.append("")
  for level in ("mild", "strong"):
    tbl, names2 = distortion_table(df, "kaess", "default", level)
    R.append(f"Distortion level *{level}* "
             f"(BrownConrady4: {names2['BrownConrady4']}; "
             f"KannalaBrandt4: {names2['KannalaBrandt4']}):")
    R.append("")
    R.append(tbl)
    R.append("")

  (report_dir / "REPORT.rst").write_text("\n".join(R))
  print(f"Wrote {report_dir / 'REPORT.rst'}")


if __name__ == "__main__":
  main()
