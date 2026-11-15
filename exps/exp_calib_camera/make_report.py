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


def data_table(df, init, fovs):
  """Views used and corners per camera and target"""
  rows = []
  for model in MODELS:
    for level in LEVELS:
      for fov in fovs:
        cells = [model, level, str(fov)]
        rs = [row_of(df, model, level, fov, init, k) for k in KEYS]
        if all(str(r.note).startswith("skipped") for r in rs):
          rows.append(cells + ["n/a"] * len(KEYS))
          continue
        rows.append(cells + [f"{int(r.views_used)} / {int(r.corners):,}"
                             for r in rs])
  header = ["Model", "Distortion", "FOV [deg]"] + \
      [f"{LABEL[k]}: views / corners" for k in KEYS]
  return rst_table(header, rows)


def converged(d):
  return d[(d.ok == 1) & (d.model_rmse_px < CLIP_PX)]


def recovered_table(df, inits):
  rows = []
  d = df[~is_skipped(df)]
  for init in inits:
    for model in MODELS:
      counts, texts = [], []
      for key in KEYS:
        sub = d[(d.init == init) & (d.model == model) & (d.target == key)]
        n_good = int(((sub.ok == 1) & (sub.model_rmse_px <= GOOD_PX)).sum())
        counts.append(n_good)
        texts.append(f"{n_good} / {len(sub)}")
      cells = bold_best(texts, counts, "max")
      for key, text in zip(KEYS, cells):
        rows.append([init, model, LABEL[key], text])
  return rst_table(["Initial guess", "Model", "Target", "Recovered"], rows)


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
      for t, key in enumerate(KEYS):
        s = stats[t]
        rows.append([init, model, LABEL[key]] + [c[t] for c in columns] +
                    ["-" if s is None else f"{s[4]} / {s[5]}"])
  header = ["Initial guess", "Model", "Target", "fx error [%]", "cx error [px]",
            "cy error [px]", "Reproj. rmse [px]", "Converged"]
  return rst_table(header, rows)


def distortion_table(df, key, init, level, fovs):
  rows = []
  for model in MODELS:
    for fov in fovs:
      r = row_of(df, model, level, fov, init, key)
      if not r.ok:
        continue
      gt = [r[f"d{i}_gt"] for i in range(1, 5)]
      est = [r[f"d{i}_est"] for i in range(1, 5)]
      rows.append([model, str(fov)] +
                  [f"{g:+.4f} / {e:+.4f}" for g, e in zip(gt, est)])
  header = ["Model", "FOV [deg]", "d1 (gt / est)", "d2 (gt / est)",
            "d3 (gt / est)", "d4 (gt / est)"]
  return rst_table(header, rows)


# ------------------------------------------------------------------- summary


def summary_bullets(df, cfg):
  max_theta = cfg["poses"]["max_theta"]
  d = df[~is_skipped(df)].copy()
  d["cx_err"] = d.cx_est - d.cx_gt
  d["cy_err"] = d.cy_est - d.cy_gt
  out = []

  def sel(model, key, init, below=None, at=None):
    s = d[(d.model == model) & (d.target == key) & (d.init == init)]
    if below is not None:
      s = s[s.fov < below]
    if at is not None:
      s = s[s.fov == at]
    return s

  def per_target(model, init, **kwargs):
    parts = []
    for key in KEYS:
      s = sel(model, key, init, **kwargs)
      ok = converged(s)
      if ok.empty:
        parts.append(f"{LABEL[key]}: none converged")
        continue
      n_good = int((ok.model_rmse_px <= GOOD_PX).sum())
      parts.append(f"{LABEL[key]}: {ok.model_rmse_px.median():.2f} px "
                   f"median, {n_good} of {len(s)} within {GOOD_PX} px")
    return "; ".join(parts)

  fisheye = d[(d.model == "KannalaBrandt4") & (d.init == "default")]
  n_conv = len(converged(fisheye))
  if n_conv == len(fisheye):
    status = "every camera converged, for every target, up to 180 degrees"
  else:
    status = f"{n_conv} of {len(fisheye)} calibrations converged"
  out.append("**Fisheye (KannalaBrandt4), from the default initial guess:** " +
             status + ". Projection error against the ground truth, " +
             per_target("KannalaBrandt4", "default") + ".")

  out.append("**Pinhole (BrownConrady4) at 90 and 120 degrees, from the "
             "default initial guess:** " +
             per_target("BrownConrady4", "default", below=160) + ".")

  # Pinhole at 160 degrees
  default160 = []
  near160 = []
  for key in KEYS:
    sd = sel("BrownConrady4", key, "default", at=160)
    sn = sel("BrownConrady4", key, "near_truth", at=160)
    default160.append(f"{LABEL[key]} {len(converged(sd))} of {len(sd)}")
    cn = converged(sn)
    near160.append(f"{LABEL[key]} {len(cn)} of {len(sn)}, median "
                   f"{cn.model_rmse_px.median():.1f} px" if not cn.empty
                   else f"{LABEL[key]} {len(cn)} of {len(sn)}")
  out.append("**Pinhole at 160 degrees:** cameras that converged (projection "
             "error below %d px) from the default initial guess: " % CLIP_PX +
             ", ".join(default160) + ". From a guess near the ground truth: " +
             ", ".join(near160) + ". A camera that converges from the "
             "near-truth guess but not from the default one fails because of "
             "the initialisation, not the camera model. A pinhole camera "
             "cannot represent 180 degrees.")

  # Principal point
  pp = []
  for key in KEYS:
    ok = converged(d[(d.target == key) & (d.init == "default")])
    pp.append(f"{LABEL[key]} {ok.cx_err.mean():+.2f} / {ok.cy_err.mean():+.2f}")
  pp_means = []
  for key in KEYS:
    ok = converged(d[(d.target == key) & (d.init == "default")])
    pp_means += [abs(ok.cx_err.mean()), abs(ok.cy_err.mean())]
  out.append(
      "**Principal point:** mean error in x / y [px] over the converged "
      "cameras: " + "; ".join(pp) + f". No target is off by more than "
      f"{max(pp_means):.2f} px on average, so a constant corner offset does "
      f"not show up in any of them.")

  # Reprojection error
  rp = []
  for key in KEYS:
    ok = converged(d[(d.target == key) & (d.init == "default")])
    rp.append(f"{LABEL[key]} {ok.reproj_rmse_px.mean():.2f}")
  text = ("**The reprojection error is not a measure of accuracy:** the mean "
          "reprojection error of the calibrator [px] is " + ", ".join(rp) +
          ". A calibration can have a low reprojection error and still be a "
          "wrong camera.")
  wrong = converged(d)
  wrong = wrong[(wrong.reproj_rmse_px < 1.0) & (wrong.model_rmse_px > 5.0)]
  for _, w in wrong.sort_values("model_rmse_px").tail(1).iterrows():
    text += (f" The worst case is {w.model} at {int(w.fov)} degrees with "
             f"{w.level} distortion ({LABEL[w.target]}, "
             f"{w.init.replace('_', ' ')} initial guess), which reports "
             f"{w.reproj_rmse_px:.2f} px while its projection error is "
             f"{w.model_rmse_px:.1f} px.")
  out.append(text)

  # Views and coverage
  views = []
  for key in KEYS:
    s = d[(d.target == key) & (d.init == "default")]
    views.append(f"{LABEL[key]} {s.views_used.mean():.0f} views and "
                 f"{s.corners.mean():,.0f} corners")
  out.append("**Data:** the checkerboard is only detected with the whole board "
             "in the image, so it only uses poses where the whole board is "
             "visible, and its corners stay close to the image center, where "
             "the distortion is weakest (see the coverage figure). The ChArUco "
             "board can be partly visible but has fewer corners than the "
             "AprilGrid. Mean per camera: " + "; ".join(views) + ".")

  out.append(
      f"**Limits of the camera models:** both models project through the "
      f"plane ``z = 1``, so rays at or beyond 90 degrees off the optical "
      f"axis cannot be represented. Corners beyond {max_theta:.0f} degrees "
      f"are dropped, so at 180 degrees the calibration only uses the inner "
      f"{2 * max_theta:.0f} degree cone and the image corners are unused.")
  return out


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
  most_recovered = max(KEYS, key=lambda k: stats[k]["recovered"])
  worst_pp = max((k for k in KEYS if stats[k]["pp"] is not None),
                 key=lambda k: stats[k]["pp"])
  n160 = {k: len(converged(d[(d.target == k) & (d.model == "BrownConrady4") &
                             (d.fov == 160)])) for k in KEYS}
  total160 = int((d[(d.target == KEYS[0]) & (d.model == "BrownConrady4") &
                    (d.fov == 160)]).shape[0])
  recovered = (f"{stats[most_recovered]['recovered']} of "
               f"{stats[most_recovered]['total']}")
  if top == most_recovered:
    first = (f"**Lowest error most often:** {LABEL[top]}, for "
             f"{stats[top]['best_for']} of the {n_cameras} cameras, and it "
             f"recovers the most cameras within {GOOD_PX} px, {recovered}.")
  else:
    first = (f"**Lowest error most often:** {LABEL[top]}, for "
             f"{stats[top]['best_for']} of the {n_cameras} cameras. "
             f"{LABEL[most_recovered]} recovers the most cameras within "
             f"{GOOD_PX} px, {recovered}.")
  bullets = [
      first,
      f"**Principal point:** {LABEL[worst_pp]} has the largest median error, "
      f"{stats[worst_pp]['pp']:.2f} px.",
  ]
  if total160:
    bullets.append(
        "**Pinhole at 160 degrees**, cameras that converged from the default "
        "initial guess: " +
        ", ".join(f"{LABEL[k]} {n160[k]} of {total160}" for k in KEYS) + ".")
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
    fig_metric(df, init, "reproj_rmse_px", "Reprojection error [px rms, log]",
               fovs, report_dir / f"reproj_error_{init}.png")

  R = []
  R.append("=" * 72)
  R.append("Camera calibration accuracy report")
  R.append("=" * 72)
  R.append("")
  R.append(f"""Corners of an AprilGrid, a checkerboard and a ChArUco board are detected in
simulated images from cameras with horizontal fields of view of
{", ".join(str(f) for f in fovs)} degrees and with varying amounts of
distortion, and are put through ``CalibCamera``. The recovered camera is
compared against the ground truth camera that rendered the images.

Each camera is simulated with ``{cfg['num_views']}`` views of each target, with
the target placed at random locations across the whole image. The AprilGrid is
detected with both the ``kaess`` and the ``olsen`` detector. Every dataset is
calibrated twice: once from the calibrator's default initial guess, and once
from a guess near the ground truth.
""")

  R.append("Summary")
  R.append("=" * 7)
  R.append("")
  for b in summary_bullets(df, cfg):
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
      ["Views per camera and target", f"{cfg['num_views']}"],
      ["Distance to board", ", ".join(
          f"{k} deg: {v[0]}-{v[1]} m" for k, v in cfg["poses"]["dist_range"].items())],
      ["Tilt of the camera about the board", f"0 - {cfg['poses']['max_tilt']:.0f} deg"],
      ["Corners further off the optical axis than", f"{cfg['poses']['max_theta']:.0f} deg are dropped"],
      ["Image", f"blur sigma {cfg['render']['blur_sigma']} px, noise sigma "
                f"{cfg['render']['noise_sigma']} levels"],
      ["Corner refinement", refine_text(cfg)],
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
  R.append("""All targets are detected with the library detectors and calibrated with
``CalibCamera``. The target sizes are set in ``exps/common``.

- **AprilGrid**: 6x6 tags of 80 mm with 25 % spacing, plain (no black squares
  between the tags). Detected with ``kaess`` and ``olsen``, which need
  different tag code tables, so the rendered grid is a different one for each.
- **Checkerboard**: 9x8 squares of 70 mm, 8x7 inner corners. The detector
  needs the whole board in the image, so only poses where the whole board is
  visible are used.
- **ChArUco**: 9x8 squares of 70 mm with 56 mm markers (5x5 dictionary). The
  markers identify the corners, so the board can be partly visible.

The kalibr style grid (black squares between the tags) is shown for reference
only, ``olsen`` detects few of its tags.
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
  R.append(f"""Example calibration views of a 120 degree fisheye camera with strong
distortion, for each target. Detected corners are drawn in green, and corners
that do not match the ground truth in red. Corners further than
{cfg['poses']['max_theta']:.0f} degrees off the optical axis are dropped, since
``KannalaBrandt4`` and ``BrownConrady4`` project through the plane ``z = 1`` and
cannot represent rays at or beyond 90 degrees.
""")
  R.append(figure("calibration_views.png", "Three calibration views per target."))
  R.append("""The detected corners over all views of each camera (mild distortion). The
AprilGrid corners cover most of the image. The checkerboard corners stay near
the center because the whole board has to be visible, and the ChArUco corners
are spread out but fewer. For the 160 degree pinhole camera, which is not
shown, detections are limited to a ring around the image center.
""")
  R.append(figure("coverage.png", "Detected corners over the image, all views."))
  R.append("Views used and corners per camera (default initial guess):")
  R.append("")
  R.append(data_table(df, inits[0], fovs))
  R.append("")

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

In the tables, the best target of each row or column is in **bold**: the lowest
error, or the most recovered cameras. A calibration that failed or diverged
never counts as the best.
""")
  table, bullets = quick_summary(df, inits[0])
  R.append("Quick summary")
  R.append("-" * 13)
  R.append("")
  R.append("Main results of every target over all simulated cameras, from the "
           "default initial guess. The projection and principal point errors "
           "are medians over the cameras that converged.")
  R.append("")
  R.append(table)
  R.append("")
  for b in bullets:
    R.append("- " + b)
    R.append("")

  R.append("Cameras recovered")
  R.append("-" * 17)
  R.append("")
  R.append("Cameras recovered (rms projection error <= %.1f px), out of the "
           "simulated ones:" % GOOD_PX)
  R.append("")
  R.append(recovered_table(df, inits))
  R.append("")
  R.append("Parameter errors of the recovered camera, as the median over the "
           "cameras that converged (rms projection error below %d px). The "
           "focal length ``fx`` is a relative error and the principal point "
           "``cx, cy`` an absolute one." % CLIP_PX)
  R.append("")
  R.append(param_table(df, inits))
  R.append("")

  names = {"default": "the default initial guess",
           "near_truth": "an initial guess near the ground truth"}
  for init in inits:
    title = f"Calibrating from {names[init]}"
    R.append(title)
    R.append("-" * len(title))
    R.append("")
    R.append(figure(f"model_error_{init}.png",
                    "Projection error against the ground truth camera, by "
                    "field of view. Crosses at the top mark cameras where the "
                    "calibration diverged or failed. Pinhole at 180 degrees "
                    "is not simulated."))
    R.append("Projection error, rms over the image [px]:")
    R.append("")
    R.append(summary_table(df, init, "model_rmse_px", fovs))
    R.append("")
    R.append("Projection error, max over the image [px]:")
    R.append("")
    R.append(summary_table(df, init, "model_max_px", fovs))
    R.append("")
    R.append(figure(f"reproj_error_{init}.png",
                    "Reprojection error of the calibrator, by field of view. "
                    "A low reprojection error does not mean the camera was "
                    "recovered, see the projection error."))

  R.append("Distortion coefficients")
  R.append("=" * 23)
  R.append("")
  R.append("Ground truth against estimated distortion coefficients (AprilGrid "
           "with the ``kaess`` detector, default initial guess).")
  R.append("")
  names2 = {"BrownConrady4": "k1, k2, p1, p2", "KannalaBrandt4": "k1, k2, k3, k4"}
  for level in ("mild", "strong"):
    R.append(f"Distortion level *{level}* "
             f"(BrownConrady4: {names2['BrownConrady4']}; "
             f"KannalaBrandt4: {names2['KannalaBrandt4']}):")
    R.append("")
    R.append(distortion_table(df, "aprilgrid_kaess", "default", level, fovs))
    R.append("")

  (report_dir / "REPORT.rst").write_text("\n".join(R))
  print(f"Wrote {report_dir / 'REPORT.rst'}")


if __name__ == "__main__":
  main()
