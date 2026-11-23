#!/usr/bin/env python3
r"""
Script to generate a grid of AprilTags, aka AprilGrid.

Example:

  ./scripts/make_aprilgrid.py --num-targets 2 --tag-rows 6 --tag-cols 6 \
      --output-dir ./targets
"""
import argparse
import sys
from pathlib import Path
from typing import NoReturn

from PIL import Image, ImageDraw

TAG_SIZE = 8  # Size of a tag in pixels, without its white border


def make_aprilgrid(
    target_id: int,
    tag_rows: int,
    tag_cols: int,
    tag_img_dir: str,
    tag_family: str,
    tag_spacing: int,
    output_path: str | Path,
    black_squares: bool = False,
) -> None:
  """Generate an AprilGrid image and save it to disk.

    Parameters
    ----------
    target_id: int
        Calibration target id
    tag_rows : int
        Number of tag rows in the grid.
    tag_cols : int
        Number of tag columns in the grid.
    tag_img_dir : str
        Path to the apriltag-imgs directory containing tag images.
    tag_family : str
        AprilTag family name (e.g. ``tag36h11``).
    tag_spacing : int
        Spacing between tags in pixels.
    output_path : str | Path
        Path to save the output PNG image.
    black_squares : bool
        Add black squares between the tags, in the gaps where four tags meet,
        and around the outside of the grid, where the gaps would continue
        (kalibr style AprilGrid). The squares are ``tag_spacing`` pixels wide,
        so the spacing must be greater than 0, and the image gets a margin of
        ``tag_spacing`` pixels on every side to fit the outer squares.
    """
  if black_squares and tag_spacing <= 0:
    raise ValueError("black squares need a tag spacing greater than 0!")

  # Tag images are named after the family, e.g. tag36h11 -> tag36_11_00000.png
  img_prefix = tag_family
  for name in ("Circle", "Custom", "Standard"):
    img_prefix = img_prefix.replace(name, "")
  img_prefix = img_prefix.replace("h", "_")

  # Paste the tags, the first tag is at the bottom left. The black squares on
  # the perimeter go in a margin around the tags
  pitch = TAG_SIZE + tag_spacing
  margin = tag_spacing if black_squares else 0
  img_w = TAG_SIZE * tag_cols + tag_spacing * (tag_cols - 1) + 2 * margin
  img_h = TAG_SIZE * tag_rows + tag_spacing * (tag_rows - 1) + 2 * margin
  mosaic = Image.new(mode="RGB", size=(img_w, img_h), color=(255, 255, 255))

  tag_id = target_id * (tag_rows * tag_cols)
  for i in range(tag_rows - 1, -1, -1):
    for j in range(tag_cols):
      tag_img_path = (f"{tag_img_dir}/{tag_family}/"
                      f"{img_prefix}_{str(tag_id).zfill(5)}.png")
      tag_img = Image.open(tag_img_path)

      # Crop the white border of the tag image
      tag_w, tag_h = tag_img.size
      tag = tag_img.crop((1, 1, tag_w - 1, tag_h - 1))
      mosaic.paste(tag, box=(margin + pitch * j, margin + pitch * i))
      tag_id += 1

  # Black squares at every intersection of the gaps between the tags, the
  # outer ones are on the perimeter of the grid
  if black_squares:
    draw = ImageDraw.Draw(mosaic)
    for i in range(tag_rows + 1):
      for j in range(tag_cols + 1):
        x = margin + pitch * j - tag_spacing
        y = margin + pitch * i - tag_spacing
        draw.rectangle((x, y, x + tag_spacing - 1, y + tag_spacing - 1),
                       fill=(0, 0, 0))

  mosaic.save(output_path)


class HelpFormatter(argparse.RawDescriptionHelpFormatter):
  """Print the help 80 columns wide, and keep the format of the example."""

  def __init__(self, prog: str) -> None:
    super().__init__(prog, width=80)


class ArgumentParser(argparse.ArgumentParser):
  """Print the full help, with the example, if the arguments are not valid."""

  def error(self, message: str) -> NoReturn:
    self.print_help(sys.stderr)
    self.exit(2, f"\n{self.prog}: error: {message}\n")


EXAMPLE = """\
example:
  Make 2 targets of 6x6 tags, with black squares between and around the tags
  (kalibr style), as aprilgrid-target0.png and aprilgrid-target1.png in
  ./targets:

    mkdir -p targets
    ./scripts/make_aprilgrid.py --num-targets 2 --tag-rows 6 --tag-cols 6 \\
        --output-dir ./targets
"""

if __name__ == "__main__":
  parser = ArgumentParser(
      description="Generate AprilGrid calibration targets.",
      epilog=EXAMPLE,
      formatter_class=HelpFormatter,
  )
  parser.add_argument("--num-targets", type=int, required=True)
  parser.add_argument("--tag-rows", type=int, default=10)
  parser.add_argument("--tag-cols", type=int, default=10)
  parser.add_argument("--tag-img_dir", default="./deps/src/apriltag-imgs")
  parser.add_argument("--tag-family", default="tag36h11")
  parser.add_argument("--tag-spacing", type=int, default=2)
  parser.add_argument("--black-squares", type=bool, default=True)
  parser.add_argument("--output-dir", type=Path, default="./")
  args = parser.parse_args()

  # Make calibration targets
  for target_id in range(args.num_targets):
    make_aprilgrid(
        target_id=target_id,
        tag_rows=args.tag_rows,
        tag_cols=args.tag_cols,
        tag_img_dir=args.tag_img_dir,
        tag_family=args.tag_family,
        tag_spacing=args.tag_spacing,
        black_squares=args.black_squares,
        output_path=args.output_dir / f"./aprilgrid-target{target_id}.png",
    )
