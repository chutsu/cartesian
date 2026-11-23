#!/usr/bin/env python3
"""
Download a dataset: EuRoC MAV, KITTI Odometry or KITTI Raw.

  ./download_dataset.py --dataset euroc                     # EuRoC (zip)
  ./download_dataset.py --dataset euroc --format rosbags    # EuRoC ROS bags
  ./download_dataset.py --dataset kitti-odom                # KITTI Odometry
  ./download_dataset.py --dataset kitti-raw                 # KITTI Raw
  ./download_dataset.py --dataset kitti-raw --dst-dir /tmp  # Into /tmp

The datasets are downloaded into `<dst-dir>/euroc`, `<dst-dir>/kitti_odometry`
or `<dst-dir>/kitti_raw`.
"""
import argparse
import os
import sys
import urllib.request
import zipfile
from typing import Dict, List, Optional

from tqdm import tqdm

################################################################################
# DOWNLOAD UTILS
################################################################################


def download(url: str, save_dir: str) -> None:
  """Download a file with a progress bar.

  The file is saved as `save_dir/<basename of url>`.

  Args:
    url: URL of the file to download.
    save_dir: Directory to save the file in, it must already exist.

  Returns:
    None.
  """
  # Get the size of the file first
  response = urllib.request.urlopen(url)
  total_size = int(response.info().get('Content-Length').strip())

  # Initialize the progress bar
  desc = f"Downloading [{os.path.basename(url)}]"
  pbar = tqdm(total=total_size, unit='B', unit_scale=True, desc=desc)
  response = None
  out_file = None

  try:
    # Open the URL and the output file
    response = urllib.request.urlopen(url)
    save_path = os.path.join(save_dir, os.path.basename(url))
    out_file = open(save_path, 'wb')

    # Read and write the file in chunks
    chunk_size = 1024
    while True:
      chunk = response.read(chunk_size)
      if not chunk:
        break
      out_file.write(chunk)
      pbar.update(len(chunk))

  finally:
    # Clean up: Close the file and the response
    pbar.close()

    if out_file:
      out_file.close()

    if response:
      response.close()


def extract_zip(src: str, dst: Optional[str] = None) -> None:
  """Extract a zip file.

  Args:
    src: Path to the zip file.
    dst: Directory to extract into. Defaults to a directory next to the zip
      file, named after it without the `.zip` extension.

  Returns:
    None.
  """
  dst_path = dst
  if dst_path is None:
    dir_name = os.path.basename(src).replace(".zip", "")
    dst_path = os.path.join(os.path.dirname(src), dir_name)

  with zipfile.ZipFile(src, 'r') as zip_ref:
    zip_ref.extractall(dst_path)


################################################################################
# EUROC DATASET
################################################################################


def download_euroc_sequences(dst_dir: str) -> None:
  """Download the EuRoC MAV sequences as zip files and extract them.

  The zip files are saved in `<dst_dir>/euroc/archive`, and each sequence is
  extracted into `<dst_dir>/euroc/<name>`, where the name has the `_easy`,
  `_medium` and `_difficult` tags removed, e.g. `MH_01`.

  Args:
    dst_dir: Directory to download the dataset into.

  Returns:
    None.
  """
  base_url = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
  seqs = [
      "calibration_datasets/cam_april/cam_april.zip",
      "calibration_datasets/cam_checkerboard/cam_checkerboard.zip",
      "calibration_datasets/imu_april/imu_april.zip",
      "machine_hall/MH_01_easy/MH_01_easy.zip",
      "machine_hall/MH_02_easy/MH_02_easy.zip",
      "machine_hall/MH_03_medium/MH_03_medium.zip",
      "machine_hall/MH_04_difficult/MH_04_difficult.zip",
      "machine_hall/MH_05_difficult/MH_05_difficult.zip",
      "vicon_room1/V1_01_easy/V1_01_easy.zip",
      "vicon_room1/V1_02_medium/V1_02_medium.zip",
      "vicon_room1/V1_03_difficult/V1_03_difficult.zip",
      "vicon_room2/V2_01_easy/V2_01_easy.zip",
      "vicon_room2/V2_02_medium/V2_02_medium.zip",
      "vicon_room2/V2_03_difficult/V2_03_difficult.zip",
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "euroc")
  archive_path = os.path.join(ds_path, "archive")
  os.makedirs(ds_path, exist_ok=True)
  os.makedirs(archive_path, exist_ok=True)

  # Download sequences
  for seq in seqs:
    # Download
    download(os.path.join(base_url, seq), archive_path)

    # Create sequence directory
    seq_name = os.path.basename(seq).replace(".zip", "")
    seq_name = seq_name.replace("_easy", "")
    seq_name = seq_name.replace("_medium", "")
    seq_name = seq_name.replace("_difficult", "")
    seq_dir = os.path.join(ds_path, seq_name)
    os.makedirs(seq_dir, exist_ok=True)

    # Extract sequence into directory
    zip_path = os.path.join(archive_path, os.path.basename(seq))
    extract_zip(zip_path, seq_dir)


def download_euroc_rosbags(dst_dir: str) -> None:
  """Download the EuRoC MAV ROS bags.

  The bags are saved in `<dst_dir>/euroc`, renamed without the `_easy`,
  `_medium` and `_difficult` tags, e.g. `MH_01.bag`.

  Args:
    dst_dir: Directory to download the dataset into.

  Returns:
    None.
  """
  base_url = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
  bags = [
      "machine_hall/MH_01_easy/MH_01_easy.bag",
      "machine_hall/MH_02_easy/MH_02_easy.bag",
      "machine_hall/MH_03_medium/MH_03_medium.bag",
      "machine_hall/MH_04_difficult/MH_04_difficult.bag",
      "machine_hall/MH_05_difficult/MH_05_difficult.bag",
      "vicon_room1/V1_01_easy/V1_01_easy.bag",
      "vicon_room1/V1_02_medium/V1_02_medium.bag",
      "vicon_room1/V1_03_difficult/V1_03_difficult.bag",
      "vicon_room2/V2_01_easy/V2_01_easy.bag",
      "vicon_room2/V2_02_medium/V2_02_medium.bag",
      "vicon_room2/V2_03_difficult/V2_03_difficult.bag",
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "euroc")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequences
  for bag in bags:
    # Download
    download(os.path.join(base_url, bag), ds_path)

    # Rename bag file without the easy, medium and difficult tags
    old_bag_path = os.path.join(ds_path, os.path.basename(bag))
    new_bag_path = old_bag_path.replace("_easy", "")
    new_bag_path = new_bag_path.replace("_medium", "")
    new_bag_path = new_bag_path.replace("_difficult", "")
    os.rename(old_bag_path, new_bag_path)


################################################################################
# KITTI ODOMETRY DATASET
################################################################################


def download_kitti_odometry(dst_dir: str) -> None:
  """Download the KITTI Odometry dataset and extract it.

  The calibration, grayscale images and poses zip files are downloaded and
  extracted into `<dst_dir>/kitti_odometry`.

  Args:
    dst_dir: Directory to download the dataset into.

  Returns:
    None.
  """
  kitti_url = "https://s3.eu-central-1.amazonaws.com/avg-kitti"
  sequences = [
      "data_odometry_calib.zip", "data_odometry_gray.zip",
      "data_odometry_poses.zip"
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "kitti_odometry")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequence
  for seq in sequences:
    download(f"{kitti_url}/{seq}", ds_path)
    extract_zip(f"{ds_path}/{seq}", ds_path)


################################################################################
# KITTI RAW DATASET
################################################################################


def download_kitti_raw(dst_dir: str) -> None:
  """Download the KITTI Raw dataset and extract it.

  The calibration files, the synchronized drives and the extracted drives used
  for calibration are downloaded and extracted into `<dst_dir>/kitti_raw`.

  Args:
    dst_dir: Directory to download the dataset into.

  Returns:
    None.
  """
  kitti_url = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"
  sequences = [
      "2011_09_26_calib.zip", "2011_09_26_drive_0001", "2011_09_26_drive_0002",
      "2011_09_26_drive_0005", "2011_09_26_drive_0009", "2011_09_26_drive_0011",
      "2011_09_26_drive_0013", "2011_09_26_drive_0014", "2011_09_26_drive_0015",
      "2011_09_26_drive_0017", "2011_09_26_drive_0018", "2011_09_26_drive_0019",
      "2011_09_26_drive_0020", "2011_09_26_drive_0022", "2011_09_26_drive_0023",
      "2011_09_26_drive_0027", "2011_09_26_drive_0028", "2011_09_26_drive_0029",
      "2011_09_26_drive_0032", "2011_09_26_drive_0035", "2011_09_26_drive_0036",
      "2011_09_26_drive_0039", "2011_09_26_drive_0046", "2011_09_26_drive_0048",
      "2011_09_26_drive_0051", "2011_09_26_drive_0052", "2011_09_26_drive_0056",
      "2011_09_26_drive_0057", "2011_09_26_drive_0059", "2011_09_26_drive_0060",
      "2011_09_26_drive_0061", "2011_09_26_drive_0064", "2011_09_26_drive_0070",
      "2011_09_26_drive_0079", "2011_09_26_drive_0084", "2011_09_26_drive_0086",
      "2011_09_26_drive_0087", "2011_09_26_drive_0091", "2011_09_26_drive_0093",
      "2011_09_26_drive_0095", "2011_09_26_drive_0096", "2011_09_26_drive_0101",
      "2011_09_26_drive_0104", "2011_09_26_drive_0106", "2011_09_26_drive_0113",
      "2011_09_26_drive_0117", "2011_09_26_drive_0119", "2011_09_28_calib.zip",
      "2011_09_28_drive_0001", "2011_09_28_drive_0002", "2011_09_28_drive_0016",
      "2011_09_28_drive_0021", "2011_09_28_drive_0034", "2011_09_28_drive_0035",
      "2011_09_28_drive_0037", "2011_09_28_drive_0038", "2011_09_28_drive_0039",
      "2011_09_28_drive_0043", "2011_09_28_drive_0045", "2011_09_28_drive_0047",
      "2011_09_28_drive_0053", "2011_09_28_drive_0054", "2011_09_28_drive_0057",
      "2011_09_28_drive_0065", "2011_09_28_drive_0066", "2011_09_28_drive_0068",
      "2011_09_28_drive_0070", "2011_09_28_drive_0071", "2011_09_28_drive_0075",
      "2011_09_28_drive_0077", "2011_09_28_drive_0078", "2011_09_28_drive_0080",
      "2011_09_28_drive_0082", "2011_09_28_drive_0086", "2011_09_28_drive_0087",
      "2011_09_28_drive_0089", "2011_09_28_drive_0090", "2011_09_28_drive_0094",
      "2011_09_28_drive_0095", "2011_09_28_drive_0096", "2011_09_28_drive_0098",
      "2011_09_28_drive_0100", "2011_09_28_drive_0102", "2011_09_28_drive_0103",
      "2011_09_28_drive_0104", "2011_09_28_drive_0106", "2011_09_28_drive_0108",
      "2011_09_28_drive_0110", "2011_09_28_drive_0113", "2011_09_28_drive_0117",
      "2011_09_28_drive_0119", "2011_09_28_drive_0121", "2011_09_28_drive_0122",
      "2011_09_28_drive_0125", "2011_09_28_drive_0126", "2011_09_28_drive_0128",
      "2011_09_28_drive_0132", "2011_09_28_drive_0134", "2011_09_28_drive_0135",
      "2011_09_28_drive_0136", "2011_09_28_drive_0138", "2011_09_28_drive_0141",
      "2011_09_28_drive_0143", "2011_09_28_drive_0145", "2011_09_28_drive_0146",
      "2011_09_28_drive_0149", "2011_09_28_drive_0153", "2011_09_28_drive_0154",
      "2011_09_28_drive_0155", "2011_09_28_drive_0156", "2011_09_28_drive_0160",
      "2011_09_28_drive_0161", "2011_09_28_drive_0162", "2011_09_28_drive_0165",
      "2011_09_28_drive_0166", "2011_09_28_drive_0167", "2011_09_28_drive_0168",
      "2011_09_28_drive_0171", "2011_09_28_drive_0174", "2011_09_28_drive_0177",
      "2011_09_28_drive_0179", "2011_09_28_drive_0183", "2011_09_28_drive_0184",
      "2011_09_28_drive_0185", "2011_09_28_drive_0186", "2011_09_28_drive_0187",
      "2011_09_28_drive_0191", "2011_09_28_drive_0192", "2011_09_28_drive_0195",
      "2011_09_28_drive_0198", "2011_09_28_drive_0199", "2011_09_28_drive_0201",
      "2011_09_28_drive_0204", "2011_09_28_drive_0205", "2011_09_28_drive_0208",
      "2011_09_28_drive_0209", "2011_09_28_drive_0214", "2011_09_28_drive_0216",
      "2011_09_28_drive_0220", "2011_09_28_drive_0222", "2011_09_28_drive_0225",
      "2011_09_29_calib.zip", "2011_09_29_drive_0004", "2011_09_29_drive_0026",
      "2011_09_29_drive_0071", "2011_09_29_drive_0108", "2011_09_30_calib.zip",
      "2011_09_30_drive_0016", "2011_09_30_drive_0018", "2011_09_30_drive_0020",
      "2011_09_30_drive_0027", "2011_09_30_drive_0028", "2011_09_30_drive_0033",
      "2011_09_30_drive_0034", "2011_09_30_drive_0072", "2011_10_03_calib.zip",
      "2011_10_03_drive_0027", "2011_10_03_drive_0034", "2011_10_03_drive_0042",
      "2011_10_03_drive_0047", "2011_10_03_drive_0058"
  ]

  calib_sequences = [
    f"{kitti_url}/2011_09_26_drive_0119/2011_09_26_drive_0119_extract.zip",
    f"{kitti_url}/2011_09_28_drive_0225/2011_09_28_drive_0225_extract.zip",
    f"{kitti_url}/2011_09_29_drive_0108/2011_09_29_drive_0108_extract.zip",
    f"{kitti_url}/2011_09_30_drive_0072/2011_09_30_drive_0072_extract.zip",
    f"{kitti_url}/2011_10_03_drive_0058/2011_10_03_drive_0058_extract.zip",
  ]

  # Make destination folder if it doesn't exist already
  ds_path = os.path.join(dst_dir, "kitti_raw")
  os.makedirs(ds_path, exist_ok=True)

  # Download sequences
  for seq in sequences:
    # Download
    url_path = seq if seq[-4:] == ".zip" else f"{seq}/{seq}_sync.zip"
    download(f"{kitti_url}/{url_path}", ds_path)

    # Extract zip file
    zip_file = os.path.basename(url_path)
    extract_zip(f"{ds_path}/{zip_file}", ds_path)

  # Download calib seqeunces
  for url_path in calib_sequences:
    # Download
    download(url_path, ds_path)

    # Extract zip file
    zip_file = os.path.basename(url_path)
    extract_zip(f"{ds_path}/{zip_file}", ds_path)


################################################################################
# COMMAND LINE
################################################################################

DATASETS: Dict[str, str] = {
    "euroc": "EuRoC MAV dataset",
    "kitti-odom": "KITTI Odometry dataset",
    "kitti-raw": "KITTI Raw dataset",
}


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
  """Parse the command line arguments.

  Args:
    argv: Arguments to parse. Defaults to `sys.argv[1:]`.

  Returns:
    The parsed arguments, with the attributes `dataset`, `dst_dir` and
    `format`.
  """
  parser = argparse.ArgumentParser(
      description="Download the EuRoC MAV, KITTI Odometry or KITTI Raw dataset")
  parser.add_argument("-s",
                      "--dataset",
                      required=True,
                      choices=DATASETS,
                      help=", ".join(f"{k}: {v}" for k, v in DATASETS.items()))
  parser.add_argument("-d",
                      "--dst-dir",
                      default="/data",
                      help="directory to download into (default: /data)")
  parser.add_argument("-f",
                      "--format",
                      choices=["sequences", "rosbags"],
                      default="sequences",
                      help="euroc only, download the sequences as zip files "
                      "or as ROS bags (default: sequences)")
  return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
  """Download the dataset chosen on the command line.

  Args:
    argv: Arguments to parse. Defaults to `sys.argv[1:]`.

  Returns:
    Exit code, 0 on success and 2 if the arguments are not valid.
  """
  args = parse_args(argv)
  if args.format != "sequences" and args.dataset != "euroc":
    print("error: --format is only for euroc", file=sys.stderr)
    return 2

  if args.dataset == "euroc" and args.format == "rosbags":
    download_euroc_rosbags(args.dst_dir)
  elif args.dataset == "euroc":
    download_euroc_sequences(args.dst_dir)
  elif args.dataset == "kitti-odom":
    download_kitti_odometry(args.dst_dir)
  elif args.dataset == "kitti-raw":
    download_kitti_raw(args.dst_dir)
  return 0


if __name__ == "__main__":
  sys.exit(main())
