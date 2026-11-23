/**
 * Detect the calibration targets in a dataset
 *
 * Detects the calibration targets in the images of every camera in a dataset,
 * and saves the detections next to the images, in
 * `<data_path>/target<id>/cam<id>/<timestamp>.csv`. Frames where the target is
 * not found are saved too. The tests that use the detections load them from
 * there. If the `target<id>/cam<id>` directory already exists the detections in
 * it are loaded and nothing is detected, remove it to detect again.
 *
 * The config is the one of `CalibProblem`, see
 * tests/test_data/calib_camera_imu.yaml.
 *
 * Usage: detect_targets <config.yaml>
 */
#include <cstdio>

#include "calib/CalibProblem.hpp"

using namespace cartesian;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <config.yaml>\n", argv[0]);
    return 1;
  }

  // Detects the targets in every image, or loads them if they were saved
  CalibProblem problem{argv[1]};

  // Report
  printf("data_path: %s\n", problem.data_path.c_str());
  for (const auto &[camera_id, camera_data] : problem.camera_data) {
    size_t num_detected = 0;
    for (const auto &[ts, targets] : camera_data) {
      for (const auto &[target_id, target] : targets) {
        if (target->detected()) {
          num_detected++;
        }
      }
    }
    printf("cam%d: %zu frames, target detected in %zu\n",
           camera_id,
           camera_data.size(),
           num_detected);
  }

  return 0;
}
