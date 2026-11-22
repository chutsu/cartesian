Experiments
===========

Simulation experiments built on top of ``cartesian``. Each experiment simulates
data (images of calibration targets, camera and IMU measurements), so the ground
truth is known exactly, runs the library on it, and writes a report.

==========================  =====================================================
Experiment                  Question
==========================  =====================================================
``exp_detector``            How accurate, reliable and fast are the AprilGrid
                            (kalibr style and plain), checkerboard and ChArUco
                            corner detectors?
``exp_calib_camera``        Does ``CalibCamera`` recover the ground truth camera
                            from AprilGrid, checkerboard and ChArUco detections,
                            for fields of view of 90 - 180 degrees and different
                            amounts of distortion, including fisheye?
``exp_calib_camera_imu``    How well does ``CalibCameraImu`` recover the time
                            delay between camera and IMU, with each of its time
                            delay methods, for different delays and amounts of
                            noise?
==========================  =====================================================

The reports are checked in:

- `Detector report <exp_detector/report/REPORT.rst>`_: accuracy, detection rate and
  time of the corner detectors.
- `Camera calibration report <exp_calib_camera/report/REPORT.rst>`_: how well
  ``CalibCamera`` recovers the ground truth camera.
- `Camera-IMU time delay report <exp_calib_camera_imu/report/REPORT.rst>`_: how
  well ``CalibCameraImu`` recovers the time delay, by method, delay and noise.


Run
---

The experiments use the dependencies and build directory of the main project,
so build the dependencies first (``make deps`` in the root of the repository).
The reports are generated with Python 3 and need ``numpy``, ``pandas``,
``matplotlib`` and ``pyyaml``.

.. code-block::

  cd exps
  make help              # List targets
  make build             # Build the experiments
  make exp_detector      # Run the detector experiment and write its report
  make exp_calib_camera  # Run the calibration experiment and write its report
  make exp_calib_camera_imu  # Run the camera-IMU time delay benchmark
  make all               # Run both
  make clean             # Remove the outputs (the reports are kept)

Each experiment takes a few minutes. Raw results (CSV files, sample
renders) are written to ``exps/output/``, which is not checked in, and the
report is written over the checked in one.

The programs can also be run directly. They take a single argument, the path to
a YAML file, and write to ``output_dir`` of that file relative to the current
directory:

.. code-block::

  ./build/exp_detector exps/exp_detector/config.yaml
  ./build/exp_calib_camera exps/exp_calib_camera/config.yaml
  ./build/exp_calib_camera_imu exps/exp_calib_camera_imu/config.yaml


Settings
--------

All settings are read from YAML files, there are no command line options. Every
setting is required, so the files are the complete description of a run.

- ``exp_detector/config.yaml``: number of poses, seed, AprilGrid detectors,
  camera field of view, pose sampling range and image blur and noise.
- ``exp_calib_camera/config.yaml``: views per camera, calibration targets,
  initial guesses, fields of view, distortion levels per camera model, and pose
  sampling.

- ``exp_calib_camera_imu/config.yaml``: time delay methods, true delays, noise
  levels of the pixels and of the IMU, number of datasets per setting, the
  trajectory and the initial guess of the camera-IMU extrinsic.

The figures of the calibration report are made from the sample renders, so leave
``save_samples`` on if you regenerate that report.


Layout
------

.. code-block::

  exps/
    Makefile                       Build, run and report targets
    exp_detector/
      config.yaml                  Settings
      ExpDetector.cpp              The experiment
      make_report.py               Report generator (reStructuredText)
      report/                      Checked in report
    exp_calib_camera/
      config.yaml                  Settings
      ExpCalibCamera.cpp           The experiment
      make_report.py               Report generator (reStructuredText)
      report/                      Checked in report
    exp_calib_camera_imu/
      config.yaml                  Settings
      ExpCalibCameraImu.cpp        The benchmark, and its errors
      make_report.py               Report generator (reStructuredText)
      report/                      Checked in report

The simulation is part of the library, and both experiments link against it:

- ``lib/sim/Renderer``: every pixel is ray cast onto the board plane and the
  board texture is sampled there, with supersampling, blur and noise. The ray
  through a pixel is exact for fisheye lenses too, including rays 90 degrees or
  more off the optical axis.
- ``lib/sim/PoseSampling``: ``sample_poses()`` looks at the board center, which
  is what the detector experiment needs. ``sample_poses_spread()`` puts the
  board center at a random image location, so the board is seen across the
  whole image, which a calibration needs.
- ``lib/sim/SimTarget``: a board to simulate, with its size, ground truth
  corners, texture and detector. ``SimTarget::aprilgrid()``,
  ``SimTarget::checkerboard()`` and ``SimTarget::charuco()`` make one. The board
  geometry lives in the calibration targets of ``lib/calib``
  (``AprilGridConfig``, ``CheckerboardConfig`` and ``CharucoConfig``), the
  texture is drawn in ``SimTarget.cpp``.
- **Target frame.** Origin at the bottom left corner of the board, x right,
  y up and z out of the board. A ``SimTarget``'s ``corners`` (ground truth) and
  ``detect()`` use the same corner ids.
- **Pixel convention.** The center of pixel (0, 0) is at (0, 0), as in OpenCV.

Adding a target means adding its config's ``get_board_points()`` and a
``SimTarget`` factory that draws its texture.


Things to know
--------------

- The two AprilGrid detectors use different tag code tables and borders:
  ``kaess`` (``ethz_apriltag``) expects a 2 module black border and its own
  codes, ``olsen`` (``apriltag``) a 1 module border. ``SimTarget::aprilgrid()`` renders
  the texture that matches the detector, so a tag drawn for one is not found by
  the other.
- The corners of ``olsen`` and of OpenCV's ChArUco detector are half a pixel
  off in x and y, with respect to the convention where the center of pixel
  (0, 0) is at (0, 0). ``lib/calib`` shifts them, so all targets use the same
  convention.
- Corner refinement with ``cv::cornerSubPix`` is a setting per target type
  (``refine_corners``). With the default config it reduces the corner error of
  the checkerboard from 0.16 px to 0.07 px and of the ChArUco board from
  0.38 px to 0.25 px, and increases the error of the AprilGrid corners from
  0.09 px to 0.3 px (``kaess``) and from 0.18 px to 0.3 px (``olsen``), whose
  corners come from lines fitted to whole tag edges. So it is on for the
  checkerboard and ChArUco and off for the AprilGrid. The window is a fraction
  of the distance to the nearest corner, a larger window is worse when the
  squares are small or foreshortened.
- The calibration experiment calibrates with the AprilGrid (both detectors), the
  checkerboard and the ChArUco board, using the detectors of ``lib/calib``. The
  checkerboard detector needs the whole board in the image, so all targets are
  rendered from the same poses, where the whole board of every target is
  visible, and every target keeps the same number of corners in every view (a
  random subset), so that they are compared on the same data. The corners stay
  close to the image center because of this.
- The views are random (seeded), not a grid: the distance, viewing angle, roll
  and the image location of the board are sampled uniformly.
- ``BrownConrady4`` projects through the plane ``z = 1``, so corners 90 degrees
  or more off the optical axis cannot be used. ``KannalaBrandt4`` and
  ``UnifiedCameraModel`` can. The calibration experiment drops corners beyond
  ``poses.max_theta`` (85 degrees), and beyond ``poses.max_theta_wide`` (105
  degrees) for those two models on cameras with a field of view above 180
  degrees.
- Pinhole cameras cannot see 180 degrees or more, so that combination is
  skipped.
- ``exp_calib_camera_imu`` has its own camera error with the pixel velocity
  (``CalibCameraImuErrorPixelVelocity``), imu error with a time delay
  (``ImuErrorTimeDelay``) and a ``CalibCameraImu`` that uses them
  (``ExpCalibCameraImu``), all in ``ExpCalibCameraImu.cpp``, none in ``lib``.
  It turns off ``estimate_time_delay`` of ``CalibCameraImu``, and overrides its
  ``create_camera_error()`` and ``add_imu_error()``. The imu error needs the
  measurements after its window, so it is added late, feed the measurements
  with ``add_imu_measurement()``.
- ``exp_calib_camera_imu`` calibrates simulated data (``SimCalib``): an
  AprilGrid, a camera at 10 Hz, an IMU at 1 kHz with time stamps that are late
  by the true delay. The corners and IMU measurements are exact unless a noise
  level adds noise, so the ``clean`` level shows the error of a method itself.
  Without IMU noise the calibration is given the noise density of the
  simulation, with it, the noise it was given. A time delay error of more than
  ``tolerance_ms`` is a failure. It runs one calibration after the other, which
  takes several minutes.
- OpenCV and the project link different OpenMP runtimes. Do not use
  ``#pragma omp`` in the experiments, use ``cv::parallel_for_``, otherwise the
  program crashes in the OpenMP runtime.


Adding an experiment
--------------------

1. Add the sources under ``exps/<name>/`` and an executable to the
   ``BUILD EXPERIMENTS`` section of the top level ``CMakeLists.txt``. Link it
   with ``${PROJECT_NAME} ${DEPS}``, which has the targets and the simulation.
2. Read settings from a YAML file with ``config_t`` and ``parse()``, and write
   results as CSV to ``output_dir``.
3. Write a script that turns the CSV files into a report.
4. Add a target to ``exps/Makefile``, and list the binary in the ``build``
   target.
