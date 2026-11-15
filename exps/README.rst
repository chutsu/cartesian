Experiments
===========

Simulation experiments built on top of ``cartesian``. Each experiment renders
images of calibration targets from simulated cameras, so the ground truth is
known exactly, runs the library on them, and writes a report.

======================  =====================================================
Experiment              Question
======================  =====================================================
``detector``            How accurate, reliable and fast are the AprilGrid
                        (kalibr style and plain), checkerboard and ChArUco
                        corner detectors?
``calib_camera``        Does ``CalibCamera`` recover the ground truth camera
                        for fields of view of 90 - 180 degrees and different
                        amounts of distortion, including fisheye?
======================  =====================================================

The reports are checked in:

- ``detector/report/REPORT.md``
- ``calib_camera/report/REPORT.rst``


Run
---

The experiments use the dependencies and build directory of the main project,
so build the dependencies first (``make deps`` in the root of the repository).
The reports are generated with Python 3 and need ``numpy``, ``pandas``,
``matplotlib`` and ``pyyaml``.

.. code-block::

  cd exps
  make help           # List targets
  make build          # Build exp_detector and exp_calib_camera
  make detector       # Run the detector experiment and write its report
  make calib_camera   # Run the calibration experiment and write its report
  make all            # Run both
  make clean          # Remove the outputs (the reports are kept)

Each experiment takes a few minutes. Raw results (CSV files, sample
renders) are written to ``exps/output/``, which is not checked in, and the
report is written over the checked in one.

The programs can also be run directly. They take a single argument, the path to
a YAML file, and write to ``output_dir`` of that file relative to the current
directory:

.. code-block::

  ./build/exp_detector exps/detector/config.yaml
  ./build/exp_calib_camera exps/calib_camera/config.yaml


Settings
--------

All settings are read from YAML files, there are no command line options. Every
setting is required, so the files are the complete description of a run.

- ``detector/config.yaml``: number of poses, seed, AprilGrid detectors,
  camera field of view, pose sampling range and image blur and noise.
- ``calib_camera/config.yaml``: views per camera, detectors, initial guesses,
  fields of view, distortion levels per camera model, and pose sampling.

The figures of the calibration report are made from the sample renders, so leave
``save_samples`` on if you regenerate that report.


Layout
------

.. code-block::

  exps/
    Makefile                       Build, run and report targets
    common/                        Shared by the experiments
      DetectorTarget.{hpp,cpp}     Target interface (struct)
      AprilGridTarget.{hpp,cpp}    AprilGrid, with the kaess or olsen detector
      CheckerboardTarget.{hpp,cpp} Checkerboard (OpenCV)
      CharucoTarget.{hpp,cpp}      ChArUco (OpenCV)
      Sim.{hpp,cpp}                Cameras, pose sampling and the renderer
    detector/
      config.yaml                  Settings
      ExpDetector.cpp              The experiment
      make_report.py               Report generator (Markdown)
      report/                      Checked in report
    calib_camera/
      config.yaml                  Settings
      ExpCalibCamera.cpp           The experiment
      make_report.py               Report generator (reStructuredText)
      report/                      Checked in report

The targets and the simulation in ``common/`` are built into the ``exps_common``
library that both experiments link against:

- **Rendering.** Every pixel is ray cast onto the board plane and the target
  texture is sampled there, with supersampling, blur and noise. The ray through
  a pixel is exact for fisheye lenses too, including rays 90 degrees or more
  off the optical axis.
- **Target frame.** Origin at the bottom left corner of the board, x right,
  y up and z out of the board. A target's ``corners()`` (ground truth) and
  ``detect()`` use the same corner ids.
- **Pixel convention.** The center of pixel (0, 0) is at (0, 0), as in OpenCV.
- **Poses.** ``sample_poses()`` looks at the board center, which is what the
  detector experiment needs. ``sample_poses_spread()`` puts the board center at
  a random image location, so the board is seen across the whole image, which
  a calibration needs.

Each target is a ``DetectorTarget``, which can render its own texture, return
its ground truth corners and run its detector. Adding a target means
implementing those four functions.


Things to know
--------------

- The two AprilGrid detectors use different tag code tables and borders:
  ``kaess`` (``ethz_apriltag``) expects a 2 module black border and its own
  codes, ``olsen`` (``apriltag``) a 1 module border. ``AprilGridTarget`` renders
  the texture that matches the detector, so a tag drawn for one is not found by
  the other.
- The ``olsen`` and ChArUco corners are about 0.5 px off in x and y, compared with
  ``kaess`` and the checkerboard. The reports show this as bias, separately from
  noise.
- ``CalibCamera`` only accepts AprilGrid targets, so the calibration experiment
  uses the plain AprilGrid. The other targets are only in the detector
  experiment.
- ``BrownConrady4`` and ``KannalaBrandt4`` project through the plane ``z = 1``,
  so corners 90 degrees or more off the optical axis cannot be used. The
  calibration experiment drops corners beyond ``poses.max_theta`` (85 degrees).
- Pinhole cameras cannot see 180 degrees, so that combination is skipped.
- OpenCV and the project link different OpenMP runtimes. Do not use
  ``#pragma omp`` in the experiments, use ``cv::parallel_for_``, otherwise the
  program crashes in the OpenMP runtime.


Adding an experiment
--------------------

1. Add the sources under ``exps/<name>/`` and an executable to the
   ``BUILD EXPERIMENTS`` section of the top level ``CMakeLists.txt``. Link it
   with ``exps_common`` to get the targets and the simulation, which also
   brings in ``${PROJECT_NAME} ${DEPS}``.
2. Read settings from a YAML file with ``config_t`` and ``parse()``, and write
   results as CSV to ``output_dir``.
3. Write a script that turns the CSV files into a report.
4. Add a target to ``exps/Makefile``, and list the binary in the ``build``
   target.
