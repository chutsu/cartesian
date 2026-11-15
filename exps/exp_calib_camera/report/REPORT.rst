========================================================================
Camera calibration accuracy report
========================================================================

Corners of an AprilGrid, a checkerboard and a ChArUco board are detected in
simulated images of cameras with horizontal fields of view of
90, 120, 160, 180, 200, 220 degrees and different amounts of distortion,
and calibrated with ``CalibCamera``. The result is compared with the ground
truth camera. Every dataset is calibrated with all three camera models, to see
which model fits best.

Setup
=====

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%
   :widths: 40 60

   * - Setting
     - Value
   * - Resolution
     - 752x480
   * - Principal point offset
     - (+4.0, -3.0) px
   * - fy / fx
     - 1.004
   * - Views per camera and target
     - 80, random poses, the same for all targets
   * - Distance to board
     - 90 deg: 0.5-1.5 m, 120 deg: 0.4-1.2 m, 160 deg: 0.3-0.8 m, 180 deg: 0.25-0.6 m, 200 deg: 0.2-0.5 m, 220 deg: 0.2-0.45 m
   * - Tilt
     - 0 - 60 deg
   * - Image
     - blur sigma 0.8 px, noise sigma 2.0 levels
   * - Corner refinement
     - cv::cornerSubPix for checkerboard, ChArUco
   * - Corners dropped beyond
     - 85 deg off the axis (105 deg for KannalaBrandt4 and UnifiedCameraModel cameras above 180 deg)
   * - Seed
     - 42

Simulated lenses, with three distortion levels each (none, mild, strong). The
pinhole lens does not exist at 180 degrees or more. Every lens is calibrated with its
own model and with the other two.

- **BrownConrady4**: pinhole with radial-tangential distortion, -3 % (mild) and
  -12 % (strong) radial distortion at the horizontal image edge.
- **KannalaBrandt4**: equidistant fisheye, ``k1..k4`` = [-0.005, 0.001, 0.0, 0.0]
  (mild) and [-0.03, 0.01, -0.002, 0.0002] (strong).
- **UnifiedCameraModel** (UCM): sphere projection with xi = 0.9
  and the distortion of the pinhole lens.

Targets, all with the detectors of ``lib/calib``:

- **AprilGrid**: 6x6 tags of 80 mm, detected with ``kaess`` and ``olsen``.
- **Checkerboard**: 9x8 squares of 70 mm. Its detector needs the whole board in
  the image.
- **ChArUco**: 9x8 squares of 70 mm, 56 mm markers.

The targets are compared on the same data. They are rendered from the same
poses, where the whole board of every target is in the image, so a board that
can be partly visible does not get views the others do not have. Only the views
where every target is detected with at least 10 corners are used, and in each
view every target keeps the same number of corners (the smallest number of any
target, a random subset), so the AprilGrid, which has the most corners, is not
favoured by having more of them.

BrownConrady4 projects through the plane ``z = 1``, so it cannot use corners 90
degrees or more off the optical axis. KannalaBrandt4 and UnifiedCameraModel can,
but the same limit (``poses.max_theta``) is applied to all models so that they
are fitted to the same corners.

.. figure:: targets.png
   :width: 80%
   :align: center

   The calibration targets.

Camera views
============

A view of the board, looking straight at it, for each camera.

.. figure:: camera_views_BrownConrady4.png
   :width: 80%
   :align: center

   Pinhole + radial-tangential (BrownConrady4): rows are distortion levels, columns are fields of view.

.. figure:: camera_views_KannalaBrandt4.png
   :width: 80%
   :align: center

   Equidistant fisheye (KannalaBrandt4): rows are distortion levels, columns are fields of view.

.. figure:: camera_views_UnifiedCameraModel.png
   :width: 80%
   :align: center

   Sphere projection (UnifiedCameraModel): rows are distortion levels, columns are fields of view.

Calibration data
================

Detected corners (green) in three views of a 120 degree fisheye camera with strong distortion, and over all views (mild distortion).

.. figure:: calibration_views.png
   :width: 80%
   :align: center

   Three calibration views per target.

.. figure:: coverage.png
   :width: 80%
   :align: center

   Detected corners over the image, all views.

Results
=======

The **projection error** compares the estimated and the ground truth camera:
rays over the image are projected with both and the pixel difference is
measured (rms and max). A camera is *recovered* when the rms is at most
0.5 px. The best result of each row or column is in **bold**.

Target ranking
--------------

Targets ranked by their mean rank over all simulated cameras (1 = lowest projection error of the four, default initial guess).

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Rank
     - Target
     - Mean rank per camera
     - Median error [px]
     - Recovered (<= 0.5 px)
     - Lowest error for cameras
   * - **1 (best)**
     - **Checkerboard**
     - **1.76**
     - **0.11**
     - 33 / 45
     - **30**
   * - 2
     - AprilGrid (olsen)
     - 2.29
     - 0.32
     - 31 / 45
     - 9
   * - 3
     - AprilGrid (kaess)
     - 2.82
     - 0.39
     - **35 / 45**
     - 6
   * - 4
     - ChArUco
     - 3.13
     - 0.33
     - 29 / 45
     - 3

- **Checkerboard is the most accurate**, followed by AprilGrid (olsen), AprilGrid (kaess), ChArUco.

The best target for every camera model and field of view, by the mean projection error over the three distortion levels:

Pinhole
~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - Best target
     - Mean error [px]
     - Second
     - Mean error [px]
   * - 90
     - **Checkerboard**
     - 0.09
     - AprilGrid (kaess)
     - 0.17
   * - 120
     - **AprilGrid (kaess)**
     - 0.30
     - ChArUco
     - 0.51
   * - 160
     - **AprilGrid (olsen)**
     - 0.79
     - AprilGrid (kaess)
     - 2.26

Fisheye
~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - Best target
     - Mean error [px]
     - Second
     - Mean error [px]
   * - 90
     - **AprilGrid (kaess)**
     - 0.15
     - AprilGrid (olsen)
     - 0.22
   * - 120
     - **Checkerboard**
     - 0.07
     - AprilGrid (olsen)
     - 0.09
   * - 160
     - **Checkerboard**
     - 0.06
     - ChArUco
     - 0.12
   * - 180
     - **Checkerboard**
     - 0.12
     - ChArUco
     - 0.16
   * - 200
     - **ChArUco**
     - 0.20
     - AprilGrid (olsen)
     - 0.38
   * - 220
     - **AprilGrid (kaess)**
     - 0.48
     - AprilGrid (olsen)
     - 0.53

UCM
~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - Best target
     - Mean error [px]
     - Second
     - Mean error [px]
   * - 90
     - **Checkerboard**
     - 0.06
     - AprilGrid (kaess)
     - 0.13
   * - 120
     - **AprilGrid (olsen)**
     - 0.06
     - Checkerboard
     - 0.06
   * - 160
     - **Checkerboard**
     - 0.05
     - AprilGrid (olsen)
     - 0.10
   * - 180
     - **Checkerboard**
     - 0.18
     - ChArUco
     - 0.20
   * - 200
     - **Checkerboard**
     - 0.57
     - AprilGrid (olsen)
     - 0.69
   * - 220
     - **AprilGrid (olsen)**
     - 0.56
     - AprilGrid (kaess)
     - 0.86

Quick summary
-------------

Default initial guess. Errors are medians over the cameras that converged.

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Target
     - Pinhole 90-120 deg: median error [px]
     - Fisheye: median error [px]
     - UCM: median error [px]
     - Cameras recovered (<= 0.5 px)
     - Lowest error for cameras
     - Principal point: median error [px]
   * - AprilGrid (kaess)
     - 0.23
     - 0.39
     - 0.38
     - **35 / 45**
     - 6
     - 0.07
   * - AprilGrid (olsen)
     - 0.49
     - 0.26
     - 0.27
     - 31 / 45
     - 9
     - 0.07
   * - Checkerboard
     - **0.10**
     - **0.14**
     - **0.09**
     - 33 / 45
     - **30**
     - **0.05**
   * - ChArUco
     - 0.46
     - 0.29
     - 0.28
     - 29 / 45
     - 3
     - 0.13

- **Best:** Checkerboard, lowest error for 30 of 45 cameras, 33 of 45 recovered.

- **Fisheye and UCM cameras:** every fisheye and UCM camera converges, median error 0.09 - 0.39 px, up to 220 degrees.

- **Pinhole at 160 degrees:** 11 of 12 calibrations converge from the default initial guess, 12 from a guess near the truth, so most failures are the initialisation, not the model.

- **Reprojection error is not accuracy:** BrownConrady4 at 160 degrees (Checkerboard) reports 0.59 px but is off by 25.5 px.

Best camera model by field of view
----------------------------------

Every dataset is calibrated with all three models from the default initial guess. The error is the median projection error over the lens types, targets and distortion levels, where a calibration that failed counts as 100 px.

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - BrownConrady4
     - KannalaBrandt4
     - UnifiedCameraModel
     - Best
     - Converged [%] Pinhole
     - Converged [%] Fisheye
     - Converged [%] UCM
   * - 90
     - 1.91
     - 0.38
     - **0.23**
     - **UnifiedCameraModel**
     - 100
     - 100
     - 100
   * - 120
     - 42.16
     - 0.66
     - **0.25**
     - **UnifiedCameraModel**
     - 69
     - 94
     - 100
   * - 160
     - >100
     - 0.67
     - **0.29**
     - **UnifiedCameraModel**
     - 31
     - 72
     - 100
   * - 180
     - >100
     - 0.36
     - **0.26**
     - **UnifiedCameraModel**
     - 0
     - 100
     - 100
   * - 200
     - >100
     - **0.58**
     - 0.63
     - **KannalaBrandt4**
     - 0
     - 100
     - 100
   * - 220
     - >100
     - **0.81**
     - 0.89
     - **KannalaBrandt4**
     - 0
     - 100
     - 100

- **Best model:** UnifiedCameraModel at 90 degrees, UnifiedCameraModel at 120 degrees, UnifiedCameraModel at 160 degrees, UnifiedCameraModel at 180 degrees, KannalaBrandt4 at 200 degrees, KannalaBrandt4 at 220 degrees.

- **BrownConrady4:** fits a pinhole lens up to 120 degrees, but not a fisheye or UCM lens, with a median error of 3.60 px already at 90 degrees.

- **Fisheye against UCM:** on a fisheye lens the UnifiedCameraModel reaches a median error of 0.27 px (KannalaBrandt4: 0.28 px), on a UnifiedCameraModel lens the KannalaBrandt4 reaches 0.76 px (UnifiedCameraModel: 0.29 px).

.. figure:: model_fit.png
   :width: 80%
   :align: center

   Median projection error of every camera model, for every lens type, by field of view.

By lens type:

Pinhole
~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - BrownConrady4
     - KannalaBrandt4
     - UnifiedCameraModel
   * - 90
     - **0.29**
     - 2.55
     - 0.49
   * - 120
     - **0.35**
     - 19.08
     - 1.09
   * - 160
     - **1.32**
     - >100
     - 13.91

Fisheye
~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - BrownConrady4
     - KannalaBrandt4
     - UnifiedCameraModel
   * - 90
     - 3.88
     - 0.24
     - **0.21**
   * - 120
     - >100
     - **0.14**
     - 0.16
   * - 160
     - >100
     - **0.18**
     - 0.22
   * - 180
     - >100
     - **0.24**
     - 0.26
   * - 200
     - >100
     - **0.39**
     - 0.49
   * - 220
     - >100
     - **0.55**
     - 0.87

UCM
~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - FOV [deg]
     - BrownConrady4
     - KannalaBrandt4
     - UnifiedCameraModel
   * - 90
     - 1.91
     - 0.54
     - **0.16**
   * - 120
     - 42.16
     - 0.66
     - **0.11**
   * - 160
     - >100
     - 0.67
     - **0.14**
   * - 180
     - >100
     - 0.63
     - **0.27**
   * - 200
     - >100
     - 1.75
     - **0.66**
   * - 220
     - >100
     - 1.02
     - **0.89**

Parameter errors
----------------

Median over the cameras that converged. ``fx`` is a relative error, the principal point ``cx, cy`` an absolute one.

Default initial guess
~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Model
     - Target
     - fx error [%]
     - cx error [px]
     - cy error [px]
     - Reproj. rmse [px]
     - Converged
   * - BrownConrady4
     - AprilGrid (kaess)
     - 0.05
     - 0.06
     - 0.06
     - 0.083
     - **9 / 9**
   * - BrownConrady4
     - AprilGrid (olsen)
     - 0.11
     - **0.05**
     - 0.08
     - 0.155
     - **9 / 9**
   * - BrownConrady4
     - Checkerboard
     - **0.02**
     - 0.07
     - **0.03**
     - **0.040**
     - **9 / 9**
   * - BrownConrady4
     - ChArUco
     - **0.02**
     - 0.31
     - 0.30
     - 0.239
     - 8 / 9
   * - KannalaBrandt4
     - AprilGrid (kaess)
     - 0.25
     - **0.03**
     - **0.02**
     - **0.082**
     - **18 / 18**
   * - KannalaBrandt4
     - AprilGrid (olsen)
     - 0.19
     - 0.04
     - 0.04
     - 0.126
     - **18 / 18**
   * - KannalaBrandt4
     - Checkerboard
     - **0.03**
     - 0.04
     - 0.03
     - 0.087
     - **18 / 18**
   * - KannalaBrandt4
     - ChArUco
     - 0.08
     - 0.05
     - 0.07
     - 0.209
     - **18 / 18**
   * - UnifiedCameraModel
     - AprilGrid (kaess)
     - 9.63
     - 0.04
     - 0.05
     - 0.082
     - **18 / 18**
   * - UnifiedCameraModel
     - AprilGrid (olsen)
     - **9.55**
     - 0.03
     - 0.04
     - 0.142
     - **18 / 18**
   * - UnifiedCameraModel
     - Checkerboard
     - 10.10
     - **0.02**
     - **0.02**
     - **0.066**
     - **18 / 18**
   * - UnifiedCameraModel
     - ChArUco
     - 12.33
     - 0.06
     - 0.07
     - 0.225
     - **18 / 18**

Initial guess near the ground truth
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Model
     - Target
     - fx error [%]
     - cx error [px]
     - cy error [px]
     - Reproj. rmse [px]
     - Converged
   * - BrownConrady4
     - AprilGrid (kaess)
     - 0.05
     - 0.06
     - 0.06
     - 0.083
     - **9 / 9**
   * - BrownConrady4
     - AprilGrid (olsen)
     - 0.11
     - **0.05**
     - 0.08
     - 0.155
     - **9 / 9**
   * - BrownConrady4
     - Checkerboard
     - **0.02**
     - 0.07
     - **0.03**
     - **0.040**
     - **9 / 9**
   * - BrownConrady4
     - ChArUco
     - 0.03
     - 0.16
     - 0.28
     - 0.241
     - **9 / 9**
   * - KannalaBrandt4
     - AprilGrid (kaess)
     - 0.25
     - **0.03**
     - **0.02**
     - **0.082**
     - **18 / 18**
   * - KannalaBrandt4
     - AprilGrid (olsen)
     - 0.19
     - 0.04
     - 0.04
     - 0.126
     - **18 / 18**
   * - KannalaBrandt4
     - Checkerboard
     - **0.03**
     - 0.04
     - 0.03
     - 0.087
     - **18 / 18**
   * - KannalaBrandt4
     - ChArUco
     - 0.08
     - 0.05
     - 0.07
     - 0.209
     - **18 / 18**
   * - UnifiedCameraModel
     - AprilGrid (kaess)
     - **1.09**
     - 0.03
     - 0.05
     - 0.082
     - **18 / 18**
   * - UnifiedCameraModel
     - AprilGrid (olsen)
     - 1.65
     - 0.03
     - 0.04
     - 0.142
     - **18 / 18**
   * - UnifiedCameraModel
     - Checkerboard
     - 1.73
     - **0.02**
     - **0.02**
     - **0.066**
     - **18 / 18**
   * - UnifiedCameraModel
     - ChArUco
     - 2.70
     - 0.06
     - 0.08
     - 0.224
     - **18 / 18**

Default initial guess
---------------------

.. figure:: model_error_default.png
   :width: 80%
   :align: center

   Projection error (rms) by field of view. Crosses mark calibrations that diverged or failed.

BrownConrady4
~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Distortion
     - FOV [deg]
     - AprilGrid (kaess)
     - AprilGrid (olsen)
     - Checkerboard
     - ChArUco
   * - none
     - 90
     - 0.22
     - 0.56
     - **0.11**
     - 0.52
   * - none
     - 120
     - **0.42**
     - 0.86
     - 1.65
     - 0.81
   * - none
     - 160
     - 4.90
     - **1.18**
     - 25.53
     - >100
   * - none
     - 180
     - n/a
     - n/a
     - n/a
     - n/a
   * - none
     - 200
     - n/a
     - n/a
     - n/a
     - n/a
   * - none
     - 220
     - n/a
     - n/a
     - n/a
     - n/a
   * - mild
     - 90
     - **0.14**
     - 0.42
     - 0.15
     - 0.77
   * - mild
     - 120
     - 0.25
     - 0.63
     - **0.08**
     - 0.32
   * - mild
     - 160
     - 1.46
     - 0.61
     - **0.57**
     - 19.59
   * - mild
     - 180
     - n/a
     - n/a
     - n/a
     - n/a
   * - mild
     - 200
     - n/a
     - n/a
     - n/a
     - n/a
   * - mild
     - 220
     - n/a
     - n/a
     - n/a
     - n/a
   * - strong
     - 90
     - 0.16
     - 0.36
     - **0.02**
     - 0.39
   * - strong
     - 120
     - 0.24
     - 0.19
     - **0.06**
     - 0.39
   * - strong
     - 160
     - 0.42
     - 0.57
     - **0.11**
     - 2.22
   * - strong
     - 180
     - n/a
     - n/a
     - n/a
     - n/a
   * - strong
     - 200
     - n/a
     - n/a
     - n/a
     - n/a
   * - strong
     - 220
     - n/a
     - n/a
     - n/a
     - n/a

KannalaBrandt4
~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Distortion
     - FOV [deg]
     - AprilGrid (kaess)
     - AprilGrid (olsen)
     - Checkerboard
     - ChArUco
   * - none
     - 90
     - **0.07**
     - 0.24
     - **0.07**
     - 0.59
   * - none
     - 120
     - 0.20
     - **0.09**
     - **0.09**
     - 0.28
   * - none
     - 160
     - 0.38
     - 0.26
     - **0.05**
     - 0.07
   * - none
     - 180
     - 0.39
     - 0.34
     - 0.16
     - **0.13**
   * - none
     - 200
     - 0.48
     - 0.36
     - 0.49
     - **0.16**
   * - none
     - 220
     - 0.48
     - **0.43**
     - 2.46
     - 2.45
   * - mild
     - 90
     - **0.18**
     - 0.23
     - 0.34
     - 0.35
   * - mild
     - 120
     - 0.18
     - 0.10
     - **0.07**
     - 0.30
   * - mild
     - 160
     - 0.40
     - 0.22
     - **0.07**
     - 0.14
   * - mild
     - 180
     - 0.41
     - 0.32
     - **0.14**
     - 0.20
   * - mild
     - 200
     - 0.52
     - 0.37
     - **0.15**
     - 0.30
   * - mild
     - 220
     - **0.50**
     - 0.55
     - 0.54
     - 0.56
   * - strong
     - 90
     - 0.19
     - **0.18**
     - 0.29
     - 0.37
   * - strong
     - 120
     - 0.27
     - 0.09
     - **0.06**
     - 0.36
   * - strong
     - 160
     - 0.29
     - 0.20
     - **0.06**
     - 0.16
   * - strong
     - 180
     - 0.42
     - 0.27
     - **0.07**
     - 0.15
   * - strong
     - 200
     - 0.55
     - 0.41
     - 0.61
     - **0.15**
   * - strong
     - 220
     - **0.47**
     - 0.61
     - 3.19
     - 1.29

UnifiedCameraModel
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Distortion
     - FOV [deg]
     - AprilGrid (kaess)
     - AprilGrid (olsen)
     - Checkerboard
     - ChArUco
   * - none
     - 90
     - 0.11
     - 0.32
     - **0.04**
     - 0.41
   * - none
     - 120
     - 0.17
     - **0.07**
     - **0.07**
     - 0.29
   * - none
     - 160
     - 0.35
     - 0.08
     - **0.06**
     - 0.16
   * - none
     - 180
     - 0.44
     - 0.29
     - **0.09**
     - 0.16
   * - none
     - 200
     - 0.80
     - 0.64
     - **0.57**
     - 0.81
   * - none
     - 220
     - 1.13
     - **0.65**
     - 1.50
     - 2.10
   * - mild
     - 90
     - 0.10
     - 0.26
     - **0.08**
     - 0.49
   * - mild
     - 120
     - 0.14
     - 0.05
     - **0.02**
     - 0.23
   * - mild
     - 160
     - 0.32
     - 0.07
     - **0.05**
     - 0.13
   * - mild
     - 180
     - 0.40
     - **0.18**
     - 0.35
     - 0.20
   * - mild
     - 200
     - 0.77
     - 0.60
     - **0.53**
     - 0.64
   * - mild
     - 220
     - 0.87
     - **0.52**
     - 0.92
     - 1.31
   * - strong
     - 90
     - 0.18
     - 0.15
     - **0.05**
     - 0.21
   * - strong
     - 120
     - 0.21
     - **0.05**
     - 0.08
     - 0.27
   * - strong
     - 160
     - 0.37
     - 0.16
     - **0.05**
     - 0.21
   * - strong
     - 180
     - 0.46
     - 0.30
     - **0.10**
     - 0.25
   * - strong
     - 200
     - 0.97
     - 0.82
     - **0.62**
     - 0.69
   * - strong
     - 220
     - 0.58
     - 0.50
     - **0.45**
     - 1.08

Initial guess near the ground truth
-----------------------------------

.. figure:: model_error_near_truth.png
   :width: 80%
   :align: center

   Projection error (rms) by field of view. Crosses mark calibrations that diverged or failed.

BrownConrady4
~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Distortion
     - FOV [deg]
     - AprilGrid (kaess)
     - AprilGrid (olsen)
     - Checkerboard
     - ChArUco
   * - none
     - 90
     - 0.22
     - 0.56
     - **0.11**
     - 0.52
   * - none
     - 120
     - **0.42**
     - 0.86
     - 1.65
     - 0.81
   * - none
     - 160
     - **1.11**
     - 1.18
     - 25.53
     - 3.33
   * - none
     - 180
     - n/a
     - n/a
     - n/a
     - n/a
   * - none
     - 200
     - n/a
     - n/a
     - n/a
     - n/a
   * - none
     - 220
     - n/a
     - n/a
     - n/a
     - n/a
   * - mild
     - 90
     - **0.14**
     - 0.42
     - 0.15
     - 0.77
   * - mild
     - 120
     - 0.25
     - 0.63
     - **0.08**
     - 0.32
   * - mild
     - 160
     - 1.46
     - 0.61
     - **0.57**
     - 2.90
   * - mild
     - 180
     - n/a
     - n/a
     - n/a
     - n/a
   * - mild
     - 200
     - n/a
     - n/a
     - n/a
     - n/a
   * - mild
     - 220
     - n/a
     - n/a
     - n/a
     - n/a
   * - strong
     - 90
     - 0.16
     - 0.36
     - **0.02**
     - 0.39
   * - strong
     - 120
     - 0.24
     - 0.19
     - **0.06**
     - 0.39
   * - strong
     - 160
     - 0.42
     - 0.57
     - **0.11**
     - 2.22
   * - strong
     - 180
     - n/a
     - n/a
     - n/a
     - n/a
   * - strong
     - 200
     - n/a
     - n/a
     - n/a
     - n/a
   * - strong
     - 220
     - n/a
     - n/a
     - n/a
     - n/a

KannalaBrandt4
~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Distortion
     - FOV [deg]
     - AprilGrid (kaess)
     - AprilGrid (olsen)
     - Checkerboard
     - ChArUco
   * - none
     - 90
     - **0.07**
     - 0.24
     - **0.07**
     - 0.59
   * - none
     - 120
     - 0.20
     - **0.09**
     - **0.09**
     - 0.28
   * - none
     - 160
     - 0.38
     - 0.26
     - **0.05**
     - 0.07
   * - none
     - 180
     - 0.39
     - 0.34
     - 0.16
     - **0.13**
   * - none
     - 200
     - 0.48
     - 0.36
     - 0.49
     - **0.16**
   * - none
     - 220
     - 0.48
     - **0.43**
     - 2.46
     - 2.45
   * - mild
     - 90
     - **0.18**
     - 0.23
     - 0.34
     - 0.35
   * - mild
     - 120
     - 0.18
     - 0.10
     - **0.07**
     - 0.30
   * - mild
     - 160
     - 0.40
     - 0.22
     - **0.07**
     - 0.14
   * - mild
     - 180
     - 0.41
     - 0.32
     - **0.14**
     - 0.20
   * - mild
     - 200
     - 0.52
     - 0.37
     - **0.15**
     - 0.30
   * - mild
     - 220
     - **0.50**
     - 0.55
     - 0.54
     - 0.56
   * - strong
     - 90
     - 0.19
     - **0.18**
     - 0.29
     - 0.37
   * - strong
     - 120
     - 0.27
     - 0.09
     - **0.06**
     - 0.36
   * - strong
     - 160
     - 0.29
     - 0.20
     - **0.06**
     - 0.16
   * - strong
     - 180
     - 0.42
     - 0.27
     - **0.07**
     - 0.15
   * - strong
     - 200
     - 0.55
     - 0.41
     - 0.61
     - **0.15**
   * - strong
     - 220
     - **0.47**
     - 0.61
     - 3.19
     - 1.29

UnifiedCameraModel
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :align: center
   :width: 80%

   * - Distortion
     - FOV [deg]
     - AprilGrid (kaess)
     - AprilGrid (olsen)
     - Checkerboard
     - ChArUco
   * - none
     - 90
     - 0.11
     - 0.32
     - **0.04**
     - 0.41
   * - none
     - 120
     - 0.16
     - **0.07**
     - **0.07**
     - 0.28
   * - none
     - 160
     - 0.35
     - 0.08
     - **0.06**
     - 0.17
   * - none
     - 180
     - 0.44
     - 0.29
     - **0.08**
     - 0.14
   * - none
     - 200
     - 0.65
     - 0.41
     - 0.43
     - **0.17**
   * - none
     - 220
     - 0.59
     - 0.28
     - **0.10**
     - 0.94
   * - mild
     - 90
     - 0.09
     - 0.26
     - **0.08**
     - 0.50
   * - mild
     - 120
     - 0.14
     - 0.05
     - **0.02**
     - 0.23
   * - mild
     - 160
     - 0.30
     - 0.06
     - **0.03**
     - 0.11
   * - mild
     - 180
     - 0.39
     - 0.17
     - 0.35
     - **0.15**
   * - mild
     - 200
     - 0.68
     - 0.51
     - **0.12**
     - 0.50
   * - mild
     - 220
     - 0.55
     - 0.30
     - **0.19**
     - 0.67
   * - strong
     - 90
     - 0.18
     - 0.15
     - **0.05**
     - 0.21
   * - strong
     - 120
     - 0.21
     - **0.05**
     - 0.08
     - 0.27
   * - strong
     - 160
     - 0.37
     - 0.16
     - **0.05**
     - 0.21
   * - strong
     - 180
     - 0.44
     - 0.28
     - **0.08**
     - 0.21
   * - strong
     - 200
     - 0.60
     - 0.50
     - **0.04**
     - 0.25
   * - strong
     - 220
     - 0.52
     - **0.30**
     - 0.87
     - 0.79
