cartesian
=====

.. image:: https://github.com/chutsu/cartesian/actions/workflows/ci.yml/badge.svg
  :target: https://github.com/chutsu/cartesian/actions?query=ci

``cartesian`` contain notes and code used to learn about different aspects of
robotics.


Build
-----

For convenience there is a ``Makefile`` that automates the installation of
dependencies and building of ``cartesian``. On Ubuntu 22.04, install the system
packages, then build the dependencies from source, and build and test
``cartesian``:

.. code-block::

  make -C deps apt   # System packages, uses sudo if not root
  make deps          # Build the dependencies in deps/
  make release       # Build cartesian
  make unittests     # Build and run the tests

``make all`` is ``make deps`` followed by ``make release``. There are also
``make debug`` and ``make relwithdeb`` builds, alternatively just type
``make help`` to bring up info on make targets.

Some tests (``Sfm``, ``CalibInit`` and the ``CalibCameraImu`` ones with
``euroc`` in the name) use the EuRoC ``imu_april`` sequence in
``/data/euroc/imu_april``, and the AprilGrid detections of it in
``mav0/target0``. Download the sequence and detect the targets with:

.. code-block::

  ./scripts/download_dataset.py --dataset euroc --sequence imu_april
  ./build/detect_targets tests/test_data/calib_camera_imu.yaml

The detections are saved next to the images, and loaded instead of detected the
next time, remove ``mav0/target0`` to detect them again.


Docker
------

The ``Dockerfile`` sets up the build environment, the system packages and the
dependencies, and runs ``bash``. The repo is not part of the image, mount it to
build and test in the container:

.. code-block::

  make docker
  docker run -it --rm -v $(pwd):/cartesian -v /cartesian/deps cartesian
  make release && ./build/unittests

The dependencies are in ``/cartesian/deps`` in the image, so ``-v
/cartesian/deps`` keeps the mount of the repo from hiding them. If the repo
already has a ``build`` directory from your machine, delete it first, or add
``-v /cartesian/build``. The ``ci`` workflow builds the image, downloads the
``imu_april`` sequence, detects the targets and runs the tests the same way,
with the data mounted at ``/data`` (``-v ~/data:/data``).


Visualization
-------------

``Logger`` logs to the `rerun <https://rerun.io>`_ viewer. By default it
connects to a viewer that is already running, at
``rerun+http://127.0.0.1:9876/proxy``, so start one first with ``rerun``. The
environment variable ``CARTESIAN_VIZ`` changes this:

.. code-block::

  rerun &                                 # Connect to the running viewer
  CARTESIAN_VIZ=spawn ./build/unittests   # Start a new viewer
  CARTESIAN_VIZ=0 ./build/unittests       # Log nothing, e.g. in docker or CI

``CARTESIAN_VIZ_URL`` sets the address to connect to, e.g. a viewer on the host
from inside a container: ``CARTESIAN_VIZ=1
CARTESIAN_VIZ_URL=rerun+http://<host ip>:9876/proxy``. The Docker image sets
``CARTESIAN_VIZ=0``.


License
-------

.. code-block::

  MIT License

  Copyright (c) <2025> <Christopher Choi>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
