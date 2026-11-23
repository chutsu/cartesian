# cartesian build environment
#
#   docker build -t cartesian .
#   docker run --rm cartesian
#
# Ubuntu 22.04 matches CMakeLists.txt, which uses clang and looks for libomp in
# /usr/lib/llvm-14/lib. The dependencies in deps/ are built from source and
# take a long time, so they are built before the sources are copied in: the
# layer is only rebuilt when deps/Makefile or deps/src/ethz_apriltag change.
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
# Number of parallel jobs, `docker build --build-arg JOBS=$(nproc) .`
ARG JOBS=4

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    git \
    libblas-dev \
    libboost-all-dev \
    libflann-dev \
    libgflags-dev \
    libgmp-dev \
    libgoogle-glog-dev \
    liblapack-dev \
    libmpfr-dev \
    libomp-dev \
    pkg-config \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /cartesian

# Dependencies: yaml-cpp, googletest, eigen, suitesparse, ceres, opencv,
# apriltag, ethz_apriltag, pcl and the rerun sdk. The sources are cloned by the
# makefile, only ethz_apriltag is part of the repo. The makefile hides the
# output in deps/log, so print the end of the logs if it fails.
COPY deps/Makefile deps/Makefile
COPY deps/src/ethz_apriltag deps/src/ethz_apriltag
RUN make -C deps all NUM_PROCS=${JOBS} \
    || { tail -n 50 deps/log/*.log; exit 1; }

# Build cartesian
COPY . .
RUN mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/cartesian/deps .. \
    && make -j${JOBS}

# The Logger connects to a rerun viewer, there is none in the container, so turn
# it off. To log to a viewer on the host instead, run the container with
# `-e CARTESIAN_VIZ=1 -e CARTESIAN_VIZ_URL=rerun+http://<host ip>:9876/proxy`.
# Some tests read the EuRoC dataset from /tmp/euroc, mount it with
# `-v /path/to/euroc:/tmp/euroc`.
ENV CARTESIAN_VIZ=0
CMD ["./build/unittests", "--gtest_filter=MarginalError.*:ImuError.*:TiltManifold.*:ReprojError.*"]
