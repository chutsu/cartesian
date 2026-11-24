FROM ubuntu:22.04

# Build settings
ARG NPROCS=4
WORKDIR /cartesian

# Install make
RUN apt-get update \
    && apt-get install -y --no-install-recommends make \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
COPY deps/Makefile deps/Makefile
COPY deps/src/ethz_apriltag deps/src/ethz_apriltag
RUN make -C deps apt && rm -rf /var/lib/apt/lists/*
# One layer per dependency, make builds the ones that are not installed yet
RUN make -C deps NUM_PROCS=${NPROCS} yaml clean-src
RUN make -C deps NUM_PROCS=${NPROCS} googletest clean-src
RUN make -C deps NUM_PROCS=${NPROCS} eigen clean-src
RUN make -C deps NUM_PROCS=${NPROCS} suitesparse clean-src
RUN make -C deps NUM_PROCS=${NPROCS} ceres clean-src
RUN make -C deps NUM_PROCS=${NPROCS} opencv clean-src
RUN make -C deps NUM_PROCS=${NPROCS} apriltag clean-src
RUN make -C deps NUM_PROCS=${NPROCS} ethz_apriltag clean-src
RUN make -C deps NUM_PROCS=${NPROCS} pcl clean-src
RUN make -C deps NUM_PROCS=${NPROCS} rerun clean-src

# Run bash
CMD ["bash"]
