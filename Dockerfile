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
RUN make -C deps NUM_PROCS=${NPROCS} all \
    || { tail -n 50 deps/log/*.log; exit 1; } \
    && rm -rf deps/log deps/src/ethz_apriltag/build \
    && find deps/src -mindepth 1 -maxdepth 1 ! -name ethz_apriltag \
        -exec rm -rf {} +

# Run bash
CMD ["bash"]
