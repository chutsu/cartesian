MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
PREFIX := $(MKFILE_DIR)
SRC_PATH := $(PREFIX)/src
LOG_PATH := $(PREFIX)/log
NUM_PROCS := $(shell expr `nproc` / 2)

# System packages (Ubuntu 22.04) to build the dependencies and cartesian
APT_PACKAGES := \
	build-essential ca-certificates clang cmake git unzip wget \
	libblas-dev libboost-all-dev libflann-dev libgflags-dev libgmp-dev \
	libgoogle-glog-dev liblapack-dev libmpfr-dev libomp-dev
SUDO := $(shell [ `id -u` -ne 0 ] && echo sudo)

YAML_VERSION=0.8.0
ABSEIL_VERSION=20240116
GTEST_VERSION=v1.17.0
EIGEN_VERSION=3.4.0
SUITESPARSE_VERSION=v7.11.0
CERES_VERSION=2.2.0
OPENCV_VERSION=5.0.0
APRILTAG_VERSION=v3.4.5
APRILTAG_IMGS_VERSION=master
PCL_VERSION=pcl-1.15.0
RERUN_VERSION=0.25.1

YAML_REPO=https://github.com/jbeder/yaml-cpp
ABSEIL_REPO=https://github.com/abseil/abseil-cpp
GTEST_REPO=https://github.com/google/googletest
EIGEN_REPO=https://gitlab.com/libeigen/eigen
SUITESPARSE_REPO=https://github.com/DrTimothyAldenDavis/SuiteSparse
CERES_REPO=https://github.com/ceres-solver/ceres-solver
OPENCV_REPO=https://github.com/opencv/opencv
APRILTAG_REPO=https://github.com/AprilRobotics/apriltag
APRILTAG_IMGS_REPO=https://github.com/AprilRobotics/apriltag-imgs
PCL_REPO=https://github.com/PointCloudLibrary/pcl
RERUN_URL=https://github.com/rerun-io/rerun/releases/download/$(RERUN_VERSION)/rerun_cpp_sdk.zip

define get_repo_name
$(basename $(notdir $(patsubst %/,%,$(1))))
endef

define git_clone
	@( \
	if [ ! -d "$(SRC_PATH)/$(call get_repo_name,$(1))" ]; then \
		cd $(SRC_PATH) && git clone $(1); \
	fi \
	&& cd $(SRC_PATH)/$(call get_repo_name,$(1)) \
		&& git checkout $(2) \
		&& { if [ -f .gitmodules ]; then git submodule update --init; fi; } \
	) > "$(LOG_PATH)/$(call get_repo_name,$(1)).log" 2>&1
endef

define cmake_build
	@echo "Building [$(1)] ..."
	@( \
	cd $(SRC_PATH)/$(1) \
		&& mkdir -p build \
		&& cd build || return \
		&& cmake \
			-DCMAKE_COLOR_MAKEFILE=OFF \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH=$(PREFIX) \
			-DCMAKE_INSTALL_PREFIX=$(PREFIX) \
			.. \
		&& make -j$(NUM_PROCS) \
		&& make install; \
	) > "$(LOG_PATH)/$(1).log" 2>&1
endef

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-15s\033[0m%s\n", $$1, $$2}'

default: setup

setup:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@mkdir -p src

apt: ## Install the system packages with apt
	$(SUDO) apt-get update
	$(SUDO) env DEBIAN_FRONTEND=noninteractive apt-get install -y \
		--no-install-recommends $(APT_PACKAGES)

clean: ## Clean
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share
	@rm -rf src

yaml: setup ## Build yaml
	$(call git_clone,$(YAML_REPO),$(YAML_VERSION))
	$(call cmake_build,yaml-cpp)

abseil: setup ## Build abseil
	$(call git_clone,$(ABSEIL_REPO),$(ABSEIL_VERSION))
	$(call cmake_build,abseil-cpp)

googletest: setup ## Build googletest
	$(call git_clone,$(GTEST_REPO),$(GTEST_VERSION))
	$(call cmake_build,googletest)

eigen: setup ## Build eigen
	$(call git_clone,$(EIGEN_REPO),$(EIGEN_VERSION))
	$(call cmake_build,eigen)

suitesparse: setup eigen ## Build suitesparse
	$(call git_clone,$(SUITESPARSE_REPO),$(SUITESPARSE_VERSION))
	$(call cmake_build,SuiteSparse)

ceres: setup suitesparse ## Build ceres-solver
	$(call git_clone,$(CERES_REPO),$(CERES_VERSION))
	$(call cmake_build,ceres-solver)

opencv: setup eigen ## Build opencv
	$(call git_clone,$(OPENCV_REPO),$(OPENCV_VERSION))
	$(call cmake_build,opencv)

apriltag: setup opencv  ## Build apriltag
	$(call git_clone,$(APRILTAG_REPO),$(APRILTAG_VERSION))
	$(call git_clone,$(APRILTAG_IMGS_REPO),$(APRILTAG_IMGS_VERSION))
	$(call cmake_build,apriltag)

ethz_apriltag: setup opencv  ## Build ethz_apriltag
	$(call cmake_build,ethz_apriltag)

pcl: setup eigen ## Build pcl
	$(call git_clone,$(PCL_REPO),$(PCL_VERSION))
	$(call cmake_build,pcl)

rerun: setup ## Build rerun
	@wget $(RERUN_URL) -O $(SRC_PATH)/$(shell basename $(RERUN_URL))
	@unzip -o $(SRC_PATH)/$(shell basename $(RERUN_URL)) -d $(SRC_PATH)
	@rm $(SRC_PATH)/$(shell basename $(RERUN_URL))
	$(call cmake_build,rerun_cpp_sdk)

BUILD_TARGETS := \
	yaml \
	googletest \
	eigen \
	suitesparse \
	ceres \
	opencv \
	apriltag \
	ethz_apriltag \
	pcl \
	rerun
all: $(BUILD_TARGETS)  ## Build all
