MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))
ROOT_DIR := $(patsubst %/,%,$(dir $(MKFILE_DIR)))
BUILD_DIR := $(ROOT_DIR)/build
PREFIX := $(ROOT_DIR)/deps
OUT_DIR := $(MKFILE_DIR)/output
NUM_PROCS := $(shell expr `nproc` / 2)

EXP_DETECTOR_DIR := $(MKFILE_DIR)/exp_detector
EXP_DETECTOR_CONFIG := $(EXP_DETECTOR_DIR)/config.yaml
EXP_DETECTOR_REPORT := $(EXP_DETECTOR_DIR)/report

EXP_CALIB_CAMERA_DIR := $(MKFILE_DIR)/exp_calib_camera
EXP_CALIB_CAMERA_CONFIG := $(EXP_CALIB_CAMERA_DIR)/config.yaml
EXP_CALIB_CAMERA_REPORT := $(EXP_CALIB_CAMERA_DIR)/report

.PHONY: help
help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-24s\033[0m%s\n", $$1, $$2}'

.PHONY: build
build: ## Build the experiments
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) \
		&& cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_PREFIX_PATH=$(PREFIX) \
			-DCMAKE_LIBRARY_PATH=$(PREFIX)/lib \
			.. \
		&& make -j$(NUM_PROCS) exp_detector exp_calib_camera

.PHONY: exp_detector
exp_detector: build ## Run the detector experiment and generate its report
	@mkdir -p $(OUT_DIR)
	@cd $(OUT_DIR) && $(BUILD_DIR)/exp_detector $(EXP_DETECTOR_CONFIG)
	@python3 $(EXP_DETECTOR_DIR)/make_report.py \
		$(OUT_DIR)/exp_detector_output \
		$(EXP_DETECTOR_CONFIG) \
		$(EXP_DETECTOR_REPORT)

.PHONY: exp_calib_camera
exp_calib_camera: build ## Run the camera calibration experiment and generate its report
	@mkdir -p $(OUT_DIR)
	@cd $(OUT_DIR) && $(BUILD_DIR)/exp_calib_camera $(EXP_CALIB_CAMERA_CONFIG)
	@python3 $(EXP_CALIB_CAMERA_DIR)/make_report.py \
		$(OUT_DIR)/exp_calib_camera_output \
		$(EXP_CALIB_CAMERA_CONFIG) \
		$(EXP_CALIB_CAMERA_REPORT)

.PHONY: all
all: exp_detector exp_calib_camera ## Run all experiments

.PHONY: clean
clean: ## Remove experiment outputs (reports are kept)
	@rm -rf $(OUT_DIR)
