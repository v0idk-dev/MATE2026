# Makefile — overhauled Task 1.2 pipeline (Apple Silicon).
#
# Layout differs from the original (sources moved into src/, headers into
# include/, Apple-only bridges in objc/, Metal kernels in metal/) but the
# overall convention — including the OPENCV FIX block, the `dist` target
# that runs fix-opencv.sh, and the help/run/clean targets — is preserved
# from the previous Makefile so existing scripts that invoke `make dist`
# continue to work.

CXX      ?= clang++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-parameter -O2 -fPIC
INCLUDES := -I./include -I.

##############################
# OPENCV FIX - DO NOT REMOVE #
##############################
# Resolve OpenCV via (in order): explicit OPENCV_PREFIX env var → pkg-config
# opencv4 → Homebrew default. The same `make dist` rule from the previous
# Makefile is preserved at the bottom; it shells out to fix-opencv.sh so
# the binary's dyld @rpath is patched for distribution outside the dev box.
ifneq ($(OPENCV_PREFIX),)
  OPENCV_CFLAGS := -I$(OPENCV_PREFIX)/include/opencv4
  OPENCV_LIBS   := -L$(OPENCV_PREFIX)/lib -lopencv_core -lopencv_imgproc \
                   -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d \
                   -lopencv_flann -lopencv_highgui
else ifeq ($(shell pkg-config --exists opencv4 && echo yes),yes)
  OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
  OPENCV_LIBS   := $(shell pkg-config --libs opencv4)
else
  OPENCV_PREFIX_DEFAULT := $(shell brew --prefix opencv 2>/dev/null)
  ifeq ($(OPENCV_PREFIX_DEFAULT),)
    OPENCV_PREFIX_DEFAULT := /opt/homebrew/opt/opencv
  endif
  OPENCV_CFLAGS := -I$(OPENCV_PREFIX_DEFAULT)/include/opencv4
  OPENCV_LIBS   := -L$(OPENCV_PREFIX_DEFAULT)/lib -lopencv_core -lopencv_imgproc \
                   -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d \
                   -lopencv_flann -lopencv_highgui
endif

# Optional: opencv_ximgproc for WLS post-filter + FastLineDetector.
# Modules gracefully no-op if the header isn't present (#if __has_include).
ifeq ($(shell pkg-config --exists opencv4 && pkg-config --variable=libdir opencv4 2>/dev/null),)
  XIMGPROC_LIB := $(shell ls $(OPENCV_PREFIX_DEFAULT)/lib/libopencv_ximgproc.* 2>/dev/null | head -n1)
  ifneq ($(XIMGPROC_LIB),)
    OPENCV_LIBS += -lopencv_ximgproc
  endif
else
  ifneq ($(shell pkg-config --libs opencv4 | grep -o ximgproc),)
    OPENCV_LIBS += -lopencv_ximgproc
  endif
endif
##############################
#       END OPENCV FIX       #
##############################

CXXFLAGS += $(INCLUDES) $(OPENCV_CFLAGS)
LDFLAGS  := $(OPENCV_LIBS)

UNAME_S := $(shell uname -s)
TARGET  := task1_2

# Pure C++ sources (always compiled).
CPP_SRCS := \
    src/calibration_io.cpp \
    src/stereo_rectifier.cpp \
    src/plate_detector.cpp \
    src/pipe_detector.cpp \
    src/triangulator.cpp \
    src/scale_estimator.cpp \
    src/underwater.cpp \
    src/wireframe_builder.cpp \
    src/dense_mvs.cpp \
    src/scene_io.cpp \
    src/model3d.cpp \
    src/manual_scale.cpp \
    src/image_undistort.cpp \
    src/accelerate_utils.cpp \
    src/per_pair_model.cpp \
    src/manhattan_calib.cpp \
    src/multi_pair_fuse.cpp \
    src/ai_enhancer.cpp \
    src/stereo_math.cpp \
    src/plate_pnp.cpp \
    src/refine_scale.cpp \
    src/lab_segment.cpp \
    src/plate_fusion.cpp \
    src/pipe_lsd.cpp \
    src/match_ransac.cpp \
    src/underwater_restore.cpp \
    src/depth_segment.cpp \
    src/bundle_adjust.cpp \
    src/pvc_segment.cpp \
    src/sgbm_disparity.cpp \
    src/pipe_lines_multi.cpp \
    src/pipe_ransac.cpp \
    src/pipe_diameter.cpp \
    src/pipe_match_stereo.cpp \
    src/pipe_sampson.cpp \
    src/pipe_cylinder3d.cpp \
    src/pipe_graph.cpp \
    src/pipe_bundle.cpp \
    src/pipe_parallel_pair.cpp \
    src/pipe_pink_tape.cpp \
    src/pipe_template.cpp \
    src/image_quality.cpp \
    src/pipe_graph_validate.cpp \
    src/pipe_pipeline.cpp \
    src/pipeline.cpp \
    src/main.cpp

# Eigen (header-only) for Umeyama + bundle adjust
EIGEN_PREFIX ?= $(shell brew --prefix eigen 2>/dev/null)
ifneq ($(EIGEN_PREFIX),)
  CXXFLAGS += -I$(EIGEN_PREFIX)/include/eigen3
endif

CPP_OBJS := $(CPP_SRCS:.cpp=.o)

# ── Apple Silicon extras ───────────────────────────────────────────────
ifeq ($(UNAME_S),Darwin)
  OBJCXX_SRCS := objc/metal_compute.mm objc/apple_vision.mm objc/apple_intelligence.mm objc/vision_rectangles.mm
  OBJCXX_OBJS := $(OBJCXX_SRCS:.mm=.o)
  METAL_SRC   := metal/kernels.metal
  METAL_AIR   := metal/kernels.air
  METAL_LIB   := task1_2_kernels.metallib

  CXXFLAGS += -DMATE_APPLE=1
  LDFLAGS  += -framework Foundation -framework Metal -framework CoreVideo \
              -framework CoreImage -framework Vision -framework AppKit \
              -framework Accelerate \
              -weak_framework FoundationModels
  ALL_OBJS := $(CPP_OBJS) $(OBJCXX_OBJS)
  EXTRA    := $(METAL_LIB)
else
  ALL_OBJS := $(CPP_OBJS)
  EXTRA    :=
endif

.PHONY: all dist clean run help

all: $(TARGET) $(EXTRA)

$(TARGET): $(ALL_OBJS)
	$(CXX) $(ALL_OBJS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

ifeq ($(UNAME_S),Darwin)
%.o: %.mm
	$(CXX) $(CXXFLAGS) -ObjC++ -fobjc-arc -c $< -o $@

$(METAL_AIR): $(METAL_SRC)
	xcrun -sdk macosx metal -c -O3 -o $@ $<

$(METAL_LIB): $(METAL_AIR)
	xcrun -sdk macosx metallib -o $@ $<
endif

##############################
# OPENCV FIX - DO NOT REMOVE #
##############################
# `dist` builds the binary AND rewrites its OpenCV dyld paths via the
# project-level fix-opencv.sh helper (lives at <repo_root>/fix-opencv.sh,
# i.e. two levels up from this Makefile).
dist: $(TARGET) $(EXTRA)
	@bash ../../fix-opencv.sh "$(CURDIR)/$(TARGET)"
##############################
#       END OPENCV FIX       #
##############################

clean:
	rm -f $(ALL_OBJS) $(TARGET) $(METAL_AIR) $(METAL_LIB)

run: $(TARGET)
	./$(TARGET) --help

help:
	@echo "MATE 2026 — Photogrammetry Pipeline (Task 1.2, overhauled)"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build the binary + Metal kernels (default)"
	@echo "  dist    - Build + run fix-opencv.sh for distribution"
	@echo "  clean   - Remove build artifacts"
	@echo "  run     - Build and show usage"
	@echo "  help    - Show this message"
	@echo ""
	@echo "Quick test:"
	@echo "  ./task1_2 \\"
	@echo "    --pair pair0_L.png pair0_R.png \\"
	@echo "    --pair pair1_L.png pair1_R.png \\"
	@echo "    --left-calib left.yaml --right-calib right.yaml \\"
	@echo "    --stereo stereo.yaml \\"
	@echo "    --out model.json --glb model.glb --debug-dir /tmp/dbg"
