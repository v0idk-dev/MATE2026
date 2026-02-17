###########################################
# OPENCV FIX - MODIFIED, DO NOT OVERWRITE #
###########################################

OPENCV_CFLAGS = -I../../app/Resources/opencv-include
OPENCV_LIBS = -L../../app/Resources/opencv-libs -lopencv_core -lopencv_imgproc -lopencv_calib3d

all: stereo_distance
	@bash ../../fix-opencv.sh

stereo_distance: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

###########################################
#             END OPENCV FIX              #
###########################################

CXX = clang++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I./include $(shell pkg-config --cflags opencv4)
LDFLAGS = $(shell pkg-config --libs opencv4)

SRCDIR = src
OBJDIR = build
TARGET = stereo_distance

SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SOURCES))

.PHONY: all clean run help

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR) $(TARGET) output/

run: $(TARGET)
	./$(TARGET) --help

help:
	@echo "Stereo Vision Distance Calculator"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build the program (default)"
	@echo "  clean   - Remove build artifacts"
	@echo "  run     - Build and show help"
	@echo "  help    - Show this message"
	@echo ""
	@echo "Usage after build:"
	@echo "  ./stereo_distance <image1> <image2> [options]"
