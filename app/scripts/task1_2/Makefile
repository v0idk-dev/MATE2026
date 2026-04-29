CXX = clang++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I./include $(shell pkg-config --cflags opencv4)
LDFLAGS = $(shell pkg-config --libs opencv4)

SRCDIR = src
OBJDIR = build
TARGET = stereo_distance

SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SOURCES))

##############################
# OPENCV FIX - DO NOT REMOVE #
##############################

.PHONY: all dist clean run help

all: $(TARGET)

dist: $(TARGET)
	@bash ../../fix-opencv.sh

##############################
#       END OPENCV FIX       #
##############################

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
