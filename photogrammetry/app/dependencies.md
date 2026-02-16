# Python

```bash
app/Resources/python-runtime/bin/pip3 install opencv-python blinker click colorama flask gunicorn itsdangerous jinja2 markupsafe packaging werkzeug flask numpy
```

# C++

```bash
brew install opencv
mkdir -p app/Resources/opencv-libs
cp -R /opt/homebrew/Cellar/opencv/*/lib/* app/Resources/opencv-libs/
cp -R /opt/homebrew/Cellar/opencv/*/include/* app/Resources/opencv-include/
```

# Makefile

Add:

```makefile
OPENCV_CFLAGS = -I../../app/Resources/opencv-include
OPENCV_LIBS = -L../../app/Resources/opencv-libs -lopencv_core -lopencv_imgproc -lopencv_calib3d
```

# Run Command

```bash
make -C scripts/task1_2 all && app/Resources/python-runtime/bin/python3 web/app.py
```
