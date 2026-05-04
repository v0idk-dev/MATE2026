# AI Model Training

## Setup

First, `cd /crab-detect/train/` and install python 3.11. Then, run the following commands:

```bash
python3.11 -m venv venv
venv/bin/python3 -m pip install --upgrade pip
venv/bin/python3 -m pip install ultralytics opencv-python matplotlib
```

Create the following directories according to this file structure:

```
---------------------------------------  PREEXISTING FILES  --------------------------------------
data.yaml                         - data definitions and structure
README.md                         - this file
check_labels.py                   - label validator
fix_tal.py                        - tal patch applier
----------------------------------------  INSTALLED FILES  ---------------------------------------
venv/                             - python environment
yolov8n.pt                        - base YOLO model
-------------------------------------------  NEW FILES  ------------------------------------------
dataset/                          - all data
  images/                         - all images
    train/                        - training images (~75% of your data)
    val/                          - validation images (~25% of your data)
  labels/                         - annotation files
    train/                        - put corresponding annotations from training images
    val/                          - put corresponding annotations from validation images
--------------------------------------------------------------------------------------------------
```

Now run these commands:

```bash
rm dataset/labels/*.cache
venv/bin/python3 ./check_labels.py
venv/bin/python3 ./fix_tal.py
```

Make sure the `check_labels` python script returns no issues AND the `fix_tal` applies both patches (on first run), otherwise the model may fail while training.

## Training

Run the following command to train:

```bash
venv/bin/yolo detect train \
model=yolov8n.pt \
data=data.yaml \
epochs=100 \ # 100-200 works best; 50-100 works
imgsz=640 \
batch=16 \
device=mps \
workers=4 \
fliplr=0.5 \
scale=0.3 # Higher values may increase chance of failing
```

## Running

```bash
cd ../
venv/bin/yolo detect predict model=models/M26.CD.P1.300__3.6.2026_21.54/outputs/main.3_81.pt source=train/dataset/images/val conf=0.50
```
