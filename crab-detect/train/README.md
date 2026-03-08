# AI Model Training

## Setup

First, install python 3.11. Then, run the following commands:

```bash
python3.11 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install ultralytics opencv-python matplotlib
```

Create the following directories according to this file structure:

```
---------------------------------------  PREEXISTING FILES  --------------------------------------
data.yaml                         - data definitions and structure
README.md                         - this file
check_labels.py                   - label validator
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
python ./check_labels.py
```

Make sure the python script returns no issues, otherwise the model may fail to finish full training.

## Training

Run the following command to train:

```bash
yolo detect train \
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
