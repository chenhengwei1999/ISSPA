# Usage Guide

## Install

```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt 
```

## Training

Use `./data/VOC.yaml` to save as ./data/data.yaml for modification.

```python
# Train/val/test sets as 
# 1) dir: path/to/imgs, 
# 2) file: path/to/imgs.txt, or 
# 3) list: [path/to/imgs1, path/to/imgs2, ..]

path: ./cinTA_v2-1   ## your dataset path
train: # train images 
  - train/images
  - valid/images
val: # val images (relative to 'path')  
  - test/images
test: # test images (optional)
  - test/images

# Classes
nc: 3  # number of classes
names: ['green', 'red', 'yellow']  # class names
```



Use `./model/yolov5s.yaml` to save as `./model/yolov5s_traffic_lights.yaml` for modification

```python
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license

# Parameters
# Change nc to the number of types that need to be trained
nc: 3  # number of classes
```

Dataset link:

```python
# paste its in jupyter notebook and run
from roboflow import Roboflow
rf = Roboflow(api_key="iOqPD5zonb3Ou6XZ2Xpe")
project = rf.workspace("wawan-pradana").project("cinta_v2")
dataset = project.version(1).download("yolov5")
```



## Weights

You can find our trained weights [here](https://drive.google.com/drive/folders/1h8PMiA1As6Gy__V9bRYqrgFeHsgx3Uh2?usp=drive_link).



## Detect Result

Use ```python train.py --ephos 100 --data ./data/data.yaml --model yolov5l_traffic_lights.yaml``` to train:

![red](/docs/source/imgs/yolov5_traffic_light_red.jpg)

![yellow](/docs/source/imgs/yolov5_traffic_light_yellow.jpg)

![green](/docs/source/imgs/yolov5_traffic_light_green.jpg)

## Issues & Solution

Q1：

```cmd
OSError: [WinError 1455] 页面文件太小，无法完成操作。 Error loading "C:\Users\86433\.conda\envs\yolov5-7.0\lib\site-packages\torch\lib\caffe2_detectron_ops_gpu.dll" or one of its dependencies.
```

A1：

在```train.py	```中找到

```python
# 调小workers的默认值，我的显卡是6G的，调到4可以正常跑，可以根据自身显卡实际情况做调整
parser.add_argument('--workers', type=int, default=8, help='max dataloader workers (per RANK in DDP mode)')
# 实测12G的显卡可以将--workers 调到8
```



Q2：

```
torch装成cpu版本如何快速替换
```

A2：

```cmd
pip uninstall torch     

pip uninstall cudatoolkit

pip3 install torch==1.10.2+cu113 torchvision==0.11.3+cu113 torchaudio==0.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
```



Q3：

custom数据集训练时报错

```
assertionerror: no labels found in */jpegimages.cache. can not train without labels.
```

A3:

```
# 数据集遵循以下格式存放
├── yolov5
└── datasets
   └── test
       └── images
       └── labels
   └── train
       └── images
       └── labels
   └── vaild
       └── images
       └── labels
```



Q4：

终端报错

```cmd
AttributeError: 'FreeTypeFont' object has no attribute 'getsize'
```



A4:

Pillow等级过高，降到9.5即可

```cmd
pip install Pillow==9.5
```



