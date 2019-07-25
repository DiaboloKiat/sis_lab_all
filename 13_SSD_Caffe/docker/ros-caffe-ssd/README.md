This is based on the Dockerfile for peterx's base_image


## Build 

Get ready to run Jupyter Notebook (replace argnctu to your dockerhub account)
* install caffe with ssd
* install jupyter and gdown
* download a pretrained model and rename it for ipynb

```
$ docker build -t argnctu/ros-caffe-ssd -f Dockerfile .
```

Get ready to train
* download VOC 07 and 12 and place to /root/data
* get the rest data preparation ready
* change examples/ssd/ssd_pascal.py settings 
If want to train SSD on workstation, need to modify examples/ssd/ssd_pascal.py:
Line 332 gpus = "0, 1, 2, 3" to "0"
Line 337 batch_size = 32 to batch_size = 16
Line 338 accum_batch_size = 32 to accum_batch_size = 16

```
$ docker build -t argnctu/ros-caffe-ssd-train -f Dockerfile-train .
```

## Train

```
ws $ nvidia-docker run -it --name ros-caffe-ssd --rm -v /home/hchengwang:/hosthome -p 8888:8888 argnctu/ros-caffe-ssd-train

container # python examples/ssd/ssd_pascal.py
```


