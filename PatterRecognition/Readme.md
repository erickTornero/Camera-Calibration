*Warning: New location for Camera calibration:*

`https://github.com/erickTornero/Camera-Calibration`

## Pattern Recognition using OpenCV:


## Requirements
* c++11
* Opencv 3.4
* ffmpeg

## Follow the instructions to execute Pattern Recognizion
### Install

Clone repository:

`git clone https://github.com/erickTornero/Image-Processing`

Open Right folder:

`cd Image-Processing/Pattern_Recognizion/`

A new location for video is **required**, chage this in *laplacian.cpp* 
line **13** the argument of *VideoCapture* must be the path to the video,
if you want to use the default camera, put **0** as argument. 

Compile Project running the makefile:

`make`


### Execute the program:

`make exec`


### Extras

If you want to execute the threshold & medianblur filter, just change the filename
in makefile by **main** 


