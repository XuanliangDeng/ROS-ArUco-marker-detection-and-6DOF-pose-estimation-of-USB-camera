# Pose estimation with ROS and chArUco marker
This is the project for Summer 2018, supervised by Professor Patricio A. Vela &amp; Yipu Zhao, for ICRA 2019

# Description
Device: Parrot ARDrone 2.0   Marker: chArUco board 

## Device connection
Turn on the drone and search for ardrone wifi on the laptop, connect the right wifi and start roscore and ardrone package in terminal
Open one terminal to start roscore
```C++
$ roscore
```
Open a new terminal and type
```C++
$ roslaunch ardrone_autonomy ardrone.launch 
```
If everything goes on well, you will see some information about drone printed on the screen, it means the connection is successful.


## Run
Run the following code to start detection, the code could be similar like this 
```C++
$ ./devel/lib/ardrone_test/Detection -c='/home/parallels/opencv_contrimodules/aruco/samples/calibrate_camera.yml' -d=14 --dp='/home/parallels/opencv_contrib/modules/aruco/samples/detector_params.yml' -h=7 --ml=0.025 --sl=0.034 -w=5

```
Move the chArUco board into the sight of the camera, if everything goes well, you should see an axis(green, red, blue)shows up at the corner of the board and the translation vector and rotation vector printed on the screen


