# Good-Feature-ORB-SLAM---2018-Summer
This is the project for Summer 2018, supervised by Professor Patricio A. Vela &amp; Yipu Zhao

# Description
There is nothing fancy here, just want to write down my experience to remind me of what I have done. Hope this could helpful:) 
All related links &amp; sources are attached.

## Camera Calibration
To detect ArUco markers using usb cameras, the first step is camera calibration. I used [camera_calibration](http://wiki.ros.org/camera_calibration) , a ros package, to implement this. To be more specific, my USB camera is [UC20MPD_ND](https://store.spinelelectronics.com/UC20MPD_ND)

* Install
```C++
$ rosdep install camera_calibration
$ sudo apt-get install ros-indigo-usb-cam
```

* Callibration

Run the following code and use rostopic list to check whether the related topics are published.
```C++
$ source /opt/ros/indigo/setup.bash
$ roslaunch usb_cam usb_cam-text.launch
$ rostopic list
```
If everything is fine, you should see these two topics: /usb_cam/camera_info &amp; /usb_cam/image_raw. You should also see some warnings about head_camera.yaml, don't worry we are gonna fix these after calibration.

The next step is to calibrate the camera using the python file, the window will become grey when computing it's normal don't be worry. For this camera, I collected 100 samples for calibration from different viewpoints and size. I print the chessboard on a A4 sheet so each square is 2.4cm, converted into meters.

```C++
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024  image:=/usb_cam/image_raw camera:=/usb_cam
```
When it is done, click commit, the head_camera.yaml file will be saved.
One result could be like this：
```C++
('D = ', [0.02769347087200823, -0.001400754024811914, -0.004704394200188205, -0.0018190502592750099, 0.0])
('K = ', [477.00761304970223, 0.0, 308.4895022533115, 0.0, 479.5071943003428, 218.6693607752001, 0.0, 0.0, 1.0])
('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
('P = ', [486.17041015625, 0.0, 306.9950571527006, 0.0, 0.0, 487.277587890625, 215.99803311404685, 0.0, 0.0, 0.0, 1.0, 0.0])
None
# oST version 5.0 parameters

[image]

width
640

height
480

[narrow_stereo]

camera matrix
477.007613 0.000000 308.489502
0.000000 479.507194 218.669361
0.000000 0.000000 1.000000

distortion
0.027693 -0.001401 -0.004704 -0.001819 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
486.170410 0.000000 306.995057 0.000000
0.000000 487.277588 215.998033 0.000000
0.000000 0.000000 1.000000 0.000000
```

