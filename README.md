YDLIDAR ROS PACKAGE V1.1.2
=====================================================================

ROS node and test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

How to build YDLIDAR ros package
=====================================================================
1) Clone this project to your catkin's workspace src folder

2) Running catkin_make to build ydlidar_node and ydlidar_client

3) Create the name "/dev/ydlidar" for YDLIDAR

    $ roscd ydlidar/startup
    $ sudo chmod 777 ./*
    $ sudo sh initenv.sh

How to run YDLIDAR ros package
=====================================================================
There're two ways to run YDLIDAR ros package

1) Run YDLIDAR node and view in the rviz

    $ roslaunch ydlidar lidar_view.launch

Then , you can see YDLIDAR's scan result in the rviz.

2) Run YDLIDAR node and view using test application

    $ roslaunch ydlidar lidar.launch
    $ rosrun ydlidar ydlidar_client


Then, you can see YDLIDAR's scan result in the console


UPDATE LOG
=====================================================================
--20171112 v1.0.0 

1) the first edition

--20171122 v1.1.0 

1) Repair display flicker problem.

2) Repair the problem of open the serial port for many times.

3) Add multithreaded tag.

--20171127 v1.1.1 

1) Repair the problem of array bounds.

2) Add try catch exception.

--20171129 v1.1.2 

1) Repair the options of serial port.




