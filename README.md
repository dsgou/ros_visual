### Description ###
The concept of this project is to detect and track people in indoor environment and produce some statistics regarding
their movement and physique(e.g. height).


* **Chroma**
 * Description: 
   Processes the RGB image to be more useful and publishes the processed version and the image difference  
 * Input : 
 	   * RGB image
 * Output: 
 	   * processed RGB image
	   * RGB image difference
* **Depth**
 * Description: 
   Processes the depth image and corrects the holes and publishes the processed version and the depth difference
 * Input : 
  	   * depth image
 * Output: 
 	   * processed depth image
  	   * depth image difference
* **Fusion**
 * Description: Combines the output of the Chroma and Depth nodes and produces high level statistics
 * Input : 
  	   * RGB image
	   * RGB image difference
	   * depth image
	   * depth image difference
* **ros_visual**
 * Description: High level package to help launch all the nodes and automate the process, only contains launch files
* **vision**
 * Description: Computer vision library
  
### Set up ###

* The example instructions are for Ubuntu 14.04. Please check your os version and replace indigo with the respective version.

* Install ROS on your system by following the installation instructions (www.ros.org), install according to your OS: 

  ```
    sudo apt-get install ros-indigo-desktop-full 
  ```
* Install opencv either as a ROS package or standlone depending on your version 

  ```
    sudo apt-get install ros-indigo-vision-opencv
  ```
* Install ros-distribution-openni-launch, those are the kinect drivers 

  ```
    sudo apt-get install ros-indigo-openni-launch
  ```
* Install catkin:

  ```
    sudo apt-get install ros-indigo-catkin
    source /opt/ros/indigo/setup.bash
  ```
* Create catkin workspace:

  ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
  ```
  * check that the path is included by: ```echo $ROS_PACKAGE_PATH```
* Copy this project in ```~/catkin_ws/src```
* Install Alglib (clustering for depth estimation), in case the stdafx.h file needed is not in /usr/include/
check where your system has  stored it and add the directory in the CMAkeList.txt link_directories()

* Run in terminal: 

  ```
  catkin_make
  ```

### Development & Testing ###
* Make project : catkin_make
* Create rosbag: rosbag record [TOPICS/OPTIONS] 
* Play rosbag  : rosbag play bagfile.bag
* List topics  : rostopic list


### Run ###
* The launch/config.xml file contains the run configurations of the project
* Edit according to your needs
* Make project : catkin_make
* To run with kinect:
      
```
 roslaunch openni_launch openni.launch	(kinect drivers)
 roslaunch ros_visual config.xml		(run project)
```

* ... or in case openni_launch fails, could also try freenect instead:
```
 roslaunch freenect_launch freenect.launch
 roslaunch ros_visual config.xml
```

* To run a rosbag:
- Edit config.launch: playback_topics = True
- Run in terminal:
```
roslaunch ros_visual config.xml	
rosbag play bagfile.bag
```

### Who do I talk to? ###

* Dimitris Sgouropoulos: dsgou@hotmail.gr
