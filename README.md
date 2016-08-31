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

### Vagrant Base Box ###
If you want to test the code without setting up your own catkin workspace, you can use the provided vagrant base box. A .bag file is also included in the home folder of the ros user. In order to use this base box, you'll need to have the latest version of virtualbox installed. If you don't, please visit https://www.virtualbox.org/wiki/Downloads. After downloading virtualbox, and installing vagrant, you can run:
```
 vagrant init gstavrinos/ROS_Indigo64; vagrant up --provider virtualbox
 ```
 The above commands will initialize the base box and then run it using vitualbox. You can then open virtualbox and access the virtual machine using the graphical user interface. Log in as ros user, using the password ros. Now open a terminal, and test the ros_visual package:
 ```
roslaunch ros_visual ros_visual.launch
 ```
 Now you need to provide some input to ros_visual, using the test.bag file (You might need to play the test.bag file multiple times, since it is ~7.5 seconds long). On a new terminal, run:
 ```
rosbag play /home/ros/test.bag
 ```
 In order to see the results of the rgb-depth fusion, on a new terminal, run:
 ```
rostopic echo /fusion/results
 ```
  
### Set up ###

* The example instructions are for Ubuntu 14.04. Please check your os version and replace indigo with the respective version.

* Install ROS on your system by following the installation instructions (www.ros.org), install according to your OS: 

  ```
    sudo apt-get install ros-indigo-desktop-full 
  ```
* Install opencv either as a ROS package or standalone depending on your version 

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
 roslaunch openni_launch openni.launch  (kinect drivers)
 roslaunch ros_visual ros_visual.launch    (run project)
```

* ... or in case openni_launch fails, could also try freenect instead:
```
 roslaunch freenect_launch freenect.launch
 roslaunch ros_visual ros_visual.launch
```

* To run a rosbag:
- Edit config.launch: playback_topics = True
- Run in terminal:
```
roslaunch ros_visual ros_visual.launch
rosbag play bagfile.bag
```

### Who do I talk to? ###

* Dimitris Sgouropoulos: dsgou@hotmail.gr
