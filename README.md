### Description ###
The concept of this project is to detect and track people in an indoor environment and recognize events regarding their movement. The visual information used consists of an RGB stream and a depth stream from an ASUS Xtion Pro or Microsoft Kinect.

* **Chroma**
 * Description: 
   This node is responsible for the preprocessing step of the RGB image. It performs some transformations on the RGB image(contrast, gamma) and publishes the processed version along with the frame difference of each RGB image from its previous. 
 * Input : 
     * RGB image
 * Output: 
     * processed RGB image
     * RGB image difference
* **Depth**
 * Description: 
   This node is responsible for the preprocessing step of the depth image. It attempts to fill areas where the sensor cannot calculate their depth(e.g. reflective surfaces) and publishes the processed version along with the frame difference of each depth image from its previous.
 * Input : 
       * depth image
 * Output: 
     * processed depth image
     * depth image difference
* **Fusion**
 * Description: This node is responsible for detecting and tracking people and calculating their physical measurements(e.g. locations in meters, height). The detection and tracking are performed on the RGB image which is more consistent and less error prone while the physical measurements are produced from the depth image. 
 * Input : 
      * processed RGB image
      * RGB image difference
      * processed depth image
      * depth image difference
 * Output:
      * Bounded boxes(areas of detected people)
* **Decision_making**
 * Description: This node is responsible for detecting events of the people in our view. The events recognized are sitting up and walking 4 meters. To do so it collects the output of the Fusion node(bounded boxes) and uses their location(x,y,z) and height.  
 * Input : 
      * Bounded boxes(areas of detected people)
 * Output:
      * Event detected
* **Ros_visual**
    * Description: Meta package that serves to ease the initilization process. Contains launch files to avoid the burden of launching each node seperately.
* **Vision**
    * Description: Computer vision library that contains the core functionality of the project e.g. detecting and tracking people, producing measurements from the depth image etc.

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
