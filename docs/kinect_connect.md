## Asus Xtion Kinect Connection
In this part, we will see how to use the Asus Xtion kinect (image below) with the RPi and ros.  
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/3de4d65a-b65a-4f5c-b68d-fbe63f11b64f" width="400" height="300">  

First, we will use a repository that specifically made for using this kinect and ros. Clone the following [repository](https://github.com/mgonzs13/ros2_asus_xtion) into your source directory using this command:  
`git clone --recurse-submodules https://github.com/mgonzs13/ros2_asus_xtion`  
Next, make sure to install depth-image-proc package:  
`sudo apt install ros-iron-depth-image-proc`  
Now, we will install a few necessary packages:  
1. Gazebo - using the following command we will install *Ignition Gazebo*, which is the default version for ros iron: `sudo apt-get install ros-iron-ros-gz`  
2. Camera Info Manager package: `sudo apt install ros-iron-camera-info-manager`
3. Image Transport Package: `sudo apt install ros-iron-image-transport`

Next, I made a few adjustments to the `CMakeLists.txt` and `package.xml` files inside the repository, in order to make it compatible with ros iron rather than other distributions. Everywhere the package `gazebo` is required -> replace it with `ros_gz`, and everywhere that the package `gazebo_ros2_control` is required -> replace it with `gz_ros2_control`. *Everywhere else that requires a package called `gazebo_XXX` (e.g. `gazebo_plugins`) you can simply delete it. *Remember to make sure to go through all of the asus_xtion packages, both the CMakeLists.txt and the package.xml files.*
After installing the necessary files and modifying the cmake and xml files, restart the terminal (or source the ros iron environment file) and use `colcon build` to build the package. If more packages are needed, make sure to install them.  
Now, take the kinect and plug it in one of the RPi USB ports. Then, use `lsusb | grep -i "ASUS Xtion"` to make sure it is visible and connected correctly. Next, make sure the *install/setup.bash* file is sourced, and run the following command:  
`ros2 launch asus_xtion asus_xtion.launch.py`  
Next, open a second terminal and ssh to the RPi ***USING THE -X FLAG***, or the next command will fail. e.g.: `ssh -X pi@192.168.0.11`  
From the second terminal, after making sure the *setup* file is sourced, run the following command:  
`ros2 launch asus_xtion_visualization rviz2.launch.py`
You should see the rviz2 running, and the camera output should appear. *Note: using rviz over the ssh with the RPi can be very slow.* It should look something like this (I ticked off the *RobotModel* and *TF*):
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/81b5eebe-4070-4b66-955b-fe2076269ee1">
