# Modified EGO-Swarm
This repository mainly modify the source code of EGO-Swarm in three aspects:
* rename the catkin workspace
  * `mv ego-planner-swarm egoswarm`
* modify the extrinsic of depth camera to imu
  * `gedit ~/catkin_ws/src/egoswarm/src/planner/plan_env/src/grid_map.cpp`
  * modify line 109-110
    ```
    extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
        "extrinsic", 10, &GridMap::extrinsicCallback, this); //sub
    ```
  * `gedit ~/catkin_ws/src/egoswarm/src/planner/plan_manage/src/traj_server.cpp`
  * Comment line 254 and add
    ```
    nh.param("traj_server/init_yaw", last_yaw_, 0.0);
    ```
* Add or modify **.launch files** for ego_planner.
  * `cd ~/catkin_ws/src/egoswarm/src/planner/plan_manage/launch/`
  * `touch advanced_param_exp.xml`
  * `gedit ~/catkin_ws/src/egoswarm/src/planner/plan_manage/launch/advanced_param_exp.xml`
  * fill with
    ```
    <launch>
        <arg name="map_size_x_"/>
        <arg name="map_size_y_"/>
        <arg name="map_size_z_"/>

        <arg name="odometry_topic"/>
        <arg name="camera_pose_topic"/>
        <arg name="depth_topic"/>
        <arg name="cloud_topic"/>
        <arg name="extrinsic_topic"/>

        <arg name="cx"/>
        <arg name="cy"/>
        <arg name="fx"/>
        <arg name="fy"/>

        <arg name="max_vel"/>
        <arg name="max_acc"/>
        <arg name="planning_horizon"/>

        <arg name="point_num"/>
        <arg name="point0_x"/>
        <arg name="point0_y"/>
        <arg name="point0_z"/>
        <arg name="point1_x"/>
        <arg name="point1_y"/>
        <arg name="point1_z"/>
        <arg name="point2_x"/>
        <arg name="point2_y"/>
        <arg name="point2_z"/>
        <arg name="point3_x"/>
        <arg name="point3_y"/>
        <arg name="point3_z"/>
        <arg name="point4_x"/>
        <arg name="point4_y"/>
        <arg name="point4_z"/>

        <arg name="flight_type"/>
        <arg name="use_distinctive_trajs"/>

        <arg name="obj_num_set"/>

        <arg name="drone_id"/>


        <!-- main node -->
        <!-- <node pkg="ego_planner" name="ego_planner_node" type="ego_planner_node" output="screen" launch-prefix="valgrind"> -->
        <node pkg="ego_planner" name="drone_$(arg drone_id)_ego_planner_node" type="ego_planner_node" output="screen">
        
            <remap from="~odom_world" to="/drone_$(arg drone_id)_$(arg odometry_topic)"/>
            <remap from="~planning/bspline" to = "/drone_$(arg drone_id)_planning/bspline"/>
            <remap from="~planning/data_display" to = "/drone_$(arg drone_id)_planning/data_display"/>
            <remap from="~planning/broadcast_bspline_from_planner" to = "/broadcast_bspline"/>
            <remap from="~planning/broadcast_bspline_to_planner" to = "/broadcast_bspline"/>

            <remap from="~grid_map/odom" to="/drone_$(arg drone_id)_$(arg odometry_topic)"/>
            <remap from="~grid_map/cloud" to="/drone_$(arg drone_id)_$(arg cloud_topic)"/>
            <remap from="~grid_map/pose"   to = "/drone_$(arg drone_id)_$(arg camera_pose_topic)"/> 
            <remap from="~grid_map/depth" to = "/drone_$(arg drone_id)_$(arg depth_topic)"/>
            <remap from="~grid_map/extrinsic" to = "/drone_$(arg drone_id)_$(arg extrinsic_topic)"/>
            

            <!-- planning fsm -->
            <param name="fsm/flight_type" value="$(arg flight_type)" type="int"/>
            <param name="fsm/thresh_replan_time" value="1.0" type="double"/>
            <param name="fsm/thresh_no_replan_meter" value="1.0" type="double"/>
            <param name="fsm/planning_horizon" value="$(arg planning_horizon)" type="double"/> <!--always set to 1.5 times grater than sensing horizen-->
            <param name="fsm/planning_horizen_time" value="3" type="double"/>
            <param name="fsm/emergency_time" value="1.0" type="double"/>
            <param name="fsm/realworld_experiment" value="false"/>
            <param name="fsm/fail_safe" value="true"/>

            <param name="fsm/waypoint_num" value="$(arg point_num)" type="int"/>
            <param name="fsm/waypoint0_x" value="$(arg point0_x)" type="double"/>
            <param name="fsm/waypoint0_y" value="$(arg point0_y)" type="double"/>
            <param name="fsm/waypoint0_z" value="$(arg point0_z)" type="double"/>
            <param name="fsm/waypoint1_x" value="$(arg point1_x)" type="double"/>
            <param name="fsm/waypoint1_y" value="$(arg point1_y)" type="double"/>
            <param name="fsm/waypoint1_z" value="$(arg point1_z)" type="double"/>
            <param name="fsm/waypoint2_x" value="$(arg point2_x)" type="double"/>
            <param name="fsm/waypoint2_y" value="$(arg point2_y)" type="double"/>
            <param name="fsm/waypoint2_z" value="$(arg point2_z)" type="double"/>
            <param name="fsm/waypoint3_x" value="$(arg point3_x)" type="double"/>
            <param name="fsm/waypoint3_y" value="$(arg point3_y)" type="double"/>
            <param name="fsm/waypoint3_z" value="$(arg point3_z)" type="double"/>
            <param name="fsm/waypoint4_x" value="$(arg point4_x)" type="double"/>
            <param name="fsm/waypoint4_y" value="$(arg point4_y)" type="double"/>
            <param name="fsm/waypoint4_z" value="$(arg point4_z)" type="double"/>

            <param name="grid_map/resolution"      value="0.1" /> 
            <param name="grid_map/map_size_x"   value="$(arg map_size_x_)" /> 
            <param name="grid_map/map_size_y"   value="$(arg map_size_y_)" /> 
            <param name="grid_map/map_size_z"   value="$(arg map_size_z_)" /> 
            <param name="grid_map/local_update_range_x"  value="5.5" /> 
            <param name="grid_map/local_update_range_y"  value="5.5" /> 
            <param name="grid_map/local_update_range_z"  value="4.5" /> 
            <param name="grid_map/obstacles_inflation"     value="0.099" /> 
            <param name="grid_map/local_map_margin" value="10"/>
            <param name="grid_map/ground_height"        value="-0.01"/>
            <!-- camera parameter -->
            <param name="grid_map/cx" value="$(arg cx)"/>
            <param name="grid_map/cy" value="$(arg cy)"/>
            <param name="grid_map/fx" value="$(arg fx)"/>
            <param name="grid_map/fy" value="$(arg fy)"/>
            <!-- depth filter -->
            <param name="grid_map/use_depth_filter" value="true"/>
            <param name="grid_map/depth_filter_tolerance" value="0.15"/>
            <param name="grid_map/depth_filter_maxdist"   value="5.0"/>
            <param name="grid_map/depth_filter_mindist"   value="0.2"/>
            <param name="grid_map/depth_filter_margin"    value="2"/>
            <param name="grid_map/k_depth_scaling_factor" value="1000.0"/>
            <param name="grid_map/skip_pixel" value="2"/>
            <!-- local fusion -->
            <param name="grid_map/p_hit"  value="0.65"/>
            <param name="grid_map/p_miss" value="0.35"/>
            <param name="grid_map/p_min"  value="0.12"/>
            <param name="grid_map/p_max"  value="0.90"/>
            <param name="grid_map/p_occ"  value="0.80"/>
            <param name="grid_map/min_ray_length" value="0.1"/>
            <param name="grid_map/max_ray_length" value="4.5"/>

            <param name="grid_map/virtual_ceil_height"   value="2.9"/>
            <param name="grid_map/visualization_truncate_height"   value="1.8"/>
            <param name="grid_map/show_occ_time"  value="false"/>
            <param name="grid_map/pose_type"     value="1"/>  
            <param name="grid_map/frame_id"      value="world"/>

        <!-- planner manager -->
            <param name="manager/max_vel" value="$(arg max_vel)" type="double"/>
            <param name="manager/max_acc" value="$(arg max_acc)" type="double"/>
            <param name="manager/max_jerk" value="4" type="double"/>
            <param name="manager/control_points_distance" value="0.4" type="double"/>
            <param name="manager/feasibility_tolerance" value="0.05" type="double"/>
            <param name="manager/planning_horizon" value="$(arg planning_horizon)" type="double"/>
            <param name="manager/use_distinctive_trajs" value="$(arg use_distinctive_trajs)" type="bool"/>
            <param name="manager/drone_id" value="$(arg drone_id)"/>

        <!-- trajectory optimization -->
            <param name="optimization/lambda_smooth" value="1.0" type="double"/>
            <param name="optimization/lambda_collision" value="0.5" type="double"/>
            <param name="optimization/lambda_feasibility" value="0.1" type="double"/>
            <param name="optimization/lambda_fitness" value="1.0" type="double"/>
            <param name="optimization/dist0" value="0.5" type="double"/>
            <param name="optimization/swarm_clearance" value="0.5" type="double"/>
            <param name="optimization/max_vel" value="$(arg max_vel)" type="double"/>
            <param name="optimization/max_acc" value="$(arg max_acc)" type="double"/>

            <param name="bspline/limit_vel" value="$(arg max_vel)" type="double"/>
            <param name="bspline/limit_acc" value="$(arg max_acc)" type="double"/>
            <param name="bspline/limit_ratio" value="1.1" type="double"/>

        <!-- objects prediction -->
            <param name="prediction/obj_num" value="$(arg obj_num_set)" type="int"/>
            <param name="prediction/lambda" value="1.0" type="double"/>
            <param name="prediction/predict_rate" value="1.0" type="double"/>
        </node>
    </launch>
    ```
  * `touch single_run_in_exp.launch`
  * `gedit ~/catkin_ws/src/egoswarm/src/planner/plan_manage/launch/single_run_in_exp.launch`
  * fill with
    ```
    <launch>
        <!-- number of moving objects -->
        <arg name="obj_num" value="10" />
        <arg name="drone_id" value="0"/>

        <arg name="map_size_x" value="100"/>
        <arg name="map_size_y" value="50"/>
        <arg name="map_size_z" value="3.0"/>
        <arg name="odom_topic" value="/vins_estimator/imu_propagate"/>
        
        <!-- main algorithm params -->
        <include file="$(find ego_planner)/launch/advanced_param_exp.xml">
            <arg name="drone_id" value="$(arg drone_id)"/>
            <arg name="map_size_x_" value="$(arg map_size_x)"/>
            <arg name="map_size_y_" value="$(arg map_size_y)"/>
            <arg name="map_size_z_" value="$(arg map_size_z)"/>
            <arg name="odometry_topic" value="$(arg odom_topic)"/>
            <arg name="obj_num_set" value="$(arg obj_num)" />
            <!-- camera pose: transform of camera frame in the world frame -->
            <!-- depth topic: depth image, 640x480 by default -->
            <!-- don't set cloud_topic if you already set these ones! -->
            <arg name="camera_pose_topic" value="nouse1"/>
            <arg name="depth_topic" value="/camera/depth/image_rect_raw"/>
            <arg name="extrinsic_topic" value="/vins_estimator/extrinsic"/>
            <!-- topic of point cloud measurement, such as from LIDAR  -->
            <!-- don't set camera pose and depth, if you already set this one! -->
            <arg name="cloud_topic" value="nouse2"/>
            <!-- intrinsic params of the depth camera -->
            <arg name="cx" value="323.3316345214844"/>
            <arg name="cy" value="234.95498657226562"/>
            <arg name="fx" value="384.39654541015625"/>
            <arg name="fy" value="384.39654541015625"/>
            <!-- maximum velocity and acceleration the drone will reach -->
            <arg name="max_vel" value="0.5" />
            <arg name="max_acc" value="6.0" />
            <!--always set to 1.5 times grater than sensing horizen-->
            <arg name="planning_horizon" value="6" />
            <arg name="use_distinctive_trajs" value="false" />
            <!-- 1: use 2D Nav Goal to select goal  -->
            <!-- 2: use global waypoints below  -->
            <arg name="flight_type" value="1" />
            <!-- global waypoints -->
            <!-- It generates a piecewise min-snap traj passing all waypoints -->
            <arg name="point_num" value="1" />
            <arg name="point0_x" value="15" />
            <arg name="point0_y" value="0" />
            <arg name="point0_z" value="1" />
            <arg name="point1_x" value="0.0" />
            <arg name="point1_y" value="0.0" />
            <arg name="point1_z" value="1.0" />
            <arg name="point2_x" value="15.0" />
            <arg name="point2_y" value="0.0" />
            <arg name="point2_z" value="1.0" />
            <arg name="point3_x" value="0.0" />
            <arg name="point3_y" value="0.0" />
            <arg name="point3_z" value="1.0" />
            <arg name="point4_x" value="15.0" />
            <arg name="point4_y" value="0.0" />
            <arg name="point4_z" value="1.0" />
        </include>
        <!-- trajectory server -->
        <node pkg="ego_planner" name="drone_$(arg drone_id)_traj_server" type="traj_server" output="screen">
            <!-- <remap from="position_cmd" to="/setpoints_cmd"/> -->
            <remap from="~planning/bspline" to="drone_$(arg drone_id)_planning/bspline"/>
            <param name="traj_server/time_forward" value="1.0" type="double"/>
            <param name="traj_server/init_yaw" value="1.57" type="double"/>
        </node>
    </launch>
    ```
  * `cd ~/catkin_ws/src/ego-planner-swarm/ && catkin_make`
  * `source ~/catkin_ws/src/ego-planner-swarm/devel/setup.bash`
  * `roslaunch ego_planner simple_run.launch`


If you find this work useful or interesting, please kindly give us a star :star:, thanks!

# Update: ROS2 Support
For the ROS2 version, please refer to the branch [ros2_version](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version).

# Quick Start within 3 Minutes 
Compiling tests passed on ubuntu **16.04, 18.04, and 20.04** with ros installed.
You can just execute the following commands one by one.
```
sudo apt-get install libarmadillo-dev
git clone https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git
cd ego-planner-swarm
catkin_make -j1
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```
<!If your network to github is slow, We recommend you to try the gitee repository [https://gitee.com/iszhouxin/ego-planner-swarm](https://gitee.com/iszhouxin/ego-planner-swarm). They synchronize automatically./>

If you find this work useful or interesting, please kindly give us a star :star:, thanks!:grinning:

# Acknowledgements

- This work extends [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) to swarm navigation.

# EGO-Swarm
EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments

**EGO-Swarm** is a decentralized and asynchronous systematic solution for multi-robot autonomous navigation in unknown obstacle-rich scenes using merely onboard resources.

<p align = "center">
<img src="pictures/title.gif" width = "413" height = "232" border="5" />
<img src="pictures/outdoor.gif" width = "413" height = "232" border="5" />
<img src="pictures/indoor1.gif" width = "413" height = "232" border="5" />
<img src="pictures/indoor2.gif" width = "413" height = "232" border="5" />
</p>

**Video Links:** [YouTube](https://www.youtube.com/watch?v=K5WKg8meb94&ab_channel=FeiGao), [bilibili](https://www.bilibili.com/video/BV1Nt4y1e7KD) (for Mainland China)

## 1. Related Paper
EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments, Xin Zhou, Jiangchao Zhu, Hongyu Zhou, Chao Xu, and Fei Gao (Published in ICRA2021). [Paper link](https://ieeexplore.ieee.org/abstract/document/9561902) and [Science](https://www.sciencemag.org/news/2020/12/watch-swarm-drones-fly-through-heavy-forest-while-staying-formation) report.

## 2. Standard Compilation

**Requirements**: ubuntu 16.04, 18.04 or 20.04 with ros-desktop-full installation.

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), which is required by **uav_simulator**.
```
sudo apt-get install libarmadillo-dev
``` 

**Step 2**. Clone the code from github or gitee. These two repositories synchronize automatically.

From github,
```
git clone https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git
```

<!--Or from gitee,
```
git clone https://gitee.com/iszhouxin/ego-planner-swarm.git
```
/-->

**Step 3**. Compile,
```
cd ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release -j1
```

**Step 4**. Run.

In a terminal at the _ego-planner-swarm/_ folder, open the rviz for visualization and interactions
```
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

In another terminal at the _ego-planner-swarm/_, run the planner in simulation by
```
source devel/setup.bash
roslaunch ego_planner swarm.launch
```

Then you can follow the gif below to control the drone.

<p align = "center">
<img src="pictures/sim_demo.gif" width = "640" height = "360" border="5" />
</p>

## 3. Using an IDE
We recommend using [vscode](https://code.visualstudio.com/), the project file has been included in the code you have cloned, which is the _.vscode_ folder.
This folder is **hidden** by default.
Follow the steps below to configure the IDE for auto code completion & jump.
It will take 3 minutes.

**Step 1**. Install C++ and CMake extentions in vscode.

**Step 2**. Re-compile the code using the command
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```
It will export a compile commands file, which can help vscode to determine the code architecture.

**Step 3**. Launch vscode and select the _ego-planner_ folder to open.
```
code ~/<......>/ego-planner-swarm/
```

Press **Ctrl+Shift+B** in vscode to compile the code. This command is defined in _.vscode/tasks.json_.
You can add customized arguments after **"args"**. The default is **"-DCMAKE_BUILD_TYPE=Release"**.

**Step 4**. Close and re-launch vscode, you will see the vscode has already understood the code architecture and can perform auto completion & jump.

 ## 4. Use GPU or Not
 Packages in this repo, **local_sensing** have GPU, CPU two different versions. By default, they are in CPU version for better compatibility. By changing
 
 ```
 set(ENABLE_CUDA false)
 ```
 
 in the _CMakeList.txt_ in **local_sensing** packages, to
 
 ```
 set(ENABLE_CUDA true)
 ```
 
CUDA will be turned-on to generate depth images as a real depth camera does. 

Please remember to also change the 'arch' and 'code' flags in the line of 
```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 
``` 
in _CMakeList.txt_. If you encounter compiling error due to different Nvidia graphics card you use or you can not see proper depth images as expected, you can check the right code via [link1](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/) or [link2](https://github.com/tpruvot/ccminer/wiki/Compatibility).
 
Don't forget to re-compile the code!

**local_sensing** is the simulated sensors. If ```ENABLE_CUDA``` **true**, it mimics the depth measured by stereo cameras and renders a depth image by GPU. If ```ENABLE_CUDA``` **false**, it will publish pointclouds with no ray-casting. Our local mapping module automatically selects whether depth images or pointclouds as its input.

For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 5. Use Drone Simulation Considering Dynamics or Not
Typical simulations use a dynamic model to calculate the motion of the drone under given commands.
However, it requires continuous iterations to solve a differential equation, which consumes quite a lot computation.
When launching a swarm of drones, this computation burden may cause significant lag.
On an i7 9700KF CPU I use, 15 drones are the upper limit.
Therefore, for compatibility and scalability purposes, I use a "[fake_drone](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/master/src/uav_simulator/fake_drone)" package to convert commands to drone odometry directly by default.

If you want to use a more realistic quadrotor model, you can un-comment the node `quadrotor_simulator_so3` and `so3_control/SO3ControlNodelet` in [simulator.xml](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/blob/master/src/planner/plan_manage/launch/simulator.xml) to enable quadrotor simulation considering dynamics.
Please don't forget to comment the package `poscmd_2_odom` right after the above two nodes.

## 6. Utilize the Full Performance of CPU
The computation time of our planner is too short for the OS to increase CPU frequency, which makes the computation time tend to be longer and unstable.

Therefore, we recommend you to manually set the CPU frequency to the maximum.
Firstly, install a tool by
```
sudo apt install cpufrequtils
```
Then you can set the CPU frequency to the maximum allowed by
```
sudo cpufreq-set -g performance
```
More information can be found in [http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils](http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils).

Note that CPU frequency may still decrease due to high temperature in high load.

<!--
# Improved ROS-RealSense Driver

We modified the ros-realsense driver to enable the laser emitter strobe every other frame, allowing the device to output high quality depth images with the help of emitter, and along with binocular images free from laser interference.

<p align = "center">
<img src="pictures/realsense.PNG" width = "640" height = "158" border="5" />
</p>

This ros-driver is modified from [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) and is compatible with librealsense2 2.30.0.
Tests are performed on Intel RealSense D435 and D435i.

Parameter ```emitter_on_off``` is to turn on/off the added function.
Note that if this function is turned on, the output frame rate from the device will be reduced to half of the frame rate you set, since the device uses half of the stream for depth estimation and the other half as binocular grayscale outputs.
What's more, parameters ```depth_fps``` and ```infra_fps``` must be identical, and ```enable_emitter``` must be true as well under this setting.

##  Install

The driver of librealsense2 2.30.0 should be installed explicitly.
On a x86 CPU, this can be performed easily within 5 minutes.
Firstly, remove the currently installed driver by 
```
sudo apt remove librealsense2-utils
```
or manually remove the files if you have installed the librealsense from source.
Then, you can install the library of version 2.30.0 by
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
For ubuntu 16.04
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
```
For ubuntu 18.04
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
```
Then continue with
```
sudo apt-get install librealsense2-dkms
sudo apt install librealsense2=2.30.0-0~realsense0.1693
sudo apt install librealsense2-gl=2.30.0-0~realsense0.1693
sudo apt install librealsense2-utils=2.30.0-0~realsense0.1693
sudo apt install librealsense2-dev=2.30.0-0~realsense0.1693
sudo apt remove librealsense2-udev-rules
sudo apt install librealsense2-udev-rules=2.30.0-0~realsense0.1693
``` 
Here you can verify the installation by 
```
realsense_viewer
```

##  Run

If everything looks well, you can now compile the ros-realsense package named _modified_realsense2_camera.zip_ by ```catkin_make```, then run ros realsense node by 
```
roslaunch realsense_camera rs_camera.launch
```
Then you will receive depth stream along with binocular stream together at 30Hz by default.
-->

# Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

# Maintenance
We are still working on extending the proposed system and improving code reliability. 

For any technical issues, please contact Xin Zhou (iszhouxin@zju.edu.cn) or Fei GAO (fgaoaa@zju.edu.cn).

For commercial inquiries, please contact Fei GAO (fgaoaa@zju.edu.cn).
