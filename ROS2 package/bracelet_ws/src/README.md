# [RRbot](https://moodle.unistra.fr/pluginfile.php/1530049/mod_resource/content/1/Example.pdf) lab for the [Robot Control](https://moodle.unistra.fr/course/view.php?id=21431#) course 

The purpose of this lab session is to implement and test the following [Motion Control](https://moodle.unistra.fr/pluginfile.php/1531393/mod_resource/content/1/Motion%20Control.pdf) strategies studied in the Robot Control course :
1. Proportional-Derivative (PD) Control with gravity compensation
2. Inverse Dynamics Control 
3. Operational Space Control

The controlled system is RR planar robotic manipulator as described in the course [Example](https://moodle.unistra.fr/pluginfile.php/1530049/mod_resource/content/1/Example.pdf). 


## Getting Started
***Required setup : Ubuntu 22.04 LTS and ROS2 Humble***

1.  If not yet done, install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. If not yet done, install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://git.unistra.fr/ros2_tutorials/rrbot.git
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --symlink-install
    source install/setup.bash
    ```




## 1. Proportional-Derivative (PD) Control with gravity compensation 
This joint torque controller implements the regulation problem of bringing the robot joint configuration to a desired constant position while compensating for gravity. 

For this controller, a `ros2_control` controller structure is already available and called `rrbot_pdg_controller`. Implement the control logic in the [rrbot_pdg_controller.cpp](rrbot_controllers/rrbot_pdg_controller/src/rrbot_pdg_controller.cpp) file in the following part:
```cpp
 // ---------------------- START TODO ------------------------------------

    g_j1_ = 0;
    g_j2_ = 0;

    tau_j1_ = 0;
    tau_j2_ = 0;

  // ---------------------- STOP TODO ------------------------------------
```
The robot configuration parameters are given in the [rrbot_pdg_controller.cpp](rrbot_controllers/rrbot_pdg_controller/include/rrbot_pdg_controller/rrbot_pdg_controller.hpp) file such that 
```cpp
  double m1_ = 1.0;
  double l1_ = 1.0;
  double lc1_ = l1_/2; 
  double i1_ = m1_/12*l1_*l1_;
  double m2_ = 1.0;
  double l2_ = 1.0;
  double lc2_ = l2_/2; 
  double i2_ = m2_/12*l2_*l2_;
  double g_ = 9.8;  
``` 

Once implemented, set up your controller using the [configuration](rrbot_bringup/config/rrbot_controllers.yaml) file and specify the `KP` and `KD` parameters. 

You can test you controller by sending a set of joint reference positions by running :
```shell
$ ros2 topic pub --once <topic_nalme> <type> <data>
```

You can test the influence of different `KP` and `KD` parameters by changing them online using the commands :
```shell
$ ros2 param set <param_name> <value>
```

How does the choice of these parameters impact the regulation ? 

## 2. Inverse Dynamics Control 
This joint torque controller implements the regulation problem of tracking a desired trajectory. 

For this controller, a `ros2_control` controller structure is already available and called `rrbot_idyn_controller`. Implement the control logic in the [rrbot_idyn_controller.cpp](rrbot_controllers/rrbot_idyn_controller/src/rrbot_idyn_controller.cpp) file in the following part:
```cpp
 // ---------------------- START TODO ------------------------------------
    
    u_ = ??;

    bmat_(0,0) = ??;
    bmat_(0,1) = ??;
    bmat_(1,0) = ??;
    bmat_(1,1) = ??;

    cvec_(0) = ??;
    cvec_(1) = ??;

    gvec_(0) = ??;
    gvec_(1) = ??;

    tau_ = ??;

  // ---------------------- STOP TODO ------------------------------------
```

**Note: This code uses the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) C++ library to facilitate matrix computation.**

The robot configuration parameters are given in the [rrbot_idyn_controller.hpp](rrbot_controllers/rrbot_idyn_controller/include/rrbot_idyn_controller/rrbot_idyn_controller.hpp) file.

Once implemented, set up your controller using the [configuration](rrbot_bringup/config/rrbot_controllers.yaml) file and specify the `KP` and `KD` parameters. 

Also, modify the joint trajectory publishing [node](rrbot_nodes/scripts/trajectory_publisher.py) to make your robot follow a circular trajectory. To do so, you will need to compute the robot kinematic model and implement it [here](rrbot_nodes/scripts/trajectory_publisher.py)  
```python
def rrbot_direct_kinematics(q1, q2):
    px = ??
    py = ??
    return px, py

def rrbot_inverse_kinematics(px, py):
    q1 = ??
    q2 = ??
    return q1, q2
```

You can test the influence of different `KP` and `KD` parameters by changing them online using the commands :
```shell
$ ros2 param set <param_name> <value>
```

How does the choice of these parameters impact the reference tracking ? 

## 3. Operational Space Control
Based on the 2 previous controllers, create 2 new controllers implementing the same control strategies as above, but this time taking as input a reference cartesian position/trajectory. You can find some hints at the end of the [Motion Control](https://moodle.unistra.fr/pluginfile.php/1531393/mod_resource/content/1/Motion%20Control.pdf) course file. 



## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](https://github.com/mcbed)
