# PND Adam Humanoid Robot Deploy Private #
This repository provides the experimental code for PND humanoid robot.  
- The program includes simulation validation in MuJoCo, as well as real-world testing.

## Install Dependencies by Script
we offer script for install the Dependencies below:
- "The `install.sh` script below will build RBDL, download LibTorch, and provide an option to compile MuJoCo.".
- Before running install.sh, execute the following commands first:
  - `sudo apt update`
  - `sudo apt upgrade`
  - `sudo apt install wget libeigen3-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libssl-dev libx11-dev`
- run the install.sh
```
sh install.sh
```
## Manual install Dependencies 
- [Eigen](https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
- [rbdl](https://github.com/rbdl/rbdl)
- [libtorch](https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcpu.zip)
  - (You need to download and unzip libtorch and place it in the pnd_adam_deploy_public floder).

### Directory Structure ###
- `example`: The main program of the sdk.
- `python_scripts`: Contains .py scripts, including the transtojit.py to trace original .pt to libtorch .pt for c++.
- `robot_interface`: Contains joint config and required libraries.
- `src`: Contains joystick code and state machine code.
  - There are three preliminary states: ZERO, MLP and STOP.

## If you run with ROS2
```
#build the ROS2 publisher
  cd robotPublisher
  colcon build
  source install/setup.bash
  cd ../example/python
  python Controller.py
```

## Build
To build the code:  
- which will create folder build and folder bin in your project and install the required files into folder bin.
```
cd robotPublisher
source install/setup.bash
cd ..
sh build.sh adam_lite|adam_inspire|adam_standard real|mujoco ros2
```
## Before Run
```
#open one more terminal
ros2 run joy joy_node 

```
## Run ROS2 code
```
##in mujoco
cd build_adam_lite_mujoco
./pnd_adam_deploy_public

##in real
sh run.sh
```


If you need to clean your build results:
- which will delete all contents in folder build and bin. If you just want to normal rebuild, you do not need to do this, but simply run `sh build.sh adam_lite real` again. 
```
sh cleanrobot.sh
```

## If you run without ROS2

## Build
To build the code:  
- which will create folder build and folder bin in your project and install the required files into folder bin.
```
sh build.sh adam_lite|adam_inspire|adam_standard real|mujoco
```


If you need to clean your build results:
- which will delete all contents in folder build and bin. If you just want to normal rebuild, you do not need to do this, but simply run `sh build.sh adam_lite real` again. 
```
sh cleanrobot.sh
```

## Run
To run this sdk, you should make sure the robot and the joystick are power on and the robot arms are in zero positions, then run:
```
sh run.sh
```

After the terminal displays the absolute positions of the joints and the following instruction :
``` 
confirm the motor para, press 1 and enter to confirm:
```
check if all joints encoder are showing the right positions, and then press 1 and enter. The output should hopefully end with

```
FSM start!
```
Then press button A to go to state ZERO (all joints will go to zero positions). Button X to go to state MLP (for demo motion or MLP). Button B to go to state STOP (all joints will stop move in current positions).
Press button LT and RT simultaneously to disable all joints. Press button LB and RB simultaneously to power off all joints. 
