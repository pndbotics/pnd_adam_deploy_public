# 🛠️PND Adam Deploy SDK

There are two ways to use PND Adam Deploy SDK: Build from source or use the docker.

## ⚓Use the docker

Download link:

[pnd_adam_deploy_public_docker](https://pndwiki.oss-cn-beijing.aliyuncs.com/sdk_adlskjfas412838_sakjfhrjdsaljf_skfj3jskdjfd32-38439/Dockers-master.zip)

### Install docker

```
sudo apt-get update
sudo apt-get install docker docker.io docker-buildx
```

### 🧑‍🏫Getting started

#### If your computer has Nvidia_GPU

- install NVIDIA Container Toolkit first:

```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
      sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

- Update the package list and install the NVIDIA Container Toolkit:

```
sudo apt update
sudo apt install -y nvidia-container-toolkit
```

- Configure Docker to use the NVIDIA runtime and restart docker server:

```
sudo nvidia-ctk runtime configure --runtime=docker

sudo systemctl restart docker
```

- run the scripts for nvidia

```
sudo sh runDockerNvidia.sh
```

#### If your computer has no GPU

- run the script for Docker

```
docker build -t pndbotics:v121 .
sh runDocker.sh
```

### After you are in docker env

```
cd pnd_adam_deploy_public-1.2.1
sh install.sh
```

### For Mujoco

```
sh build.sh adam_lite mujoco
sh run_mujoco.sh adam_lite
```

### Exit docker

```
#ctrl+d to exit the docker env 
```

### Run the docker container

```
docker ps -a #find the docker id
docker start ${docker id}
sudo docker exec -it ${docker id} bash
```

## 🏗️Build from source

[pnd_adam_deploy_public](https://pndwiki.oss-cn-beijing.aliyuncs.com/sdk_adlskjfas412838_sakjfhrjdsaljf_skfj3jskdjfd32-38439/pnd_adam_deploy_public-1.4.0.tar.gz) It is the SDK developed for PND humanoid robot, and it is contain the sim2real/sim2sim experiment program of the robot.

This article describes how to use pnd_adam_deploy_public for rapid development and application of PND humanoid robots.

### Environment

- Ubuntu 20.04 is recommended for development. Mac and Windows are not currently supported for development.
- If you want to run it on real robot please Connect the user computer to the same network as the robot NUC first.

### Install Dependencies by Script

- we offer script for install the Dependencies in code
- "The `install.sh` script in code will build RBDL, download LibTorch, and provide an option to compile MuJoCo."
- Before running install.sh, execute the following commands first
- `sudo apt update`
- `sudo apt upgrade`
- `sudo apt install wget libeigen3-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libssl-dev libx11-dev`
- Get code then run the install.sh
- x.x.x indicates the version number

```sh
##Enter path
cd ~/Docments

##get code
wget https://pndwiki.oss-cn-beijing.aliyuncs.com/sdk_adlskjfas412838_sakjfhrjdsaljf_skfj3jskdjfd32-38439/pnd_adam_deploy_public-1.4.0.tar.gz
tar -xzf pnd_adam_deploy_public-1.2.2.tar.gz
cd pnd_adam_deploy_public

## run scripts
sh install.sh
```

### Manual install Dependencies

- [Eigen](https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
- [rbdl](https://github.com/rbdl/rbdl)
- [libtorch](https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcpu.zip)
  - (You need to download and unzip libtorch and place it in the pnd_adam_deploy_public floder).

### 🐢If you want to build with ROS2

```
#build the ros2 publisher
  cd robotPublisher
  colcon build
  source install/setup.bash
  cd ../example/python
  python Controller.py
```

#### Build

To build the code:

- which will create folder build and folder bin in your project and install the required files into folder bin.

```
cd robotPublisher
source install/setup.bash
cd ..
sh build.sh adam_lite|adam_inspire|adam_standard real|mujoco ros2
```

#### Before Run

```
#open one more terminal
ros2 run joy joy_node 

```

#### Run ros2 code

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

### 🤖Build & Run Mujoco

- urdf2mujoco
  - Before using Mujoco for simulation verification, all axes in the floating_joint in your urdf need to be commented out and then converted to xml. The provided script file is located at `mujoco/urdf2mujocoXml.py`

#### build by script

```sh
sh build.sh adam_lite mujoco
```

#### Run simulation in Mujoco

- Run the test
  - The robot operation process can be found in the robot operation instruction document.
- Enter the build path then run :

```sh
cd build_adam_lite_mujoco
./pnd_adam_deploy_public
```

After entering the Mujoco simulation environment, press `A` first and then `X` on xbox. Then click `Reset` several times or press the Backspace key on the `keyboard`, then press `Y` on xbox to let adam walk."
![mujocoSim](../img/mujoco_sim/adam_lite.gif)

### Build & Run Real Robot

```sh
sh build.sh adam_lite|adam_inspire|adam_standard real
# before v1.1.0 sh (build_adam_lite.sh | build_adam_standard.sh | build_adam_inspire.sh) # before v0.4.0 sh buildrobot.sh
sh run.sh
```

When the terminal displays 'FSM start!' After the information can start to control.
Press button A to make all joint of the robot return to the semi-squat zero position; Press button X to make the robot run MLP or DEMO; Press button B to make all joints stop at the current position; Press LT and RT keys at the same time disables the joint and exits the control program; Press LB and RB at the sime time to power on and off the joint.


### 🔣Kp,Kd Paramerter Explanation

#### In Isaac Gym

The PD controller:

 $\tau=K_p*e_p+K_d*e_v$ (1)

$K_p$ : stiffness of PD controller

$K_d$ : damping of PD controller

$e_p$ : position error

$e_v$ : velocity error

$\tau$ : the torque worked on end

#### In Real Robot Actuator

The PID controller:
$I=P_p*P_v*e_p+P_v*e_v$ (2)

$\tau'=g*I*K_t=g*(P_p*P_v*K_t*e_p+P_v*K_t*e_v)$ (3)

$P_p$ : proportional parameter of position control loop PID controller

$P_v$ : proportional parameter of velocity control loop PID controller

$e_p$ : position error

$e_v$ : velocity error

$g$ : the gear ratio between end and motor

$K_t$ : torque constant,this can be get from experiment

$I$ : the current output of position and velocity control loop PID controller

$\tau'$ : the torque worked on end

For sim to real purpose, we need $\tau=\tau'$, then we can get:

$K_p=g*P_p*P_v*K_t$ (4)

$K_d=g*P_v*K_t$ (5)

$P_v=K_d/(K_t*g)$ (6)

$P_p=K_p/K_d$ (7)


