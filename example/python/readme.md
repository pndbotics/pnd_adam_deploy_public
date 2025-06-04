# Using LCM (Lightweight Communications and Marshalling) on Adam Lite
This is the demo on using LCM controller.  This demo is tested on Ubuntu 20.04 & 22.04. Before use, please make sure you meet the requirement.  

## ❓How to use the example
In the example, you can use the lcm_controller.py to send command to the robot. If succeeded, the robot will perform a "spacewalk".  

### 🥞Dependences
Before running, open a terminal and run the following commands:
- `pip install lcm`
- `pip install numpy==1.24`
- `pip install scipy`

### 💻Run in Mujoco
If you want to use mujoco, do the follwing:
1. `sh build.sh adam_lite mujoco lcm`
2. `cd build_adam_lite_mujoco && ./pnd_adam_deploy_public`
3. When terminal shows `FSM start!`, use the gamepad to control fsm to enter MLP mode. Press button `A` and then press `X`.  The terminal will show `Zero2MLP`.
4. Open a new terminal. Enter `pnd_adam_deploy_public` root directory, run `python example/python/lcm_controller.py`
5. press `Right` button (the key on the cross gamepad).

### 🤖Run in real robot
Before running on the real robot, make sure the robot is `HANGING` 🪢properly.  The foot should be off the ground. Double check before use!!!  ⚠️⚠️⚠️

If the preparation is finished, proceed to the following:

1. `sh build.sh adam_lite real lcm`
2. `sh run.sh`
3. When terminal shows 'FSM start!'. Use the gamepad to control fsm to enter MLP mode. Press button `A` and then press `X`.  The terminal will show `Zero2MLP`.
4. Open a new terminal. Enter `pnd_adam_deploy_public` root directory, run `python example/python/lcm_controller.py`
5. press 'Right' button (the key on the cross gamepad).
Now the robot will be performing "spacewalk"🚶‍♂️‍➡️.
