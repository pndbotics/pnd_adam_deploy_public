# examples

## lcm

### dependence

- `pip install lcm`
- `pip install inputs`
- `pip install numpy`
- `pip install scipy`

### run

1. `sh build.sh adam_lite mujoco lcm`
2. `cd build_adam_lite_mujoco && ./pnd_adam_deploy_public`
3. When terminal output 'FSM start!'. joystick control fsm to mlp mode. button 'A' -> 'X', terminal output 'Zero2MLP'.
4. new terminal. entry pnd_adam_deploy_public root directory, run `python example/python/lcm_controller.py`
5. press 'Right' button.
