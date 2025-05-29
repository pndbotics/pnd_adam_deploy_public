
import os
import sys
sys.path.append(f"{os.getcwd()}/src/msg")
from threading import Thread
import numpy as np
import time
from scipy import interpolate
from joystick_input import JoystickInput
import lcm
from pnd_lcm_types import pnd_robot_state_t as RobotState
from pnd_lcm_types import pnd_imu_t as Imu
from pnd_lcm_types import pnd_joint_cmd_t as JointCmd


DOF_NUM = 25
DT = 0.0025  # 400Hz control frequency


robot_state = None
imu_data = None
send_cmd_flag = False
joy = JoystickInput()
lc = lcm.LCM()
np_zero = np.zeros(25)
joint_pos_motion = np.array([
    # [-0.41, -0.04, -0.23, 0.81, -0.47, 0.0,
    # -0.41, 0.04, 0.23, 0.81, -0.47, 0.0,
    #     0.0, 0.0, 0.0,
    #     0.0, 0.0, 0.0, -0.3, 0.0, 0.0, -0.0, 0.0, -0.3, 0.0],
    [-0.66, -0.11, -0.35, 0.68, -0.11, 0.0,  
        -0.02, -0.003, 0.015, 0.68, -0.66, 0.0,                     
        0.0, 0.0, 0.0,                                              
        0.7, 0.3, 0.0, -0.7, 0.0, -0.7, -0.3, 0.0, -0.7, 0.0],
    [-0.02, 0.003, -0.015, 0.68, -0.66, -0.0,  
        -0.66, 0.11, 0.35, 0.68, -0.11, -0.0,                         
        0.0, 0.0, 0.0,                                                
        -0.7, 0.3, 0.0, -0.7, 0.0, 0.7, -0.3, 0.0, -0.7, 0.0],
    [-0.41, -0.04, -0.23, 0.81, -0.47, 0.0,  
        -0.41, 0.04, 0.23, 0.81, -0.47, 0.0,                        
        0.0, 0.0, 0.0,                                              
        0.0, 0.0, 0.0, -0.3, 0.0, 0.0, -0.0, 0.0, -0.3, 0.0],
    ])


def pnd_robot_state_handler(channel, data):
    global robot_state
    robot_state = RobotState.decode(data)
    # print(f"Received Robot State")
    # print(f"Number of Joints: {robot_state.num_joints}")
    # print(f"Joint Positions: {robot_state.apos}")
    # print(f"Joint Velocities: {robot_state.avel}")
    # print(f"Joint Torques: {robot_state.atau}")


def pnd_imu_handler(channel, data):
    global imu_data
    imu_data = Imu.decode(data)
    # print(f"Received IMU Data")
    # print(f"Orientation: {imu_data.yaw}, {imu_data.pitch}, {imu_data.roll}")
    # print(f"Angular Velocity: {imu_data.angular_velocity}")
    # print(f"Linear Acceleration: {imu_data.linear_acceleration}")


def send_joint_command():
    print("Sending joint commands...")
    global send_cmd_flag
    time_begin = 0.0
    time_interval = 2.0
    time_points = np.linspace(time_begin, time_interval, int(time_interval / DT) + 1)
    cur_pos = np.array(robot_state.apos)
    
    interpolators = []
    for i in range(joint_pos_motion.shape[0]):
        interpolators.clear()
        for j in range(DOF_NUM):
            interpolator = interpolate.CubicSpline(
                [0, time_interval],
                [cur_pos[j], joint_pos_motion[i][j]],
                bc_type=((1, 0), (1, 0))
            )
            interpolators.append(interpolator)
        for t in time_points:
            target_pos = np.array([interpolator(t) for interpolator in interpolators])
            joint_positions = target_pos.tolist()
            cmd = JointCmd()
            cmd.num_joints = DOF_NUM
            cmd.dpos = joint_positions
            cmd.dvel = np_zero.tolist()
            cmd.dtau = np_zero.tolist()
            # print(f"Sending joint command: {cmd.dpos}")
            lc.publish("pnd_joint_cmd", cmd.encode())
            time.sleep(DT)  # Sleep to maintain control frequency
            
        cur_pos = joint_pos_motion[i]

    send_cmd_flag = False
    joy.cmd = False
    print("Joint commands sent.")


def main():
    print("LCM Controller is running...")
    lc.subscribe("pnd_robot_state", pnd_robot_state_handler)
    lc.subscribe("pnd_imu", pnd_imu_handler)
    global send_cmd_flag
    while True:
        lc.handle()
        if joy.get_cmd() and not send_cmd_flag:
            Thread(target=send_joint_command, daemon=True).start()
            send_cmd_flag = True


if __name__ == "__main__":
    main()  
