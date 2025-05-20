import rclpy
import math
import numpy as np
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robotstatepub.msg import RobotState, Imu, JointStateCmd

class RlController(Node):
    def __init__(self):
        super().__init__('rl_controller')
        self.jointcmd_pub_ = self.create_publisher(JointStateCmd, 'joint_state_cmd', 10)
        
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.Control)
        self.mutex = threading.Lock()
        
        self.current_state = None
        self.current_imu = None

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.robotstate_sub_ = self.create_subscription(
            RobotState,
            "robot_state_actual",
            self.getRobotState,
            10)
            
        self.imu_sub_ = self.create_subscription(
            Imu,
            "imu_data",
            self.getImu,
            10)

        self.joint_num = 25  # 根据实际机器人配置修改
        self.default_positions = [
            -0.41, -0.04, -0.23, 0.81, -0.47, 0.0,
            -0.41, 0.04, 0.23, 0.81, -0.47, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -0.3, 0.0, 0.0, 0.0, 0.0, -0.3, 0.0
        ]
        self.target_positions = [0.0] * self.joint_num
        self.amplitude = 0.05
        self.frequency = 1.0 
        self.control_enabled = False
        self.sig_enabled = False

        self.last_x = 0
        self.last_y = 0
        self.last_b = 0

    def joy_callback(self, msg):
        X_BUTTON = 2
        B_BUTTON = 1
        Y_BUTTON = 3
        

        if msg.buttons[X_BUTTON] == 1 and self.last_x == 0:
            self.control_enabled = True
            print("X Pressed")
        self.last_x = msg.buttons[X_BUTTON]
    
        if msg.buttons[Y_BUTTON] == 1 and self.last_y == 0:
            print("Y Pressed")
        self.last_y = msg.buttons[Y_BUTTON]

        if msg.buttons[B_BUTTON] == 1 and self.last_b == 0:
            self.control_enabled = False
            print("B Pressed")
        self.last_b = msg.buttons[B_BUTTON]

    def Control(self):

        with self.mutex:
            robot_state = self.current_state
            imu = self.current_imu
            
        if robot_state is None or imu is None:
            self.get_logger().warn("Waiting for initial sensor data...")
            return
            


        ########您需要在此处添加您的强化学习推理代码##########
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # self.target_positions += signal_value
        signal_value = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)

        if self.sig_enabled:
            for i in range(len(self.default_positions)):
                self.target_positions[i] = self.default_positions[i] + signal_value

        ################################################

        msg = JointStateCmd()
        
        try:
            #在获取apos后应裁切掉前kBaseNum(6)
            current_pos = np.array(robot_state.apos)[6:]

            if(current_pos.tolist() == self.target_positions):
                print("pass")

            if self.control_enabled and self.sig_enabled:
                msg.dpos = self.target_positions
                msg.dvel = [0.0] * self.joint_num
                msg.dtau = [0.0] * self.joint_num
                self.jointcmd_pub_.publish(msg)

            else:
                if abs(signal_value) < 1e-3:
                    self.sig_enabled = True
                else:
                    self.sig_enabled = False
                        
        except Exception as e:
            self.get_logger().error(f"Control error: {str(e)}")

    def getRobotState(self, msg):
        with self.mutex:
            self.current_state = msg
            
    def getImu(self, msg):
        with self.mutex:
            self.current_imu = msg

def main(args=None):
    rclpy.init(args=args)
    rl_controller = RlController()
    rclpy.spin(rl_controller)
    rl_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
