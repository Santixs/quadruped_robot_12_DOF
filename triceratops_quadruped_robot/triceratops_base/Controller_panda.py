# This Controller includes Hip and Back movements. Controller.py is the original controller without Hip and Back support
#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from Triceratops_Config import RobotConfiguration, RobotDynamixel
from DXL_motor_control import DXL_Communication
from Gait_controller import GaitController
from Swing_controller import SwingController
from Stance_controller import StanceController
from Triceratops_IK import InverseKinematics
from Body_controller_panda import BodyController

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from math import pi
import math
import threading
import queue
import traceback
import atexit
import time
import concurrent.futures
import numpy as np

DEGREE_TO_SERVO = 4095/360
#ros2 run teleop_twist_keyboard teleop_twist_keyboard 
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.2


# --- ROS2 Subscriber ---
class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0

        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.get_logger().info('Vel Subscriber has been started')

    def listener_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.linear_z = msg.linear.z

        self.angular_x = msg.angular.x
        self.angular_y = msg.angular.y
        self.angular_z = msg.angular.z

# --- Robot State and Command Objects ---
class RobotState:
    def __init__(self):
        self.config = RobotConfiguration()
        self.ticks = 0                    
        self.height = -0.10
        self.foot_locations = self.config.default_stance.copy()
        self.joint_angles = np.zeros((3, 4))
        # For leg motors, we store a 3x4 array (for the three joint groups).
        self.last_goal = np.zeros((3, 4)) 

class Command:
    def __init__(self):
        # For the leg controllers, we use horizontal velocities and yaw rate.
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0
        self.height = -0.10

# --- Main Robot Control Class ---
class RobotControl:
    def __init__(self):
        # Set up velocity subscriber.
        self.cmd_vel = CmdVelSubscriber()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.cmd_vel)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()


        self.config = RobotConfiguration()
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)
        self.inverse_kinematics = InverseKinematics(self.config)

        # The BodyController uses the step_duration from configuration!
        self.body_controller = BodyController(step_duration=self.config.step_duration)

        self.state = RobotState()
        self.command = Command()
        self.control_cmd = ControlCmd()

    def get_vel_data(self):
        # Update the command object using the individual velocity attributes.
        self.command.horizontal_velocity = np.array([self.cmd_vel.linear_x, self.cmd_vel.linear_y])
        self.command.yaw_rate = self.cmd_vel.angular_z

    def cleanup(self):
        self.executor.shutdown()

    def step_gait(self, state, command):
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            # foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:  
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else: 
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command
                )
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations, contact_modes

        """

    def step_gait(self, state, command):
        # Obtain contact modes from the gait controller.
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            if contact_modes[leg_index] == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks)
                new_location = self.swing_controller.next_foot_location(swing_proportion, leg_index, state, command)
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations, contact_modes
"""
    def puppy_move(self):
        #trunk conversion parameters
        #Note:  Conversion factor: [m] -> servo counts.5000.0 means that for every meter of trunk displacement, you would offset the servo position by 5000 counts.
       #if the periodic function returns an offset of 0.02 m, then multiplying 0.02 m by 5000 gives 100 counts
       # We could also use directly servo units. based on some test this works: yaw_amplitude = 100 (servo units) vertical_amplitude = 50 (servo units)
        TRUNK_VERTICAL_SCALE = 4000.0 
        TRUNK_HORIZONTAL_SCALE = 4000.0
        TRUNK_CENTER = 2048           # Center position for trunk motors. (Standard pose of motors -- Check)

        while rclpy.ok():
            self.get_vel_data()

        

            # Asynchronously compute new foot placements and get contact modes.
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future = executor.submit(self.step_gait, self.state, self.command)
                new_foot_locations, contact_modes = future.result()

            # Obtain trunk (hip/back) offsets from the BodyController.
            trunk_offsets = self.body_controller.update(self.config.dt, self.cmd_vel, contact_modes)
            
            # Apply trunk offsets to compute the effective foot positions.
            # We assume: index 1 is lateral (y) and index 2 is vertical (z).
            effective_foot_locations = new_foot_locations.copy()
            effective_foot_locations[1, :] -= trunk_offsets['hip']
            effective_foot_locations[2, :] -= trunk_offsets['back']

            # Compute joint angles for the leg motors via inverse kinematics.
            self.state.joint_angles = self.inverse_kinematics.four_legs_inverse_kinematics(effective_foot_locations)

            # Convert leg joint angles (radians) to servo positions.
            leg_goal = (self.state.joint_angles * 180 / math.pi * DEGREE_TO_SERVO +
                        self.config.leg_center_position)

            # Compute trunk motor commands from trunk offsets:
            trunk_goal = {}
            # 'hip_vertical' motor uses the vertical (back) offset.
            trunk_goal['hip_vertical'] = int(TRUNK_CENTER + trunk_offsets['back'] * TRUNK_VERTICAL_SCALE)
            # 'hip_horizontal' motor uses the lateral (hip) offset.
            trunk_goal['hip_horizontal'] = int(TRUNK_CENTER + trunk_offsets['hip'] * TRUNK_HORIZONTAL_SCALE)

            # Update motor positions if the leg goal has changed significantly.
            if np.linalg.norm(leg_goal - self.state.last_goal) > self.config.goal_change_threshold:
                self.control_cmd.motor_position_control(leg_positions=leg_goal, trunk_positions=trunk_goal)
                self.state.last_goal = leg_goal.copy()

            self.state.foot_locations = new_foot_locations.copy()
            self.state.ticks += 1    
            time.sleep(self.config.dt)

# --- Low-Level Motor Control Class ---
class ControlCmd:
    """Low-level control of Dynamixel motors (leg + trunk)."""
    def __init__(self):
        self.setup_dynamixel()
        self.setup_motors()
        self.initialize_motor_states()
        self.walking_freq = 2000  # 2000Hz

    def setup_dynamixel(self):
        self.robot_dynamixel = RobotDynamixel()
        self.dynamixel = DXL_Communication(self.robot_dynamixel.DEVICE_NAME, 
                                           self.robot_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()
        
    def setup_motors(self):
        # Updated motor list including trunk motors.
        motor_names = ['FR_higher', 'FR_lower', 'FR_hip',  
                       'FL_higher', 'FL_lower', 'FL_hip',
                       'RR_higher', 'RR_lower', 'RR_hip',
                       'RL_higher', 'RL_lower', 'RL_hip',
                       'hip_vertical', 'hip_horizontal']
        motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        self.motors = {name: self.dynamixel.createMotor(name, motor_number=id_) 
                       for name, id_ in zip(motor_names, motor_ids)}

        # Leg motors: organized as a 3x4 list.
        self.leg_motor_list = [
            [self.motors['FR_hip'], self.motors['FL_hip'], self.motors['RR_hip'], self.motors['RL_hip']],
            [self.motors['FR_higher'], self.motors['FL_higher'], self.motors['RR_higher'], self.motors['RL_higher']],
            [self.motors['FR_lower'], self.motors['FL_lower'], self.motors['RR_lower'], self.motors['RL_lower']]
        ]
        # Trunk motors stored separately.
        self.trunk_motors = {
            'hip_vertical': self.motors['hip_vertical'],
            'hip_horizontal': self.motors['hip_horizontal']
        }
    
    def initialize_motor_states(self):
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()
        self.enable_all_motor()
        self.joint_position = np.zeros(12)

    def cleanup(self):
        self.disable_all_motor()
        self.dynamixel.closeHandler()

    def enable_all_motor(self):
        for motor in self.motors.values():
            motor.enableMotor()

    def disable_all_motor(self):
        for motor in self.motors.values():
            motor.disableMotor()

    def read_all_motor_data(self):
        self.update_joint_state()
        print(self.joint_position)
    
    def update_joint_state(self):
        self.dynamixel.updateMotorData()
        # Update only leg motor joint positions.
        self.joint_position = np.zeros((3, 4))
        self.joint_position[0] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[0]]
        self.joint_position[1] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[1]]
        self.joint_position[2] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[2]]

    def reset_to_original(self, position=None):
        self.motor_position_control()

    def motor_position_control(self, leg_positions=None, trunk_positions=None):
        # leg_positions is expected to be a 3x4 array for leg motors.
        # trunk_positions is a dictionary with keys 'hip_vertical' and 'hip_horizontal'.
        if leg_positions is None:
            leg_positions = [[2048, 2048, 2048, 2048],
                             [1992, 2047, 2092, 2099],
                             [2048, 2048, 2048, 2048]]
        if trunk_positions is None:
            trunk_positions = {'hip_vertical': 1048, 'hip_horizontal': 1048}

        # Send commands to leg motors.
        for i, motor_list in enumerate(self.leg_motor_list):
            for j, motor in enumerate(motor_list):
                motor.writePosition(int(leg_positions[i][j]))
        # Send commands to trunk motors.
        for name, motor in self.trunk_motors.items():
            motor.writePosition(int(trunk_positions[name]))
        self.dynamixel.sentAllCmd()

# --- Main ---
def main():
    rclpy.init()
    robot_control = RobotControl()

    command_dict = {
        "s": robot_control.puppy_move,
    }

    atexit.register(robot_control.cleanup)

    while True:
        try:
            cmd = input("CMD : ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break
        except Exception as e:
            traceback.print_exc()
            break

if __name__ == "__main__":
    main()
