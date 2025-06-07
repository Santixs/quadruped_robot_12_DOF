#!/usr/bin/env python3

print("Joy_controller.py: Starting script...")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
# from champ_interfaces.srv import SetMode
import sys, os
import argparse

print("Joy_controller.py: Basic imports successful.")

# Parse command line arguments
parser = argparse.ArgumentParser(description='Triceratops Joy Controller')
args, unknown = parser.parse_known_args()

# Import Unity ROS2 messages
print("Joy_controller.py: Attempting to set up sys.path for Unity messages...")
possible_paths = [
    os.path.expanduser("~/UnityRos2_ws/src/unity_robotics_demo_msgs"),
    os.path.expanduser("~/UnityRos2_ws/install/unity_robotics_demo_msgs/lib/python3.10/site-packages"),
    "/home/panda2/UnityRos2_ws/src/unity_robotics_demo_msgs",
    "/home/panda2/UnityRos2_ws/install/unity_robotics_demo_msgs/lib/python3.10/site-packages"
]
unity_msgs_found = False
for path in possible_paths:
    print(f"Joy_controller.py: Checking path: {path}")
    if path not in sys.path and os.path.exists(path):
        sys.path.append(path)
        print(f"Joy_controller.py: Added {path} to Python path")
        unity_msgs_found = True

if not unity_msgs_found:
    print("Joy_controller.py: ERROR - Could not find unity_robotics_demo_msgs in any checked path.")

try:
    print("Joy_controller.py: Attempting to import Unity messages...")
    from unity_robotics_demo_msgs.msg import JointState
    from unity_robotics_demo_msgs.srv import UnityAnimateService
    print("Joy_controller.py: Successfully imported unity_robotics_demo_msgs")
except ImportError as e:
    print(f"Joy_controller.py: ERROR - unity_robotics_demo_msgs is required but not available: {e}")
    print(f"Joy_controller.py: Current sys.path: {sys.path}")
    sys.exit(1)

class TriceratopsControlClient(Node):
    def __init__(self):
        print("Joy_controller.py: Initializing TriceratopsControlClient node...")
        try:
            super().__init__('JoyControlClient')
            print("Joy_controller.py: super().__init__('JoyControlClient') successful.")
        except Exception as e:
            print(f"Joy_controller.py: ERROR initializing Node: {e}")
            raise # Re-raise exception to ensure script exits
        
        print("Joy_controller.py: Creating subscription to /joy...")
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        print("Joy_controller.py: Creating publisher to /cmd_vel...")
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        print("Joy_controller.py: Creating publisher to /body_pose...")
        self.body_pub = self.create_publisher(String, 'body_pose', 1)
        
        print("Joy_controller.py: Creating Unity service client...")
        try:
            self.cli = self.create_client(UnityAnimateService, 'UnityAnimate_srv')
            # Wait for service to be available (optional but good practice)
            # if not self.cli.wait_for_service(timeout_sec=5.0):
            #     self.get_logger().error('UnityAnimate_srv service not available, continuing without it.')
            #     self.cli = None # Indicate service is not available
            # else:
            #     print("Joy_controller.py: Unity service client connected.")
            self.req = UnityAnimateService.Request()
            print("Joy_controller.py: Unity service client setup successful.")
        except Exception as e:
            print(f"Joy_controller.py: WARNING - Failed to create Unity service client: {e}. Continuing without it.")
            self.cli = None # Indicate service is not available
            self.req = None
        
        # Current robot mode (only panda_mode or puppy_mode)
        print("Joy_controller.py: Initializing other variables...")
        self.current_mode = "puppy_mode"
        self.previous_mode = "puppy_mode"
        
        # Track button states to prevent continuous triggering
        self.last_button_states = {}
        
        # Head and mouth positions (in degrees)
        self.head_position = 200  # Initial value between 177 and 230
        self.mouth_position = 150  # Initial value between 116 and 190
        self.head_increment = 5    # Amount to change per button press
        self.mouth_increment = 5   # Amount to change per button press
        
        self.linear_x_scale = 0.12
        self.linear_y_scale = 0.12
        self.angular_scale = 1
        feq = 500
        self.joy = None
        self.cmd = None
        
        print("Joy_controller.py: Creating timer...")
        self.timer = self.create_timer(1/feq, self.timer_callback)
        self.get_logger().info("Joy_controller.py: Client Initialized successfully.")
        
    def joy_callback(self, data: Joy):
        # Handle mode switching (button 12)
        if self.button_newly_pressed(data, 12):
            if self.current_mode == "panda_mode":
                self.current_mode = "puppy_mode"
                self.call_unity_service("puppy_move")
            else:
                self.current_mode = "panda_mode"
                self.call_unity_service("panda_move")
            self.get_logger().info(f"Mode: {self.current_mode}")
            time.sleep(0.01)
        
        # Handle other button presses
        elif self.button_newly_pressed(data, 0):
            if self.get_current_mode() == "idle":
                if self.previous_mode in ["puppy_move", "panda_move"]:
                    self.call_unity_service(self.previous_mode)
                else:
                    self.call_unity_service("puppy_move" if self.current_mode == "puppy_mode" else "panda_move")
            else:
                self.previous_mode = self.get_current_mode()
                self.call_unity_service("idle")
            self.get_logger().info(f"Button: X - Switching to {self.get_current_mode()}")
            time.sleep(0.01)
        elif self.button_newly_pressed(data, 1):
            self.call_unity_service("puppy_move")
            self.get_logger().info("Button: Circle")
            time.sleep(0.01)

        elif self.button_newly_pressed(data, 3):
            self.get_logger().info("Button: Square")
            self.call_unity_service("panda_move")
            time.sleep(0.01)
        elif self.button_newly_pressed(data, 4):
            self.get_logger().info("Button: Triangle")
            time.sleep(0.01)
        elif self.button_newly_pressed(data, 11):
            self.call_unity_service("connect")
            self.get_logger().info("Button: Start (connect)")
            time.sleep(0.01)
        
        # Directional inputs for body movements
        elif data.axes[3]==1:
            self.get_logger().info("Direction: Up")
            time.sleep(0.01)
        elif data.axes[3]==-1:
            self.get_logger().info("Direction: Down")
            time.sleep(0.01)
        elif data.axes[2]==1:
            self.get_logger().info("Direction: Left")
            time.sleep(0.1)
        elif data.axes[2]==-1:
            self.get_logger().info("Direction: Right")
            time.sleep(0.01)

        # Directional inputs for head and mouth movement
        elif data.axes[7]==1:
            # Up arrow - move head up
            self.head_position = min(230, self.head_position + self.head_increment)
            self.get_logger().info(f"Direction: Arrow-Up, Head Position: {self.head_position}")
            time.sleep(0.01)
        elif data.axes[7]==-1:
            # Down arrow - move head down
            self.head_position = max(177, self.head_position - self.head_increment)
            self.get_logger().info(f"Direction: Arrow-Down, Head Position: {self.head_position}")
            time.sleep(0.01)
        elif data.axes[6]==1:
            # Left arrow - open mouth
            self.mouth_position = min(190, self.mouth_position + self.mouth_increment)
            self.get_logger().info(f"Direction: Arrow-Left, Mouth Position: {self.mouth_position}")
            time.sleep(0.1)
        elif data.axes[6]==-1:
            # Right arrow - close mouth
            self.mouth_position = max(116, self.mouth_position - self.mouth_increment)
            self.get_logger().info(f"Direction: Arrow-Right, Mouth Position: {self.mouth_position}")
            time.sleep(0.01)
    
        
        # Update button states for next comparison
        self.update_button_states(data)
        
        # Save joystick data for the timer callback
        self.joy = data

    def call_unity_service(self, mode):
        """Helper method to call the Unity service"""
        if self.cli is None or self.req is None:
            print(f"Error: Unity service client not available")
            return
        
        try:
            self.req.mode = mode
            self.future = self.cli.call_async(self.req)
        except Exception as e:
            self.get_logger().error(f"Failed to call Unity service: {e}")
    
    def get_current_mode(self):
        """Helper method to get the current mode from the Unity request"""
        if self.req is not None:
            return self.req.mode
        return "puppy_move" if self.current_mode == "puppy_mode" else "panda_move"

    def button_newly_pressed(self, data, button_index):
        """Check if a button is newly pressed (was not pressed in previous frame)"""
        if button_index not in self.last_button_states:
            return data.buttons[button_index] == 1
        return data.buttons[button_index] == 1 and self.last_button_states[button_index] == 0

    def update_button_states(self, data):
        """Update the stored button states"""
        for i, state in enumerate(data.buttons):
            self.last_button_states[i] = state

    def timer_callback(self):
        vel = Twist()
        pose = String()
        pose.data = str(self.current_mode)
        
        if self.joy is not None:
            if self.joy.axes[1]>0.001:
                vel.linear.x = self.joy.axes[1]*self.linear_x_scale
            elif self.joy.axes[1]<-0.001:
                vel.linear.x = self.joy.axes[1]*self.linear_x_scale
            if self.joy.axes[0]>0.001:
                vel.linear.y = self.joy.axes[0]*self.linear_y_scale
            elif self.joy.axes[0]<-0.001:
                vel.linear.y = self.joy.axes[0]*self.linear_y_scale
            if self.joy.axes[2]>0.001:
                vel.angular.z = self.joy.axes[2]*self.angular_scale
            elif self.joy.axes[2]<-0.001:
                vel.angular.z = self.joy.axes[2]*self.angular_scale
            
            # Add head and mouth position information to the pose string
            # Format: "mode:head_pos:mouth_pos"
            pose.data = f"{self.current_mode}:{self.head_position}:{self.mouth_position}"

        self.cmd_pub.publish(vel)
        self.body_pub.publish(pose)
        
def main(args=None):
    print("Joy_controller.py: Entering main function...")
    try:
        print("Joy_controller.py: Calling rclpy.init()...")
        rclpy.init(args=args)
        print("Joy_controller.py: rclpy.init() successful.")
    except Exception as e:
        print(f"Joy_controller.py: ERROR during rclpy.init(): {e}")
        sys.exit(1)

    controller = None
    try:
        print("Joy_controller.py: Creating TriceratopsControlClient instance...")
        controller = TriceratopsControlClient()
        print("Joy_controller.py: TriceratopsControlClient instance created.")
        print("Joy_controller.py: Starting rclpy.spin()...")
        rclpy.spin(controller)
        print("Joy_controller.py: rclpy.spin() finished.")
    except Exception as e:
        print(f"Joy_controller.py: ERROR during node creation or spin: {e}")
    finally:
        if controller:
            print("Joy_controller.py: Destroying node...")
            controller.destroy_node()
            print("Joy_controller.py: Node destroyed.")
        print("Joy_controller.py: Shutting down rclpy...")
        rclpy.shutdown()
        print("Joy_controller.py: rclpy shut down.")

if __name__ == '__main__':
    print("Joy_controller.py: Script execution started.")
    main()
    print("Joy_controller.py: Script execution finished.")
