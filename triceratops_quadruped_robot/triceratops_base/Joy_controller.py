import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
# from champ_interfaces.srv import SetMode
import sys, os
sys.path.append(os.path.expanduser("~/UnityRos2_ws/src/unity_robotics_demo_msgs"))
from unity_robotics_demo_msgs.msg import JointState
from unity_robotics_demo_msgs.srv import UnityAnimateService


class TriceratopsControlClient(Node):
    def __init__(self):
        super().__init__('JoyControlClient')
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.body_pub = self.create_publisher(String, 'body_pose', 1)
        self.cli = self.create_client(UnityAnimateService, 'UnityAnimate_srv')
        self.req = UnityAnimateService.Request()
        
        # Current robot mode (only panda_mode or puppy_mode)
        self.current_mode = "puppy_mode"
        self.previous_mode = "puppy_mode"
        
        # Track button states to prevent continuous triggering
        self.last_button_states = {}
        
        self.linear_x_scale = 0.1
        self.linear_y_scale = 0.08
        self.angular_scale = 1
        feq = 500
        self.joy = None
        self.cmd = None
        
        self.timer = self.create_timer(1/feq, self.timer_callback)
        self.get_logger().info("Client Initialized")
        
    def joy_callback(self, data: Joy):
        # Handle mode switching (button 12)
        if self.button_newly_pressed(data, 12):
            if self.current_mode == "panda_mode":
                self.current_mode = "puppy_mode"
                self.req.mode = "puppy_move"
            else:
                self.current_mode = "panda_mode"
                self.req.mode = "panda_move"
            self.future = self.cli.call_async(self.req)
            self.get_logger().info(f"Mode: {self.current_mode}")
            time.sleep(0.01)
        
        # Handle other button presses
        elif self.button_newly_pressed(data, 0):
            if self.req.mode == "idle":
                if self.previous_mode in ["puppy_move", "panda_move"]:
                    self.req.mode = self.previous_mode
                else:
                    self.req.mode = "puppy_move" if self.current_mode == "puppy_mode" else "panda_move"
            else:
                self.previous_mode = self.req.mode
                self.req.mode = "idle"
            self.future = self.cli.call_async(self.req)
            self.get_logger().info(f"Button: X - Switching to {self.req.mode}")
            time.sleep(0.01)
        elif self.button_newly_pressed(data, 1):
            self.req.mode = "puppy_move"
            self.future = self.cli.call_async(self.req)
            self.get_logger().info("Button: Circle")
            time.sleep(0.01)

        elif self.button_newly_pressed(data, 3):
            self.get_logger().info("Button: Square")
            self.req.mode = "panda_move"
            self.future = self.cli.call_async(self.req)
            time.sleep(0.01)
        elif self.button_newly_pressed(data, 4):
            self.get_logger().info("Button: Triangle")
            time.sleep(0.01)
        elif self.button_newly_pressed(data, 11):
            self.req.mode = "connect"
            self.future = self.cli.call_async(self.req)
            self.get_logger().info("Button: Start (connect)")
            time.sleep(0.01)
        
        # Directional inputs (these can trigger continuously)
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

        elif data.axes[7]==1:
            self.get_logger().info("Direction: Arrow-Up")
            time.sleep(0.01)
        elif data.axes[7]==-1:
            self.get_logger().info("Direction: Arrow-Down")
            time.sleep(0.01)
        elif data.axes[6]==1:
            self.get_logger().info("Direction: Arrow-Left")
            time.sleep(0.1)
        elif data.axes[6]==-1:
            self.get_logger().info("Direction: Arrow-Right")
            time.sleep(0.01)
    
        
        # Update button states for next comparison
        self.update_button_states(data)
        
        # Save joystick data for the timer callback
        self.joy = data

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

        self.cmd_pub.publish(vel)
        self.body_pub.publish(pose)
        
def main(args=None):
    rclpy.init(args=args)

    controller = TriceratopsControlClient()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
