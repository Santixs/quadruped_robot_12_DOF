#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys, os
sys.path.append(os.path.expanduser("~/UnityRos2_ws/src/unity_robotics_demo_msgs"))
from unity_robotics_demo_msgs.msg import JointState
import numpy as np
from datetime import datetime

def transform_to_fr(x, y, z, leg_type):
    if leg_type == 'fr':
        return x, y, z
    elif leg_type == 'fl':
        return x, -y, z
    elif leg_type == 'hr':
        return -x, y, z
    elif leg_type == 'hl':
        return -x, -y, z
    else:
        raise ValueError(f"Unknown leg type: {leg_type}")

def inverse_transform_angles(gamma, alpha, beta, leg_type):
    if leg_type == 'fr':
        return gamma, alpha, beta
    elif leg_type == 'fl':
        return -gamma, alpha, beta
    elif leg_type == 'hr':
        return gamma, -alpha, -beta
    elif leg_type == 'hl':
        return -gamma, -alpha, -beta
    else:
        raise ValueError(f"Unknown leg type: {leg_type}")

def inverse_kinematics(x, y, z):
    h = 0.0365
    d = (y ** 2 + z ** 2) ** 0.5
    l = (d ** 2 - h ** 2) ** 0.5
    gamma1 = - np.arctan(h / l)
    gamma2 = - np.arctan(y / z)
    gamma = gamma2 - gamma1

    s = (l ** 2 + x ** 2) ** 0.5
    hu = 0.065
    hl = 0.065
    n = (s ** 2 - hl ** 2 - hu ** 2) / (2 * hu)
    beta = -np.arccos(n / hl)

    alpha1 = - np.arctan(x / l)
    alpha2 = np.arccos((hu + n) / s)
    alpha = alpha2 + alpha1

    return np.array([gamma, alpha, beta])

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')
        
        # Create the subscriber
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)
        
        # Initialize storage for frames
        self.frames = []
        self.recording = True
        self.first_frame = None
        self.min_frames = 20  # Minimum number of frames before checking for repetition
        
        # Generate timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = f'joint_angles_{timestamp}.txt'
        
        self.get_logger().info('Joint State Recorder has been started')

    def detect_pattern_completion(self, current_frame, threshold=1e-2):
        """
        Detect if the motion pattern is complete by comparing with first frame
        """
        if self.first_frame is None or len(self.frames) < self.min_frames:
            return False

        # Compare current frame with first frame
        return np.all(np.abs(current_frame - self.first_frame) < threshold)

    def process_frame(self, msg):
        """Process a single frame of joint states into angles."""
        leg_order = ['fr', 'fl', 'hr', 'hl']
        frame_angles = []
        
        # Process each leg
        for i, leg in enumerate(leg_order):
            y, z, x = msg.x[i], msg.y[i], msg.z[i]
            
            # Transform coordinates
            x_fr, y_fr, z_fr = transform_to_fr(x, y, z, leg)
            
            # Calculate angles
            gamma, alpha, beta = inverse_kinematics(x_fr, y_fr, z_fr)
            
            # Transform angles back
            gamma, alpha, beta = inverse_transform_angles(gamma, alpha, beta, leg)
            
            # Add to frame angles in order
            frame_angles.extend([gamma, alpha, beta])
        
        return np.array(frame_angles)

    def save_frames(self):
        """Save recorded frames to file."""
        if not self.frames:
            return
        
        # Convert frames to numpy array
        frames_array = np.array(self.frames)
        
        # Save to text file
        np.savetxt(
            self.output_file, 
            frames_array,
            fmt='%.6f',
            header=f'Joint angles matrix: {frames_array.shape[0]} frames, 12 joints per frame\n'
                  'Order: fr[gamma,alpha,beta] fl[gamma,alpha,beta] hr[gamma,alpha,beta] hl[gamma,alpha,beta]'
        )
        
        self.get_logger().info(f'Saved {len(self.frames)} frames to {self.output_file}')

    def joint_callback(self, msg):
        """Process incoming joint states and record angles."""
        try:
            if not self.recording:
                return
            
            # Process current frame
            current_frame = self.process_frame(msg)
            
            # Store first frame for comparison
            if self.first_frame is None:
                self.first_frame = current_frame
            
            # Store frame
            self.frames.append(current_frame)
            
            # Check for pattern completion
            if self.detect_pattern_completion(current_frame):
                self.recording = False
                self.save_frames()
                self.get_logger().info(f'Pattern completed after {len(self.frames)} frames')
                return
            
            # Log progress
            if len(self.frames) % 10 == 0:
                self.get_logger().info(f'Recorded {len(self.frames)} frames')
            
        except Exception as e:
            self.get_logger().error(f'Error processing joint states: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    joint_recorder = JointStateRecorder()
    
    try:
        rclpy.spin(joint_recorder)
    except KeyboardInterrupt:
        joint_recorder.save_frames()
    finally:
        joint_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()