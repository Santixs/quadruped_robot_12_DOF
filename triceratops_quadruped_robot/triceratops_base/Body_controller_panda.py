#!/usr/bin/env python3
import math

# Summary of the code
"""
Hip (lateral) movement: hip_offset = base_yaw_amplitude * sin(2π * phase_ratio + phase_shift)
    - When the robot is going straight (turning rate below the threshold), no phase shift is applied. This means the lateral oscillation follows a clean sine wave with an amplitude of 0.05 meters (the default base_yaw_amplitude).
    - If the robot is turning (angular rate above the threshold), a phase shift proportional to the turning rate is added. This shifts the sine wave slightly, altering the timing of the hip’s lateral movement relative to the gait cycle.
    - The code checks the contact status of the back legs. If one is off the ground while the other is on, a fixed bias (0.02 meters by default, hip_bias_amplitude) is added.

        Example: if the Back Right leg is off and Rear Left is on, the bias is negative (what means that we shift the hip to the right), and vice versa.
        Note: The final hip offset is the sum of the base oscillation and this bias.

    
Back (vertical) movement: back_offset = vertical_amplitude * cos(2π * phase_ratio)
    - Since be use cos instead of sin the oscillation is 90° out of phase with the hip movement to appear more natural
    - The amplitude of the vertical oscillation is scaled by the forward speed (base_vertical_amplitude * |forward_speed|)
    - If almost staying still, the amplitude is damped by a factor of 0.5 to reduce the back movement.

A reduction factor of 0.2X has been applied to the lienar and angular speeds. 

"""

class BodyController:
    def __init__(self, step_duration,
                 base_yaw_amplitude=0.01, #
                 base_vertical_amplitude=0.01,
                 hip_bias_amplitude=0.01, #
                 straight_threshold=0.05,
                 linear_threshold=0.01,
                 twist_linear_scale=0.2,      
                 twist_angular_scale=0.2):
        """
        Initialize the BodyController.
        
        Parameters:
        - step_duration (float): Duration (in seconds) of one full gait cycle.
        - base_yaw_amplitude (float): Base amplitude for lateral (hip) oscillations in meters.
        - base_vertical_amplitude (float): Base amplitude for vertical (back) oscillations in meters.
        - hip_bias_amplitude (float): Additional lateral offset when one hind leg is off the ground.
        - straight_threshold (float): Angular velocity (rad/s) below which the robot is considered to be going straight.
        - linear_threshold (float): Minimal linear speed (m/s) considered as significant forward movement.
        """
        self.step_duration = step_duration
        self.base_yaw_amplitude = base_yaw_amplitude
        self.base_vertical_amplitude = base_vertical_amplitude
        self.hip_bias_amplitude = hip_bias_amplitude
        self.straight_threshold = straight_threshold
        self.linear_threshold = linear_threshold
        self.twist_linear_scale = twist_linear_scale
        self.twist_angular_scale = twist_angular_scale

        self.t = 0.0  # Internal timer for gait phase synchronization

    def update(self, dt, cmd_vel, contact_modes, hip_bias_amplitude=0):
        """
        Update trunk offsets based on the current velocity attributes and leg contact modes.
        
        Parameters:
        - dt (float): Time step in seconds.
        - cmd_vel: An object with six attributes:
            - cmd_vel.linear_x: Forward speed (m/s).
            - cmd_vel.linear_y: Lateral speed (m/s).
            - cmd_vel.linear_z: Vertical speed (m/s).
            - cmd_vel.angular_x, angular_y: (unused here).
            - cmd_vel.angular_z: Yaw (turning) rate (rad/s).
        - contact_modes (list or array of 4 ints): Binary flags for each leg in the order [FR, FL, RR, RL].
          For example, one provided sequence is:
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        
        Returns:
        - dict: {'hip': <lateral offset in meters>, 'back': <vertical offset in meters>}
          These offsets should be applied to the robot’s body frame prior to computing leg inverse kinematics.
        """
        # Update the internal timer and compute normalized phase.
        self.t = (self.t + dt) % self.step_duration
        phase_ratio = self.t / self.step_duration

        # Extract movement parameters.
        forward_speed = cmd_vel.linear_x * self.twist_linear_scale
        turning_rate = cmd_vel.angular_z * self.twist_angular_scale

        go_straight = abs(turning_rate) < self.straight_threshold
        moving_forward = abs(forward_speed) > self.linear_threshold
        turning = not go_straight

        # Compute base oscillation using the normalized gait phase.
        base_sin = math.sin(2.0 * math.pi * phase_ratio)
        base_cos = math.cos(2.0 * math.pi * phase_ratio)

        # For turning, apply a phase shift proportional to the turning rate.
        phase_shift = turning_rate * 0.1 if turning else 0.0

        # Compute the base hip (lateral) offset.
        hip_offset = (self.base_yaw_amplitude + hip_bias_amplitude) * math.sin(2.0 * math.pi * phase_ratio + phase_shift)

        # --- Integrate Contact Information for Panda-style Gait ---
        # Expected contact_modes order: [Front Right, Front Left, Rear Right, Rear Left].
        # With the coordinate convention: positive lateral (y) is to the left.
        # If one hind leg is off and the other on, shift the hip toward the side of the off-ground leg.
        hind_bias = 0.0
        if contact_modes[2] != contact_modes[3]:
            # If Rear Right is off (0) and Rear Left is on (1), shift hip to the right (negative y).
            if contact_modes[2] == 0 and contact_modes[3] == 1:
                hind_bias = -self.hip_bias_amplitude
            # If Rear Left is off (0) and Rear Right is on (1), shift hip to the left (positive y).
            elif contact_modes[2] == 1 and contact_modes[3] == 0:
                hind_bias = self.hip_bias_amplitude

        final_hip_offset = hip_offset + hind_bias

        # Compute the vertical (back) offset.
        # Scale the vertical amplitude linearly with forward speed.
        if moving_forward:
            vertical_amplitude = self.base_vertical_amplitude * abs(forward_speed)
        else:
            vertical_amplitude = self.base_vertical_amplitude * 0.5  # Dampen if nearly stationary.
        back_offset = vertical_amplitude * base_cos

        return {'hip': final_hip_offset, 'back': back_offset}

    def reset(self):
        self.t = 0.0

