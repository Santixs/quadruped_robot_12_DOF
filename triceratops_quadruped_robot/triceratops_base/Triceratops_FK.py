import numpy as np
from transforms3d.euler import euler2mat
from math import pi

class ForwardKinematics():
    def __init__(self, config):
        self.config = config
        
    def leg_explicit_forward_kinematics(self, motor1, motor2, leg_index):
        """Find the body-relative foot position corresponding to the given joint angles for a given leg and configuration
        
        Parameters
        ----------
        motor1 : float
            [description]
        motor2 : float
            linkageleg - not directly controlling 
        leg_index : [type]
            [description]
        config : [type]
            [description]
        
        Returns
        -------
        numpy array (3)
            Array of corresponding joint angles.
        """

        theta_k_0 = (3*pi)/4

        theta_k = theta_k_0 + motor2 - motor1 + 0
        theta_h = motor2 + 0

        return np.array([theta_k, theta_h])
    
    def leg_explicit_forward_kinematics_2(self, theta0, theta1, theta2, leg_index):
        """Find the body-relative foot position corresponding to the given joint angles for a given leg and configuration
        
        Parameters
        ----------
        motor1 : float
            [description]
        motor2 : float
            linkageleg - not directly controlling 
        leg_index : [type]
            [description]
        config : [type]
            [description]
        
        Returns
        -------
        numpy array (3)
            Array of corresponding joint angles.
        """
        if leg_index == 0 or leg_index == 2:
            gamma = theta2
        else:
            gamma = -theta2

        if leg_index == 1 or leg_index == 3:
            alpha = theta0
        else:
            alpha = -theta0

        if leg_index == 1 or leg_index == 2:
            theta0 = -theta0
            theta1 = -theta1
        beta = 360-135-(90-theta1)-theta0


        if leg_index == 0 or leg_index == 1:
            beta = -beta


        return np.array([gamma, alpha, beta])
