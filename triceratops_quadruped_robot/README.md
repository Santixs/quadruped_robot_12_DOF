

# triceratops_robot


## **How to Run the Files**

### **Step 1: Run the Robot Program**

1. Open a terminal.
2. Navigate to the robot's program directory and execute the control script:
    
    ```bash
    cd triceratops_quadruped_robot/triceratops_base/
    python3 Controller.py
    
    ```


---

### **Step 2: Control the Robot**

You can control the robot using a joystick.

### **Joystick Controller**

1. Open another terminal.
2. Run the joystick controller script:
    
    ```bash
    cd triceratops_quadruped_robot/triceratops_base/
    python3 Joy_controller.py
    
    ```
    
3. Open another terminal.
4. Start the `joy_node` topic:
    
    ```bash
    ros2 run joy joy_node
    
    ```
    
5. Start the Computer Vision Node:

    ```bash
    ros2 launch triceratops_quadruped_robot launch_vision_integration.py
    ```


---

dt: 0.02 s
Time step for the control loop. Smaller dt means finer resolution in the gait timing.

num_phases: 4
Defines the number of distinct phases in a complete gait cycle.

contact_phases:

    [[1, 1, 1, 0],
     [1, 0, 1, 1],
     [1, 0, 1, 1],
     [1, 1, 1, 0]]

    Matrix where 1 indicates a leg in stance (in contact) and 0 indicates swing. This sets the stepping pattern for each phase.

    overlap_time: 0.2 s
    Duration when all feet are on the ground. It helps maintain stability during transitions.

    swing_time: 0.25 s
    Duration when one or more legs are off the ground (swing phase). It determines how fast the leg moves to its next position.

    overlap_ticks & swing_ticks:
        overlap_ticks: 10 ticks (≈0.2 s/0.02 s)
        swing_ticks: 12 ticks (≈0.25 s/0.02 s)
        Discrete time steps that quantify the overlap and swing durations.

    phase_ticks: [overlap_ticks, swing_ticks, overlap_ticks, swing_ticks]
    Breaks down the tick count for each of the 4 phases.

    phase_length: 44 ticks
    Total ticks in one full gait cycle (2×overlap_ticks + 2×swing_ticks).

    step_duration: 0.88 s
    Total time for one complete gait cycle (phase_length × dt).

    delta_x: 0.09067 m
    Forward/backward offset for foot placement during stance. It affects how far the legs reach.

    delta_y: 0.085 m
    Lateral (side-to-side) offset for foot placement during stance, influencing the width of the gait.

    default_z_ref: -0.10 m
    Default vertical reference for the feet in stance, determining the nominal height.

    z_clearance: 0.025 m
    Maximum vertical lift during the swing phase to clear obstacles.

    alpha: 1 (dimensionless)
    Scaling factor that adjusts the horizontal touchdown offset based on the robot’s speed.

    beta: 0.5 (dimensionless)
    Scaling factor that influences yaw-based adjustments during leg touchdown.

    z_time_constant: 0.02
    Determines the responsiveness of vertical adjustments for foot placement in stance.

Additionally, parameters in the BodyController that affect trunk (body) motion include:

    base_yaw_amplitude: 0.02 m
    Sets the amplitude of lateral (hip) oscillations.

    base_vertical_amplitude: 0.01 m
    Sets the base amplitude of vertical (back) oscillations, which is further scaled by forward speed.

    hip_bias_amplitude: 0.015 m
    Adds extra lateral offset when one hind leg is off the ground, helping with balance.

    straight_threshold: 0.05 rad/s
    Yaw rate below which the robot is considered to be moving straight, affecting phase adjustments.

    linear_threshold: 0.01 m/s
    Minimum forward speed that triggers full vertical oscillation amplitude.

    twist_linear_scale & twist_angular_scale: 0.2 (each, dimensionless)
    Scale factors that moderate the influence of commanded linear and angular velocities on the trunk’s oscillatory behavior.