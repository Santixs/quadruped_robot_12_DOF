import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # --- Configuration ---
    package_name = 'triceratops_quadruped_robot'
    joy_controller_script = 'Joy_controller.py'
    controller_script = 'Controller.py'
    scripts_subfolder = 'triceratops_base' # Subfolder within the package where scripts are
    ros_domain_id = '33'
    startup_delay_seconds = 5.0 # Delay in seconds before starting joy_node
    # ---------------------

    try:
        pkg_dir = get_package_share_directory(package_name)
        scripts_dir = os.path.join(pkg_dir, scripts_subfolder)
        print(f"Found package directory: {pkg_dir}")
        print(f"Using scripts directory: {scripts_dir}")
    except Exception as e:
        print(f"Error finding package directory '{package_name}': {e}")
        scripts_dir = f"/home/panda2/code/quadruped_robot_12_DOF-DEV/{package_name}/{scripts_subfolder}" # Fallback
        print(f"Using fallback scripts directory: {scripts_dir}")

    if not os.path.isdir(scripts_dir):
         print(f"Error: Scripts directory '{scripts_dir}' not found.")
         return LaunchDescription([])

    set_ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id)

    # --- Define Actions --- 
    # Action to start joy_node
    start_joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Action to start Joy_controller.py
    start_joy_controller = ExecuteProcess(
        cmd=['python3', os.path.join(scripts_dir, joy_controller_script)],
        name='joy_controller_process',
        cwd=scripts_dir,
        output='screen'
    )

    # Action to start Controller.py
    start_controller = ExecuteProcess(
        cmd=['python3', os.path.join(scripts_dir, controller_script)],
        name='controller_process',
        cwd=scripts_dir,
        output='screen'
    )

    # --- Arrange Startup with Delay ---
    # 1. Start Controller.py immediately 
    #    (assuming it doesn't strictly depend on joy_node starting first)
    # 2. Introduce a delay
    # 3. After delay, start joy_node
    # 4. Once joy_node starts, start Joy_controller.py (which subscribes to /joy)

    delayed_joy_node = TimerAction(
        period=startup_delay_seconds,
        actions=[start_joy_node]
    )

    # Start Joy_controller only after joy_node has successfully started
    start_joy_controller_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_joy_node,
            on_start=[start_joy_controller]
        )
    )

    return LaunchDescription([
        set_ros_domain_id,
        
        # Start controller immediately
        start_controller,
        
        # Start joy_node after a delay
        delayed_joy_node,
        
        # Start joy_controller after joy_node starts
        start_joy_controller_handler
    ]) 