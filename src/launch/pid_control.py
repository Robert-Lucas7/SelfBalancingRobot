from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
# from launch_ros.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')    
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    my_package_path = get_package_share_directory('self_balancing_robot')
    sdf_file = os.path.join(my_package_path, 'models', 'basic_sbr.sdf')

    control_exec_config = LaunchConfiguration('control_exec')

    control_exec_arg = DeclareLaunchArgument(
        'control_exec',
        default_value='robot_control_pid',
        description='Parameter to control which control method is used.'
    )
    robot_control_node = Node(
        package='self_balancing_robot',
        executable=control_exec_config,
        output='screen'
    )

    log_arg = LogInfo(
        msg=['Launching robot with controller: ', control_exec_config]
    )

    return LaunchDescription([
        control_exec_arg,
        log_arg,
        robot_control_node,
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([my_package_path, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([my_package_path, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': sdf_file,
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/world/empty/control@ros_gz_interfaces/srv/ControlWorld', 
                       '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                       '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                       '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                       '/world/empty/model/differential_drive_robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
                       ],
            output='screen'
        ),
    ])