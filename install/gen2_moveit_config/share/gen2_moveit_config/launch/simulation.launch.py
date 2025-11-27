import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    gen2_config_path = get_package_share_directory('gen2_moveit_config')
    kinova_desc_path = get_package_share_directory('kinova_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Environment
    ros_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(kinova_desc_path, '..')
    )

    # 1. URDF
    xacro_file = os.path.join(kinova_desc_path, 'urdf', 'j2n6s300_standalone.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 2. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'j2n6s300', '-z', '0.05'],
        output='screen')

    # 4. Robot State Publisher
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_description, {'use_sim_time': True}])

    # 5. Controllers
    jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    arm_ctrl = Node(package="controller_manager", executable="spawner", arguments=["arm_controller"])
    grip_ctrl = Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"])

    # 6. Bridge
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')

    return LaunchDescription([
        ros_gz_resource_path,
        gazebo, spawn, rsp, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm_ctrl, grip_ctrl]))
    ])