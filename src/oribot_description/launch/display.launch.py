import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='oribot_description').find('oribot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/oribot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    video_resource = LaunchConfiguration('video_resource', default = 'csi://0')
    video_flip = LaunchConfiguration('video-flip', default = 'rotate-180')

    print(pkg_share)
    print(default_model_path)
    print(default_rviz_config_path)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("slam_toolbox"),"launch","online_async_launch.py")
        )
    )
    # online_async_launch.py 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    oribot_node = Node(
        package='oribot',
        executable='api_node',
        name='inu_node',
        output='screen'
    )

    autodrive_node = Node(
        package='oribot_autodrive',
        executable='autodrive_node',
        name='autodrive_node',
        output='screen'
    )

    driver_node = Node(
        package='oribot_driver',
        executable='driver_node',
        name='inu_node',
        output='screen'
    )

    imu_node = Node(
        package='oribot_imu',
        executable='imu_node',
        name='inu_node',
        output='screen'
    )

    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    ld = launch.LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        DeclareLaunchArgument(
            'video_resource',
            default_value=video_resource,
            description='Specifying video source'),
        
        DeclareLaunchArgument(
            'video_flip',
            default_value=video_flip,
            description='Specifying video flip'),
        
        autodrive_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_localization_node,
        driver_node,
        imu_node,
        oribot_node,
        Node(
            package="ros_deep_learning",
            executable="video_source",
            name="video_source",
            parameters=[{
                'resource': video_resource,
                'flip': video_flip
            }],
            output="screen"
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                            'serial_port': serial_port, 
                            'serial_baudrate': serial_baudrate, 
                            'frame_id': frame_id,
                            'inverted': inverted, 
                            'angle_compensate': angle_compensate,
                            'scan_mode': scan_mode
                            }],
            output='screen',
        ),
        slam,
        rviz_node
    ])

    #ld.add_action(slam)
    return ld

# https://www.youtube.com/watch?v=idQb2pB-h2Q