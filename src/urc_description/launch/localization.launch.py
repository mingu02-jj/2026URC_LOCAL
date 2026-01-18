from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_path = '/home/mingu02/local_ws/src/urc_description/description/robot_core.xacro'

    # 런치 인자 선언
    cam_name_arg  = DeclareLaunchArgument('camera_name',  default_value='zed')
    cam_model_arg = DeclareLaunchArgument('camera_model', default_value='zed2')

    camera_name  = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    # Xacro 파일 실행 명령어 구성
    xacro_cmd = Command([
        'xacro ', urdf_path,
        ' camera_name:=', camera_name,
        ' camera_model:=', camera_model
    ])

    robot_description = ParameterValue(xacro_cmd, value_type=str)

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Joint State Publisher GUI (슬라이더)
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # EKF 로컬라이제이션 노드
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom_node',
        output='screen',
        parameters=['/home/mingu02/local_ws/src/urc_description/config/ekf_odom.yaml'],
        remappings=[('odometry/filtered', 'odometry/local')]  # 토픽 충돌 방지
    )

    # LaunchDescription 반환
    return LaunchDescription([
        cam_name_arg,
        cam_model_arg,
        rsp,
        jsp_gui,
        ekf_local
    ])

