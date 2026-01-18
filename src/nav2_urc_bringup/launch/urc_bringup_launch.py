# nav2_urc_bringup/launch/urc_bringup_launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    urc_desc_share = get_package_share_directory('urc_description')
    zed_share      = get_package_share_directory('zed_wrapper')
    bringup_share  = get_package_share_directory('nav2_urc_bringup')

    # 카메라 모델 인자(원하면 커맨드라인에서 덮어쓰기 가능)
    camera_model_arg = DeclareLaunchArgument('camera_model', default_value='zed2')

    # 1) Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urc_desc_share, 'launch', 'localization.launch.py'))
    )

    # 2) ZED
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_share, 'launch', 'zed_camera.launch.py')),
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items()
    )

    # 3) Nav2
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'urc_navigation.py'))
    )

    # 4) RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'rviz_launch.py'))
    )

    # 순차 기동: t=0 localization, t=2 ZED, t=7 Nav2, t=10 RViz
    return LaunchDescription([
        camera_model_arg,

        LogInfo(msg='[0s] start localization'),
        localization,

        TimerAction(period=2.0, actions=[
            LogInfo(msg='[2s] start ZED'),
            zed
        ]),

        TimerAction(period=7.0, actions=[
            LogInfo(msg='[7s] start Nav2'),
            nav
        ]),

        TimerAction(period=10.0, actions=[
            LogInfo(msg='[10s] start RViz'),
            rviz
        ]),
    ])

