"""
RoboCup KidSize 축구장 + kubot26 3대.

    ros2 launch kubot26_pkgs soccer.launch.py
    ros2 launch kubot26_pkgs soccer.launch.py count:=3 formation:=kickoff

제어는 로봇별 네임스페이스로:
    ros2 topic pub --once /kubot1/KubotMode std_msgs/msg/Int32 "{data: 2}"

필드 규격은 scripts/gen_field.py 의 FIELD 딕셔너리에서 수정 후 재생성:
    python3 scripts/gen_field.py > worlds/robocup_hsl_small.world
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            DeclareLaunchArgument, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory('kubot26_pkgs')

current_paths = os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
clean_paths = []
for path in current_paths:
    if not path:
        continue
    abs_path = os.path.abspath(path)
    if "install" in abs_path or "ament_index" in abs_path or "colcon-core" in abs_path:
        continue
    clean_paths.append(path)
clean_paths.append(os.path.join(os.path.expanduser('~'), 'kubot26_ws', 'src'))
os.environ["GAZEBO_MODEL_PATH"] = ":".join(list(set(clean_paths)))

SPAWN_Z = 0.60          # 기립 높이 0.4315 보다 높게 (카펫 두께 0.02 포함해도 여유)

# 자기 진영(x<0)에 서는 기본 배치. (x, y, yaw[rad])
FORMATIONS = {
    # 킥오프: 공격수 1명이 센터 근처, 수비 2명
    'kickoff': [(-0.6, 0.0, 0.0), (-2.5, 1.2, 0.0), (-2.5, -1.2, 0.0)],
    # 한 줄로 나란히 (동작 비교용)
    'line':    [(-1.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (-1.0, -1.0, 0.0)],
    # 골키퍼 + 필드 2명
    'defense': [(-4.2, 0.0, 0.0), (-2.0, 1.5, 0.0), (-2.0, -1.5, 0.0)],
}


def spawn_robots(context, *args, **kwargs):
    count = int(LaunchConfiguration('count').perform(context))
    form = LaunchConfiguration('formation').perform(context)
    slots = FORMATIONS.get(form, FORMATIONS['kickoff'])
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'kubot26_pkgs.urdf')

    actions = []
    for i in range(count):
        name = f'kubot{i + 1}'
        ns = f'/{name}'
        x, y, yaw = slots[i % len(slots)]
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, arguments=[urdf_path], output='screen'))
        actions.append(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', name, '-robot_namespace', ns, '-file', urdf_path,
                       '-x', str(x), '-y', str(y), '-z', str(SPAWN_Z), '-Y', str(yaw)],
            output='screen'))
    return actions


def generate_launch_description():
    world_path = os.path.join(pkg_share_dir, 'worlds', 'robocup_hsl_small.world')
    plugin_path = os.path.join(pkg_share_dir, '../../lib/kubot26_pkgs')
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[os.environ.get('GAZEBO_PLUGIN_PATH', ''), ':', plugin_path])

    gz = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzclient.launch.py')))

    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('formation', default_value='kickoff',
                              description='kickoff | line | defense'),
        set_plugin_path, gzserver, gzclient,
        OpaqueFunction(function=spawn_robots),
    ])
