"""
kubot26 여러 대를 각각 독립 제어하는 런치.

각 로봇은 -robot_namespace 로 분리되어 자기 네임스페이스의 토픽만 듣는다.

    ros2 launch kubot26_pkgs multi_robot.launch.py            # 기본 3대
    ros2 launch kubot26_pkgs multi_robot.launch.py count:=2   # 2대

제어 예:
    ros2 topic pub --once /kubot1/KubotMode std_msgs/msg/Int32 "{data: 2}"
    ros2 topic echo /kubot2/joint_states
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory('kubot26_pkgs')

# meshes 경로 인식용 GAZEBO_MODEL_PATH 정화 (단일 런치와 동일)
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

# 로봇을 y축으로 이 간격만큼 떨어뜨려 놓는다 (발 폭 0.14 m 이라 충분히 여유)
SPACING_Y = 0.8
SPAWN_Z = 0.60          # 기립 높이 0.4315 보다 높게 (지면 관통 방지)


def spawn_robots(context, *args, **kwargs):
    count = int(LaunchConfiguration('count').perform(context))
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'kubot26_pkgs.urdf')

    actions = []
    for i in range(count):
        name = f'kubot{i + 1}'
        ns = f'/{name}'
        y = (i - (count - 1) / 2.0) * SPACING_Y      # 가운데 정렬

        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            arguments=[urdf_path],
            output='screen',
        ))
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', name,
                '-robot_namespace', ns,
                '-file', urdf_path,
                '-x', '0.0', '-y', str(y), '-z', str(SPAWN_Z),
            ],
            output='screen',
        ))
    return actions


def generate_launch_description():
    world_path = os.path.join(pkg_share_dir, 'worlds', 'kubot26.world')

    plugin_path = os.path.join(pkg_share_dir, '../../lib/kubot26_pkgs')
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[os.environ.get('GAZEBO_PLUGIN_PATH', ''), ':', plugin_path]
    )

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3', description='스폰할 로봇 대수'),
        set_plugin_path,
        gzserver,
        gzclient,
        OpaqueFunction(function=spawn_robots),
    ])
