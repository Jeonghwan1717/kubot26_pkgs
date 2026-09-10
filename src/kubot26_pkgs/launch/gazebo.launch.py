import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 1. meshes 3D 파일 경로 인식용 환경 변수 정밀 정화 및 설정
pkg_share_dir = get_package_share_directory('kubot26_pkgs')

current_paths = os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
clean_paths = []

for path in current_paths:
    if not path:
        continue
    abs_path = os.path.abspath(path)

    # install 내부의 share나 빌드 시스템 관련 폴더들을 필터링해서 걷어냅니다.
    if "install" in abs_path or "ament_index" in abs_path or "colcon-core" in abs_path:
        continue
    clean_paths.append(path)

# 오직 순수한 src 폴더 경로만 가제보에 넣어줍니다. (package:// 메쉬 경로 해석용)
workspace_src_dir = os.path.join(os.path.expanduser('~'), 'kubot26_ws', 'src')
clean_paths.append(workspace_src_dir)
os.environ["GAZEBO_MODEL_PATH"] = ":".join(list(set(clean_paths)))


def generate_launch_description():
    # 파일 경로 추적
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'kubot26_pkgs.urdf')
    world_path = os.path.join(pkg_share_dir, 'worlds', 'kubot26.world')

    # 2. 플러그인 (.so 파일) 위치 환경 변수 주입
    plugin_path = os.path.join(pkg_share_dir, '../../lib/kubot26_pkgs')
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[os.environ.get('GAZEBO_PLUGIN_PATH', ''), ':', plugin_path]
    )

    # 3. Gazebo 공식 런치 분리 실행 (메쉬 로딩 누락 방지 안정화)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # 가제보 서버 (물리 엔진 기동 & 월드 로드)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    # 가제보 클라이언트 (GUI 화면 켜기)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py'))
    )

    # 4. 로봇 상태 퍼블리셔 노드
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_path],
        output='screen'
    )

    # 5. 로봇 스폰(Spawn) 노드 (z축을 띄워서 초기 물리 충돌 폭발 방지)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'kubot26_robot',
            '-file', urdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.60'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_plugin_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity
    ])
