"""multi_lidar_fusion 메인 launch.

이 파일의 역할:
    융합 노드 하나를 두 YAML(lidar_extrinsics.yaml, fusion_params.yaml)과 함께 띄운다.
    센서 드라이버는 별도 launch(multi_lidar_drivers.launch.py)로 분리했다 —
    드라이버가 죽어도 융합 노드는 살아 있어야 하고(요구 §20), rosbag 재생 시에는
    드라이버 없이 융합만 돌려야 하기 때문이다.

    ★ 장착값(extrinsic)의 단일 원천은 `stack_parking/config/lidar_mounts.yaml` 이다.
      이 launch 가 그 파일을 읽어 노드 파라미터로 주입하고, lidar_extrinsics.yaml 의
      값과 다르면 경고를 찍는다. 값을 고칠 곳은 lidar_mounts.yaml 한 곳뿐이다.
      (stack_parking 이 없는 환경이면 조용히 lidar_extrinsics.yaml 만 쓴다.)

실행:
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py sim:=true rviz:=true
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py use_mounts:=false

인자:
    extrinsics_file  센서 정의·토픽·거리·FOV       (기본: config/lidar_extrinsics.yaml)
    params_file      융합 알고리즘 파라미터         (기본: config/fusion_params.yaml)
    mounts_file      장착값 단일 원천               (기본: stack_parking/config/lidar_mounts.yaml)
    use_mounts       mounts_file 로 extrinsic 덮어쓰기 (기본: true)
    sim              합성 라이다 4대를 함께 띄움    (기본: false)
    sim_params_file  시뮬레이터 파라미터            (기본: config/sim_lidars.yaml)
    rviz             RViz2 동시 실행                (기본: false)
    log_level        노드 로그 레벨                 (기본: info)
"""

import math
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml

# 융합 슬롯 <-> lidar_mounts.yaml 의 키. 이름 대응은 **여기 한 곳에만** 있다.
MOUNT_OF_SLOT = {'a1': 'front', 'a2': 'rear', 'b1': 'left', 'b2': 'right'}


def _default_mounts_file():
    """stack_parking 이 설치돼 있으면 그 config 경로, 없으면 빈 문자열."""
    try:
        return os.path.join(
            get_package_share_directory('stack_parking'), 'config', 'lidar_mounts.yaml')
    except PackageNotFoundError:
        return ''


def _wrap_deg(a):
    """각도를 (-180, 180] 으로 정규화."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def _read_yaml(path):
    if not path or not os.path.isfile(path):
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _extrinsics_in_file(path):
    """lidar_extrinsics.yaml 이 들고 있는 extrinsic 값을 꺼낸다 (대조용)."""
    doc = _read_yaml(path)
    for top in doc.values():
        params = top.get('ros__parameters') if isinstance(top, dict) else None
        if isinstance(params, dict) and 'extrinsics' in params:
            return params['extrinsics'] or {}
    return {}


def _build(context, *args, **kwargs):
    extrinsics_file = LaunchConfiguration('extrinsics_file').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    mounts_file = LaunchConfiguration('mounts_file').perform(context)
    sim_params_file = LaunchConfiguration('sim_params_file').perform(context)
    use_mounts = LaunchConfiguration('use_mounts').perform(context).lower() in (
        '1', 'true', 'yes')

    notes = []
    overrides = {}

    if use_mounts:
        mounts_doc = _read_yaml(mounts_file) or {}
        mounts = mounts_doc.get('lidar_mounts') or {}
        if not mounts:
            notes.append(LogInfo(msg=(
                '[multi_lidar_fusion] 장착값 원천을 못 읽었다 '
                f'({mounts_file or "경로 없음"}) — lidar_extrinsics.yaml 값을 그대로 쓴다.')))
        else:
            have = _extrinsics_in_file(extrinsics_file)
            diffs = []
            unverified = []
            for slot, key in MOUNT_OF_SLOT.items():
                m = mounts.get(key)
                if not isinstance(m, dict):
                    continue
                x = float(m.get('x', 0.0))
                y = float(m.get('y', 0.0))
                z = float(m.get('z', 0.0))
                # 장착 yaw: yaw_deg 가 있으면 그것이 실측값이다.
                # 없으면 "유효 시야의 중심 = 센서 정면"이라는 가정으로 축퇴한다 —
                # 2026-08-13 실차에서 RPLiDAR 2대가 이 가정과 180도 어긋나 있었다
                # (센서 0도가 차체 안쪽을 봄 → FOV 가 차체만 남겨 empty).
                fov_center = float(m.get('fov_center_deg', 0.0))
                if 'yaw_deg' in m:
                    yaw_deg = float(m['yaw_deg'])
                else:
                    yaw_deg = fov_center
                    unverified.append(slot)
                yaw = math.radians(yaw_deg)
                overrides[f'extrinsics.{slot}.x'] = x
                overrides[f'extrinsics.{slot}.y'] = y
                overrides[f'extrinsics.{slot}.z'] = z
                overrides[f'extrinsics.{slot}.roll'] = 0.0
                overrides[f'extrinsics.{slot}.pitch'] = 0.0
                overrides[f'extrinsics.{slot}.yaw'] = yaw

                # 시야각도 같은 원천에서. lidar_mounts.yaml 의 시야각은 **vehicle frame**
                # 기준이므로 센서 frame 으로 옮긴다:  센서각 = vehicle각 - 장착 yaw.
                # ±180 을 가로지르면 min > max 가 되는데, AngularSector 가 그 표기를
                # 그대로 이해한다(감긴 구간).
                w = float(m.get('fov_width_deg', 360.0))
                if w < 359.0:
                    center = _wrap_deg(fov_center - yaw_deg)
                    overrides[f'sensors.{slot}.fov_enabled'] = True
                    overrides[f'sensors.{slot}.fov_min_deg'] = _wrap_deg(center - 0.5 * w)
                    overrides[f'sensors.{slot}.fov_max_deg'] = _wrap_deg(center + 0.5 * w)

                old = have.get(slot) or {}
                for name, val in (('x', x), ('y', y), ('z', z)):
                    if name in old and abs(float(old[name]) - val) > 1e-6:
                        diffs.append(f'{slot}.{name}: {old[name]} -> {val}')

            # fov_status 는 lidar_mounts 바깥 최상위 키다.
            #   geometric_upper_bound = 바퀴만 고려한 기하 상한 (실측 mask 로 줄어들 수 있음)
            #   measured               = 실측 확정
            notes.append(LogInfo(msg=(
                f'[multi_lidar_fusion] 장착값 원천: {mounts_file} '
                f'(fov_status={mounts_doc.get("fov_status", "?")})')))
            if diffs:
                notes.append(LogInfo(msg=(
                    '[multi_lidar_fusion] ! lidar_extrinsics.yaml 이 원천과 다르다 — '
                    '원천 값으로 덮어쓴다: ' + ', '.join(diffs))))
            if unverified:
                notes.append(LogInfo(msg=(
                    '[multi_lidar_fusion] ! 장착 yaw 미실측: ' + ', '.join(unverified) +
                    ' — lidar_mounts.yaml 에 yaw_deg 가 없어 fov_center_deg 로 대체했다. '
                    'FOV 가 엉뚱한 방향을 자를 수 있다(실차에서 겪음).')))

    fusion_node = Node(
        package='multi_lidar_fusion',
        executable='multi_lidar_fusion_node',
        name='multi_lidar_fusion',
        output='screen',
        emulate_tty=True,
        # 순서가 중요하다: 뒤가 앞을 덮어쓴다. 장착값 원천이 가장 마지막.
        parameters=[extrinsics_file, params_file] + ([overrides] if overrides else []),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # 시뮬레이터에도 같은 장착값을 먹인다 — 합성 데이터와 융합 노드가 같은 위치를
    # 보게 해서, 어긋남이 보이면 그건 순수하게 알고리즘 문제다.
    sim_node = Node(
        package='multi_lidar_fusion',
        executable='test_scan_publisher',
        name='test_scan_publisher',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('sim')),
        parameters=[extrinsics_file, sim_params_file] + ([overrides] if overrides else []),
    )

    return notes + [fusion_node, sim_node]


def generate_launch_description():
    pkg = get_package_share_directory('multi_lidar_fusion')

    args = [
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=os.path.join(pkg, 'config', 'lidar_extrinsics.yaml'),
            description='센서 정의(토픽/타입/frame/거리/FOV)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'fusion_params.yaml'),
            description='동기화/보상/필터/가상스캔 파라미터'),
        DeclareLaunchArgument(
            'mounts_file', default_value=_default_mounts_file(),
            description='장착값 단일 원천 (stack_parking/config/lidar_mounts.yaml)'),
        DeclareLaunchArgument(
            'use_mounts', default_value='true',
            description='mounts_file 값으로 extrinsic·FOV 를 덮어쓸지'),
        DeclareLaunchArgument(
            'sim_params_file',
            default_value=os.path.join(pkg, 'config', 'sim_lidars.yaml'),
            description='합성 라이다 시뮬레이터 파라미터'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='실 센서 없이 합성 라이다 4대로 검증'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz2 동시 실행'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'multi_lidar.rviz')],
    )

    return LaunchDescription(args + [OpaqueFunction(function=_build), rviz_node])
