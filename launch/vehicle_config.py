"""Load vehicle YAML config and resolve launch profiles / argument overrides."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

PROFILES = {
    'full': {
        'gnss': True,
        'sonar': True,
        'dvl': True,
        'mavros': True,
    },
    'sensors_only': {
        'gnss': False,
        'sonar': True,
        'dvl': True,
        'mavros': True,
    },
    'gnss_only': {
        'gnss': True,
        'sonar': False,
        'dvl': False,
        'mavros': False,
    },
    'sim': {
        'gnss': False,
        'sonar': False,
        'dvl': False,
        'mavros': False,
    },
}


def default_vehicle_config_path():
    pkg_share = get_package_share_directory('mr_pinchy_bringup')
    return os.path.join(pkg_share, 'config', 'pinchy.yaml')


SONAR_DEFAULTS = {
    'sonar_ip': '192.168.194.96',
    'frame_id': 'sonar_link',
    'acoustics_enabled': True,
    'speed_of_sound': 1480.0,
    'mode': 'low-frequency',
    'salinity': 'salt',
    'range_min': 0.3,
    'range_max': 15.0,
    'udp_mode': 'multicast',
    'interface_ip': '0.0.0.0',
    'unicast_destination_ip': '',
    'unicast_destination_port': 0,
    'topic_point_cloud': '~/point_cloud',
    'topic_range_image': '~/range_image',
    'topic_intensity_image': '~/intensity_image',
    'topic_camera_info': '~/camera_info',
    'diagnostics_period': 5.0,
}


def _load_sonar_params_file(path):
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if 'sonar_node' in data:
        return dict(data['sonar_node'].get('ros__parameters', {}))
    return dict(data)


def _resolve_sonar_frame_id(frame_id, extrinsics):
    if frame_id:
        return frame_id
    child_frame = extrinsics['sonar']['child_frame']
    if child_frame.startswith('/'):
        return child_frame
    return f'/{child_frame}'


def resolve_sonar(config, overrides, extrinsics):
    """Resolve Water Linked sonar driver parameters."""
    sonar_section = dict(config.get('sonar', {}))
    params_file = overrides.get('sonar_params_file', '')
    if not params_file:
        params_file = sonar_section.pop('params_file', '')

    if params_file:
        params = _load_sonar_params_file(params_file)
    else:
        params = sonar_section

    resolved = dict(SONAR_DEFAULTS)
    resolved.update(params)

    sonar_ip = overrides.get('sonar_ip', '')
    if sonar_ip:
        resolved['sonar_ip'] = sonar_ip

    resolved['frame_id'] = _resolve_sonar_frame_id(resolved.get('frame_id', ''), extrinsics)
    return resolved


def load_vehicle_config(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('true', '1', 'yes')


def _bool_to_launch(value):
    return 'true' if value else 'false'


def resolve_subsystems(config, profile, overrides):
    """Merge profile and launch-arg overrides onto vehicle config subsystems."""
    profile_name = profile if profile else 'full'
    if profile_name not in PROFILES:
        valid = ', '.join(sorted(PROFILES))
        raise ValueError(f"Unknown profile '{profile_name}'. Valid profiles: {valid}")

    subsystems = dict(config.get('subsystems', {}))
    subsystems.update(PROFILES[profile_name])

    for key in ('gnss', 'sonar', 'dvl', 'mavros'):
        override = overrides.get(key, '')
        if override:
            subsystems[key] = _parse_bool(override)

    return subsystems


def resolve_network(config, overrides):
    """Resolve network settings with launch-arg overrides."""
    network = dict(config.get('network', {}))

    nucleus_ip = overrides.get('nucleus_ip', '')
    if nucleus_ip:
        network['nucleus_ip'] = nucleus_ip

    fcu_url = overrides.get('fcu_url', '')
    if fcu_url:
        network['mavros_fcu_url'] = fcu_url

    network.setdefault('nucleus_ip', '192.168.32.23')
    network.setdefault('nucleus_vendor', 'nortek')
    network.setdefault('nucleus_connect_delay', 3.0)
    network.setdefault('nucleus_start_delay', 6.0)
    network.setdefault('mavros_fcu_url', 'udp-b://0.0.0.0:14550@14550')

    return network


def resolve_bridges(config):
    bridges = dict(config.get('bridges', {}))
    bridges.setdefault('dvl_type', 'nucleus')
    bridges.setdefault('dvl_frame', 'dvl_link')
    bridges.setdefault('depth_frame', 'icp_map')
    bridges.setdefault('depth_topic', '/global_position/rel_alt')
    bridges.setdefault('relative_depth', False)
    bridges.setdefault('ned', False)
    bridges.setdefault('ahrs', True)
    return bridges


def resolve_extrinsics(config):
    extrinsics = dict(config.get('extrinsics', {}))
    extrinsics.setdefault('base_frame', 'base_link')
    for name in ('sonar', 'camera', 'dvl'):
        extrinsics.setdefault(name, {})
        extrinsics[name].setdefault('child_frame', f'{name}_link')
        extrinsics[name].setdefault('xyz', [0.0, 0.0, 0.0])
        extrinsics[name].setdefault('rpy', [0.0, 0.0, 0.0])
    return extrinsics


def load_resolved_config(vehicle_config_path, profile, overrides):
    config = load_vehicle_config(vehicle_config_path)
    extrinsics = resolve_extrinsics(config)
    return {
        'subsystems': resolve_subsystems(config, profile, overrides),
        'network': resolve_network(config, overrides),
        'bridges': resolve_bridges(config),
        'extrinsics': extrinsics,
        'sonar': resolve_sonar(config, overrides, extrinsics),
    }


def static_transform_node(name, parent, child, xyz, rpy):
    from launch_ros.actions import Node

    args = [str(v) for v in (*xyz, *rpy, parent, child)]
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=args,
        output='screen',
    )
