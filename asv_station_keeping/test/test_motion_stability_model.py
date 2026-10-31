# Copyright 2026 ASV Station Keeping contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression checks for the simulation-only surge-stability configuration."""

from pathlib import Path
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).parents[1]


def test_thruster_mounts_use_the_stability_height():
    """Keep all four thrust application points on the calibrated plane."""
    root = ElementTree.parse(
        PACKAGE_ROOT / 'urdf' / 'research_catamaran.urdf').getroot()
    names = {
        'port_hull_to_port_front_thruster',
        'starboard_hull_to_starboard_front_thruster',
        'port_hull_to_port_rear_thruster',
        'starboard_hull_to_starboard_rear_thruster',
    }
    joints = {
        joint.attrib['name']: joint
        for joint in root.findall('joint')
        if joint.attrib.get('name') in names
    }
    assert set(joints) == names
    for joint in joints.values():
        xyz = joint.find('origin').attrib['xyz'].split()
        assert float(xyz[2]) == -0.05


def test_hydrodynamics_target_the_physical_body():
    """Damp the link that survives Gazebo's fixed-joint lumping.

    The URDF-to-SDF conversion merges every fixed-joint child into the root
    link, so a ``base_link`` target does not exist in the simulator: the
    plugin then logs ``no linear vel`` forever and neither damping nor the
    ocean current acts on the vehicle.
    """
    root = ElementTree.parse(
        PACKAGE_ROOT / 'urdf' / 'research_catamaran.urdf').getroot()
    plugin = next(
        plugin for plugin in root.findall('gazebo/plugin')
        if plugin.attrib.get('name', '').endswith('Hydrodynamics')
    )
    children = {j.find('child').attrib['link'] for j in root.findall('joint')}
    roots = [
        link.attrib['name'] for link in root.findall('link')
        if link.attrib['name'] not in children
    ]
    assert roots == ['base_footprint']
    assert plugin.findtext('link_name') == roots[0]
    for tag in ('xU', 'yV', 'zW', 'kP', 'mQ', 'nR'):
        assert plugin.find(tag) is not None


def test_ballast_is_low_and_sufficient_for_conservative_motion():
    """Keep the simulation centre of gravity below the deck payload."""
    root = ElementTree.parse(
        PACKAGE_ROOT / 'urdf' / 'research_catamaran.urdf').getroot()
    ballast = root.find("link[@name='simulation_ballast']/inertial/mass")
    joint = root.find("joint[@name='base_to_simulation_ballast']/origin")
    assert float(ballast.attrib['value']) == 12.0
    assert float(joint.attrib['xyz'].split()[2]) == -0.20


def test_thruster_plugins_have_a_conservative_command_cap():
    """Prevent a manual topic command from bypassing the slow baseline."""
    root = ElementTree.parse(
        PACKAGE_ROOT / 'urdf' / 'research_catamaran.urdf').getroot()
    plugins = [
        plugin for plugin in root.findall('gazebo/plugin')
        if plugin.attrib.get('name', '').endswith('Thruster')
    ]
    assert len(plugins) == 4
    for plugin in plugins:
        assert plugin.findtext('max_thrust_cmd') == '6'
        assert plugin.findtext('min_thrust_cmd') == '-6'


def test_motion_launch_uses_buoyancy_world_and_equilibrium_spawn():
    """Keep the repeatable motion launch out of rigid water contact mode."""
    launch_path = PACKAGE_ROOT / 'launch' / 'motion_demo.launch.py'
    launch_text = launch_path.read_text(encoding='utf-8')
    research_text = (
        PACKAGE_ROOT / 'launch' / 'research_catamaran.launch.py').read_text(
            encoding='utf-8')
    assert "default_value='research_pool_dynamics.sdf'" in launch_text
    assert "DeclareLaunchArgument('spawn_z', default_value='-0.10')" in (
        launch_text)
    assert "default_value='research_pool_dynamics.sdf'" in research_text
    assert "'slew_rate': LaunchConfiguration('slew_rate')" in launch_text
    assert "'enable_tilt_guard': LaunchConfiguration('enable_tilt_guard')" in (
        launch_text)


def test_repeatable_validation_launch_is_installed_in_source():
    """Keep a one-command stability validation path in the package."""
    launch_path = PACKAGE_ROOT / 'launch' / 'motion_validation.launch.py'
    launch_text = launch_path.read_text(encoding='utf-8')
    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    validator_text = (
        PACKAGE_ROOT / 'asv_station_keeping' / 'motion_validation.py'
    ).read_text(encoding='utf-8')
    assert 'verify_motion_stability' in launch_text
    assert 'verify_motion_stability' in setup_text
    assert 'MOTION_VALIDATION_PASS' in validator_text


def test_dynamic_station_keeping_experiment_is_wired():
    """Keep the controller, current disturbance, and status path connected."""
    launch_path = PACKAGE_ROOT / 'launch' / 'dynamic_station_keeping.launch.py'
    launch_text = launch_path.read_text(encoding='utf-8')
    controller_text = (
        PACKAGE_ROOT / 'asv_station_keeping' / 'station_keeping_pid.py'
    ).read_text(encoding='utf-8')
    current_text = (
        PACKAGE_ROOT / 'asv_station_keeping' / 'current_disturbance.py'
    ).read_text(encoding='utf-8')
    package_text = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert 'research_current_disturbance' in launch_text
    assert '/ocean_current@geometry_msgs/msg/Vector3]' in launch_text
    assert '/research_catamaran/station_keeping/status' in controller_text
    assert 'DiagnosticArray' in controller_text
    assert 'Vector3' in current_text
    assert '<depend>diagnostic_msgs</depend>' in package_text


def test_robot_state_publisher_receives_namespaced_joint_states():
    """Keep continuous propeller links transformable in RViz."""
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'research_catamaran.launch.py'
    ).read_text(encoding='utf-8')
    assert "'/joint_states'" in launch_text
    assert "'/research_catamaran/joint_states'" in launch_text


def test_dynamics_world_uses_the_live_demo_physics_rate():
    """Keep the interactive dynamics world at the validated 500 Hz baseline."""
    world_text = (
        PACKAGE_ROOT / 'worlds' / 'research_pool_dynamics.sdf'
    ).read_text(encoding='utf-8')
    assert '<max_step_size>0.002</max_step_size>' in world_text
    assert '<real_time_update_rate>500</real_time_update_rate>' in world_text
