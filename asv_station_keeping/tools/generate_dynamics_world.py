#!/usr/bin/env python3
"""Create the optional buoyancy world from the deterministic pool world."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE_ROOT / 'worlds' / 'research_pool.sdf'
OUTPUT = PACKAGE_ROOT / 'worlds' / 'research_pool_dynamics.sdf'


def main():
    """Write a no-contact-plane world with Fortress graded buoyancy."""
    text = SOURCE.read_text(encoding='utf-8')
    if 'ignition-gazebo-buoyancy-system' not in text:
        marker = (
            '    <plugin filename="ignition-gazebo-navsat-system"\n'
            '            name="ignition::gazebo::systems::NavSat"/>\n'
        )
        buoyancy = marker + (
            '    <plugin filename="ignition-gazebo-buoyancy-system"\n'
            '            name="ignition::gazebo::systems::Buoyancy">\n'
            '      <graded_buoyancy>\n'
            '        <default_density>1000</default_density>\n'
            '        <density_change>\n'
            '          <above_depth>0</above_depth>\n'
            '          <density>1</density>\n'
            '        </density_change>\n'
            '      </graded_buoyancy>\n'
            '    </plugin>\n'
        )
        text = text.replace(marker, buoyancy)

    water_collision = '''        <collision name="water_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 8</size>
            </plane>
          </geometry>
        </collision>
'''
    text = text.replace(water_collision, '')
    OUTPUT.write_text(text, encoding='utf-8')
    print(f'Wrote {OUTPUT}')


if __name__ == '__main__':
    main()
