#!/usr/bin/env python3
"""
Generate mesh-loading diagnostics, not a CAD-accurate assembly.

00_original is the visible, known-good two-hull/four-thruster baseline.
00_base is an inertial-only model and intentionally has no visual geometry.
The other variants use the earlier manually estimated mounting transforms.
They retain collisions and inertias but omit Gazebo controllers and sensors.
A change in rendering is a lead to investigate, not proof of a bad mesh.
"""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_URDF = PACKAGE_ROOT / 'urdf' / 'asv_station_keeping.urdf'
SOURCE_URDF = PACKAGE_ROOT.parent / 'asv_assem_sim' / 'urdf' / 'asv_assem_sim.urdf'
OUTPUT_DIR = PACKAGE_ROOT / 'diagnostics' / 'variants'


STEPS = [
    ('00_base', ['base_footprint', 'base_link']),
    ('01_hull_left', ['hull_left']),
    ('02_hull_right', ['hull_right']),
    ('03_base_deck', ['base_deck']),
    ('04_frame1_left', ['frame1_left']),
    ('05_frame1_right', ['frame1_right']),
    ('06_frame2_left', ['frame2_left']),
    ('07_frame2_right', ['frame2_right']),
    ('08_frame3_left', ['frame3_left']),
    ('09_frame3_right', ['frame3_right']),
    ('10_battery', ['battery']),
    ('11_electronics', ['electronics']),
    ('12_jetson_nano', ['jetson_nano']),
    ('13_imu_link', ['imu_link']),
    ('14_gps', ['gps']),
    ('15_gps_antenna', ['gps_antenna']),
    ('16_zed_camera', ['zed_camera']),
    ('17_clamp_left_front', ['clamp_left_front']),
    ('18_clamp_left_rear', ['clamp_left_rear']),
    ('19_clamp_right_front', ['clamp_right_front']),
    ('20_clamp_right_rear', ['clamp_right_rear']),
    ('21_left_front_mount', ['thruster_left_front_mount']),
    ('22_left_front_steer', ['thruster_left_front_steer']),
    ('23_left_front_thruster', ['thruster_left_front']),
    ('24_left_rear_mount', ['thruster_left_rear_mount']),
    ('25_left_rear_steer', ['thruster_left_rear_steer']),
    ('26_left_rear_thruster', ['thruster_left_rear']),
    ('27_right_front_mount', ['thruster_right_front_mount']),
    ('28_right_front_steer', ['thruster_right_front_steer']),
    ('29_right_front_thruster', ['thruster_right_front']),
    ('30_right_rear_mount', ['thruster_right_rear_mount']),
    ('31_right_rear_steer', ['thruster_right_rear_steer']),
    ('32_right_rear_thruster', ['thruster_right_rear']),
]


def generate() -> None:
    source_root = ET.parse(SOURCE_URDF).getroot()
    source_links = {
        link.attrib['name']: link for link in source_root.findall('link')
    }
    source_joints = source_root.findall('joint')

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    # Copy the working model byte-for-byte: it is a control for rendering,
    # not a claim that the original simplified model matches the full CAD.
    (OUTPUT_DIR / '00_original.urdf').write_bytes(ORIGINAL_URDF.read_bytes())
    selected = set()
    manifest = [
        '00_original: visible reference (original two hulls + four thrusters)',
    ]

    for variant_name, additions in STEPS:
        selected.update(additions)
        selected_links = {'base_footprint', 'base_link'} | selected
        robot = ET.Element('robot', {'name': 'asv_station_keeping_diag'})
        robot.append(ET.Comment(
            ' Mesh-loading test only. Mounting poses are estimates, not '
            'Fusion occurrence transforms. 00_base has no visuals. '))

        for name in source_links:
            if name not in selected_links:
                continue
            link = ET.fromstring(ET.tostring(source_links[name]))
            for mesh in link.findall('.//mesh'):
                filename = mesh.attrib.get('filename', '')
                mesh.attrib['filename'] = filename.replace(
                    'package://asv_assem_sim/',
                    'package://asv_station_keeping/',
                )
            robot.append(link)

        for joint in source_joints:
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            if parent in selected_links and child in selected_links:
                robot.append(ET.fromstring(ET.tostring(joint)))

        ET.indent(robot, space='  ')
        output = OUTPUT_DIR / f'{variant_name}.urdf'
        ET.ElementTree(robot).write(output, encoding='utf-8', xml_declaration=True)
        note = (
            ' (no visuals; no visible robot is expected)'
            if variant_name == '00_base' else ''
        )
        manifest.append(f'{variant_name}: adds {", ".join(additions)}{note}')

    (OUTPUT_DIR / 'README.txt').write_text(
        '00_original is a standalone, visible reference model.\n'
        '00_base is inertial-only: an invisible robot is expected.\n'
        '01 through 32 form a separate cumulative sequence. Their mounting\n'
        'positions are manually estimated, NOT recovered CAD positions.\n'
        'Controllers and sensors are omitted; collisions/inertias remain.\n'
        'Successful rendering tests mesh loading, not CAD assembly accuracy.\n\n'
        + '\n'.join(manifest) + '\n',
        encoding='utf-8',
    )
    print(
        f'Generated {len(STEPS)} variants and a visible original baseline '
        f'in {OUTPUT_DIR}'
    )


if __name__ == '__main__':
    generate()
