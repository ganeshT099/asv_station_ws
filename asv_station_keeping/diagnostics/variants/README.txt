00_original is a standalone, visible reference model.
00_base is inertial-only: an invisible robot is expected.
01 through 32 form a separate cumulative sequence. Their mounting
positions are manually estimated, NOT recovered CAD positions.
Controllers and sensors are omitted; collisions/inertias remain.
Successful rendering tests mesh loading, not CAD assembly accuracy.

00_original: visible reference (original two hulls + four thrusters)
00_base: adds base_footprint, base_link (no visuals; no visible robot is expected)
01_hull_left: adds hull_left
02_hull_right: adds hull_right
03_base_deck: adds base_deck
04_frame1_left: adds frame1_left
05_frame1_right: adds frame1_right
06_frame2_left: adds frame2_left
07_frame2_right: adds frame2_right
08_frame3_left: adds frame3_left
09_frame3_right: adds frame3_right
10_battery: adds battery
11_electronics: adds electronics
12_jetson_nano: adds jetson_nano
13_imu_link: adds imu_link
14_gps: adds gps
15_gps_antenna: adds gps_antenna
16_zed_camera: adds zed_camera
17_clamp_left_front: adds clamp_left_front
18_clamp_left_rear: adds clamp_left_rear
19_clamp_right_front: adds clamp_right_front
20_clamp_right_rear: adds clamp_right_rear
21_left_front_mount: adds thruster_left_front_mount
22_left_front_steer: adds thruster_left_front_steer
23_left_front_thruster: adds thruster_left_front
24_left_rear_mount: adds thruster_left_rear_mount
25_left_rear_steer: adds thruster_left_rear_steer
26_left_rear_thruster: adds thruster_left_rear
27_right_front_mount: adds thruster_right_front_mount
28_right_front_steer: adds thruster_right_front_steer
29_right_front_thruster: adds thruster_right_front
30_right_rear_mount: adds thruster_right_rear_mount
31_right_rear_steer: adds thruster_right_rear_steer
32_right_rear_thruster: adds thruster_right_rear
