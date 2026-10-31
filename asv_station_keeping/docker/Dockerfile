# Reproducible environment for the ASV station-keeping simulation.
# Build from the workspace root (the folder that contains src/):
#   docker build -f src/asv_station_keeping/docker/Dockerfile -t asv-sim .
# Run headless experiments:
#   docker run --rm -v $PWD/results:/results asv-sim \
#     python3 /ws/src/asv_station_keeping/tools/run_experiments.py --out /results
# Run with a GUI (X11):
#   xhost +local:docker
#   docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#     --device /dev/dri asv-sim ros2 launch asv_station_keeping \
#     visual_station_keeping.launch.py start_rviz:=false open_dashboard:=false
# NOTE: this file has not been built in the project's CI; report problems.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
      ros-humble-robot-state-publisher ros-humble-rviz2 \
      python3-matplotlib python3-numpy python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/asv_station_keeping src/asv_station_keeping
COPY src/asv_station_gz_bridge src/asv_station_gz_bridge
RUN . /opt/ros/humble/setup.sh \
 && colcon build --symlink-install \
      --packages-select asv_station_keeping asv_station_gz_bridge

COPY src/asv_station_keeping/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
