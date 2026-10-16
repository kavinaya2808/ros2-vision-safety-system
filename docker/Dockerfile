# Development image: ROS 2 Jazzy + Gazebo Harmonic + everything this project needs.
# Works on any Linux host (the host can run a different Ubuntu / ROS version).
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      # Gazebo Harmonic + ROS bridge
      ros-jazzy-ros-gz \
      # Robot description
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      # Perception
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-rqt-image-view \
      libopencv-dev \
      # Control / diagnostics
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-diagnostic-updater \
      # Testing
      ros-jazzy-ament-cmake-gtest \
      ros-jazzy-launch-testing-ament-cmake \
      python3-pytest \
      # Dev utilities
      gdb \
      mesa-utils \
      sudo \
      nano \
    && rm -rf /var/lib/apt/lists/*

# Run as a user with the HOST user's UID/GID, so files written into the mounted
# workspace belong to you on the host (not to root). Values come from docker/.env.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN existing="$(getent passwd ${USER_UID} | cut -d: -f1)"; \
    if [ -n "$existing" ] && [ "$existing" != "${USERNAME}" ]; then userdel -r "$existing"; fi; \
    (getent group ${USER_GID} || groupadd --gid ${USER_GID} ${USERNAME}) \
    && (id -u ${USERNAME} >/dev/null 2>&1 || useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}) \
    && usermod -aG video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash' >> ~/.bashrc \
    && echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> ~/.bashrc

CMD ["sleep", "infinity"]
