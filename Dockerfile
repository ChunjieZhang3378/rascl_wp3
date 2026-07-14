FROM docker.io/ros:jazzy

ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
  apt-get install -y \
  python3-pip python3-venv ros-jazzy-rviz2 ros-jazzy-rqt-common-plugins ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  python3-dev gfortran libopenblas-dev liblapack-dev \
  libserial-dev \
  curl build-essential less htop tree nano vim neovim \
  && \
  rm -rf /var/cache/apk/* && \
  apt-get autoremove -y && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

ENV VIRTUAL_ENV=/opt/rascl_venv
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"
ENV PYTHONPATH="${VIRTUAL_ENV}/lib/python3.12/site-packages:${PYTHONPATH}"

# Install python dependencies
RUN python3 -m venv --system-site-packages ${VIRTUAL_ENV} && \
  pip install --no-cache-dir --upgrade pip && \
  pip install --no-cache-dir \
  pysoem==1.1.12 \
  pytest \
  spatialmath-python \
  roboticstoolbox-python
RUN python -c "import roboticstoolbox, spatialmath; print('Robotics Toolbox installed')"

# Clone SOEM (stable tag or master as needed)
RUN git clone --depth 1 https://github.com/OpenEtherCATsociety/SOEM.git /opt/SOEM \
  && mkdir -p /opt/SOEM/build && cd /opt/SOEM/build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
  && cmake --build . -- -j$(nproc) \
  && cmake --build . --target install \
  && ldconfig

# Setup environment
RUN \
  printf "echo rosbuild - Build all packages\n" >> ~/.bashrc && \
  printf "echo rossetup - Source ROS local setup variable\n" >> ~/.bashrc && \
  printf "echo rosclean - Delete build, install and log\n" >> ~/.bashrc && \
  printf "alias rossetup='cd /root/ws/ && source ~/ws/install/local_setup.bash && ros2 daemon start'\n" >> ~/.bashrc && \
  printf "alias rosbuild='colcon build --symlink-install'\n" >> ~/.bashrc && \
  printf "alias rosclean='rm -r ~/ws/build/ ~/ws/install/ ~/ws/log/'\n" >> ~/.bashrc && \
  printf "source /opt/ros/jazzy/setup.bash\n" >> ~/.bashrc && \
  printf "export PYTHONPATH=/opt/rascl_venv/lib/python3.12/site-packages:\${PYTHONPATH}\n" >> ~/.bashrc && \
  printf "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-16}\n" >> ~/.bashrc && \
  printf "PS1='\[\e[32m\]rascl-container\[\e[0m\]:\[\e[34m\]\w\[\e[0m\]$ '\n" >> ~/.bashrc

WORKDIR /root/ws

CMD ["/bin/bash"]
