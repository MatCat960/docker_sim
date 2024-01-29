# Use the official ROS "humble-desktop-full" image
FROM ros:noetic

ARG USERNAME=user
ARG DEBIAN_FRONTEND=noninteractive

# Set the working directory
# WORKDIR /home/$USERNAME

# Set the "HOME" environment variable
ENV HOME=/home/$USERNAME

# Set root password
RUN echo 'root:root' | chpasswd

# Install some basic dependencies
RUN apt-get update && apt-get -y upgrade && apt-get -y install \
  curl ssh python3-pip git vim cmake libeigen3-dev\
  && rm -rf /var/lib/apt/lists/*

# Permit SSH root login
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config


RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list


# Install catkin tools
RUN apt-get update && apt-get install -y python3-catkin-tools tmux libtf2-ros-dev libsfml-dev\
  && rm -rf /var/lib/apt/lists/*

# Create symlink for Eigen folder
RUN cd /usr/include ; ln -sf eigen3/Eigen Eigen

RUN apt update && apt install -y ros-noetic-tf2*

# Install osqp
COPY osqp $HOME/osqp
RUN cd $HOME/osqp && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install


# Install osqp-eigen
RUN cd $HOME && git clone https://github.com/robotology/osqp-eigen.git
RUN cd $HOME/osqp-eigen && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/osqp-eigen ../ && make && make install
# RUN mkdir build
# RUN cd build
# RUN cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/osqp-eigen/build ../
# RUN make
# RUN make install

RUN echo "export OsqpEigen_DIR=$HOME/osqp-eigen" > /etc/bash.bashrc

# Get packages for building
WORKDIR /catkin_ws
RUN mkdir src
RUN catkin config --extend /opt/ros/noetic && catkin build --no-status

# Get current directory
RUN echo $PWD

# Clone your ROS packages into the workspace (replace <your_repo_url> with the actual URL)
COPY packages/coverage_unimore_nyu src/coverage_unimore_nyu
COPY packages/flightmare src/flightmare
COPY packages/flightmare_coverage src/flightmare_coverage
COPY packages/fow_control src/fow_control
COPY packages/safety_control src/safety_control
COPY packages/torch_pf src/torch_pf

RUN apt-get install ros-noetic-roscpp

# Build the workspace
RUN apt-get update \
  # && rosdep update \
  # && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/*

# Set the environment variables
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_IP=127.0.0.1

# Automatically source the workspace when starting a bash session
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc
RUN catkin build

RUN echo "--- build complete ---"

# Set the default command to run when the container starts
CMD ["/bin/bash"]
