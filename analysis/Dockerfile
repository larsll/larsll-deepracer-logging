## Run with
## docker run -ti -p 8888:8888 -v `pwd`:/workspace/analysis -v <logsdir>:/workspace/logs <imagename>

FROM ros:foxy-ros-base

LABEL maintainer="AWS Deepracer Community" \
	  description="Log Analysis for DeepRacer with ROS and TF" \
	  version=1.0

# Container Dependency Setup
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    python3-pip \
    python3-opencv \
    ros-foxy-tango-icons-vendor \
    ros-foxy-image-transport \
    ros-foxy-compressed-image-transport \
    ros-foxy-imu-tools \
    ros-foxy-cv-bridge \
    ros-foxy-pybind11-vendor \
    ros-foxy-test-msgs \
    libjsoncpp-dev \
    v4l-utils \
    ffmpeg \
    sudo \
    curl && \
	apt-get clean && rm -rf /var/lib/apt/lists/*

# Install tensorflow and visualization
RUN pip3 install -U --no-cache-dir tensorflow \
                               python-resize-image \
                               'deepracer-utils>=1.0.3' \
                               'jupyterlab>=2.0.0' \ 
                               'jupytext>=1.4.0' \
                               'ipympl' \
                               pillow \
                               pandas \
                               nbconvert \
                               ipykernel \
                               numpy \
                               python-dateutil \
                               tqdm

WORKDIR /
RUN mkdir -p /workspace/analysis /workspace/src /workspace/logs

RUN cd /workspace/src && \
    git clone -b v2 https://github.com/jochem725/deepracer-viz && \
    git clone -b compressed-image https://github.com/larsll/aws-deepracer-interfaces-pkg && \
    git clone -b foxy-future https://github.com/ros2/rosbag2 && \
    bash -c 'cd /workspace && source /opt/ros/foxy/setup.bash && colcon build --merge-install && rm -rf build && rm -rf log ' 

EXPOSE 8888
VOLUME ["/workspace/analysis", "/root/.aws", "/workspace/logs"]
CMD bash -c 'source /opt/ros/foxy/setup.bash && source /workspace/install/setup.bash && cd /workspace && jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --no-browser'
