## Build with
## docker buildx build -t local/deepracer-analysis:ros-tf -f Dockerfile .
## Run with
## docker run -ti -p 8888:8888 -v `pwd`:/workspace/analysis -v <logsdir>:/workspace/logs -v <modelsdir>:/workspace/models local/deepracer-analysis:ros-tf

FROM ros:humble-ros-base

LABEL maintainer="AWS Deepracer Community" \
	  description="Log Analysis for DeepRacer with ROS and TF" \
	  version=1.0

# Container Dependency Setup
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    python3-pip \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-cv-bridge \
    ros-humble-pybind11-vendor \
    ros-humble-test-msgs \
    libjsoncpp-dev \
    v4l-utils \
    ffmpeg \
    libx264-dev \
    sudo \
    curl && \
	apt-get clean && rm -rf /var/lib/apt/lists/*

# Install tensorflow and visualization
RUN pip3 install -U --no-cache-dir 'tensorflow-cpu<2.17' \
                               python-resize-image \
                               'deepracer-utils>=1.0.3' \
                               'jupyterlab>=2.0.0' \ 
                               'jupytext>=1.4.0' \
                               'ipympl' \
                               pillow \
                               pandas \
                               nbconvert \
                               ipykernel \
                               'numpy<2' \
                               python-dateutil \
                               tqdm

RUN mkdir -p /workspace/analysis /workspace/src /workspace/logs /workspace/models /workspace/.tmp/matplotlib && chmod o+rw /workspace/.tmp/matplotlib
ENV MPLCONFIGDIR=/workspace/.tmp/matplotlib

RUN cd /workspace/src && \
    git clone -b logging https://github.com/larsll/deepracer-viz && \
    git clone -b compressed-image https://github.com/larsll/aws-deepracer-interfaces-pkg && \
    bash -c 'cd /workspace && source /opt/ros/humble/setup.bash && colcon build --merge-install && rm -rf build && rm -rf log ' 

ADD . /workspace/analysis

EXPOSE 8888
WORKDIR /workspace/analysis
ENV PATH="$PATH:/workspace/analysis"
VOLUME ["/workspace/logs", "/workspace/models"]
ENTRYPOINT [ "/workspace/analysis/entrypoint.sh" ]
CMD bash -c 'jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --no-browser'
