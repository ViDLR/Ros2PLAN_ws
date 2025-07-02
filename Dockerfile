FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ROS 2
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    libgraphviz-dev \          
    pkg-config \
    && rm -rf /var/lib/apt/lists/*


SHELL ["/bin/bash", "-c"]
    
WORKDIR /root/PHD/Ros2PLAN_ws

#SRC copies, exec
COPY ./src ./src
COPY dockerentry.sh /dockerentry.sh
RUN chmod +x /dockerentry.sh

RUN source /opt/ros/humble/setup.bash && colcon build

ENTRYPOINT ["/dockerentry.sh"]
