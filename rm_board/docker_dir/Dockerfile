FROM ros:melodic

ENV WORKSPACE /catkin_ws/jaxon_tutorials

RUN echo "now building..."

RUN apt update && apt install -y  ros-melodic-jsk-pcl-ros ros-melodic-roseus
RUN mkdir -p catkin_ws/jaxon_tutorials/src

RUN apt-get update && apt-get install --no-install-recommends -y \
  python-catkin-tools
  
WORKDIR ${WORKSPACE}
RUN git clone https://github.com/Utaro-M/rm_debris.git ${WORKSPACE}/src/rm_debris -b emergency
RUN git clone https://github.com/Utaro-M/rm_debris_vision.git ${WORKSPACE}/src/rm_debris_vision
RUN git clone https://github.com/kosuke55/topic_publisher.git ${WORKSPACE}/src/topic_publisher
RUN pwd
RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN pwd
RUN source /opt/ros/melodic/setup.bash; catkin build rm_debris_vision topic_publisher
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh


