FROM ros:melodic

WORKDIR /home

RUN echo "now building..."

RUN apt update && apt install -y  ros-melodic-jsk-pcl-ros 
RUN mkdir -p catkin_ws/jaxon_tutorials/src

RUN cd catkin_ws/jaxon_tutorials/src && git clone https://github.com/Utaro-M/rm_debris_vision.git
RUN sudo apt install -y python-catkin-tools 
RUN source /opt/ros/melodic/setup.bash && source ~/catkin_ws/jaxon_tutorials/devel/setup.bash && catkin build rm_debris_vision

RUN mkdir research && cd research && git clone https://github.com/Utaro-M/rm_debris.git
RUN cd ~/research/rm_debris/rm_board/ && git checkout emergency
