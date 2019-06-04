# amazonwarehouse

source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh

python2 NavigationOmpl.py amazonMap.conf 

rosrun gmapping slam_gmapping scan:=scan_filtered _base_frame:=amazon_warehouse_robot/base _odom_frame:=amazon_warehouse_robot/base

rosrun tf view_frames

http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/