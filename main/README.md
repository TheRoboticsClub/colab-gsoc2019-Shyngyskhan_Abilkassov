# amazonwarehouse

source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh

python2 NavigationOmpl.py amazonMap.conf 

rosrun gmapping slam_gmapping scan:=amazon_warehouse_robot/laser/scan _base_frame:=amazon_warehouse_robot/base _maxRange:=11.0 _maxUrange:=9.0  _linearUpdate:=0.0 _angularUpdate:=0.0

rosrun tf view_frames

http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/

 _odom_frame:=amazon_warehouse_robot/odom 


rosrun map_server map_server map.yaml
