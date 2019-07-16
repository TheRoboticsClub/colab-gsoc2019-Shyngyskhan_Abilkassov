# amazonwarehouse

To replicate current implementation follow these steps:

1. Replace ```amazonrobot_1_warehouse.world``` in your root folder of JdeRobot with the file in amazon_warehouse_robot folder in /main.

2. It is currently needed to create your own package in your catkin workspace called "amazon_navigation". Copy the contents of catkin_files there. You should have config, launch, and maps folders.

2. Source that workspace.
Additionally, do following if required. 
```
source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

3. Run ```roslaunch amazonrobot_1_warehouse.launch```

4. Run ```python2 amazonWarehouse.py amazonMap.conf```
