# amazonwarehouse

Running current implementation steps:

1. It is currently needed to create your own package in your catkin workspace called "amazon_navigation". Copy the contents of catkin_files there. You should have config, launch, and maps folders.

2. Source that workspace.
Additionally, do following if required. 
```
source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

3. Run ```roslaunch amazonrobot_1_warehouse.launch```

4. Run ```python2 amazonWarehouse.py amazonMap.conf```
