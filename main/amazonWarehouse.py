#!/usr/bin/python3
# -*- coding: utf-8 -*-

#
#  Copyright (C) 1997-2015 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Samuel Rey Escudero <samuel.rey.escudero@gmail.com>
#       Alberto Martin Florido <almartinflorido@gmail.com>
#

import sys
from PyQt5.QtWidgets import QApplication

from MyAlgorithm import MyAlgorithm
from gui.threadGUI import ThreadGUI
from gui.GUI import MainWindow
from sensors.threadMotors import ThreadMotors
from sensors.threadMotors import Velocity

from sensors.sensor import Sensor
from sensors.grid import Grid

from interfaces.motors import PublisherMotors		
from interfaces.pose3d import ListenerPose3d
from interfaces.path import ListenerPath
# from interfaces.goal import PublisherGoal
from interfaces.moveBaseClient import MoveBaseClient
from interfaces.threadGoalSender import ThreadGoalSender

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

def removeMapFromArgs():
    for arg in sys.argv:
        if (arg.split(".")[1] == "conf"):
            sys.argv.remove(arg)

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('ERROR: python2 globalNavigation.py [MAP CONFIG file] [YAML CONFIG file]')
        sys.exit(-1)

    app = QApplication(sys.argv) 
    myGUI = MainWindow()

    grid = Grid(myGUI)
    
    motors = PublisherMotors("/amazon_warehouse_robot/teleop", 0.5, 0.25)
    pose = ListenerPose3d("/amazon_warehouse_robot/odom")

    pathListener = ListenerPath("/amazon_warehouse_robot/move_base/NavfnROS/plan")
    moveBaseClient = MoveBaseClient()

    vel = Velocity(0, 0, motors.getMaxV(), motors.getMaxW())
    sensor = Sensor(grid, pose, True)
    sensor.setGetPathSignal(myGUI.getPathSig)
    
    myGUI.setVelocity(vel)
    myGUI.setGrid(grid)
    myGUI.setSensor(sensor)
    algorithm = MyAlgorithm(grid, sensor, vel, pathListener, moveBaseClient)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()

    removeMapFromArgs()

    t1 = ThreadMotors(motors, vel)
    t1.daemon = True
    t1.start()
    t2 = ThreadGUI(myGUI)  
    t2.daemon = True
    t2.start()
    # t3 = ThreadGoalSender(moveBaseClient)
    # t3.daemon = True
    # t3.start()
    
    sys.exit(app.exec_()) 