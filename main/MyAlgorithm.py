import numpy as np
import cv2
import math

import sys

import threading
import time
from datetime import datetime
import yaml
import rospy
from std_msgs.msg import Float32

# from interfaces.moveBaseClient import MoveBaseClient

time_cycle = 80


def clearscreen(numlines=10):
    """Clear the console.
    numlines is an optional argument used only as a fall-back.
    """
    import os
    if os.name == "posix":
        # Unix/Linux/MacOS/BSD/etc
        os.system('clear')
    elif os.name in ("nt", "dos", "ce"):
        # DOS/Windows
        os.system('CLS')
    else:
        # Fallback for other operating systems.
        print '\n' * numlines


class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel, pathListener, moveBaseClient):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        self.path = pathListener
        sensor.getPathSig.connect(self.sendGoal)
        self.palettesList = yaml.load(open('./palettes_coords.yaml'))["coords"]
        self.jointForce = 0
        self.pub = rospy.Publisher('amazon_warehouse_robot/joint_cmd', Float32, queue_size=10)
        self.client = moveBaseClient

        self.isDelivered = False
        self.isFinished = False

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def run (self):

        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)
            self.dt = ms/1000

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Send Goal button.
        Call to grid.setPath(path) method for setting the path. """
    def sendGoal(self, list):
        print("LOOKING FOR SHORTER PATH")
        dest = self.grid.getDestiny()
        # validDest = self.destToValidLoc(dest[0], dest[1])
        # validDest = [24, 151] for new palete
        validDest = [200, 155]
        pose = self.grid.getPose() 

        if ((pose[0] == validDest[0])) and ((pose[1] == validDest[1])):
            print("Not valid dest, remaining at rest")
        else:
            destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
            self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
            self.drawPath()

    def drawPath(self):
        pathArray = self.path.getPath()

        pathlist = [[] for i in range(2)]
        num = 0
        pathX_old = -1
        pathY_old = -1

        for i in range(len(pathArray)):
            pathX = pathArray[i].pose.position.x
            pathY = pathArray[i].pose.position.y

            if pathX != pathX_old or pathY != pathY_old:
                tmp = self.grid.worldToGrid(pathX, pathY)
                self.grid.setPathVal(int(tmp[0]), int(tmp[1]), num)

                pathlist[0].append(pathX)
                pathlist[1].append(pathY)
                pathX_old = pathX
                pathY_old = pathY
                num += 1

        self.grid.setPathFinded()

        npPathList = np.array(pathlist)
        self.grid.setWorldPathArray(npPathList)


    def liftDropExecute(self):
        if self.jointForce != 25:
            self.jointForce = 25
            self.pub.publish(self.jointForce)
            print ('Platform Lifted!')
        else:
            self.jointForce = 0
            self.pub.publish(self.jointForce)
            print ('Platform Dropped!')

    def destToValidLoc(self, x, y):
        gridPos = self.grid.getPose()

        if ((x > 310) and (y > 125) and (y < 185)):
            # print("Going to pick-up room")
            return x, y
        elif ((y > 255) and (x < 315) and (x > 85)):
            # print("Going to charging point")
            return x, y
        else:
            for coordinate in self.palettesList:
                if (abs(x - coordinate['x']) < 10):
                    if (abs(y - coordinate['y']) < 10):
                        # print("Closest palette: ", coordinate['x'], ", ", coordinate['y'])
                        # print("Approximating to closest palette...")
                        return coordinate['x'], coordinate['y']
            return gridPos[0], gridPos[1]

    """ Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
        # print("Starting")
        # dest = self.grid.getDestiny()
        pose = self.grid.getPose()
        # validDest = self.destToValidLoc(dest[0], dest[1])
        validDest = [24, 151]

        # STANDARD PROCEDURE. IT IS EXPLICITLY DEFINED IN BLOG

        print(self.client.isFinished)

        # if ((self.client.getResultFromClient() != None) and (self.isDelivered == False) and (self.isFinished == False)):
        
        #     if ((pose[0] - validDest[0]) < 2) and ((pose[1] - validDest[1]) < 2):
        #         print("Reached palette, lifting it, and going to drop point")
        #         self.liftDropExecute()
        #         destInWorld = self.grid.gridToWorld(364, 175)
        #         self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
        #         self.drawPath()
        #         self.isDelivered = True
        #     else:
        #         print ("Reached given point")

        # if ((self.client.getResultFromClient() != None) and (self.isDelivered == True) and (self.isFinished == False)):
        #     print("Reached drop point, leaving palette, and going to charging point")

        #     self.liftDropExecute()

        #     destInWorld = self.grid.gridToWorld(230, 265)
        #     self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
        #     self.drawPath()

        #     self.isFinished = True


            # rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer inflation_radius 3



    # REACH PALETTE AND SEND IT TO THE PARKING PLACE
    # if ((self.client.getResultFromClient() != None) and (self.isDelivered == True) and (self.isFinished == False)):
    # def destToValidLoc(self, x, y):
    #     gridPos = self.grid.getPose()

    #     if ((x > 310) and (y > 125) and (y < 185)):
    #         print("Going to pick-up room")
    #         self.isPaletteChosen = False
    #         return x, y
    #     elif ((y > 255) and (x < 315) and (x > 85)):
    #         print("Going to charging point")
    #         self.isPaletteChosen = False
    #         return x, y
    #     else:
    #         for coordinate in self.palettesList:
    #             if (abs(x - coordinate['x']) < 10):
    #                 if (abs(y - coordinate['y']) < 10):
    #                     print("Closest palette: ", coordinate['x'], ", ", coordinate['y'])
    #                     print("Approximating to closest palette...")
    #                     self.isPaletteChosen = True
    #                     return coordinate['x'], coordinate['y']
    #         print("Not valid dest, remaining at rest")
    #         self.isPaletteChosen = False
    #         return gridPos[0], gridPos[1]

    # """ Write in this method the code necessary for going to the desired place,
    #     once you have generated the shorter path.
    #     This method will be periodically called after you press the GO! button. """
    # def execute(self):
    #     # print("Starting")      
    #     if (self.grid.getDestiny()):

    #         if (self.isPaletteChosen == False):
    #             dest = self.grid.getDestiny()
    #             validDest = self.destToValidLoc(dest[0], dest[1])
    #             destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
    #             # self.goal.setPose(destInWorld[0], destInWorld[1])
    #             self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
    #             self.drawPath()

    #         if ((self.isDelivered) and (self.isFinished)):
    #             self.grid.resetPath()
    #             print("Task finished")

    #         elif ((self.client.getResultFromClient() != None) and (self.isDelivered == False) and (self.isFinished == False)):
    #             print("Reached palette, lifting it, and going to drop point")

    #             self.liftDropExecute()

    #             destInWorld = self.grid.gridToWorld(355, 150)
    #             self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
    #             self.drawPath()

    #             self.isDelivered = True

    #         elif ((self.client.getResultFromClient() != None) and (self.isDelivered == True) and (self.isFinished == False)):
    #             print("Reached drop point, leaving palette, and going to charging point")

    #             self.liftDropExecute()

    #             destInWorld = self.grid.gridToWorld(230, 260)
    #             self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
    #             self.drawPath()

    #             self.isFinished = True