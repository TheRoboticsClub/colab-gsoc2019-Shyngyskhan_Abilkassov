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
from interfaces.moveBaseClient import clearCostmaps

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

        self.palletInGuiChosen = False

        self.pickNewPalletPressed = False
        self.pickOldPalletPressed = False
        self.storeNewPalletExecuted = False
        self.storeOldPalletExecuted = False

        self.executingTask = False
        self.taskCompleted = False

        # self.pickOldPalletExecuted = False
        self.isFinished = False

        self.pickedPallet = False

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
        print("EXECUTING GOAL")
        self.pickOldPalletPressed = True
        # self.isSendGoalPressed = True
        # dest = self.grid.getDestiny()
        # # validDest = self.destToValidLoc(dest[0], dest[1])
        # validDest = [200, 155]
        # pose = self.grid.getPose() 

        # if ((pose[0] == validDest[0])) and ((pose[1] == validDest[1])):
        #     print("Not valid dest, remaining at rest")
        # else:
        #     destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
        #     self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
        #     self.drawPath()

    def drawPath(self):
        pathArray = self.path.getPath()

        while (len(pathArray) < 10):
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
            print("Going to pick-up room")
            self.palletInGuiChosen = False
            return x, y
        elif ((y > 255) and (x < 315) and (x > 85)):
            print("Going to charging point")
            self.palletInGuiChosen = False
            return x, y
        else:
            for coordinate in self.palettesList:
                if (abs(x - coordinate['x']) < 10):
                    if (abs(y - coordinate['y']) < 10):
                        # print("Closest palette: ", coordinate['x'], ", ", coordinate['y'])
                        print("Approximating to closest palette...")
                        self.palletInGuiChosen = True
                        self.pickOldPalletExecuted = False
                        return coordinate['x'], coordinate['y']
            print("Remaiing at rest")
            self.palletInGuiChosen = False
            return gridPos[0], gridPos[1]

    def setNewPalletFlag(self, isPressed):
        self.pickNewPalletPressed = isPressed

    """ Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
        goalAchieved = self.client.client.wait_for_result(rospy.Duration(0.5))
        pose = self.grid.getPose()
    
        print ('Isexecuting: ' + str(self.executingTask))
        print ('Button: ' + str(self.pickNewPalletPressed))
        print ('Pose: ' + str(pose))
        print ('goalAchieved: ' + str(goalAchieved) + "\n")

        if goalAchieved:
            self.executingTask = False

        if not self.executingTask:
            # ## Picking a pallet
            # if (self.palletInGuiChosen and self.storeOldPalletExecuted):
            #     dest = self.grid.getDestiny()
            #     validDest = self.destToValidLoc(dest[0], dest[1])
                
            #     self.palletInGuiChosen = False
            #     if (not self.pickedPallet) and (abs(pose[0] - validDest[0]) < 2) and (abs(pose[1] - validDest[1]) < 2):
            #         ## Reached old pallet. Going from old pallet to dest
            #         print("Reached old pallet")
            #         self.liftDropExecute()
            #         destInWorld = self.grid.gridToWorld(380, 175)
            #         self.client.sendGoalToClient(destInWorld[0], destInWorld[1], yaw = 3.14)
            #         self.drawPath()
            #         self.executingTask = True
            #         self.storeOldPalletExecuted = True
            #         self.pickedPallet = True
            #     elif (self.pickedPallet) and (abs(pose[0] - 380) < 2) and (abs(pose[1] - 175) < 2):
            #         ## Dropping old pallet at dest
            #         print("Dropped old pallet")
            #         self.liftDropExecute()
            #         clearCostmaps()
            #         self.executingTask = False
            #         self.taskCompleted = True
            #         self.pickedPallet = False
            #         self.pickOldPalletExecuted = True

            # if (not self.pickOldPalletExecuted and self.grid.getDestiny()):
            #     print("got destiny")
            #     dest = self.grid.getDestiny()
            #     validDest = self.destToValidLoc(dest[0], dest[1])
            #     destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])

            #     if (not self.palletInGuiChosen):
            #         ## Simply going to location marked
            #         self.executingTask = True
            #         self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
            #         self.drawPath()
            #     else:
            #         ## Going to pick old pallet
            #         self.executingTask = True
            #         self.storeOldPalletExecuted = True
            #         self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
            #         self.drawPath()

            ## New pallet pick behavior
            if self.storeNewPalletExecuted:
                if (abs(pose[0] - 24) < 2) and (abs(pose[1] - 151) < 2):
                    ## Reached new pallet. Going from new pallet to dest
                    self.liftDropExecute()
                    validDest = [200, 41]
                    self.executingTask = True
                    destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
                    self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
                    self.drawPath()    
                elif (abs(pose[0] - 200) < 2) and (abs(pose[1] - 41) < 2):
                    ## Dropping new pallet at dest
                    self.liftDropExecute()
                    ## Important
                    clearCostmaps()
                    self.storeNewPalletExecuted = False
                    self.executingTask = False
                    self.taskCompleted = True

            elif (self.pickNewPalletPressed):
                ## Going to new pallet
                validDest =  [24, 151] 
                self.executingTask = True
                destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
                self.client.sendGoalToClient(destInWorld[0], destInWorld[1], yaw = 1.57)
                self.drawPath()    
                self.pickNewPalletPressed = False
                self.storeNewPalletExecuted = True

            ## Old pallet pick behavior
            if self.storeOldPalletExecuted:
                dest = self.grid.getDestiny()
                validDest = self.destToValidLoc(dest[0], dest[1])
                if (abs(pose[0] - validDest[0]) < 2) and (abs(pose[1] - validDest[1]) < 2):
                    ## Reached old pallet. Going from old pallet to dest
                    self.liftDropExecute()
                    
                    # clearCostmaps()

                    validDest = [380, 175]
                    self.executingTask = True
                    destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
                    self.client.sendGoalToClient(destInWorld[0], destInWorld[1], yaw = 3.14)
                    self.drawPath()    
                elif (abs(pose[0] - 380) < 2) and (abs(pose[1] - 175) < 2):
                    ## Dropping old pallet at dest
                    self.liftDropExecute()
                    
                    self.executingTask = False
                    self.taskCompleted = True
                    self.storeOldPalletExecuted = False

            elif (self.pickOldPalletPressed):
                ## Going to old pallet
                dest = self.grid.getDestiny()
                validDest = self.destToValidLoc(dest[0], dest[1])
                
                self.executingTask = True
                destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
                self.client.sendGoalToClient(destInWorld[0], destInWorld[1])
                self.drawPath()    
                self.pickOldPalletPressed = False
                self.storeOldPalletExecuted = True

            if (self.taskCompleted):
                ## going to chraging point
                validDest = [200, 265]
                if (abs(pose[0] - validDest[0]) < 2) and (abs(pose[1] - validDest[1]) < 2):
                    print("Reached point")
                    self.executingTask = False
                    self.taskCompleted = True
                    ## Reset path
                    self.grid.resetPath()
                else:
                    print("Going to charging point")
                    self.executingTask = True
                    destInWorld = self.grid.gridToWorld(validDest[0], validDest[1])
                    self.client.sendGoalToClient(destInWorld[0], destInWorld[1], yaw = 3.14)
                    self.drawPath()
                    ## Important
                    # 
                    clearCostmaps()
                    self.taskCompleted = False