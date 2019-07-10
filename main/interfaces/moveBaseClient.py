#
#  Copyright (C) 1997-2016 JDE Developers Team
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
#       Shyngyskhan Abilkassov <s.abilkassov@gmail.com>
#

import rospy
import actionlib
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import threading
from .threadGoalSender import ThreadGoalSender

class MoveBaseClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.data = [0, 0] # 
        self.goal = None
        self.isFinished = False
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadGoalSender(self)

        self.thread.daemon = True
        self.start()

    def stop(self):
        self.kill_event.set()
        self.client.cancel_all_goals()

    def start(self):
        self.kill_event.clear()
        self.thread.start()

    def publishGoalToClient(self):
        self.lock.acquire()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = self.data[0]
        self.goal.target_pose.pose.position.y = self.data[1]

        orientation_q = quaternion_from_euler(0, 0, 1.57)

        self.goal.target_pose.pose.orientation.x = orientation_q[0]
        self.goal.target_pose.pose.orientation.y = orientation_q[1]
        self.goal.target_pose.pose.orientation.z = orientation_q[2]
        self.goal.target_pose.pose.orientation.w = orientation_q[3]

        self.client.send_goal(self.goal)
        # print("Goal Sent")
        self.isFinished = self.client.wait_for_result(rospy.Duration(1))
        self.lock.release()

    def sendGoalToClient(self, posX, posY):
        self.lock.acquire()
        self.data = [posX, posY]
        self.lock.release()

    def getResultFromClient(self):
        if (self.goal):
            return self.client.get_result()
        else:
            return None