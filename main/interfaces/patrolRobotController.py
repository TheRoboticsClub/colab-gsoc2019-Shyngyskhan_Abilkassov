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

import rospy
from std_srvs.srv import Empty

## Use this if after moving pallet, there are obstacles left on costmaps, which prevent proper movement after moving and dropping pallet
def clearCostmaps():
    rospy.wait_for_service('/amazon_warehouse_robot/move_base/clear_costmaps')
    clear_costmaps = rospy.ServiceProxy('/amazon_warehouse_robot/move_base/clear_costmaps', Empty)

    try:
        print("Clearing costmap")
        clear_costmaps()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

class PatrolRobot():
    def __init__(self):
        rospy.init_node("patrolRobot")
        self.client = actionlib.SimpleActionClient('/amazon_warehouse_robot_patrol/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.data = [0, 0]
        self.goal = None
        self.points = [[3.23, -8.2], [-3.23, -8.2], [-3.23, 8.2], [3.23, 8.2]]
        self.currentPoint = 0 # or 1 2 3
        self.isFinished = True
        self.patrolAborted = False
        self.goalCompleted = False

    def sendNextGoal(self):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # if (not self.patrolAborted and self.isFinished):
        pose = self.points[0]
            
        self.goal.target_pose.pose.position.x = pose[0]
        self.goal.target_pose.pose.position.y = pose[1]

        orientation_q = quaternion_from_euler(0, 0, 3.14)

        self.goal.target_pose.pose.orientation.x = orientation_q[0]
        self.goal.target_pose.pose.orientation.y = orientation_q[1]
        self.goal.target_pose.pose.orientation.z = orientation_q[2]
        self.goal.target_pose.pose.orientation.w = orientation_q[3]

        self.client.send_goal(self.goal)

        # self.goalCompleted = self.client.wait_for_result(rospy.Duration(0.5))

        # if (self.isFinished and self.goalCompleted):
        if (self.currentPoint < 4):
            self.currentPoint = self.currentPoint + 1
        else:
            self.currentPoint = 0

    def startPatrol(self):
        while(True):
            self.sendNextGoal()
            self.client.wait_for_result()
    
    def abortPatrol(self):
        self.client.cancel_all_goals()
        self.patrolAborted = True
        
    def getResultFromClient(self):
        if (self.goal):
            return self.client.get_result()
        else:
            return None


if __name__ == '__main__':
    robot = PatrolRobot()
    robot.startPatrol()