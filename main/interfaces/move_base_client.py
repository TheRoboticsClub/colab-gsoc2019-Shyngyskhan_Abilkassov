import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveBaseClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = None

    def send_goal_to_client(self, posX, posY):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = posX
        self.goal.target_pose.pose.position.y = posY
        self.goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(self.goal)
        print("Goal Sent")
        self.client.wait_for_result(rospy.Duration(1))

    def get_result_from_client(self):
        if (self.goal):
            return self.client.get_result()
        else:
            return None