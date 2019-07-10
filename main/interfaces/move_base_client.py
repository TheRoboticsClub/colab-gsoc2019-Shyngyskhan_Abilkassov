import rospy
import actionlib
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveBaseClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = None
        self.isFinished = False

    def send_goal_to_client(self, posX, posY):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = posX
        self.goal.target_pose.pose.position.y = posY

        orientation_q = quaternion_from_euler(0, 0, 1.57)

        self.goal.target_pose.pose.orientation.x = orientation_q[0]
        self.goal.target_pose.pose.orientation.y = orientation_q[1]
        self.goal.target_pose.pose.orientation.z = orientation_q[2]
        self.goal.target_pose.pose.orientation.w = orientation_q[3]

        self.client.send_goal(self.goal)
        # print("Goal Sent")
        self.isFinished = self.client.wait_for_result(rospy.Duration(1))

    def get_result_from_client(self):
        if (self.goal):
            return self.client.get_result()
        else:
            return None