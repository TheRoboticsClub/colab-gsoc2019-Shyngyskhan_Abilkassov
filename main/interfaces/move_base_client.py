import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(posX, posY):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = posX
    goal.target_pose.pose.position.y = posY
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    print("Goal Sent")
    # wait = client.wait_for_result()
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    #     return client.get_result()