import rospy
from geometry_msgs.msg import PoseStamped
import threading

class PublisherGoal:
    '''
        ROS Goal Publisher. Path Client to Send goal from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        PublisherGoal Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = []
        self.pub = None
        self.sub = None
        self.msg = PoseStamped()
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, pose):
        '''
        Callback function to receive and save PoseStamped. 

        @param pose: ROS PoseStamped received
        
        @type pose: PoseStamped

        '''
        self.lock.acquire()
        self.data = pose
        self.lock.release()
        
    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.sub.unregister()
        self.pub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.sub = rospy.Subscriber(self.topic, PoseStamped, self.__callback)
        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=10)
        
    def getPose(self):
        '''
        Returns last PoseStamped. 

        @return last PoseStamped saved

        '''
        self.lock.acquire()
        path = self.data
        self.lock.release()
        
        return path

    def setPose(self, posX, posY):
        self.msg.header.frame_id = "/map"

        self.msg.pose.position.x = posX
        self.msg.pose.position.y = posY

        self.msg.pose.orientation.w = 1

        # print self.msg
        
        self.pub.publish(self.msg)

