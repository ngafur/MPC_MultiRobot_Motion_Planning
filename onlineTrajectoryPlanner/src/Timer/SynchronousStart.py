import rospy
from std_msgs.msg import String


class SynchronousStart:

    def __init__(self):
        self.mySub = rospy.Subscriber('/sync_topic', String)
        # self.myPub = rospy.Publisher('/confirm_topic',String)

    def wait(self):
        print('Waiting for start message.')
        rospy.wait_for_message('/sync_topic', String)

    # def finish(self):
        # print('Message is recieved')
        # myPub.publish('Message is recieved')
