import numpy as np
import ast
import rospy
from std_msgs.msg import String


class DeadlockROS_Coordinator:
    
    def __init__(self, robotparams):
        self.robotparams = robotparams
        #rospy.init_node('deadlock'+robotparam.ROS['robotNr'][1:])
        self.topic_name_raw_from_coordinator = '/deadlock_data_from_coordinator'
        self.topic_name_raw_from_robot = '/deadlock_data_from_robot'

        self.pubs = {}
        for param in robotparams:
            param.ROS['robotNr']
            self.pubs[param.ROS['robotNr']] = rospy.Publisher(param.ROS['robotNr']+self.topic_name_raw_from_coordinator,String, queue_size = 10)
        rospy.sleep(0.2)         # ROS will sleep for the specified period, will sleep for 0.2 second

        self.data = {}
        self.subs = {}
        def callback_status(msg,ind):
            data1 = ast.literal_eval(msg.data)
            data1['robot_terminalState'] = np.array(data1['robot_terminalState'])
            data1['robot_state'] = np.array(data1['robot_state'])
            self.data[ind] = data1
        for param in robotparams:
            ind = param.ROS['robotNr']
            self.subs[ind] = rospy.Subscriber(param.ROS['robotNr']+self.topic_name_raw_from_robot,String,callback=callback_status, callback_args=ind)
            rospy.sleep(0.2)


    def sendStatus(self, status, stopProcess):
        for i,param in enumerate(self.robotparams):
            status_dict = {'status':status[i], 'stopProcess':stopProcess}
            status_str = str(status_dict)
            self.pubs[param.ROS['robotNr']].publish(status_str)


    def receiveData(self):
        return self.data



class DeadlockROS_Robot:
    
    def __init__(self,  robotparam):
        #rospy.init_node('deadlock'+robotparam.ROS['robotNr'][1:])
        self.topic_name_raw_from_coordinator = '/deadlock_data_from_coordinator'
        self.topic_name_raw_from_robot = '/deadlock_data_from_robot'

        self.pub = rospy.Publisher(robotparam.ROS['robotNr']+self.topic_name_raw_from_robot,String, queue_size = 10)
        rospy.sleep(0.2)       # ROS will sleep for the specified period, will sleep for 0.2 second

        self.receivedstatus = {}
        self.subs = {}

        def callback_status(msg):
            status = ast.literal_eval(msg.data)
            self.receivedstatus = status
        self.sub = rospy.Subscriber(robotparam.ROS['robotNr']+self.topic_name_raw_from_coordinator,String,callback = callback_status)

    def sendDeadlock(self, deadlock, finalStatus, robot):
        data = {'deadlock':deadlock, 'finalStatus':finalStatus, 'robot_residual':robot.residual(), 'robot_state':robot.state[0:6].tolist(), 'robot_terminalState':robot.terminalState[0:6].tolist()}
        data_str = str(data)
        self.pub.publish(data_str)

    def receiveStatus(self):
        return self.receivedstatus