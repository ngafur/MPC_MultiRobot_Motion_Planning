import numpy as np
import ast
import rospy
from std_msgs.msg import String


class NetworkROS:
    
    def __init__(self, robotparam, otherrobotsparams):
        #rospy.init_node('network'+robotparam.ROS['robotNr'][1:])
        self.topic_name_raw = '/trajectory_data'

        self.pub = rospy.Publisher(robotparam.ROS['robotNr']+self.topic_name_raw,String, queue_size = 10)
        rospy.sleep(0.2)

        self.receivedTrajectories = {}
        self.subs = {}
        for param in otherrobotsparams:
            def callback_traj(msg):
                traj = ast.literal_eval(msg.data)
                for k in list(traj.keys()):
                    traj[k] = np.array(traj[k])
                self.receivedTrajectories[param.ROS['robotNr']] = traj
            self.subs['robotNr'] = rospy.Subscriber(param.ROS['robotNr']+self.topic_name_raw,String,callback=callback_traj)
            rospy.sleep(0.2)


    def broadcastData(self,trajectory):
        controls = trajectory.controls
        states = trajectory.states
        traj_dict = {'controls':controls.tolist(), 'states':states.tolist()}
        traj_str = str(traj_dict)
        self.pub.publish(traj_str)


    def receiveData(self):
        return self.receivedTrajectories
