# Copyright (c) 2023, Nigora Gafur - Rheinland-Pfälzische Technische Universität Kaiserslautern-Landau

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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