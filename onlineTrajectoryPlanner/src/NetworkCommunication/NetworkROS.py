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
