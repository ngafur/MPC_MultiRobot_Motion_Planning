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

class Sequence:
    def __init__(self,param):
        self.param = param
        self.states = []
        self.actions = []
        self.times = []

    def move(self, xyzpos, timeStart = -1):
        state = self.param.invKinematics(xyzpos)
        self.addTask(state.T, 'MPC', timeStart)

    def addPickTask(self, xyzpos, timeStart):

        if len(xyzpos)==4:
            offset = np.vstack((self.param.ROS['offset'],np.array([0])))
        else:
            offset = self.param.ROS['offset']
        steps = self.param.ROS['steps']
        state1 = self.param.invKinematics(xyzpos + offset)

        self.addTask(state1.T, 'MPC', timeStart)

        for i in range(1,steps+1):
            state2 = self.param.invKinematics(xyzpos - self.param.ROS['PickingOffset'] )   #(xyzpos + ((steps-i)/steps)*offset)
            self.addTask(state2.T, 'transition', -1)

        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'close', -1)           
        self.addTask(state2.T, 'idle', -1) 
        self.addTask(state2.T, 'idle', -1) 
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)

        for i in range(1,steps+1):
            state2 = self.param.invKinematics(xyzpos + ((i)/steps)*offset)
            self.addTask(state2.T, 'transition', -1)

        self.addTask(state2.T, 'done', -1)


    def addPlaceTask(self, xyzpos, timeStart):

        if len(xyzpos)==4:
            offset = np.vstack((self.param.ROS['offset'],np.array([0])))
        else:
            offset = self.param.ROS['offset']
            xyzpos = xyzpos[0:3] + self.param.ROS['placeOffset']
        steps = self.param.ROS['steps']
        state1 = self.param.invKinematics(xyzpos + offset)
        

        self.addTask(state1.T, 'MPC', timeStart)

        for i in range(1,steps+1):
            state2 = self.param.invKinematics(xyzpos + ((steps-i)/steps)*offset)
            self.addTask(state2.T, 'transition', -1)

        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'open', -1)           
        self.addTask(state2.T, 'idle', -1) 
        self.addTask(state2.T, 'idle', -1) 
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)
        self.addTask(state2.T, 'idle', -1)

        for i in range(1,steps+1):
            state2 = self.param.invKinematics(xyzpos + ((i)/steps)*offset)
            self.addTask(state2.T, 'transition', -1)

        self.addTask(state2.T, 'done', -1)

    def addTransition(self, xyzpos, timeStart = -1):
        state1 = self.param.invKinematics(xyzpos.reshape((-1,1)))
        self.addTask(state1.T, 'MPC', timeStart)


    def addTask(self, state, action , time):
        if state.shape[0] == 6:
            state = np.vstack((state,np.zeros((6,1))))
        self.states.append(state)
        self.actions.append(action)
        self.times.append(time)
        # self.states = np.vstack((self.states,state))
        # self.actions = np.vstack((self.actions,action))
        # self.times = np.vstack((self.times,time))
    def get_states(self):
        return np.array(self.states).reshape((-1,12)).T

    def getTask(self, currentTime):
        if currentTime < self.times[0]:
            # idle
            state = np.zeros((12,1))
            action = 'idle'
        else:
            # action zuruckgeben
            state = self.get_states()[:,0]
            action = self.actions[0]

            if len(self.actions) > 1:
                self.states = self.states[1:]
                self.actions = self.actions[1:]
                self.times = self.times[1:]
        return state, action

