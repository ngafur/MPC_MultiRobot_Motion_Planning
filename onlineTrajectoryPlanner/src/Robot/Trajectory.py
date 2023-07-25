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

from src.Parameter.UR5eParameter import UR5eParameter
import numpy as np
from numpy.matlib import repmat

class Trajectory:

    def __init__(self,states,controls,objValue):
        self.states = states
        self.controls = controls
        self.objValue = objValue


    def controlInputRobot(self):
        u_opt = self.controls[:,0]
        return u_opt

    def targetStateRobot(self):
        q_target = self.states[:,1]

    def extrapolate(self,robotParam,Nsteps):
        # We extrapolate the last Nsteps time steps based on system dynamics
        # Should yield better results than a constant extrapolation
        A_d = robotParam.Dynamics['A_d']
        B_d = robotParam.Dynamics['B_d']

        # Constant extrapolation of controls
        cE1 = self.controls[:,Nsteps:]
        cE2 = repmat(self.controls[:,-1].reshape(-1,1),1,Nsteps)
        controlsExtrapolated = np.concatenate([cE1,cE2],axis = 1)

        # Extrapolate states using system dynamics
        statesExtrapolated = self.states[:,Nsteps:]
        lastControl = self.controls[:,-1]
        for i in range(0,Nsteps):
            lastState = statesExtrapolated[:,-1]
            newState = np.matmul(A_d,lastState.reshape((-1,1)))+np.matmul(B_d,lastControl.reshape((-1,1)))
            statesExtrapolated = np.concatenate([statesExtrapolated,newState],axis = 1)

        # update with the extrapolated Trajectory
        newTraj = Trajectory(statesExtrapolated,controlsExtrapolated,self.objValue)

        return newTraj

