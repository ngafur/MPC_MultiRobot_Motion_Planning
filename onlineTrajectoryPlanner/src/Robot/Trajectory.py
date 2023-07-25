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

