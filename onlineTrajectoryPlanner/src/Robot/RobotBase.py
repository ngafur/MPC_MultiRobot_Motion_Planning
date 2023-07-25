from email.mime import base
from src.Parameter.UR5eParameter import UR5eParameter
import numpy as np
from math import pi

class RobotBase(object):

    def __init__(self,param):
        # Assign robot parameters
        self.param = param

        # Init state and terminal state
        self.state = np.zeros((param.Dynamics['Nq'],1))
        self.terminalState = np.zeros((param.Dynamics['Nq'],1))
        self.status = 1
        self.terminalStateGoal = np.zeros((param.Dynamics['Nq'],1))

    def residual(self):
        res = abs(self.terminalStateGoal-self.state).max()
        return res

    def receiveState(self):
        raise NotImplementedError('Class method has not been implemented yet')

    def transition(self):
        raise NotImplementedError('Class method has not been implemented yet')

    def setState(self):
        raise NotImplementedError('Class method has not been implemented yet')

    def activate(self):
        if self.status == 0:
            self.terminalState = self.terminalStateGoal
            self.status = 1

    def deactivate(self):
        if self.status == 1:
            self.terminalState = self.shiftState(np.concatenate([self.state[0].reshape((-1,1)),self.param.pose['deadlock'][1:].reshape((-1,1)),np.zeros((6,1))]),False)
            self.status = 0


    def setTerminalState(self,terminalState,shift):
        if self.status ==1 :
            newState = self.shiftState(terminalState,shift)
            self.terminalState = newState.reshape((-1,1))
            self.terminalStateGoal = newState.reshape((-1,1))

    def shiftState(self,newState,shift):
        res = newState
        if shift:
            # Base joint
            base_alpha_1 = self.state[0]
            base_alpha_2 = newState[0]
            while (base_alpha_2 - base_alpha_1) > pi:
                base_alpha_2 = base_alpha_2 - 2. * pi
                if base_alpha_2 < (-2.*pi):
                    base_alpha_2 = base_alpha_2 + 2.*pi
                    break
            while( (base_alpha_1 - base_alpha_2) > pi ):
                backup_angle_2 = base_alpha_2
                base_alpha_2 = base_alpha_2 + 2.0 * pi
                if base_alpha_2  > (2*pi) :
                    base_alpha_2 = backup_angle_2
                    break
            res[0] = base_alpha_2


            # Endeffector joint
            EF_alpha_1 = self.state[5]
            EF_alpha_2 = newState[5]
            while (EF_alpha_2 -EF_alpha_1) > pi:
                EF_alpha_2 = EF_alpha_2 - 2.*pi
            res[5] = EF_alpha_2
            if res[5] > 2.*pi:
                res[5] = res[5]%(2*pi)

        return res
