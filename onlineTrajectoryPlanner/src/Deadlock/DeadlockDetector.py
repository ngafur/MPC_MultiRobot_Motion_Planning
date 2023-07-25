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

class DeadlockDetectorStatic:
    def __init__(self,*args):
        self.robots = [args[0]]
        self.tolResGrasp = 0.02   #0.02
        self.tolRes = 0.12 #0.12
        self.tolTraj = 0.02
        self.tolVel = 0.015
        self.tolDist = 0.5


    def checkForDeadlock(self,*args):
        # Check for deadlocks
        Ninputs = len(args)
        deadlocks = []
        for i in range(0,Ninputs):
            # Check for deadlocks
            robot = self.robots[0]
            trajectory = args[i]
            checkRes = self.checkTerminalState(robot,False)
            checkResGrasp = self.checkTerminalState(robot,True)
            checkTraj = self.checkTrajectory(trajectory)
            checkVel = self.checkVelocity(trajectory)
            condVel = checkResGrasp & checkVel
            condTraj = checkRes & checkTraj
            deadlocks = condVel | condTraj

        return deadlocks

    def checkTrajectory(self,trajectory):
        states = trajectory.states
        print(('res traj: ',np.linalg.norm(states[0:6,0]-states[0:6,-1])))
        deadlock = np.linalg.norm(states[0:6,0]-states[0:6,-1]) < self.tolTraj
        return deadlock
    
    def checkVelocity(self, trajectory):
        states = trajectory.states
        print(('res vel: ',np.linalg.norm(states[6:12,0])))
        deadlock = np.linalg.norm(states[6:12,0]) < self.tolVel
        return deadlock

    def checkTerminalState(self,robot,grasp):
        if grasp:
            deadlock = robot.residual() > self.tolResGrasp
            print(('res distance grasping: ',robot.residual()))
        else:
            deadlock = robot.residual() > self.tolRes
            print(('res distance: ',robot.residual()))
        return deadlock

    def checkTerminalPose(self,robot):
        state = robot.state
        cartesianPoseC = robot.param.forwardKinematics(state)
        state = robot.terminalState
        cartesianPoseD = robot.param.forwardKinematics(state)
        dist = np.linalg.norm(cartesianPoseC-cartesianPoseD)
        print(('res cartesian distance: ',dist))
        deadlock = dist > self.tolDist
        return deadlock
        