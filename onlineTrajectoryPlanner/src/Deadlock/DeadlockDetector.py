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
        