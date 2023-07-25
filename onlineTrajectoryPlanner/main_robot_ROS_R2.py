import rospy
from src.Timer.SynchronousStart import SynchronousStart
rospy.init_node('main_robot_ROS2')
import numpy as np
import time
from RobotSetup import  Np, Ts, maxRange, minRange, setup, options, rate, R10, R20, testbed, controllerID, BaseRotation_R1, BaseRotation_R2, err_tol, Setpoints
from src.Robot.RobotROSactionClient import RobotROSactionClient
from src.Robot.RobotBase import RobotBase
from src.Robot.Trajectory import Trajectory
from src.Parameter.UR5eParameter import UR5eParameter
from src.Scheduling.Sequence import Sequence
from src.Solver.NonCooperativeSolver2Robots import NonCooperativeSolver2Robots
from src.Plots.Recorder import Recorder
from src.NetworkCommunication.NetworkROS import NetworkROS
from src.Deadlock.DeadlockDetector import DeadlockDetectorStatic
from src.NetworkCommunication.DeadlockROS import DeadlockROS_Robot



## Some file specific settings
robotID = 1
robotNeighborID = 0
robotCoordinatorID = 2


## Init Robot with Parameters 
robotParameter = setup['robot']['params'][robotID]
print((robotParameter.ROS['robotNr']))


if not testbed:
    robot = RobotROSactionClient(robotParameter, False)

robotParameterNeighbour = setup['robot']['params'][robotNeighborID]
robotNeighbour = {}
robotNeighbour['param'] = robotParameterNeighbour

## Init communicator object & synchronizing robots
Nq = robotParameter.Dynamics['Nq']
Nu = robotParameter.Dynamics['Nu']
rosNetwork = NetworkROS(robotParameter,[robotNeighbour['param']])
deadlock_network = DeadlockROS_Robot(robotParameter)
syncStart = SynchronousStart()


## Initial pose of the robot
if robotID == 0:
    R0 = R10
elif robotID ==1:
    R0 = R20


## Setpoints
if not testbed:
    robotSequenze = Sequence(robotParameter)
    ## Call constructor for the sequence class
    robotSequenze = Sequence(robotParameter)
    Setpoints = Setpoints[robotID]
    #Setpoints = Setpoints['Robot'+str(robotID+1)]

    ## Generate a sequence
    for j in range(0,int(len((Setpoints)+1)/2)):
        robotSequenze.addPickTask(Setpoints[2*j,:].reshape((-1,1)),-1)
        robotSequenze.addPlaceTask(Setpoints[2*j+1,:].reshape((-1,1)),-1)
    RqE = robotParameter.invKinematics(Setpoints[0,:].reshape((-1,1)) + np.vstack((robotParameter.ROS['offset'],np.array([0]))))

## Add last transition which is the initial pose of the robot
robotSequenze.addTransition(R0)

## Inverse Kinematics
Rq0 = robotParameter.invKinematics(R0)
# Rq0[0,5] = Rq0[0,5] % (2*np.pi)
if (Setpoints[-1,0] < 0):
    Rq0[0,0] = Rq0[0,0].T % (2*np.pi)
elif (Setpoints[-1,0] > 0):
    Rq0[0,0] = Rq0[0,0].T % (-2*np.pi)
#print(Setpoints[0,:].T,np.array([[0.1]]))
#RqE = robotParameter.invKinematics(np.vstack((Setpoints[0,:].reshape((-1,1)),np.array([[0.1]]))) + robotParameter.ROS['offset'])
#RqE = robotSequenze.get_states()[0:6,0]

Robotq0 = np.vstack((Rq0.reshape((-1,1)),np.zeros((6,1))))
RobotqE = np.vstack((RqE.reshape((-1,1)),np.zeros((6,1))))

## Init Robots with initial & final states
robot.setState(Robotq0, False, 10, True)
robot.receiveState()
robot.setTerminalState(RobotqE,False)

## Init states and controls for the optimizer
states = np.matlib.repmat(robot.state, 1, Np  + 1)
controls = np.matlib.repmat(np.zeros((Nu,1)), 1, Np)
trajRobot = Trajectory(states, controls, 0)

## Init the properties of the solver class (done with constructor)
solver = NonCooperativeSolver2Robots(options, robot, robotNeighbour)

# Wait for all clients
syncStart.wait()

# Broadcast initial trajectory to robot
rosNetwork.broadcastData(trajRobot)
time.sleep(1)
trajNeigbour = rosNetwork.receiveData()

# Solve NLP
solver.solve(trajRobot, trajNeigbour)

# Send new optimal trajectory to other robot
rosNetwork.broadcastData(trajRobot)

# Init deadlock detection
deadlockDetector = DeadlockDetectorStatic(robot)

## Init the simulation
solver_times = []
recorder = Recorder()
i = 0
status = 1
finalStatus = 0
deadlock = 0
action = 'MPC'
statusBackup = []
deadlock_network.sendDeadlock(deadlock,finalStatus,robot)
data = []
data_operationalspace = []

## The control loop
syncStart.wait()
while True:
    time_start = time.time()

    # Receive sensor data
    robot.receiveState()

    ## Records measured and predicted states, control inputs, objValues and times
    recorder.pushBack(Ts*i, robot.state, action, trajRobot, 0)

    # Residual
    residualRobot = robot.residual()

    print(('Step: '+str(i)+', Time: '+str(time_start)+', Residuals: '+str(residualRobot)))

    # Receive trajectory from other robot
    trajNeighbour = rosNetwork.receiveData()

    # Receive status from coordinator
    statusProcess = deadlock_network.receiveStatus()
    print(statusProcess)
    status = statusProcess['status']

    ## Actions
    if status == 0:
        robot.deactivate()
        action = 'MPC'
    elif status == 1:
        robot.activate()

    if ((status == 1) & (residualRobot <= err_tol)) | (action == 'idle'):
        print('Setting new terminal state for robot')
        state, action = robotSequenze.getTask(time_start)
        robot.setTerminalState(state, True)

    if action == 'MPC':
        # Solve MPC
        solver.solveNLP(trajRobot, trajNeigbour)
        # Sending new trajectory to the robot
        time_now = time.time() - time_start
        robot.transition([0.0, 4*Ts], time_now, trajRobot)

    if action == 'transition':
        time_now = time.time()-time_start
        robot.setState(robot.terminalState, True, 2, True)

    if (action == 'open') & (not testbed):
        robot.openGripperGazebo()

    if (action == 'close') & (not testbed):
        robot.closeGripperGazebo()

    if action == 'idle':
        pass

    ## Shift trajecories for warmstart
    if action == 'MPC':
        time_now = time.time()-time_start
        Nsteps = int(np.ceil(time_now/Ts))
        if Nsteps > 1:
            print('********************************************')
            print('* MPC time greater than a sampling time Ts *')
            print('********************************************')
        trajRobot = trajRobot.extrapolate(robotParameter, Nsteps)
        solver_times.append(solver.times[-1])
    else:
        solver_times.append(0)
        
    ## Check for deadlocks
    deadlock = deadlockDetector.checkForDeadlock(trajRobot)
    if deadlock == 1:
        print('**************************************')
        print('Deadlock is detected')
        print('**************************************') 
    deadlock_network.sendDeadlock(deadlock,finalStatus,robot)

    ## Broadcast trajectory to other robots
    rosNetwork.broadcastData(trajRobot)
    statusBackup.append(status)

    ## Wait for executing one MPC loop  
    i += 1
    arg1 = (len(robotSequenze.actions) == 1)
    arg2 = (robotSequenze.get_states()[0:5,-1]==robot.terminalState[0:5].T).all()
    arg3 = (robotSequenze.get_states()[0:5,-1]==np.vstack(((robot.terminalState[0] % 2*np.pi).reshape((-1,1)), robot.terminalState[1:5].reshape((-1,1)))).T).all()
    arg4 = (robotSequenze.get_states()[0:5,-1]==np.vstack(((robot.terminalState[0] % (-2*np.pi)).reshape((-1,1)),robot.terminalState[1:5].reshape((-1,1)))).T).all()
    if ((robotSequenze.get_states()[0,-1]- 2*np.pi)==robot.terminalState[0]):
        arg2 = True
    if  arg1 & arg2 | arg3 | arg4:
        total_time = time.time()-time_start
        if max(abs(robot.terminalState - robot.state)) <= err_tol:
            finalStatus = 1
            deadlock = 0
            deadlock_network.sendDeadlock(deadlock,finalStatus,robot)
            time.sleep(2)
            break
    #print(action)
    print(('Time Elapsed: '+str(time.time()- time_start)))
    

    try:
        solverTimes = solver.solverTimes()
        minTime = solverTimes['tmin']
        maxTime = solverTimes['tmax']
        meanTime = solverTimes['tmean']
        stdDeviation = solverTimes['tstd']
        print(('Solver times 1: min = ',minTime, 'max = ',maxTime, 'mean = ',meanTime,))
    except ValueError:
        pass

    rate.sleep()

robot.actClient.cancel_goal()

solverTimes = solver.solverTimes()
minTime = solverTimes['tmin']
maxTime = solverTimes['tmax']
meanTime = solverTimes['tmean']
stdDeviation = solverTimes['tstd']
print(('Solver times 1: min = ',minTime, 'max = ',maxTime, 'mean = ',meanTime,))