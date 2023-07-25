import rospy
rospy.init_node('Coordinator')
from RobotSetup import  Np, Ts, maxRange, minRange, setup, options, rate, R10, R20, testbed, controllerID, BaseRotation_R1, BaseRotation_R2, err_tol, NrRobots, BaseRobot, RobotName
from src.NetworkCommunication.DeadlockROS import DeadlockROS_Coordinator
from src.Timer.SynchronousStart import SynchronousStart
from src.Deadlock.DeadlockResolver import DeadlockResolver
import time

if NrRobots == 2:
    robotCoordinatorID = 2
    robotIDs = [0,1]
    robotParam1 = setup['robot']['params'][robotIDs[0]]
    robotParam2 = setup['robot']['params'][robotIDs[1]]
    print((robotParam1.ROS['robotNr']))
    print((robotParam2.ROS['robotNr']))


# Communication setup
deadlock_ros = DeadlockROS_Coordinator([robotParam1,robotParam2])

if NrRobots == 2:
    deadlock_resolver = DeadlockResolver(robotParam1, robotParam2)

syncStart = SynchronousStart()

## Wait for all clients
syncStart.wait()
time.sleep(1)

## The control loop
recordDeadlocks = []
i = 0
stopProcess = False
syncStart.wait()
while True:
    time_start = time.time()
    print(('Step: ',i,', Time: ', time_start))

    if NrRobots == 2:
        dataRobots = deadlock_ros.receiveData()
        if dataRobots == {}:
            print('next')
            continue
        dataRobot1 = dataRobots[robotParam1.ROS['robotNr']]
        dataRobot2 = dataRobots[robotParam2.ROS['robotNr']]
        print(('Deadlock Robot1:',dataRobot1['deadlock'],'Deadlock Robot2:',dataRobot2['deadlock']))
        status = deadlock_resolver.updateStatus(dataRobot1,dataRobot2)
        print(('status:',status))
        finalStatus = [dataRobot1['finalStatus'],dataRobot2['finalStatus']]
        print(('FinalStatus Robot1:',dataRobot1['finalStatus'],'FinalStatus Robot2:',dataRobot2['finalStatus']))

    if all(finalStatus):
        stopProcess = True
        deadlock_ros.sendStatus(status, stopProcess)
        print('*************************************')
        print('*********Sending stopProcess*********')
        print('*************************************')
        stopProcess = False
        time.sleep(2)
        break
    else:
        deadlock_ros.sendStatus(status,stopProcess)

    i += 1

    rate.sleep()
