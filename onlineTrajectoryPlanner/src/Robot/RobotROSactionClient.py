from src.Robot.RobotBase import RobotBase
from src.Robot.Trajectory import Trajectory
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, JointTrajectoryControllerState, GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


import numpy as np



class RobotROSactionClient(RobotBase):
    def __init__(self,param,gripper):
        # Call constructor of base class
        super(RobotROSactionClient,self).__init__(param)
        self.actClient = actionlib.SimpleActionClient(param.ROS['robotNr']+param.ROS['controller'], FollowJointTrajectoryAction)
        self.actClient.feedback_cb = None
        self.actClient.wait_for_server()
        self.param = param

        # Create subscriber
        self.mySub = rospy.Subscriber(param.ROS['robotNr']+param.ROS['subscriber'],JointTrajectoryControllerState)

        # Create message
        self.msg = FollowJointTrajectoryGoal()
        # Create publisher
        self.msg.trajectory.joint_names = param.ROS['jointNames']
        self.msg.trajectory.points.append(JointTrajectoryPoint())

        # Create publisher with flexible number of messages
        self.msgNum = FollowJointTrajectoryGoal()
        self.msgNum.trajectory.joint_names = param.ROS['jointNames']
        self.Npoints = self.param.ROS['points']
        self.trackingPoints = []
        for i in range(0,self.Npoints):
            self.trackingPoints.append(JointTrajectoryPoint())

        # Set initial gripper state to be fully open
        if gripper:
            self.gazebo = False
            self.gripper_init()
        else:
            self.gazebo = True
            self.gripper_gazbeo_init()


    def receiveState(self):
        #print(self.param.ROS['robotNr']+self.param.ROS['subscriber'])
        sensor = rospy.wait_for_message(self.param.ROS['robotNr']+self.param.ROS['subscriber'],JointTrajectoryControllerState,timeout = 5)
        position = np.array(sensor.actual.positions).T
        velocity = np.array(sensor.actual.velocities).T
        self.state = np.concatenate([position,velocity],axis = 0).reshape(-1,1)

    def transitionGazebo(self,t_span,trajectory):
        targetState = trajectory
        # Package ROS message and send to the robot
        self.msg.trajectory.points[0].time_from_start = rospy.Duration(t_span[1]-t_span[0])
        self.msg.trajectory.points[0].positions = targetState[0:6]
        self.msg.trajectory.points[0].velocities = targetState[6:12]

        #Publish message
        self.actClient.send_goal(self.msg)


    def transition(self,t_span,t_now,trajectory):
        Ts = self.param.Dynamics['Ts']
        timesStart = np.linspace(t_span[0], t_span[1], int(t_span[1]/Ts +1) ) #  timesStart = np.linspace(t_span[0], t_span[1], t_span[1]/Ts+1) 
        targetStates = trajectory.states[:,0:len(timesStart)]
        filter = timesStart > (t_now + Ts)
        times = timesStart[filter]
        targetStates = targetStates[:,filter]
        NpointsReduced = len(times)

        # Only use NpointsReduced rossmessages from obj.trackingPoints
        #trackingPointsReduced = self.trackingPoints[0:NpointsReduced]
        self.msgNum.trajectory.points = self.trackingPoints[0:NpointsReduced]
        for i in range(0,NpointsReduced):
            self.msgNum.trajectory.points[i].time_from_start = rospy.Duration(times[i]-t_now)
            self.msgNum.trajectory.points[i].positions = targetStates[0:6,i]
            self.msgNum.trajectory.points[i].velocities = targetStates[6:12,i]

        self.goal_handle = self.actClient.send_goal(self.msgNum)


    def transition_callback(self, goal_handle):
        # Check if the received goal handle matches the tracked goal handle
        if goal_handle == self.goal_handle:
            # Process the transition callbacks for the tracked goal handle
            # You can add your own custom logic here
            # For example, print the goal status or handle goal completion

            # Check the goal status
            if goal_handle.get_goal_status().status == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("robot goal succeeded!")

            # Clear the goal handle once it has completed
            if goal_handle.get_goal_status().status in [actionlib.GoalStatus.SUCCEEDED,
                                                        actionlib.GoalStatus.ABORTED,
                                                        actionlib.GoalStatus.REJECTED,
                                                        actionlib.GoalStatus.PREEMPTED]:
                self.goal_handle = None

        else:
            rospy.logwarn("Received a transition callback for an untracked goal handle")

    def setState(self,state,shift,time,blocking):
        self.msg.trajectory.points[0].time_from_start = rospy.Duration(time)

        if shift:
            self.msg.trajectory.points[0].positions = self.shiftState(state[0:6],shift)
            self.msg.trajectory.points[0].velocities = np.zeros(6)
        else:
            self.msg.trajectory.points[0].positions = state[0:6]
            self.msg.trajectory.points[0].velocities = np.zeros(6)


        self.actClient.send_goal(self.msg)
        if blocking:
            self.actClient.wait_for_result()



    def gripper_gazbeo_init(self):
        ## Publisher
        self.pub_gripper = rospy.Publisher(self.param.ROS['robotNr']+self.param.ROS['gripper'], JointTrajectory,queue_size=10)
        rospy.sleep(0.2)
        self.gripper_gazebo_control(self.param.ROS['gripperStroke'])
        rospy.sleep(0.5)
        self.gripper_gazebo_control([0.0 , 0.0])  # oeffnen
        rospy.sleep(3)

    def gripper_gazebo_control(self,pos):

        if self.gazebo:
            msg = JointTrajectory()
            msg.joint_names = self.param.ROS['gripperNames']
            point = JointTrajectoryPoint()
            point.positions = pos
            point.time_from_start = rospy.Duration(.1)
            msg.points.append(point)
            self.pub_gripper.publish(msg)
            return 'Success'

    def openGripperGazebo(self):
        self.gripper_gazebo_control([0.0 ,  0.0]) 
        rospy.sleep(2.4)
        # pass

    def closeGripperGazebo(self):
        self.gripper_gazebo_control(self.param.ROS['gripperStroke']+[0.01,0.01]) 
        rospy.sleep(2.4)
        # pass








