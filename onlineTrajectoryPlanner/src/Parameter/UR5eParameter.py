import numpy as np
from math import pi, sqrt
from scipy.integrate import ode
from scipy.signal import cont2discrete
from ur_ikfast import ur_kinematics
from src.CollisionAvoidance.robotCoordinates import robotCoordinates



class UR5eParameter:
    def __init__(self,robotNr,BaseShift, BaseRotation, Np,Ts,plotColor, controllerID):
        # System Dynamics 
        A = np.block([[np.zeros((6,6)),np.eye(6)],
                         [np.zeros((6,12))]])
        B = np.block([[np.zeros((6,6))],[np.eye(6)]])
        C = np.eye(12)
        D = 0
        (A_d, B_d, C_d, D_d, dt) = cont2discrete((A,B,C,D), Ts)    

        self.Dynamics = {}
        self.Dynamics['Ts'] = Ts
        self.Dynamics['A_c'] = A
        self.Dynamics['B_c'] = B
        self.Dynamics['A_d'] = A_d
        self.Dynamics['B_d'] = B_d
        (Nq,Nu) = B_d.shape
        self.Dynamics['Nq'] = Nq
        self.Dynamics['Nu'] = Nu

        # Controller
        if controllerID == 1:
            self.Controller = {}
            self.Controller['Np'] = Np
            self.Controller['Qq'] = np.diag([1.0, 1.0, 1.0, 0.2, 0.2, 1.0, 2.0, 0.1, 0.1, 0.01, 0.01, 0.01])
            self.Controller['Qf'] = 3*np.diag([1.0, 1.0, 1.0, 0.2, 0.2, 1.0, 2.0, 0.1, 0.1, 0.01, 0.01, 0.01])
            self.Controller['Ru'] = 0.1*np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
            self.Controller['Rd'] = 5*np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
            # self.Controller['lbx'] = np.array([-2*np.pi,   -np.pi,             0,   -160./180.*np.pi, -140./180.*np.pi, -2*np.pi,    -pi,-pi,-pi,-2*pi,-2*pi,-2*pi])
            self.Controller['lbx'] = np.array([-2*np.pi, -np.pi, -2*np.pi/3, -np.pi, -140./180.*np.pi, -2*np.pi,    -np.pi/2,-np.pi,-np.pi,-np.pi,-np.pi,-np.pi])
            self.Controller['lbx'][6:12] = 1*self.Controller['lbx'][6:12]
            # self.Controller['ubx'] = np.array([ 2*np.pi,     0,  155./180.*np.pi,    -90./180.*np.pi,  140./180.*np.pi,  2*np.pi,     pi, pi, pi, 2*pi, 2*pi, 2*pi])
            self.Controller['ubx'] = np.array([ 2*np.pi,  0,    2.4,  np.pi,  140./180.*np.pi,  2*np.pi,    np.pi/2, np.pi, np.pi, np.pi, np.pi, np.pi])
            self.Controller['ubx'][6:12] = 1*self.Controller['ubx'][6:12]
            self.Controller['lbu'] = np.array([-pi/2,-pi,-pi,-pi,-pi,-pi])
            self.Controller['ubu'] = np.array([ pi/2, pi, pi, pi, pi, pi])
        elif controllerID == 2:
            self.Controller = {}
            self.Controller['Np'] = Np
            self.Controller['Qq'] = np.diag([1.0, 5.0, 1.0, 0.2, 0.2, 1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
            self.Controller['Qf'] = 10*np.diag([1.0, 5.0, 1.0, 0.2, 0.2, 1.0, 1.0, 0.2, 1.0, 0.1, 0.1, 0.1])
            self.Controller['Ru'] = 0.1*np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
            self.Controller['Rd'] = 5*np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
            # self.Controller['lbx'] = np.array([-2*np.pi,   -np.pi,             0,   -160./180.*np.pi, -140./180.*np.pi, -2*np.pi,    -pi,-pi,-pi,-2*pi,-2*pi,-2*pi])
            self.Controller['lbx'] = np.array([-2*np.pi, -np.pi, -2*np.pi/3, -np.pi, -np.pi, -2*np.pi,    -np.pi/4,-np.pi,-np.pi,-2.0*np.pi,-2.0*np.pi,-2.0*np.pi])
            self.Controller['lbx'][6:12] = 1*self.Controller['lbx'][6:12]
            # self.Controller['ubx'] = np.array([ 2*np.pi,     0,  155./180.*np.pi,    -90./180.*np.pi,  140./180.*np.pi,  2*np.pi,     pi, pi, pi, 2*pi, 2*pi, 2*pi])
            self.Controller['ubx'] = np.array([ 2*np.pi,  0,    2.3,  np.pi,  np.pi,  2*np.pi,    np.pi/4, np.pi, np.pi, 2.0*np.pi, 2.0*np.pi, 2.0*np.pi])
            self.Controller['ubx'][6:12] = 1*self.Controller['ubx'][6:12]
            self.Controller['lbu'] = np.array([-pi/4,-pi,-pi,-pi,-pi,-pi])
            self.Controller['ubu'] = np.array([ pi/4, pi, pi, pi, pi, pi])

        # Base
        self.Base = {}
        self.Base['Shift'] = BaseShift
        self.Base['Rotation'] = BaseRotation

        # Minimum height from surface
        self.Base['zmin'] = 0.11
        self.Base['zmax'] = 0.9
        self.Base['xmax'] = 0.6


        # Mass
        self.Mass = {}
        self.Mass['m1'] = 3.761
        self.Mass['m2'] = 8.058
        self.Mass['m3'] = 2.846
        self.Mass['m4'] = 1.37
        self.Mass['m5'] = 1.3
        self.Mass['m6'] = 0.365

        # Length
        self.Length = {}
        self.Length['l1'] = 0.163
        self.Length['l2'] = 0.425
        self.Length['l3'] = 0.3922
        self.Length['l4'] = 0.127
        self.Length['l5'] = 0.1
        self.Length['l6'] = 0.1

        # Radius of links
        self.Radius = {}
        self.Radius['r1'] = 0.058  
        self.Radius['r2'] = 0.0425
        self.Radius['r3'] = 0.0375 
        self.Radius['r4'] = 0.0375  
        self.Radius['r5'] = 0.0375 
        self.Radius['r6'] = 0.0375 

        # Gripper
        self.Length['lGripper'] = 0.177
        self.Radius['rGripper'] = 0.152/2
        self.Length['lGripperOpen'] = 0.162
        self.Length['lGripperFinger'] = 0.037


        # Denavit Hartenberg 
        self.DH = {}
        self.DH['d1'] = 0.1625
        self.DH['a2'] = -0.425
        self.DH['a3'] = -0.3922
        self.DH['d4'] = 0.1267
        self.DH['d5'] = 0.0997
        self.DH['d6'] = 0.0996
        
        # For Collision Avoidance
        self.DH['d0'] = 0.138
        self.DH['d3'] = 0.131

        # Ellipsoids for collision avoidance between the robots
        eps_r = 2.0
        self.Ellipse = {}
        self.Ellipse['H1l'] = np.sqrt(2)*self.DH['d1'] + 4*self.Radius['r1'] + 0.1*self.DH['d1']
        self.Ellipse['H1d'] = np.sqrt(2)*2*self.Radius['r1'] + 2*eps_r*self.Radius['r1']
        self.Ellipse['H2l'] = sqrt(2)*abs(self.DH['a2']) + 4*self.Radius['r2'] + 0.1*abs(self.DH['a2'])
        self.Ellipse['H2d'] = sqrt(2)*2*self.Radius['r2'] + 2*eps_r*self.Radius['r2']
        self.Ellipse['H3l'] = sqrt(2) * (sqrt(2)*abs(self.DH['a3']) + 4*self.Radius['r3'] + 0.1*abs(self.DH['a3']))
        self.Ellipse['H3d'] = sqrt(2) * (sqrt(2)*2*self.Radius['r3'] + 2*eps_r*self.Radius['r3'])
        self.Ellipse['H4l'] = sqrt(2)*self.DH['d4'] + 4*self.Radius['r4'] + 0.1*self.DH['d4']
        self.Ellipse['H4d'] = sqrt(2)*2*self.Radius['r4'] + 2*eps_r*self.Radius['r4']
        self.Ellipse['H5l'] = sqrt(2)*self.DH['d5'] + 4*self.Radius['r5'] + 0.2*self.DH['d5']
        self.Ellipse['H5d'] = sqrt(2)*2*self.Radius['r5'] + 2*eps_r*self.Radius['r5']
        self.Ellipse['H6l'] = sqrt(2)*(self.DH['d6'] + self.Length['lGripper']) + 4*self.Radius['rGripper'] + 2*0.1*(self.DH['d6']+self.Length['lGripper'])
        self.Ellipse['H6d'] = sqrt(2)*2*self.Radius['rGripper'] + 2*eps_r*self.Radius['rGripper']

        #Inertia expresed in the body attached frame
        self.Inertia = {}
        self.Inertia['I1x'] = 1/12*self.Mass['m1']*(3*self.Radius['r1']**2 + self.Length['l1']**2)
        self.Inertia['I1y'] = 1/12*self.Mass['m1']*(3*self.Radius['r1']**2 + self.Length['l1']**2)
        self.Inertia['I1z'] = 1/2*self.Mass['m1']*self.Radius['r1']**2
        self.Inertia['I2x'] = 1/12*self.Mass['m2']*(3*self.Radius['r2']**2 + self.Length['l2']**2)
        self.Inertia['I2y'] = 1/12*self.Mass['m2']*(3*self.Radius['r2']**2 + self.Length['l2']**2)
        self.Inertia['I2z'] = 1/2*self.Mass['m2']*self.Radius['r2']**2
        self.Inertia['I3x'] = 1/12*self.Mass['m3']*(3*self.Radius['r3']**2 + self.Length['l3']**2)
        self.Inertia['I3y'] = 1/12*self.Mass['m3']*(3*self.Radius['r3']**2 + self.Length['l3']**2)
        self.Inertia['I3z'] = 1/2*self.Mass['m3']*self.Radius['r3']**2
        self.Inertia['I4x'] = 1/12*self.Mass['m4']*(3*self.Radius['r4']**2 + self.Length['l4']**2)
        self.Inertia['I4y'] = 1/12*self.Mass['m4']*(3*self.Radius['r4']**2 + self.Length['l4']**2)
        self.Inertia['I4z'] = 1/2*self.Mass['m4']*self.Radius['r4']**2
        self.Inertia['I5x'] = 1/12*self.Mass['m5']*(3*self.Radius['r5']**2 + self.Length['l5']**2)
        self.Inertia['I5y'] = 1/12*self.Mass['m5']*(3*self.Radius['r5']**2 + self.Length['l5']**2)
        self.Inertia['I5z'] = 1/2*self.Mass['m5']*self.Radius['r5']**2
        self.Inertia['I6x'] = 1/12*self.Mass['m6']*(3*self.Radius['r6']**2 + self.Length['l6']**2)
        self.Inertia['I6y'] = 1/12*self.Mass['m6']*(3*self.Radius['r6']**2 + self.Length['l6']**2)
        self.Inertia['I6z'] = 1/2*self.Mass['m6']*self.Radius['r6']**2

        # Poses
        self.pose = {}
        self.pose['initial'] = np.array([0,-pi/2,0,0,0,pi/4])
        self.pose['deadlock'] = np.array([0.0, -2.3713, 1.9292, -1.1287, -1.5708, -0.2025])
        
        # Mass Center Points
        self.CP = {}
        self.CP['ly1'] = -0.02
        self.CP['lz1'] = 0.0
        self.CP['lx2'] = 0.13
        self.CP['lz2'] = 0.1157
        self.CP['lx3'] = 0.05
        self.CP['lz3'] = 0.0238
        self.CP['ly4'] = 0.0
        self.CP['lz4'] = 0.01
        self.CP['lz5'] = 0.01
        self.CP['lz6'] = -0.02
        
        # Minimum distance between the robots for line segments
        self.Dmin = 2.5*self.Radius['r3'] 
        
        # Grasping steps
        self.ROS = {}
        self.ROS['steps'] = 1
        self.ROS['offset'] = np.array([0, 0, 0.13]).reshape((-1,1))
        self.ROS['PickingOffset'] = np.array([0, 0, 0.090, 0 ]).reshape((-1,1))
        deltaLGripper = self.Length['lGripper'] - self.Length['lGripperOpen']
        self.ROS['placeOffset'] = np.array([0, 0, -deltaLGripper-0.01]).reshape((-1,1))

        # Inverse Kinematics
        self.kin = {}
        self.kin['theta_min'] = np.array([-2*pi, -2.6,       0,   -4.0, -1.6, -2*pi])  
        self.kin['theta_max'] = np.array([ 2*pi,     0,  2.6,  -1.0,  1.6,  2*pi])
        #self.kin['theta_min'] = np.array([-2*pi,   -pi,             0,   -pi, -140./180.*pi, -2*pi])  
        #self.kin['theta_max'] = np.array([ 2*pi,     0,  130./180.*pi,     0,  140./180.*pi,  2*pi])   

        # ROS Action Client
        if controllerID == 1:
            self.ROS['controller'] = '/scaled_vel_joint_traj_controller/follow_joint_trajectory'
            self.ROS['subscriber'] = '/scaled_vel_joint_traj_controller/state'
        elif controllerID == 2:
            self.ROS['controller'] = '/vel_joint_traj_controller/follow_joint_trajectory'
            self.ROS['subscriber'] = '/vel_joint_traj_controller/state'
        else:
            print('Choose one of the available controllers!')
        self.ROS['jointNames'] = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        if (robotNr == 1) & (controllerID == 1):
            self.ROS['robotNr'] = '/robot030'
        elif (robotNr == 2) & (controllerID == 1):
            self.ROS['robotNr'] = '/robot029'
        else:
            self.ROS['robotNr'] = '/robot' + str(robotNr)
        self.ROS['gripper'] = '/gripper_controller/command'
        self.ROS['gripperNames'] = ['gripper_finger_1_joint','gripper_finger_2_joint']
        self.ROS['gripperStroke'] = [-0.034, -0.034]
        self.ROS['points'] = 10

        # Misc
        self.PlotColor = plotColor
        self.ur5e_arm = ur_kinematics.URKinematics('ur5e')

    def intergrateModel(self,t_span,initialState,controls):
        # (n,m) = controls.shape
        # state = initialState.reshape(-1,1)
        # for i in range(0,m):
        #     state = np.matmul(self.Dynamics['A_d'],state)+np.matmul(self.Dynamics['B_d'],controls[:,i].reshape(-1,1))
        # return state
        t_arr = np.linspace(t_span[0],t_span[1],self.ROS['points'])
        q_int = np.zeros((12,self.ROS['points']))
        A_c = self.Dynamics['A_c']
        B_c = self.Dynamics['B_c']
        Ts = self.Dynamics['Ts']
        def model(t,q):
            return np.matmul(A_c,q.reshape((-1,1)))+np.matmul(B_c,controls[:,0+int(np.floor(t/Ts))].reshape((-1,1)))
        for i in range(0,self.ROS['points']):
            r = ode(model).set_integrator('vode',method='bdf')
            r.set_initial_value(initialState,t_span[0])
            q_int[:,i] = r.integrate(t_span[1]).reshape((-1))
        return q_int

    def invKinematics(self,Pos):
        Pos = np.array(Pos).reshape((-1,1))
        #print(Pos)
        if Pos.shape[0] == 4:
            xyz = Pos[0:3] - self.Base['Shift']
        else:
            xyz = Pos - self.Base['Shift']
        xyz = xyz.reshape(-1,1)


        # Dimension of the gripper
        dim = {}
        dim['gripper'] = np.array([0,0,-self.Length['lGripperOpen']]).reshape((-1,1))

        # Rotation matrix of the endeffector
        Rot = {}

        Rot['EF'] = np.array([[-0.00650204, -0.99997807, -0.0012669],
                        [-0.99996871,  0.00649626,  0.0045156],
                        [-0.00450727,  0.00129622, -0.99998897]])
        
        
        Rot['gripper'] = np.array([[1,0,0],[0,1,0],[0,0,1]])
        H = {}
        H['gripper'] = np.array([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,-self.Length['lGripperOpen']],
                              [0,0,0,1]])

        d = np.array([[0,0,0,1]])

        # TCP Position
        # print(Rot['EF'].shape,xyz.shape,d.shape)
        H['all'] = np.concatenate([np.concatenate([Rot['EF'],xyz],axis = 1),
                                    d],axis = 0)


        # Homogenous tranformation matrix for the gripper
        if Pos.shape[0] == 4:
            theta = -Pos[3,0]
            rotz = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                            [np.sin(theta),np.cos(theta),0,0],
                            [0,0,1,0],
                            [0,0,0,1]])
            H['all'] = np.matmul(H['all'],rotz) 
        

        # Homogenous tranformation matrix for the ee
        H['EF'] = np.matmul(np.linalg.inv(H['gripper']),H['all'])


        qs = self.ur5e_arm.inverse(H['EF'][0:3,:],True)
        #print(self.kin['theta_min'])
        #print(self.kin['theta_max'])
        #print(qs)


        q = [x for x in qs if (x<= self.kin['theta_max']).all() & (x >= self.kin['theta_min']).all()]
        q = q[0]
        
        EF60 = self.ur5e_arm.forward(q,'matrix')
        EF60 = np.concatenate([EF60,d],axis = 0)
        EF_tcp0 = np.matmul(H['gripper'],EF60)

        if np.linalg.norm(EF_tcp0[0:3,3].reshape((-1,1))-xyz) > 1e-2:
            print('Wrong solution of the inverse kinematics')
        else:
            q_desired = q + np.array([self.Base['Rotation']-np.pi,0,0,0,0,0])
        
        return np.array(q_desired).reshape((1,-1))

    def forwardKinematics(self,state):
        rob = robotCoordinates(state[0],state[1],state[2],state[3],state[4],state[5],self)
        cartesianPose = rob['B06']
        return cartesianPose