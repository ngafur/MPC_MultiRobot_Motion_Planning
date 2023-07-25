from casadi import SX, mtimes, vertcat, reshape, horzcat, vcat, nlpsol, hcat
from src.Solver.SolverBase import SolverBase
import numpy as np
from src.CollisionAvoidance.lineSegments import lineSegments
from src.CollisionAvoidance.ellipses import ellipses
from src.CollisionAvoidance.Distance import Distance

class NonCooperativeSolver2Robots(SolverBase):
    
    def __init__(self,opts,robot1,robot2):
        super(NonCooperativeSolver2Robots,self).__init__(opts)
        self.robot1 = robot1
        self.robot2 = robot2
        self.setup()
        
    def setup(self):
        # Read parameters from associated robot
        A_d = self.robot1.param.Dynamics['A_d']
        B_d = self.robot1.param.Dynamics['B_d']
        Nq = self.robot1.param.Dynamics['Nq']
        Nu = self.robot1.param.Dynamics['Nu']
        Np = self.robot1.param.Controller['Np']
        Qq = self.robot1.param.Controller['Qq']
        Qf = self.robot1.param.Controller['Qf']
        Ru = self.robot1.param.Controller['Ru']
        Rd = self.robot1.param.Controller['Rd']

        NqRobot2 = self.robot2['param'].Dynamics['Nq']

        if self.robot1.param.Controller['Np'] != self.robot1.param.Controller['Np']:
            raise ValueError('Parameter:Inconsistency, Both robots need to have identical optimization horizons!')
        
        # Optimization Variables
        U = SX.sym('U', Nu, Np)                      # control variables for the length of the horizon Np
        P = SX.sym('P', 2*Nq)                        # parameters, which include the initial and the reference state of the robot
        Q = SX.sym('Q', Nq, Np+1)                    # state variables for the length of the horizon Np
        Q_prev = SX.sym('Q_prev', NqRobot2, Np+1)    # for collision constraints 
        
        # Objective and constraints 
        obj_func = 0
        g = []
        g.append(Q[:,0]-P[0:Nq])
        lambda_reward = 1
        for k in range(0,Np):
            obj_func += mtimes(mtimes( (Q[:,k]-P[Nq:2*Nq]).T,Qq),(Q[:,k]-P[Nq:2*Nq])) + mtimes(mtimes(U[:,k].T,Ru),U[:,k]) + mtimes(mtimes((U[:,k]-U[:,k-1]).T,Rd),(U[:,k]-U[:,k-1]))
            g.append(Q[:,k+1] - mtimes(A_d,Q[:,k]) + mtimes(B_d,U[:,k]))
        obj_func += mtimes(mtimes((Q[:,Np]-P[Nq:2*Nq]).T,Qf),(Q[:,Np]-P[Nq:2*Nq]))
        
        
        
        ## Collision Avoidance
        # Number of collision constraints
        N_col = 7+14
        
        
        for k in range(1,Np+1):
            rob1 = lineSegments(Q[0,k],Q[1,k],Q[2,k],Q[3,k],Q[4,k],Q[5,k],self.robot1.param)
            # with the table
            g.append(-rob1['B07'][2] + self.robot1.param.Base['zmin'])
            g.append(-rob1['B05'][2] + self.robot1.param.Base['zmin'])
            g.append(-rob1['B06'][2] + self.robot1.param.Base['zmin'])
            # with the camera
            g.append( rob1['B06'][2] - self.robot1.param.Base['zmax'])
            g.append( rob1['B07'][2] - self.robot1.param.Base['zmax'])
            g.append( rob1['B06'][0] - self.robot1.param.Base['xmax'])
            g.append( rob1['B07'][0] - self.robot1.param.Base['xmax'] + self.robot1.param.Radius['rGripper'])
        
        ## Line modeling for an active robot -- Ellipse modeling for inactive robot
        for k in range(1,Np+1):
            rob1 = lineSegments(Q[0,k],Q[1,k],Q[2,k],Q[3,k],Q[4,k],Q[5,k],self.robot1.param)
            rob1_ell = ellipses(Q[0,k],Q[1,k],Q[2,k],Q[3,k],Q[4,k],Q[5,k],self.robot1.param)
            rob2 = ellipses(Q_prev[0,k],Q_prev[1,k],Q_prev[2,k],Q_prev[3,k],Q_prev[4,k],Q_prev[5,k],self.robot2['param'])
            #Collision avoidance between basis and gripper of the same robot
            #g.append(-Distance(rob1_ell['R1'], rob1_ell['H1'], rob1_ell['rm001'], rob1['B06'], rob1['r6t']) + 1)
            # Ellipse Arm 2
            g.append(-Distance(rob2['R2'], rob2['H2'], rob2['rm023'], rob1['B06'], rob1['r6t']) + 1)
            #Ellipse Arm 3
            g.append(-Distance(rob2['R3'], rob2['H3'], rob2['rm03t4'], rob1['B03t'], rob1['r3t4']) + 1)
            g.append(-Distance(rob2['R3'], rob2['H3'], rob2['rm03t4'], rob1['B04'], rob1['r45']) + 1)
            g.append(-Distance(rob2['R3'], rob2['H3'], rob2['rm03t4'], rob1['B05'], rob1['r56']) + 1)
            g.append(-Distance(rob2['R3'], rob2['H3'], rob2['rm03t4'], rob1['B06'], rob1['r6t']) + 1)
            #Ellipse Arm 5  
            g.append(-Distance(rob2['R5'], rob2['H5'], rob2['rm056'], rob1['B03t'], rob1['r3t4']) + 1)
            g.append(-Distance(rob2['R5'], rob2['H5'], rob2['rm056'], rob1['B04'], rob1['r45']) + 1)
            g.append(-Distance(rob2['R5'], rob2['H5'], rob2['rm056'], rob1['B05'], rob1['r56']) + 1)
            g.append(-Distance(rob2['R5'], rob2['H5'], rob2['rm056'], rob1['B06'], rob1['r6t']) + 1)
            #Ellipse Arm 6
            g.append(-Distance(rob2['R6'], rob2['H6'], rob2['rm06t'], rob1['B02'], rob1['r23']) + 1)
            g.append(-Distance(rob2['R6'], rob2['H6'], rob2['rm06t'], rob1['B03t'], rob1['r3t4']) + 1)
            g.append(-Distance(rob2['R6'], rob2['H6'], rob2['rm06t'], rob1['B04'], rob1['r45']) + 1)
            g.append(-Distance(rob2['R6'], rob2['H6'], rob2['rm06t'], rob1['B05'], rob1['r56']) + 1)
            g.append(-Distance(rob2['R6'], rob2['H6'], rob2['rm06t'], rob1['B06'], rob1['r6t']) + 1)

        g = vcat(g)
        
        # Define the optimization variables in one vector
        OPT_variables = vertcat(reshape(Q,Nq*(Np+1),1), reshape(U,Nu*Np,1))
        
        # setup NLP 
        nlp = {'f':obj_func, 'x':OPT_variables, 'g':g, 'p':horzcat(P[0:Nq],P[Nq:], Q_prev)}
        
        # Define arguments
        args = {}
        args['p'] = []
        args['x0'] = np.zeros((Np*(Nq+Nu)+Nq,1))
        
        #Equality constraints
        args['lbg'] = np.zeros((Nq*(Np+1),1))
        args['ubg'] = np.zeros((Nq*(Np+1),1))
        
        # inequality constraints
        args['lbg'] = vertcat(args['lbg'], -np.ones(N_col*Np)*np.inf)
        args['ubg'] = vertcat(args['ubg'], np.zeros(N_col*Np))
        
        # constraints in states
        lbx_step = self.robot1.param.Controller['lbx']
        ubx_step = self.robot1.param.Controller['ubx']
        lbu_step = self.robot1.param.Controller['lbu']
        ubu_step = self.robot1.param.Controller['ubu']
        args['lbx'] = vcat([lbx_step for i in range(0,Np+1)] + [lbu_step for i in range(0,Np)])
        args['ubx'] = vcat([ubx_step for i in range(0,Np+1)] + [ubu_step for i in range(0,Np)])
        
        
        self.args = args
        
        # Setup solver
        self.solver = nlpsol('solver','ipopt',nlp,self.opts)
        
        
    def solve(self,trajRobot1,trajRobot2):
        # Assemble parameter vector
        trajRobot2 = trajRobot2[[i for i in list(trajRobot2.keys())][0]]
        self.args['p'] = hcat([self.robot1.state,self.robot1.terminalState,trajRobot2['states']])
        # Get current state from robot
        
        a1 = reshape(hcat([self.robot1.state,trajRobot1.states[:,1:]]), (-1,1))
        a2 = reshape(trajRobot1.controls, (-1,1))
        self.args['x0'] = vcat([a1,a2 ])

        # solve nlp
        sol = self.solver(x0 = self.args['x0'], lbx = self.args['lbx'], ubx = self.args['ubx'], lbg = self.args['lbg'], ubg = self.args['ubg'], p = self.args['p'])
        print((str(self.solver.stats()['return_status'])))
        # Raise an exception is from is infeasible
        if str(self.solver.stats()['return_status']) != 'Solve_Succeeded':
            print('###############################################################')
            print('###############################################################')
            print('################### Solver did not converge ###################')
            print('###############################################################')
            print('###############################################################')
            #raise RuntimeError ('Solution:Infeasible, The solver did not converge') 

        # Extract optimal solution
        Nq = self.robot1.param.Dynamics['Nq']
        Nu = self.robot1.param.Dynamics['Nu']
        Np = self.robot1.param.Controller['Np']
        
        q_opt = np.array(sol['x'][0:Nq*(Np+1)]).reshape((Np+1,Nq)).T
        u_opt = np.array(sol['x'][Nq*(Np+1):]).reshape((Np,Nu)).T
        

        # Update trajRobot1
        trajRobot1.states = q_opt
        trajRobot1.controls = u_opt
        trajRobot1.objValue = sol['f']
    
    

    



        
        
        
        
        
        
        
        
        
        
        
        
        
        

       