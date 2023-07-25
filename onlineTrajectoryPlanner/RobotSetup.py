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
import rospy
from math import pi
from src.Parameter.UR5eParameter import UR5eParameter

RobotName = "UR5e"
NrRobots = 2
Np = 15
err_tol = 5e-2


# simulation or experiments
testbed = False

if not testbed:
    Ts = 0.5
    controllerID = 2
    BaseRotation_R1 = 0
    BaseRotation_R2 = 2*pi

# Network Setup
# IP Adresse von ROS PC
setup1 = {}
setup1['network'] = {}
setup1['network']['ROSmasterURI'] = 'http://localhost:11311/'
setup1['network']['ROSIPs'] = ['192.168.0.24', '192.168.0.240', '192.168.0.24']
setup1['network']['UDPhosts'] = [['IPV4', 'localhost', 5000],
                                 ['IPV4', 'localhost', 5000], ['IPV4', 'localhost', 5000]]

setup = setup1

if (RobotName == "UR5e") & (NrRobots == 2):
    BaseRobot = np.array([[0.0, 0.0, 0.0], [0.0, 1.1, 0.0]])

    setup['robot'] = {}
    setup['robot']['params'] = [UR5eParameter(1, BaseRobot[0, :].reshape((-1, 1)), BaseRotation_R1, Np, Ts, 'black', controllerID),
                                UR5eParameter(2, BaseRobot[1, :].reshape((-1, 1)), BaseRotation_R2, Np, Ts, 'red', controllerID)]

    # Kartesische Koordinaten der Anfangsposition der Endeffektoren
    R10 = np.array([0.0,  -0.4,  0.12]).reshape((-1, 1))
    R20 = np.array([0.0,  1.5,  0.12]).reshape((-1, 1))

    #  Essential points/1st use_Case
    Setpoints = {}
    Setpoints[0] = np.array([[0.3, 0.6, 0.1, pi/2],[-0.3,  0.5, 0.1, 0]])
    Setpoints[1] = np.array([[-0.3,  0.5, 0.1, pi/2],[0.3, 0.6, 0.1, 0]])


# Minimale und Maximale Reichweite des Roboters
maxRange = 0.9
minRange = 0.37


rate = rospy.Rate(1/Ts)

# Number of iterations
iter_end = 100  

# Init Solver
options = {}
ipopt_options = {}
ipopt_options["max_iter"] = iter_end
ipopt_options["print_level"] = 0
options["print_time"] = False
ipopt_options["acceptable_tol"] = 1e-4
ipopt_options["acceptable_obj_change_tol"] = 1e-6
ipopt_options["linear_solver"] = 'mumps' 
options["jit"] = True
ipopt_options['warm_start_init_point'] = 'yes'
ipopt_options['fast_step_computation'] = 'yes'
options["ipopt"] = ipopt_options
