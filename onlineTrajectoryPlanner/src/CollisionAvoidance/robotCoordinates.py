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

from casadi import cos, sin, SX, DM, vcat, hcat, mtimes, sqrt
import numpy as np

def robotCoordinates(theta1,theta2,theta3,theta4,theta5,theta6,Param):
    
    d0 = Param.DH['d0']
    d1 = Param.DH['d1']
    d3 = Param.DH['d3']
    d4 = Param.DH['d4']
    d5 = Param.DH['d5']
    d6 = Param.DH['d6'] + Param.Length['lGripper']
    a2 = Param.DH['a2']
    a3 = Param.DH['a3']
    
    x0 = Param.Base['Shift'][0]
    y0 = Param.Base['Shift'][1]
    z0 = Param.Base['Shift'][2]
    theta_base = Param.Base['Rotation']
    
    res = {}
    
    
    # Basisvektoren der Gelenke
    A00 = vcat([hcat([cos(theta_base), -sin(theta_base), 0, x0]),hcat([sin(theta_base), cos(theta_base), 0, y0]), hcat([0,0,1,z0]), hcat([0,0,0,1])])
    A10 = vcat([hcat([cos(theta1), 0, sin(theta1), 0]),hcat([sin(theta1), 0, -cos(theta1), 0]), hcat([0,1,0,d1]), hcat([0,0,0,1])])
    A21_0 = vcat([hcat([cos(theta2), -sin(theta2), 0, 0]),hcat([sin(theta2), cos(theta2), 0, 0]), hcat([0,0,1,d0]), hcat([0,0,0,1])])
    A21_1 = vcat([hcat([cos(theta2), -sin(theta2), 0, cos(theta2)*a2]),hcat([sin(theta2), cos(theta2), 0, sin(theta2)*a2]), hcat([0,0,1,d0]), hcat([0,0,0,1])])
    A32_0 = vcat([hcat([cos(theta3), -sin(theta3), 0, 0]),hcat([sin(theta3), cos(theta3), 0, 0]), hcat([0,0,1,-d3]), hcat([0,0,0,1])])
    A32_1 = vcat([hcat([cos(theta3), -sin(theta3), 0, cos(theta3)*a3]),hcat([sin(theta3), cos(theta3), 0, sin(theta3)*a3]), hcat([0,0,1,-d3]), hcat([0,0,0,1])])
    A43 = vcat([hcat([cos(theta4), 0, sin(theta4), 0]),hcat([sin(theta4), 0, -cos(theta4), 0]), hcat([0,1,0,d4]), hcat([0,0,0,1])])
    A54 = vcat([hcat([cos(theta5), 0, -sin(theta5), 0]),hcat([sin(theta5), 0, cos(theta5), 0]), hcat([0,-1,0,d5]), hcat([0,0,0,1])])
    A65 = vcat([hcat([cos(theta6), -sin(theta6), 0, 0]),hcat([sin(theta6), cos(theta6), 0, 0]), hcat([0,0,1,d6]), hcat([0,0,0,1])])
    
    
    # Transformationsmatrizen
    T01 = mtimes(A00,A10)
    T02 = mtimes(T01,A21_0)
    T03 = mtimes(T01,A21_1)
    T03t = mtimes(T03,A32_0)
    T04 = mtimes(T03, A32_1)
    T05 = mtimes(T04, A43)
    T06 = mtimes(T05, A54)
    T07 = mtimes(T06, A65)
    
    # Basisvektoren der Gelenke
    res['B00'] = Param.Base['Shift']
    res['B01'] = T01[0:3,3]
    res['B02'] = T02[0:3,3]
    res['B03'] = T03[0:3,3]
    res['B03t'] = T03t[0:3,3]
    res['B04'] = T04[0:3,3]
    res['B05'] = T05[0:3,3]
    res['B06'] = T06[0:3,3]
    res['B07'] = T07[0:3,3]
    
    return res
    
    
    
    