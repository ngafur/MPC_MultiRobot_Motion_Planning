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

from src.CollisionAvoidance.lim_func import lim_func
from casadi import mtimes,dot,sqrt


def Line_Distance(B1,r1,B2,r2):
    
    cc = 10.0
    epsilon = 0.001
    alpha1 = 1/(dot(r1,r1)*dot(r2,r2)-dot(r1,r2)*dot(r1,r2)+epsilon) * dot(dot(r2,r2)*r1+dot(r1,r2)*(-r2),(B2-B1))
    alpha1_c = lim_func(alpha1)
    
    alpha2 = 1/(dot(r1,r1)*dot(r2,r2)-dot(r1,r2)*dot(r1,r2)+epsilon) * dot(dot(r1,r2)*r1+dot(r1,r1)*(-r2),(B2-B1))
    alpha2_c = lim_func(alpha2)
    
    dd_q = dot((B1 + alpha1_c*r1 - B2 - alpha2_c*r2),(B1 + alpha1_c*r1 - B2 - alpha2_c*r2))
    dd = sqrt(dd_q)
    
    return dd