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

from casadi import mtimes
from src.CollisionAvoidance.lim_func import lim_func


def Distance(Rot,H,x0,B_r,r):
    
    alpha1 = -mtimes(mtimes(mtimes(mtimes(((B_r-x0).T),Rot),H),Rot.T),r)/mtimes(mtimes(mtimes(mtimes(r.T,Rot),H),Rot.T),r)
    
    alpha1_c = lim_func(alpha1)
    
    dd = mtimes(mtimes(mtimes(mtimes((B_r+alpha1_c*r-x0).T,Rot),H),Rot.T),(B_r+alpha1_c*r-x0))
    
    return dd