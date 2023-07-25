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

import time
import numpy as np


class SolverBase(object):
    
    def __init__(self,opts):
        self.opts = opts
        self.times = []
        
        
    def setup(self):
        raise NotImplementedError('Class method has not been implemented yet')
        
    def solveNLP(self,*varargin):
        t0 = time.time()
        self.solve(*varargin)
        deltaT = time.time()-t0
        
        # Limit the number of saved times to N elements
        N = 1000
        Ntimes = len(self.times)
        if Ntimes >= N:
            del self.times[0]
            
        self.times.append(deltaT)
        
    def solve(self,varargin):
        raise NotImplementedError('Class method has not been implemented yet')
        
    
    def solverTimes(self):
        times = {}
        times['tmin'] = min(self.times[1:])
        times['tmax'] = max(self.times[1:])
        times['tmean'] = np.mean(self.times[1:])
        times['tstd'] = np.std(self.times[1:])
        
        return times
        
        
        
        

        
        
        
        