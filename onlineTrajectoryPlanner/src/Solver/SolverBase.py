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
        
        
        
        

        
        
        
        