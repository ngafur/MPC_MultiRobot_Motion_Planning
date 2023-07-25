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

class Recorder:
    def __init__(self):        
        self.times = []
        self.states = []       #measured states
        self.actions = []
        self.predStates = []
        self.predControls = []
        self.objValue = []
        self.deadlocks = []

    def pushBack(self, time, state, action, predTraj, deadlock):
        self.times.append(time)                                # Add time
        self.states.append(state)                              # Add state
        self.actions.append(action)                            # Add control-inputs
        self.predStates.append(predTraj.states[:,1])           # Add predicted trajectory-State
        self.predControls.append(predTraj.controls[:,0])       # Add predicted trajectory-control
        self.objValue.append(predTraj.objValue)                # Add predicted objective values
        self.deadlocks.append(deadlock)                        # Add deadlock


        # self is a inner flag (i.e., variable), which won't be called outside