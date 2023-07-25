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