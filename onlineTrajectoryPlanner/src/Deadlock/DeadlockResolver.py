import numpy as np 
from src.CollisionAvoidance.robotCoordinates import robotCoordinates

class DeadlockResolver:
    def __init__(self,*args):
        self.distTol = 0.23 #0.23
        self.resTol = 0.04
        self.Nrobots = len(args)
        self.robotParams = []
        self.states = []
        self.terminalStates = []
        self.robotClusters = []
        for i in range(0,self.Nrobots):
            self.robotParams.append(args[i])
            self.states.append(np.zeros(int(self.robotParams[i].Dynamics['Nq']/2)))
            self.terminalStates.append(np.zeros(int(self.robotParams[i].Dynamics['Nq']/2)))
            self.robotClusters.append([i])

    def updateStatus(self,*args):
        local_deadlocks = np.zeros(self.Nrobots)
        test = np.zeros((self.Nrobots,1))
        residuals = np.zeros(self.Nrobots)
        for i in range(0,self.Nrobots):
            data = args[i]
            
            local_deadlocks[i] = data['deadlock']
            test[i] = data['finalStatus']
            residuals[i] = data['robot_residual']
            self.states[i] = data['robot_state']
            self.terminalStates[i] = data['robot_terminalState']

        #Check for local deadlocks and cluster robots accordingly
        if any(local_deadlocks):
            # Determin set of neighboring robots
            robotsByDist = self.getNeighboringRobots(local_deadlocks)

            # Loop over all groups of robots and update global cluster cell array
            for i in range(0,len(robotsByDist)):
                robotGroup = robotsByDist[i]
                print(('robotGroup:',robotGroup))
                self.updateClusters(robotGroup)
                print(('robotclusters',self.robotClusters))

        updatedStatus = self.statusFromClusters(residuals,test)

        return updatedStatus

    def getNeighboringRobots(self, deadlocks):
        robotIds = np.array(list(range(0,self.Nrobots)),dtype = int)
        robotsByDist = np.array([])
        for i in range(0,self.Nrobots):
            if deadlocks[i] > 0.0:
                distances = self.computeAllDistances(i)
                print(('dist:',distances))
                robotCluster = robotIds[np.array(distances) < self.distTol]
                robotsByDist = np.append(robotsByDist,robotCluster).astype(int)
        print(robotsByDist)
        return [robotsByDist.tolist()]

    def computeAllDistances(self,robIdx):
        pos1Struct = self.forwardKinematics(robIdx,True)
        distances = []
        for i in range(0,self.Nrobots):
            pos2Struct = self.forwardKinematics(i,True)
            minDist = self.computeMinDistance(pos1Struct, pos2Struct)
            distances.append(minDist)

        return distances

    def computeMinDistance(self, pos1Struct, pos2Struct):
        fn1 = list(pos1Struct.keys())
        fn2 = list(pos2Struct.keys())

        distances = []
        for i in range(0,len(fn1)):
            pos1 = pos1Struct[fn1[i]]
            for j in range(0,len(fn2)):
                pos2 = pos2Struct[fn2[j]]
                dis = np.linalg.norm(pos1 - pos2)
                distances.append(dis)
        minDist = min(distances)
        return minDist

    def getIntersectionPairs(self):
        # Check for all active robots, if trajectories potentially intersect
        initPoint = []
        finalPoint = []
        dirVec = []
        for i in range(0,self.Nrobots):
            initPoint.append(self.forwardKinematics(i,True)['B06'])
            finalPoint.append(self.forwardKinematics(i,False)['B06'])
            dirVec.append(finalPoint[:,i]-initPoint[:,i])

        intersectionPairs = []
        for i in range(0,self.Nrobots):
            for j in range(i+1,self.Nrobots):
                # Compute alpha for robot pair (i,j)
                nom = np.cross(initPoint[:,j]-initPoint[:,i], dirVec[:,j])
                den = np.cross(dirVec[:,i], dirVec[:,j])
                alphaIntersect = nom[2]/den[2]

                #Intersection detected
                if (alphaIntersect > 0.0) & (alphaIntersect < 1.0):
                    print(('Potential intersection between robot' ,i, 'and' ,j, 'detected'))
                    intersectionPairs.append([i,j])
        return intersectionPairs

    def updateClusters(self, group):
        # Loop over all clusters
        for i in range(0,self.Nrobots):
            # Current cluster
            clu = self.robotClusters[i]
            print(('clu:',clu))
            # Check if clu and group intersect
            contained = [c in group for c in [clu]]
            # If an intersection is detected
            if any(contained):
                # Replace group at index i with empty array
                self.robotClusters[i] = []
                # Append all not contained elements in clu to group array
                group.append([clu[k] for k in range(0,len(clu)) if not contained[k]])
                print(('group:',group))

        if len(group) > 0:
            # Make sure that group array is always sorted
            group.sort()
            # Add group array to cell array
            idx = int(group[0])
            self.robotClusters[idx] = group



    def statusFromClusters(self, residuals,test):
        # Initialize status
        robotStatus = list(np.zeros(self.Nrobots))

        # Loop over all clusters
        for i in range(0,self.Nrobots):
            # Current cluster
            clu = self.robotClusters[i]
            res_clu = []
            test_clu = []
            for clu_element in clu:
                # residuals of current cluster
                res_clu.append(residuals[clu_element])
                test_clu.append(test[clu_element])

            # Compute ID of robot with minimal residual in cluster
            min_res = np.min(res_clu)
            minIdx = np.argmin(res_clu)

            # If smalles residual in cluster is smaller than the tolerance, reset the cluster
            #if min_res < self.resTol:
            if any(test_clu):
                for clu_element in clu:
                    # Reset all robot states of cluster
                    robotStatus[clu_element] = 1
                # Reset clusters
                for rID in clu:
                    self.robotClusters[rID] = [rID]
            else:
                activeRobot = clu[minIdx]
                # Set status accordingly
                for clu_element in clu:
                    robotStatus[clu_element] = 0
                robotStatus[activeRobot] = 1
        return robotStatus
        
    def forwardKinematics(self, robIdx,trigger):
        param = self.robotParams[robIdx]
        if trigger:
            state = self.states[robIdx]
        else: 
            state = self.terminalStates[robIdx]
        pos = robotCoordinates(state[0], state[1],state[2], state[3], state[4], state[5], param)

        return pos
        
