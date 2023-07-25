import socket
class NetworkUDP:
    
    def __init__(self,hosts, localID, remoteIDs, Nq, Nu, Np, robotParams):
        self.UDPchannels = []

        self.localID = localID
        self.remoteIDs = remoteIDs
        self.robotParams = robotParams
        self.localIP = '127.0.0.1'

        localBasePort = [i[2] for i in hosts]

        self.localPorts = [localBasePort+10*localID+remoteID for remoteID in remoteIDs]

        for i in range(0,len(self.localPorts)):
            channel = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            channel.bind((self.localIP, self.localPorts[i]))
            self.UDPchannels.append(channel)

    def broadcastData(self,trajectory):
        pass


    def receiveData(self):
        pass
