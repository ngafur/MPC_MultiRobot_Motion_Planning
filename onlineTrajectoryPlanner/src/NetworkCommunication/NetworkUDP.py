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
