import rospy
rospy.init_node('startSynchronized')
from std_msgs.msg import String
from RobotSetup import *



## Setup net topic
pub = rospy.Publisher('/sync_topic',String)
stop = False
while True:
    def buttonCallback(pub):
        pub.publish('run')
        print('Message has been sent!')

    while True:
        btn = input('Zum Starten "go" eingeben: ')
        if btn == 'go': 
            buttonCallback(pub)
            break
        elif btn in ['exit','quit']:
            stop = True
            break
        else:
            print('Zum Starten "go", zum Beenden "exit"')
    if stop:
        break
