#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
##################################
'''
everything should now be working as outlined in the lab sheet!
the trouble was actually in remote_control: all I did was add a loop to continue prompting the user for commands until shutdown.  I guess the node was terminating right after the stop command was sent, which cancelled the command? Whatever it was, it's fixed now, and we know that constant_command2 is/was OK, which is the important part.
'''
################################## 

command = Twist()
command.linear.x = 0.0
command.angular.z = 0.0
prevCommandX = 0.0
prevAcc = 0.0
goalVel = 0.0

def messengerCallback(data):
    global goalVel
    #global pub
    global command
    print "got new command!"
    goalVel = data.linear.x
    command.angular.z = data.angular.z
    command.linear.z = data.linear.z
    #print "the command is:"
    #print command.linear.x , command.angular.z, command.linear.z


def bumperCallback(data):
	global pub, goalVel, command
        
        if data.state == BumperEvent.PRESSED:
	        goalVel = 0
	        command.angular.z = 0



def send_commands():
    global goalVel, prevCommandX, prevAcc
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber("kobuki_command",Twist,messengerCallback, queue_size = 10)
    rospy.Subscriber('mobile_base/events/bumper',BumperEvent,bumperCallback)
    rospy.init_node('constant_command', anonymous=True)
    rate = rospy.Rate(1)
    while pub.get_num_connections() == 0:
        pass
    while not rospy.is_shutdown():
        maxAcc = .05        
        command.linear.x = goalVel
        if prevCommandX != goalVel and prevAcc != 0 and (prevCommandX - goalVel) / abs(prevCommandX - goalVel) != prevAcc / abs(prevAcc):
                maxAcc = .025

        if prevCommandX == 0 and abs(goalVel) > maxAcc:
                command.linear.x = goalVel / abs(goalVel) * maxAcc
        elif goalVel == 0 and abs(prevCommandX) > maxAcc:
                command.linear.x = prevCommandX - prevCommandX/abs(prevCommandX) * maxAcc 
        elif prevCommandX != 0 and goalVel != prevCommandX and abs(goalVel) > abs(prevCommandX + (goalVel - prevCommandX)/abs(goalVel - prevCommandX) * maxAcc):
                command.linear.x = prevCommandX + (goalVel - prevCommandX)/abs(goalVel - prevCommandX) * maxAcc
        
        prevAcc = prevCommandX - goalVel
        prevCommandX = command.linear.x

    	pub.publish(command)
	print "publishing " ,command.linear.x, command.angular.z, command.linear.z
    	rospy.sleep(.1)

if __name__ == '__main__':
    try:
        send_commands()
    except rospy.ROSInterruptException:
        pass

