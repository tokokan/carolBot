#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob

import math
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import pygame
import random

musicList = ["BabyItsColdOutside.wav","FelizNavidad.wav","JingleBellRock.wav","LastChristmas.wav","SantaClausIsComingToTown.wav","SilentNight.wav","WeWishYouAMerryChristmas.wav"]

depthData = Image()
isDepthReady = False
state = 0
prevavg = 0
hallwayDist = -1
hallwayWidth = 0
prevdepth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def depthCallback(data):
    global depthData, isDepthReady, dist, pub
    command = Twist()
    depthData = data
    isDepthReady = True

def bumperCallback(data):
        global pub, state
        command = Twist()
        if data.state == BumperEvent.PRESSED :
                if pygame.mixer.music.get_busy():
                        pygame.mixer.music.stop()
                else:        
                        command.linear.x = 0
                        command.linear.z = 0
                        pub.publish(command)
                        state = -1
                       

def blobsCallback(data):
        pass

def odomCallback(data):
        global depthData, isDepthReady, pub, odompub, state, prevdepth, hallwayDist, hallwayWidth, prevavg
        command = Twist()
        totalcount = 0

        if state == 0:
                command.linear.x = 0
                isDepthReady = False
                while not isDepthReady:
                        pass
                
                step = depthData.step
                midX = 320
                midY = 240

                depth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
                for x in range(1, 11):
                        offset = (240 * step) + ((60*x-20) * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                        depth[x-1] = dist

                count = 0.0
                depthsum = 0.0
                leftavg = 0.0
                rightavg = 0.0
                for x in range(0, 4):
                        if not math.isnan(depth[x]):
                                count = count + 1
                                depthsum = depthsum + depth[x]
                                prevdepth[x] = depth[x]

                print count
                if count > 0:
                        leftavg = depthsum / count

                count = 0.0
                depthsum = 0.0

                for x in range(5, 10):
                        if not math.isnan(depth[x]):
                                count = count + 1
                                depthsum = depthsum + depth[x]
                                prevdepth[x] = depth[x]
                print count
                
                if count > 0:
                        rightavg = depthsum / count
        
                error = leftavg - rightavg

                if -.01 < error < .01:
                        command.angular.z = 0
                        odompub.publish(Empty())
                        prevavg = (leftavg + rightavg) / 2
                        state = 1
                        print 'Aligned!'
                else:
                        command.angular.z = -2.5 * error
        elif state == 1:
                command.linear.x = .3
                print hallwayDist
                if not hallwayDist == -1 and data.pose.pose.position.x > hallwayDist - .5:
                        command.linear.x = 0
                        state = 12
                        odompub.publish(Empty())
                isDepthReady = False
                while not isDepthReady:
                        pass
                
                step = depthData.step
                midX = 320
                midY = 240

                depth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
                for x in range(1, 11):
                        offset = (240 * step) + ((60*x-20) * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                        depth[x-1] = dist

                count = 0
                depthsum = 0.0
                leftavg = 0.0
                rightavg = 0.0
                for x in range(0, 4):
                        if depth[x] < prevdepth[x] + .5 and not math.isnan(depth[x]):
                                count = count + 1
                                depthsum = depthsum + depth[x]
                                prevdepth[x] = depth[x]
                        else:
                                prevdepth[x] = -10
                leftcount = count

                #print count
                totalcount = count + totalcount
                if count > 0:
                        leftavg = depthsum / count
                        
                count = 0.0
                depthsum = 0.0

                for x in range(5, 10):
                        if depth[x] < prevdepth[x] + .5 and not math.isnan(depth[x]):
                                count = count + 1
                                depthsum = depthsum + depth[x]
                                prevdepth[x] = depth[x]
                        else:
                                prevdepth[x] = -10
                
                if count > 0:
                        rightavg = depthsum / count
        
                totalcount = count + totalcount

                if leftcount <= 1:
                        command.angular.z = 0
                        command.linear.x = 0
                        state = 2
                        odompub.publish(Empty())
                        prevdepth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        print 'Doorway detected'
                        time.sleep(.5)

                if state == 1:
                        previousDepth = 0.0
                        if count >0:
                                previousDepth = depthsum/count

                        error = (rightavg - leftavg) 
                        print("count is: ",count, " total count: ", totalcount)
                        if  -0.1 < error < 0.1:
                                if -0.01 < error < 0.01:
                                        command.angular.z = 0
                                #elif totalcount < 6:
                                #        command.angular.z = 0
                                #else:
                                #        command.angular.z = -2.5 * error
                                #elif error < -0.01:
                                #        command.angular.z = 2.5*error
                                else:
                                        command.angular.z = 2.5*error
        elif state == 2:
                command.linear.x = 0
                command.angular.z = 0

                isDepthReady = False
                while not isDepthReady:
                        pass
                
                step = depthData.step
                midX = 320
                midY = 240
                depth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                for x in range(1, 11):
                        offset = (240 * step) + ((60*x-20) * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                        depth[x-1] = dist

                clear = True 
                for x in range(4, 6):
                        if not math.isnan(depth[x]) and depth[x] < 1.5:
                                clear = False
                print clear                
                if clear:
                        state = 3
                else:
                        state = 8
                print("state is: ",state)
        elif state == 3:
                q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
	        roll, pitch, yaw = euler_from_quaternion(q)
    	        odomDegree = yaw * 180 / math.pi

                if odomDegree < 85:
                        command.angular.z = .5
                else:
                        command.angular.z = 0
                        pub.publish(command)
                        time.sleep(.5)
                        state = 4
        elif state == 4:
                command.angular.z = 0.0
                pub.publish(command)
                odomDist = data.pose.pose.position.y

                if odomDist < .95:
                        command.linear.x = .3
                else:
                        command.linear.x = 0
                        state = 5
                        pub.publish(command)
        elif state == 5:
                '''
                isDepthReady = False
                while not isDepthReady:
                        pass
                
                step = depthData.step
                midX = 320
                midY = 240
                depth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                for x in range(1, 11):
                        offset = (240 * step) + ((60*x-20) * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                        depth[x-1] = dist

                depthavg = 0.0
                count = 0.0
                for x in range(2, 8):
                        if not math.isnan(depth[x]):
                                depthavg = depthavg + depth[x]
                                count = count + 1

                if count > 0:
                        depthavg = depthavg / count

                print depthavg
'''
                #PLAY MUSIC
                song = musicList[random.randint(0,6)]
                pygame.mixer.music.load("/home/student/carolMusic/"+song)
                pygame.mixer.music.play()
                
                while pygame.mixer.music.get_busy():
                        pass

                state = 6
        elif state == 6:
                odomDist = data.pose.pose.position.y

                if odomDist > .05:
                        command.linear.x = -.3
                else:
                        command.linear.x = 0
                        state = 8
        elif state == 7:
                q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
	        roll, pitch, yaw = euler_from_quaternion(q)
    	        odomDegree = yaw * 180 / math.pi

                if odomDegree > 0:
                        command.angular.z = -.5
                else:
                        command.angular.z = 0
                        state = 8
        elif state == 8:
                q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
	        roll, pitch, yaw = euler_from_quaternion(q)
    	        odomDegree = yaw * 180 / math.pi

                if odomDegree > -90:
                        command.angular.z = -.5
                else:
                        command.angular.z = 0
                        state = 9
        elif state == 9:
                isDepthReady = False
                while not isDepthReady:
                        pass
                
                step = depthData.step
                midX = 320
                midY = 240

                depth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                for x in range(1, 11):
                        offset = (240 * step) + ((60*x-20) * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                        depth[x-1] = dist

                count = 0
                depthsum = 0

                for x in range(4, 6):
                        if not math.isnan(depth[x]):
                                count = count + 1
                                depthsum = depthsum + depth[x]

                if count > 3:
                        hallwayDist = depthsum / count
                else:
                        hallwayDist = -1
                state = 10
        elif state == 10:
                q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
	        roll, pitch, yaw = euler_from_quaternion(q)
    	        odomDegree = yaw * 180 / math.pi

                if odomDegree < 0.05:
                        command.angular.z = .5
                else:
                        command.angular.z = 0
                        state = 11
                pub.publish(command)
        elif state == 11:
                if hallwayDist == -1 or hallwayDist > 2.5:
                        if data.pose.pose.position.x < 1.2:
                                command.linear.x = .3
                        else:
                                if hallwayDist != -1:
                                        hallwayDist = hallwayDist - 1.2
                                command.linear.x = 0
                                state = 0
                else:
                        state = 12
        elif state == 12:
                #END OF HALLWAY
                q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
	        roll, pitch, yaw = euler_from_quaternion(q)
    	        odomDegree = yaw * 180 / math.pi

                if odomDegree < 89:
                        command.angular.z = .5
                else:
                        command.angular.z = 0
                        pub.publish(command)
                        state = 13
        elif state == 13:
                isDepthReady = False
                while not isDepthReady:
                        pass
                
                step = depthData.step
                midX = 320
                midY = 240

                depth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
                for x in range(1, 11):
                        offset = (240 * step) + ((60*x-20) * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                        depth[x-1] = dist

                count = 0
                depthsum = 0

                for x in range(0, 10):
                        if not math.isnan(depth[x]) and depth[x] < 2.5:
                                count = count + 1
                                depthsum = depthsum + depth[x]

                if count > 0:
                        hallwayWidth = depthsum / count

                odompub.publish(Empty())
                state = 14

        elif state == 14:
                odomDist = data.pose.pose.position.x
                if odomDist < hallwayWidth - .5:
                        command.linear.x = .2
                else:
                        command.linear.x = 0
                        state = 15
        elif state == 15:
                q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
	        roll, pitch, yaw = euler_from_quaternion(q)
    	        odomDegree = yaw * 180 / math.pi

                if odomDegree > -90:
                        command.angular.z = -.5
                else:
                        command.angular.z = 0
                        state = 0
                pub.publish(command)
                

                
                
                
                
        pub.publish(command)

def detect_blob():
        global pub
        global blobpub
        global odompub
        pygame.mixer.init()
        rospy.init_node('carolbot', anonymous = True)
        rospy.Subscriber('/blobs', Blobs, blobsCallback,queue_size=10)
        rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
        rospy.Subscriber('mobile_base/events/bumper',BumperEvent,bumperCallback)
        defaultImageTopic = "/camera/rgb/image_color"
        rospy.Subscriber('/odom', Odometry, odomCallback)
        odompub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)  
        pub = rospy.Publisher('kobuki_command', Twist, queue_size = 10)
        #pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        blobpub = rospy.Publisher('/our_blobs', Blob, queue_size = 10)
        rospy.spin()


if __name__ == '__main__':
        detect_blob()
