#!/usr/bin/env python

'''
jr-ros drone python ROS 
Reading ESP8266 over wifi Data and publish the topic
code rev 4
github : jihadrahmawan
last_update = 1/1/2023
'''
##
import socket
import struct
import contextlib 
import rospy
from std_msgs.msg import Int32MultiArray


if __name__=="__main__":
    rospy.init_node('ESP8266')
    
    pub = rospy.Publisher("espData", Int32MultiArray, queue_size=10)
    UDP_IP=""
    UDP_PORT=9000

    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind((UDP_IP,UDP_PORT))
    sock.setblocking(0)
    data = ''
    addr = ''
    r = rospy.Rate(30)
    pritlogConecting = True
    pritlogConected = False
    rospy.loginfo('Waiting Connection from ESP8266...')
    array=[]
    cnt = 0
    with contextlib.closing(sock):
        while not rospy.is_shutdown():
            try:
	            data,addr = sock.recvfrom(1024)
            except socket.error as e:
                pass
            else:
                if cnt == 0 :
                    pritlogConected = True
                
                cnt=cnt+1
                if cnt>5:
                    cnt = 3
                    
                array=[]
                start_charracters = 0
                numITemsData = 3
                for p in range(numITemsData):
                    try:
                       encodedData = int (data[start_charracters:start_charracters+3])
                    except ValueError:
                       rospy.logerr('Decoding ESP data failed!')
                       
                    encodedData = encodedData - 512
                    array.append(encodedData)
                    start_charracters=start_charracters+3

            if pritlogConected:
               rospy.loginfo('Connection succesfully !') 
               pritlogConected = False      
                
            array_forPublish = Int32MultiArray(data=array)
            pub.publish(array_forPublish)
            r.sleep()



