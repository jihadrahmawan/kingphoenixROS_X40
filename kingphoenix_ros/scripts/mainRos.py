#!/usr/bin/env python

'''
jr-ros drone python ROS 
code rev 4
github : jihadrahmawan
last_update = 1/1/2023
'''
##
#cd ~/catkin_ws/src
#catkin_create_pkg kingphoenix_ros rospy geometry_msgs std_msgs tf2_msgs nav_msgs mavros mavros_msgs sensor_msgs tf pandas cv2 math
#cd kingphoenix_ros

'''
touch scripts/esp8266pub.py
jetson@jr-ros:~/catkin_ws/src/kingphoenix_ros$ chmod +x scripts/esp8266pub.py 
jetson@jr-ros:~/catkin_ws/src/kingphoenix_ros$ touch scripts/mainRos.py
jetson@jr-ros:~/catkin_ws/src/kingphoenix_ros$ chmod +x scripts/mainRos.py 
catkin_ws = catkin_make
roslaunch kingphoenix_ros master.launch

mavproxy.py --master /dev/ttyTHS1 --baud 57600 --out udp:127.0.0.1:14550 

roslaunch mavros apm.launch fcu_url:=udp://:14550@ gcs_url:=udp://:14560@192.168.100.30:14560

'''
import os
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from std_msgs.msg import String, Int32MultiArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import BatteryStatus
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState
import sys
import select
import tty
import termios
from datetime import datetime
import numpy as np
import pandas as pd
import cv2
import math 

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


keyin = ''
pi = math.pi
pi_2 = pi / 2.0
rc = RCIn()
pose_ = Pose()
timestamp_pose = rospy.Time()
lidar= Range()
magneto = MagneticField()
esp_data = Int32MultiArray()
imudata = Imu()
battery = BatteryState()
vehicle_states = State()

def rc_callback(data):
    global rc
    rc = data

def lidar_callback(data):
    global lidar
    lidar = data
    #.range

def magneto_callback(data):
    global magneto
    magneto = data
       
def pose_callback(data):
    global timestamp_pose, pose_
    timestamp = data.header.stamp
    pose_ = data.pose  
    
    
def esp8266_callback(data):
    global esp_data
    esp_data = data
    
def imu_callback (data):
    global imudata
    imudata = data
    #.linear_acceleration

def battery_stat_callback(data):
    global battery
    battery = data

def vehiclestatus_callback(data):
    global vehicle_states
    vehicle_states = data

def goto(pose):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = timestamp
    pose_stamped.pose = pose
    cmd_pos_pub.publish(pose_stamped) 
    
def goto_xyz_rpy(x, y, z, ro, pi, ya):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    goto(pose)

def goto_xyz(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    quat = tf.transformations.quaternion_from_euler(0, 0, 0 + pi_2)

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    goto(pose)

def set_vel(vx, vy, vz, avx=0, avy=0, avz=0):
    cmd_vel = Twist()

    cmd_vel.linear.x = vx
    cmd_vel.linear.y = vy
    cmd_vel.linear.z = vz

    cmd_vel.angular.x = avx
    cmd_vel.angular.y = avy
    cmd_vel.angular.z = avz

    cmd_vel_pub.publish(cmd_vel)

def arm():
    return arm_service(True)

def disarm():
    return arm_service(False)

def takeoff(height=1.0):
        
    #mode_resp = self.mode_service(custom_mode="0")
    mode_resp = mode_service(custom_mode="4")
    arm()
    takeoff_resp = takeoff_service(altitude=height+0.5)

    return mode_resp

def land():
    resp = mode_service(custom_mode="9")
    disarm()

  
def next_trigger (Xtarget, Ytarget, Ztarget, Xreal, Yreal, Zreal, tolerant):

    if (((abs(Xreal)<=(abs(Xtarget)+tolerant)) and (abs(Xreal)>=(abs(Xtarget)-tolerant))) and
       ((abs(Yreal)<=(abs(Ytarget)+tolerant)) and (abs(Yreal)>=(abs(Ytarget)-tolerant))) and
       ((abs(Zreal)<=(abs(Ztarget)+tolerant)) and (abs(Zreal)>=(abs(Ztarget)-tolerant)))):

        return True

    else :

        return False

def scaling (inp, in_min, in_max, out_min, out_max):
 
    out = (abs(inp) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    if inp<0 :
        return -out
    else:
        return out




if __name__=="__main__":
     
    
    rospy.init_node("master")
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
    rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, lidar_callback)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, magneto_callback)
    rospy.Subscriber("/mavros/imu/data_raw", Imu, imu_callback)
    rospy.Subscriber("/mavros/state", State, vehiclestatus_callback)
    rospy.Subscriber("/mavros/battery", BatteryState, vehiclestatus_callback)
    rospy.Subscriber("/espData", Int32MultiArray, esp8266_callback)

    cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
    cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
    rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
    mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)


    '''
    flight mode control tabel source: http://wiki.ros.org/mavros/CustomModes
    Numeric     String      Description and notes
    0           STABILIZE
	1           ACRO
	2           ALT_HOLD
	3           AUTO
	4           GUIDED
	5           LOITER
	6           RTL
	7           CIRCLE
	8           POSITION    not in list
    9           LAND
	10          OF_LOITER
	11          DRIFT       renamed, prev name: APPROACH
    13          SPORT
	14          FLIP
	15          AUTOTUNE
	16          POSHOLD
	17          BRAKE
	18          THROW
	19          AVOID_ADSB
	20          GUIDED_NOGPS
    '''
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)  
    
    listener_t265 = tf.TransformListener()
    listener_t265.waitForTransform("/camera_odom_frame", "/camera_link", rospy.Time(), rospy.Duration(3.0)) 
    
    
    old_settings = termios.tcgetattr(sys.stdin)
   
    
    target_posX =[0.0, 1.2, 3.5, 4.5]
    target_posY =[0.0, 1.2, 4.4, 5.6]
    target_posZ =[2.0, 2.0, 1.2, 1.5]
    r = rospy.Rate(15) #speed loop in Hz
    delay_ = 40
    cnt = 0
    saveposX=[]
    saveposY=[]
    saveposZ=[]
    saverotX=[]
    saverotY=[]
    saverotZ=[]
    saveimuaccX=[]
    saveimuaccY=[]
    saveimuaccZ=[]
    saveLidar=[]
    saveTime=[]
    saveBattVol=[]


    pitch = 0.0
    roll = 0.0
    keyTakingOffEsp = 0
    max_speed_moveESP = 1.5
    max_angle = 30.0
    targetAltitude = 1.5
    takingoff = False
    landing = True
    
    here = os.path.abspath(os.path.dirname(__file__))
    parent = os.path.dirname(here)

    #rospy.loginfo("Welcome to JR_ROS Kingphoenix X40")
    #rospy.loginfo("Ready for take off, Enter s to start and c for cancel the mision!")
    print ("[INFO] HEllo, KIngphoenix team")
    print ("[INFO] Press m for manual (remote) or a for auto (GUided mission)")
    modeRemote = None
    modeAuto = None
    pathFile = '/home/jetson/Desktop/'
    
    mission_start = False
    rostime = 0
    trigger_task = False
    step = 0
    index_mision = 1
    trigger_saveLog = False
    RecordingData = False



    try:

        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
        
            if len (esp_data.data) >= 3 :
            	roll, pitch, keyTakingOffEsp = esp_data.data
            else:
                pitch, roll, keyTakingOffEsp = 0,0,1 
            
            if isData():
                keyin = sys.stdin.read(1)
            
            try :
                (trans,rot) = listener_t265.lookupTransform("/camera_odom_frame", "/camera_link",rospy.Time(0))
                posx,posy,posz = trans
                rotx,roty,rotz, _ = rot	   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


            pitch = float (pitch)
            if pitch >= max_angle:
                pitch = max_angle
            if pitch <= -max_angle:
                pitch = -max_angle

            roll = float (roll)
            if roll >= max_angle:
                roll = max_angle
            if roll <= -max_angle:
                roll = -max_angle

            velocityX = round (scaling(pitch, 0.0, max_angle, 0.0, max_speed_moveESP),3)  
            velocityY = round (scaling(roll, 0.0, max_angle, 0.0, max_speed_moveESP),3)  

            #=====================================================================================
            #                                  Mission flow
            #=====================================================================================

            if keyin == 'm' or keyin == 'M' :   #memilih mode manual dengan ESP REmote
                modeRemote = True
                modeAuto = False
                mission_start = False

                step = 0
                print ("[INFO] MODE ESP On")
                keyin = '' 

            if keyin == 'a' or keyin == 'A' : #memilih mode otomatis dengan guided mission
                modeAuto = True
                modeRemote = False
                mission_start = False
                trigger_saveLog = True

                step = 0
                print ("[INFO] MODE Auto On, Non ESP remote")
                keyin = ''   

            if keyin == 'c' or keyin == 'C' : #memilih pendaratan darutan, Drone landing
                modeAuto = None
                modeRemote = None
                rospy.logwarn("CANCEL KEY PRESSED! CANCEL THE MISSION, ROBOT LANDING ..")
                land()
                keyin=''


            #==================================== Mode Remote ==================================
            if modeRemote and modeAuto == False: 

                print ("[INFO] Kecepatan Maju Mundur = ",velocityX)
                print ("[INFO] Kecepatan Kanan Kiri = ",velocityY)
                print ("[INFO] Tombol Takeoff State = ",keyTakingOffEsp)
                print ("[INFO] Lidar Ketingian (m) = ",lidar.range)

                if keyin == 's' or keyin == 'S':
                    mission_start = True
                    keyin = ''

                if mission_start:
                    if step == 0:
                        if keyTakingOffEsp == 0 :
                            takingoff = True
                            landing = False
                            takeoff(height=targetAltitude)
                        if takingoff:
                            print ("[INFO] ============== Taking off ...")
                            if lidar.range >= targetAltitude:
                                landing = False
                                step = 1
                        else:
                            print ("[INFO] REady for take off, press esp button for takingoff")
                    
                    if step == 1:
                        print ("[INFO] ESP COntrol active, please be aware with KIngphoenix vehicle")
                        set_vel(velocityX, velocityY, 0.0, avx=0, avy=0, avz=0)
                        if keyTakingOffEsp == 0 :
                            landing = True
                            takingoff = False
                            land()
                        if landing:
                            print ("[INFO] ================== Landing...")
                            if lidar.range <= 0.3:
                                #akhiri dan matikan mode remot setelah landing
                                modeRemote = None
                else:
                    print ("[INFO] Pres s atau S to active controll")


            #==================================== Mode Auto ==================================
           
            if modeAuto and modeRemote == False: 

                print ("[INFO] X T265 = ",posx)
                print ("[INFO] Y T265 = ",posy)
                print ("[INFO] Z T265 = ",posz)
                print ("[INFO] Z lidar= ",lidar.range)

                if keyin == 's' or keyin == 'S':
                    mission_start = True
                    RecordingData = True
                    takeoff(height=target_posZ[0])
                    keyin = ''


                if mission_start:
                    
                    if step == 0 :
                        print ("[INFO] ================== Taking off ...")
                        if lidar.range >= target_posZ[0]:
                            trigger_task = True
                            step = 1
                    
                    if step == 1:
                        if trigger_task:
                            goto_xyz(target_posX[index_mision],target_posY[index_mision],target_posX[index_mision])
                            trigger_task = False
                       
                        if next_trigger(target_posX[index_mision],target_posY[index_mision],target_posX[index_mision],posx,posy,posz,0.3):
                            print ("[INFO]      Sampai di titik = ",index_mision)
                            index_mision=index_mision+1

                            if index_mission<len(target_posX):
                                trigger_task = True
                                step = 2
                            else:
                                #full mission, index mission set 0 return to HOme
                                trigger_task = True
                                index_mision=0
                                step = 3
                        else:
                            print ("[INFO]      Menuju ke titik = ", index_mision)

                    if step == 2:
                        cnt=cnt+1
                        print ("[INFO] ================== DROP LOGISTIK")
                        if (cnt>=delay_):
                            cnt = 0
                            step=1
                    
                    if step == 3:
                        if trigger_task:
                            
                            goto_xyz(target_posX[index_mision],target_posY[index_mision],target_posZ[index_mision])
                            tigger_task = False

                        if next_trigger(target_posX[index_mision],target_posY[index_mision],target_posX[index_mision],posx,posy,posz,0.3):
                            land()
                            step = 4
                        else:
                            print ("[INFO] ================== Kembali ke HOme")

                    if step == 4:
                        if lidar.range <= 0.3:
                            print ("[INFO] Landed ! Mission done, data save..")
                            RecordingData = False

                else:
                    print ("[INFO] Pres s atau S to takeoff")
                    print ("[INFO] Data sensor stop recording")


                if RecordingData:

                    print ("[INFO] SEdang merekam data sensor...")
                    saveposX.append(round(posx,3))
                    saveposY.append(round(posy,3))
                    saveposZ.append(round(posz,3))
                    saverotX.append(round(rotx,3))
                    saverotY.append(round(roty,3))
                    saverotZ.append(round(rotz,3))
                    saveimuaccX.append(round(imudata.linear_acceleration.x,3))
                    saveimuaccY.append(round(imudata.linear_acceleration.y,3))
                    saveimuaccZ.append(round(imudata.linear_acceleration.z,3))
                    saveLidar.append(round(lidar.range,3))
                    now = rospy.get_rostime()
                    saveTime.append(now.secs)
                    saveBattVol.append(battery.voltage) 

                else:
                    if trigger_saveLog:
                        rospy.logwarn("Data excel saving...")
                        dataSave = {'rosTime(ns)':saveTime,
                                    'positionX(m)':saveposX,
                                    'positionY(m)':saveposY,
                                    'positionZ(m)':saveposZ,
                                    'rotationX(rad)':saverotX,
                                    'rotationY(rad)':saverotY,
                                    'rotationZ(rad)':saverotZ,
                                    'imuaccX(rad)':saveimuaccX,
                                    'imuaccX(rad)':saveimuaccY,
                                    'imuaccX(rad)':saveimuaccZ,
                                    'lidar(m)':saveLidar,
                                    'battery(v)':saveBattVol,
                                    }
                                
                        dataframe = pd.DataFrame(dataSave) 
                        print ("[INFO] Save data = ", dataframe)
                        
                        filenamecsv = pathFile+'percobaanTanggal_'+datetime.now().strftime("%d-%m-%-Y%H-%M-%S")+'.csv'
                        filenamexls=  pathFile+'percobaanTanggal_'+datetime.now().strftime("%d-%m-%-Y%H-%M-%S")+'.xls'
                        dataframe.to_csv(filenamecsv) 
                        dataframe.to_excel(filenamexls) 
                        saveposX=[]
                        saveposY=[]
                        saveposZ=[]
                        saverotX=[]
                        saverotY=[]
                        saverotZ=[]
                        saveimuaccX=[]
                        saveimuaccY=[]
                        saveimuaccZ=[]
                        saveLidar=[]
                        saveTime=[]
                        saveBattVol=[]
                        print ("[INFO] Data TErsimpan")
                        trigger_saveLog=False

            r.sleep()

    finally:
        rospy.logwarn("JR_ROS Shuting down")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
 
