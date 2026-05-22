
# Benchmark runs can suppress diagnostic prints to reduce stdout overhead.
try:
    import os as _benchmark_os
    if _benchmark_os.environ.get("WEBOTS_BENCHMARK_QUIET") == "1":
        def print(*args, **kwargs):
            pass
except Exception:
    pass

# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# CACC car-following model based on [1], [2].
# [1] Milanes, V., and S. E. Shladover. Handling Cut-In Vehicles in Strings
#    of Cooperative Adaptive Cruise Control Vehicles. Journal of Intelligent
#     Transportation Systems, Vol. 20, No. 2, 2015, pp. 178-191.
# [2] Xiao, L., M. Wang and B. van Arem. Realistic Car-Following Models for
#    Microscopic Simulation of Adaptive and Cooperative Adaptive Cruise
#     Control Vehicles. Transportation Research Record: Journal of the
#     Transportation Research Board, No. 2623, 2017. (DOI: 10.3141/2623-01).

"""vehicle_driver controller."""
from vehicle import Driver
from controller import Supervisor, Robot, Node, RadarTarget, Radar, Receiver, Emitter, LED, Joystick, Display, ImageRef
import array
import os
import struct
import time
import numpy as np
import math
#from playsound2 import playsound
import playsound as playsound
import threading
import csv
import sys 
sys.path.append("..") 
from ContrlPara import ContrlPara
#global_lock = threading.Lock()
from openal import *

import pygame
import threading

def play_sound(filepath):
    pygame.mixer.init()
    pygame.mixer.music.load(filepath)
    pygame.mixer.music.play()


def get_latest_file(dir):
    '''search latest file'''
    file_lists = os.listdir(dir)
    file_lists.sort(key=lambda fn: os.path.getmtime(dir + "\\" + fn)\
                    if not os.path.isdir(dir + "\\" + fn) else 0)
    file = os.path.join(dir, file_lists[-1])
    return file


def apply_PID(position, targetPosition, curAngle, ctrlPara):
    """Apply the PID controller and return the angle command."""

    if abs(curAngle)>=0.01:
        P = ctrlPara.LaneChg_PID_P
        I = ctrlPara.LaneChg_PID_I
        D = ctrlPara.LaneChg_PID_D
        K = ctrlPara.LaneChg_PID_K    
    
    else:
        
        P = ctrlPara.LaneChg_PID_P_S
        I = ctrlPara.LaneChg_PID_I_S
        D = ctrlPara.LaneChg_PID_D_S
        K = ctrlPara.LaneChg_PID_K_S
        
    diff = position - targetPosition
    if apply_PID.previousDiff is None:
        apply_PID.previousDiff = diff
    # anti-windup mechanism
    if diff > 0 and apply_PID.previousDiff < 0:
        apply_PID.integral = 0
    if diff < 0 and apply_PID.previousDiff > 0:
        apply_PID.integral = 0
    apply_PID.integral += diff
    # compute angle
    angle = P * diff + I * apply_PID.integral + D * (diff - apply_PID.previousDiff)+K*curAngle
    apply_PID.previousDiff = diff
    if angle >0.1:
        angle = 0.1
    if angle <-0.1:
        angle = -0.1
    return angle



def getLeaderInfo(radar, singleGap, mySpeed, ctrlPara):
    numVeh=radar.getNumberOfTargets()
    preVeh=False
    gap=ctrlPara.maxDetectRange
    speedDiff=ctrlPara.myMaxSpeed-mySpeed
    if numVeh>0:
        preObj=radar.getTargets()
        for i in range(0,numVeh):
            #if preObj[i].distance<15:
                #print("preObj[i].distance",preObj[i].distance, "preObj[i].azimuth", preObj[i].azimuth, "received_power", preObj[i].received_power,"gap",gap, "singleGap",singleGap )
            if abs(preObj[i].azimuth) == 0.2 and abs(preObj[i].azimuth) > (0.5*ctrlPara.laneWidth-0.3)/(preObj[i].distance+ctrlPara.vehLength-1):
                continue
            if (preObj[i].distance<gap and abs(preObj[i].azimuth) < (0.5*ctrlPara.laneWidth-0.3)/preObj[i].distance and abs(preObj[i].azimuth) <= 0.2):# or preObj[i].distance<singleGap:
            #if (preObj[i].distance<gap and abs(preObj[i].azimuth) < 0.015):
                #print("preObj[i].distance",preObj[i].distance, "preObj[i].azimuth", preObj[i].azimuth, "received_power", preObj[i].received_power,"gap",gap, "singleGap",singleGap )
                
                    
                preVeh=True
                if preObj[i].distance<gap:
                    gap = preObj[i].distance
                    speedDiff=preObj[i].speed
                
    return preVeh, gap, speedDiff



def speedSpeedControl(speed, vErr, mySpeedControlGain):
    sclAccel = mySpeedControlGain * vErr
    newSpeed = speed + sclAccel
    return newSpeed
    
    
def speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action, time_step, myHeadwayTime, ctrlPara):
    if preVeh:
        desHeadwayTime = myHeadwayTime
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 5
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*ctrlPara.cutinDuration)*(myHeadwayTime-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
        
        desSpacing = desHeadwayTime * speed
        gap = gap2pred - ctrlPara.myMinGap
        spacingErr = gap - desSpacing
        #######original spacingErr1 = predSpeed - speed + myHeadwayTime * accel;##########
        #spacingErr1 = predSpeed - speed + myHeadwayTime * accel
        spacingErr1 = -vErr - desHeadwayTime * accel
        #print("desHeadwayTime", desHeadwayTime,"desSpacing", desSpacing, "gap",gap)

        if spacingErr > 0 and spacingErr < 0.2 and vErr < 0.1:#gap mode newSpeed = speed + 0.45 * spacingErr + 0.0125 *spacingErr1;
            newSpeed = speed + ctrlPara.myGapControlGainGap * spacingErr + ctrlPara.myGapControlGainGapDot * spacingErr1
        elif spacingErr < 0:# collision avoidance mode newSpeed = speed + 0.45 * spacingErr + 0.05 *spacingErr1;
            newSpeed = speed + ctrlPara.myCollisionAvoidanceGainGap * spacingErr + ctrlPara.myCollisionAvoidanceGainGapDot * spacingErr1
        else:# gap closing mode 
            newSpeed = speed + ctrlPara.myGapClosingControlGainGap * spacingErr + ctrlPara.myGapClosingControlGainGapDot * spacingErr1
        
    else:
        newSpeed = speedSpeedControl(speed, vErr, ctrlPara.mySpeedControlGain)
    
    
    return newSpeed

def speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,myGps_z):
    if preVeh:
        desHeadwayTime = myHeadwayTime
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 10
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*ctrlPara.cutinDuration)*(myHeadwayTime-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
            
        gap = gap2pred - ctrlPara.myMinGap
        optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime))
        
       #ctrlPara.myDesSpeed=25 m/s
       # Gap_max=400
       #ctrlPara.myMaxSpeed=40:
        if gap < ctrlPara.myMaxGap:   
            optSpeed = max(0, min(ctrlPara.myDesSpeed[ringNum],gap/desHeadwayTime)) #Added 
        else:    
            optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime)) #Added 

        
        accel = (optSpeed-speed)/ctrlPara.T_adpt + ctrlPara.Lamda*(predSpeed-speed)
        
        if accel>ctrlPara.maxAccel:
            accel = ctrlPara.maxAccel
        if accel<ctrlPara.minAccel:
            accel = ctrlPara.minAccel
            
        newSpeed = speed + accel*time_step/1000
        #print("fvdm-newspeed", newSpeed)
      
    else:
        #newSpeed = speedSpeedControl(speed, vErr, ctrlPara.mySpeedControlGain)
        
        desHeadwayTime = myHeadwayTime
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 10
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*ctrlPara.cutinDuration)*(myHeadwayTime-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
            
        gap = gap2pred - ctrlPara.myMinGap
        optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime))
        
        accel = (optSpeed-speed)/ctrlPara.T_adpt + ctrlPara.Lamda*(predSpeed-speed)
        if accel>ctrlPara.maxAccel:
            accel = ctrlPara.maxAccel
        if accel<ctrlPara.minAccel:
            accel = ctrlPara.minAccel
            
        newSpeed = speed + accel*time_step/1000
        #print("fvdm-gap", gap,"fvdm-optSpeed", optSpeed,"fvdm-accel", accel,"fvdm-newSpeed", newSpeed)
    
    if myGps_z>ctrlPara.Pos_Msg6[ringNum] and myGps_z<ctrlPara.Pos_Msg7[ringNum]:
        newSpeed=speed
    
    return newSpeed



def speed_CACC(preVeh, gap2pred, predSpeed, speed, accel,CACC_ControlMode, lane_change_action,time_step, myHeadwayTime, ctrlPara):
    newSpeed = 0.0

    if speed >0.001:
        time_gap = gap2pred / speed
    else:
        time_gap = 5
    
    if preVeh and time_gap<=5:
        vErr = speed - predSpeed
    else:
        vErr = speed - ctrlPara.myMaxSpeed
        
    if time_gap > 3:
        #newSpeed = speedSpeedControl(speed, vErr, lane_change_action)
        #newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara)        
        newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,myGps_z)


        CACC_ControlMode[0] = 0
    elif time_gap <= 2:
        #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel,lane_change_action,time_step, myHeadwayTime,ctrlPara)
        #newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara)                
        newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,myGps_z)
        CACC_ControlMode[0] = 1
    else:
        if CACC_ControlMode[0]==0:
            #newSpeed = speedSpeedControl(speed, vErr,lane_change_action)
            #newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara)        
            newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,myGps_z)

            
        else:
            #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action,time_step, myHeadwayTime, ctrlPara)
            #newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara)        
            newSpeed =speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,myGps_z)

    return newSpeed

        

#Previous 
#display function================================================        
# def display_update(display, speed, myGps_z, ringNum, ctrlPara):

    # display.imagePaste(DefaultIcon, 0, 0, False)
    
    # if myGps_z>= ctrlPara.RRX_TLonrmp_trans_z[ringNum]-ctrlPara.DeltaZ_TL_onramp and myGps_z < ctrlPara.RRX_TLonrmp_trans_z[ringNum]:
        # display.imagePaste(MergeIcon, 0, 0, False)
    # elif myGps_z >=ctrlPara.Pos_Msg2b[ringNum] and myGps_z < ctrlPara.pos_cadTohd[ringNum]:
        # display.imagePaste(MoveToRightIcon, 0, 0, False)
    # elif myGps_z >=ctrlPara.Pos_Msg7[ringNum] and myGps_z < ctrlPara.Pos_Msg8[ringNum]:
        # display.imagePaste(DivergeIcon, 0, 0, False)
    
    # display.drawText("{:.0f}".format(ctrlPara.mySpeedLimit[ringNum]*3.6)+' Km/h', 534, 138)    
    # display.drawText("{:.1f}".format(speed*3.6)+' Km/h', 500, 340)
#========================================================================

#Revised
#New display function          
#Display function was revised : inevitable revisions were deemed necessary after after pretesting because:
    #There should be two default icons:
        # one default in the main (load 
        # one default outside the mainfreeway   
def display_update(display, speed, myGps_z, ringNum, ctrlPara, Main_road):

    if Main_road:  # The vehicle is driving within the main freeway sections
        # Move to right condition
        if myGps_z >=ctrlPara.Pos_Msg2b[ringNum] and myGps_z < ctrlPara.pos_cadTohd[ringNum]:
            display.imagePaste(MoveToRightIcon, 0, 0, False)
        # Diverge condition    
        elif myGps_z >=ctrlPara.Pos_Msg7[ringNum] and myGps_z < ctrlPara.Pos_Msg8[ringNum]:
            display.imagePaste(DivergeIcon, 0, 0, False)        
       # Default in the main freeway    
        else: 
            display.imagePaste(MainDefaultIcon, 0, 0, False) # This should be deafualt.  
            
    else: # The vehicle is not driving in the in the main freeway     
        if myGps_z>= ctrlPara.RRX_TLonrmp_trans_z[ringNum]-ctrlPara.DeltaZ_TL_onramp and myGps_z < ctrlPara.RRX_TLonrmp_trans_z[ringNum]:
            display.imagePaste(MergeIcon, 0, 0, False) # merge condition
        else:
            display.imagePaste(RampDefaultIcon, 0, 0, False) # This should be deafualt.      
        
    #display.drawText("{:.0f}".format(ctrlPara.mySpeedLimit[ringNum]*3.6)+' Km/h', 290, 50)
    display.setColor(0xFF0000)    
    display.drawText("{:.0f}".format(ctrlPara.mySpeedLimit[ringNum]*3.6), 15, 20)    
    

    #display.drawText("{:.1f}".format(speed*3.6)+' Km/h', 290, 220)
    display.setColor(0xABA7AA)
    display.drawText("{:.1f}".format(speed*3.6)+' Km/h', 130, 125)
    
   
# play message and return messageId& switchHD
def playMessage(myGps_z, ctrlPara, toPlayMessageId, ringNum):
    Message_Played_bool = False

    messageId = ''
    nextMessageId = toPlayMessageId
    if  toPlayMessageId == 'Msg0' and myGps_z >= ctrlPara.Pos_Msg0:
        messageId = 'Msg0'
        nextMessageId = 'Msg1'

    if  toPlayMessageId == 'Msg4' and myGps_z >= ctrlPara.Pos_Msg4[ringNum]:
        messageId = 'Msg4'
        if ctrlPara.RRX_NDRT[ringNum] ==1:
            nextMessageId = 'Msg5b'        
        else:
            nextMessageId = 'Msg5a'        
        print("current Message ID is",messageId,
        "Next Msg ID is",nextMessageId,
        "ctrlPara.RRX_NDRT[ringNum] is",ctrlPara.RRX_NDRT[ringNum])
    if  toPlayMessageId == 'Msg5a' and myGps_z >= ctrlPara.Pos_Msg5a[ringNum]:
        messageId = 'Msg5a'    
        nextMessageId = 'Msg6'
        
    if  toPlayMessageId == 'Msg5b' and myGps_z >= ctrlPara.Pos_Msg5b[ringNum]:
        messageId = 'Msg5b'    
        nextMessageId = 'Msg6'
        
    if  toPlayMessageId == 'Msg6' and myGps_z >= ctrlPara.Pos_Msg6[ringNum]:
        messageId = 'Msg6'    
        nextMessageId = 'Msg7'
        
    if messageId != '':
        filepath = f'../soundPlay/{messageId}.mp3'
        threading.Thread(target=play_sound, args=(filepath,), daemon=True).start()
        message_played_bool = True
        # threading.Thread(target=playsound, args=('../soundPlay/'+ messageId +'.mp3',), daemon=True).start()
        # Message_Played_bool = True

    return Message_Played_bool, messageId, nextMessageId


def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))




##### program start here #########
###import system parameters

ctrlPara = ContrlPara()
paraFilename = '../Controller_input_param.csv'
#read csv, and split on "," the line
P_File = open(paraFilename, "r")
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadVehInfo(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.IsMsgPlay(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadGeoPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadVehPosPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.load_TL_PosPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadTimeSyn(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadCtrlPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.load_Msg_PosPara(paraFile)
P_File.seek(0)

#ctrlPara.numTrafficLightRoad=3 #Added

## define cut_in state
cutIn_action = ('CUT_NO', 'CUT_IN', 'CUT_OUT')

#################### define controller (CACC and FVDM)parameters   #######################################

CACC_ControlMode=[0]


#################### define display information  #######################################
RampDefaultIcon_file='../icon/RampDefault.png' #Added
MainDefaultIcon_file = '../icon/MainDefault.png' #Added
MergeIcon_file = '../icon/Merge.png'
MoveToRightIcon_file = '../icon/MoveToRight.png'
DivergeIcon_file = '../icon/Diverge.png'
#DefaultIcon_file = '../icon/Default.png' #Previous


TurningLeftIcon_file = '../icon/left-arrow.png'
TurningRightIcon_file = '../icon/right-arrow.png'
apply_PID.integral = 0
apply_PID.previousDiff = None

get_filtered_speed.previousSpeeds = []

###########define drivng mode switch button###############
SwitchMode = 23


driver = Driver()


veh = driver.getSelf()
controllerName = veh.getField("controller")
#print("current controller is ",controllerName.getSFString())
myPosition = veh.getField("translation")




time_step = int(driver.getBasicTimeStep()) 


driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0)


joystick = driver.getJoystick()
joystick.enable(time_step)



radar = driver.getRadar("radar")
radar.enable(time_step)
preObj0=radar.getTargets()
#print("radar", radar)

gps = driver.getGPS("gps")
gps.enable(time_step)

recv = driver.getReceiver("receiver")
recv.enable(time_step)


display = driver.getDisplay("display")
display.setColor(0xFF0000)
display.setFont("Arial", 20, True)

#DefaultIcon = display.imageLoad(DefaultIcon_file) #Revised: No longer needed

RampDefaultIcon=display.imageLoad(RampDefaultIcon_file) #Added
MainDefaultIcon=display.imageLoad(MainDefaultIcon_file) #Added 
MergeIcon = display.imageLoad(MergeIcon_file)
MoveToRightIcon = display.imageLoad(MoveToRightIcon_file)
DivergeIcon = display.imageLoad(DivergeIcon_file)

#display.imagePaste(DefaultIcon, 0, 0, False) #Revised: Changed to:
display.imagePaste(RampDefaultIcon, 0, 0, False)



myTrafficLight = None

driver.step()

#Revised: Added the off-ramp traffic light
## select which traffic light should be controlled by participant vehicle
TL_Pos = [[ctrlPara.RRX_TLonrmp_trans_z[0], ctrlPara.RRX_TLImgnry_StnGo_trans_z[0], ctrlPara.RRX_TLImgnry_TakeOver_trans_z[0],ctrlPara.RRX_TLoffrmp_trans_z[0]],\
					[ctrlPara.RRX_TLonrmp_trans_z[1], ctrlPara.RRX_TLImgnry_StnGo_trans_z[1], ctrlPara.RRX_TLImgnry_TakeOver_trans_z[1],ctrlPara.RRX_TLoffrmp_trans_z[1]],\
					[ctrlPara.RRX_TLonrmp_trans_z[2], ctrlPara.RRX_TLImgnry_StnGo_trans_z[2], ctrlPara.RRX_TLImgnry_TakeOver_trans_z[2],ctrlPara.RRX_TLoffrmp_trans_z[2]]]


#Revised: Added the off-ramp traffic light
# Traffic-light DEF names must match the compiled racing_wheel_com controller.
# Road RR1 -> suffix 0, RR2 -> suffix 1, RR3 -> suffix 2.
TL_Name = [['TL_onrmp0', 'TL_img_StnGo0', 'TL_img_takeOver0','TL_offrmp0'],\
									['TL_onrmp1', 'TL_img_StnGo1', 'TL_img_takeOver1','TL_offrmp1'],\
									['TL_onrmp2', 'TL_img_StnGo2', 'TL_img_takeOver2','TL_offrmp2']]
                                    
print(ctrlPara.numRingRoad)




## initialize onramp traffic light with red state
print(f"Number of traffic lights per road is: {ctrlPara.numTrafficLightRoad}")
if gps.getValues()[2]<TL_Pos[0][0]:
    for i in range (0,ctrlPara.numRingRoad): #
        for j in range (0, ctrlPara.numTrafficLightRoad): #
           # print ("TL_Pos",TL_Pos[i][j])
            print("myTLName",TL_Name[i][j])
            myTrafficLight = driver.getFromDef(TL_Name[i][j])
            state_field = myTrafficLight.getField("state") if myTrafficLight is not None else None
            if state_field is not None:
                state_field.setSFString("red")
            else:
                print("[auto_ringroad_driver] WARNING: cannot set red state for", TL_Name[i][j])
        
## set initial traffic light with red





toCtrlTLIndex = [ctrlPara.numRingRoad,ctrlPara.numTrafficLightRoad] #Revised 



afterCtrlTLIndex = [0,0]

for i in range(0,ctrlPara.numRingRoad):
    for j in range(0,ctrlPara.numTrafficLightRoad):
        #myTrafficLight1 = driver.getFromDef(TL_Name[i][j])
        #print ("traffic light state",myTrafficLight1.getField("state").getSFString(), "i = ",i, "j=", j)
        if gps.getValues()[2]<TL_Pos[i][j]:
            myTrafficLight = driver.getFromDef(TL_Name[i][j])
            if myTrafficLight.getField("state").getSFString() == "red":
                toCtrlTLIndex[0]=i
                toCtrlTLIndex[1]=j
                #print("hi",TL_Name[i][j],"",toCtrlTLIndex[0],toCtrlTLIndex[1],",", TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]])
                break
    else:
        continue
    break            

print("participant position ",gps.getValues()[2], "toCtrlTLIndex[0] ",toCtrlTLIndex[0], "toCtrlTLIndex[1] ",toCtrlTLIndex[1])

if joystick.isConnected():
    print(joystick.getModel(),"detected")

print("getNumberOfAxes is ",joystick.getNumberOfAxes())

print("Steering number is",joystick.getAxisValue(0))

print("Throttle number is",joystick.getAxisValue(1))

print("Brake number is",joystick.getAxisValue(2))
   
#decide which ring road number is based on the current location of participant. 
ringNum = int((gps.getValues()[2]-ctrlPara.zoffsetRoad[0])/(ctrlPara.zoffsetRoad[1]-ctrlPara.zoffsetRoad[0]))

##### play message autodriving reminder #########
toPlayMessageId = ''

if gps.getValues()[2]<TL_Pos[0][0]:
   # threading.Thread(target=playsound, args=('../soundPlay/Auto.mp3',), daemon=True).start() #AutO
    filepath = f'../soundPlay/Auto.mp3'
    threading.Thread(target=play_sound, args=(filepath,), daemon=True).start() 
    message_played_bool = True
    toPlayMessageId = 'Msg0'
else:
    toPlayMessageId = 'Msg4'

### initialize messageId to be played first.

print("ctrlPara.RRX_NDRT[0] is", ctrlPara.RRX_NDRT)
print("RRX_TLImgnry_StnGo_trans_z is", ctrlPara.RRX_TLImgnry_StnGo_trans_z)


### 
fileDate = time.strftime("%Y%m%d")
fileTime = time.strftime("%H%M%S")
fileDir = '../../Extracted data/Participant/'
if toPlayMessageId == 'Msg0':

    id = veh.getField("name").getSFString()
    print(id)
    recFile = fileDir + 'Run-Data-' + fileDate + '-Time-' + fileTime + '-PT' +'.csv'
    fp = open(recFile, 'a', newline = '')

    fields = ['Local_time', 'Main_road', 'Road ID', 'Lane_ID', 'Position_X', 'Position_Z', 'Speed', \
            'Participant_leader_ID', 'Participant_follower_ID', 'Message_Played', 'messageId', 'Driving_Mode', \
            'Participant_Presses_button', 'Participant_Indicates', 'Participant_Indicate_back', 'Participant_Brakes', 
            'Participant_is_hit_from_behind','Participant_crashes_its_leader', 'Participants_TrafficLightAhead_ID', 'Participants_TrafficLightAhead_State', \
            'Participants_TrafficLightBehind_ID','Participants_TrafficLightBehind_State','Spacing','Speed_Difference','Steering_Wheel_Angle','Participant_HandsOn_SteeringWheel','Partiticipant_Press_Pedal']    
    csvwriter = csv.writer(fp)
    csvwriter.writerow(fields)
else:
    recFile = get_latest_file(fileDir)
    fp = open(recFile, 'a', newline = '')
    csvwriter = csv.writer(fp)





isReadyToAutodriving = False


numStep=0

myPreSpeed = 0

## initialise take over warning
takeoverWarning = False

mTimer=0
Time_Threshold = ctrlPara.switchWaitingTime/time_step


pvInfo = []
pv_brake_act = 0
pv_reaction_pv = 0


### take over driving 
isOnramp_takeover = False
isOnbroken_takeover_req = False
isOnbroken_takeover = False



##### initial value for recordings
Main_road = False
Message_Played = False
Participant_Presses_button = 3
Participant_Indicates = 0
Participant_Indicate_back = -1
Participant_Brakes = False


Participants_TrafficLightAhead_ID = 0
Participants_TrafficLightBehind_ID = 0
Participants_TrafficLightBehind_State = 0
Participants_TrafficLightAhead_State = 0

# ### add  engine background sound 
engineSndSource = oalOpen("../soundPlay/engine.wav")
engineSndSource.play()
listener = Listener()
THROTTLE_TO_VOLUME_GAIN = 0.5
RPM_TO_VOLUME_GAIN = 0.2
engine_max_rpm = 6500
engine_min_rpm = 1000
engine_sound_rpm_reference = 1000
newSpeed=25
while driver.step() != -1:
    ## first step is used for sensor warming up 
    myCurSpeed = gps.getSpeed()
    #here myGps_z is the GPS postion of vehicle, different from vehicle position in Webots. GPS offset is needed.
    myGps_x = gps.getValues()[0]
    myGps_z = gps.getValues()[2]
    
    gear_ratio=5
    average_speed = myCurSpeed*60/(2*3.14159*0.374)
    rpm = average_speed * gear_ratio
    if rpm >engine_max_rpm:
        rpm=engine_max_rpm
    speed_diff = newSpeed - myCurSpeed
    if speed_diff < 0:
        speed_diff = 0.0
    if speed_diff > 25.0:
        speed_diff = 25.0
    
    engineVolume = (1.0 - THROTTLE_TO_VOLUME_GAIN) + THROTTLE_TO_VOLUME_GAIN * speed_diff / 25.0
    engineVolume = engineVolume*((1.0 - RPM_TO_VOLUME_GAIN) + RPM_TO_VOLUME_GAIN * rpm/engine_max_rpm)
    
    if rpm < engine_min_rpm:
        rpm = engine_min_rpm
    enginePitch = rpm / engine_sound_rpm_reference
    ###play engine sound
    
    listener.set_position([myGps_x, 1.54, myGps_z])
    
    # check if the file is still playing
    if engineSndSource.get_state() != AL_PLAYING:
    # and start playback
        engineSndSource.play()
    engineSndSource.set_looping(True)
    engineSndSource.set_pitch(enginePitch*0.5)
    engineSndSource.set_gain(engineVolume*0.5)
    
    
    ### decide the current ring road number
    if not isReadyToAutodriving:
        ringNum = int((myGps_z-ctrlPara.zoffsetRoad[0])/(ctrlPara.zoffsetRoad[1]-ctrlPara.zoffsetRoad[0]))
        if ringNum> ctrlPara.numRingRoad-1:
            ringNum = ctrlPara.numRingRoad-1
        if ringNum<0:
            ringNum = 0
        isReadyToAutodriving = True
       
    lane_index = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)  

    Main_road = True
    if lane_index>ctrlPara.numLane-1:
        lane_index = ctrlPara.numLane-1
        Main_road = False
    if lane_index<0:
        lane_index = 0
        Main_road = False
        

    if numStep%(ctrlPara.infoDispTime/time_step) ==0:      
    #Previous
        #display_update(display, myCurSpeed, myGps_x, myGps_z, ringNum, ctrlPara)
    #Revised
        display_update(display, myCurSpeed, myGps_z, ringNum, ctrlPara, Main_road)            
    
    #decide which lane the vehicle is located in.
    singleGap = ctrlPara.myDesSpeed[ringNum]*ctrlPara.myHeadwayTime[ringNum][lane_index] + ctrlPara.myMinGap
    preVeh, gap2pred, speedDiff = getLeaderInfo(radar, singleGap, myCurSpeed, ctrlPara)
    #print("preVeh", preVeh,"gap2pred", gap2pred, "speedDiff",speedDiff)
    gap2pred = gap2pred - 0.5*ctrlPara.vehLength
    predSpeed = speedDiff + myCurSpeed
    myAccel = (myCurSpeed-myPreSpeed)/(time_step/1000)

    myPreSpeed = myCurSpeed

    lane_change_action = cutIn_action[0]



    #implement desired longitudinal speed.
#    newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, ctrlPara.myHeadwayTime[ringNum][lane_index], ctrlPara)
    newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, ctrlPara.myHeadwayTime[ringNum][lane_index], ctrlPara)

    if newSpeed-myCurSpeed<-0.01 or myCurSpeed<=0.001:
        driver.setBrakeIntensity(0.001)
    if newSpeed-myCurSpeed>=0:
        driver.setBrakeIntensity(0)            
    
    if (not math.isnan(newSpeed)):
        driver.setCruisingSpeed(newSpeed*3.6)
    
        #print('newSpeed,', newSpeed,'predSpeed,' , predSpeed, 'myCurSpeed,', myCurSpeed,'myAccel,' ,myAccel)    

    # calculate desired angle 
    curAngle=round(driver.getSteeringAngle(), 3)-veh.getField("rotation").getSFRotation()[3]
    desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index], curAngle, ctrlPara)
        
    if abs(desAngle-curAngle)>0.000001 and myGps_z >= ctrlPara.RRX_TLonrmp_trans_z[0]:
        driver.setSteeringAngle(desAngle)
        #print("newSpeed", newSpeed,"desAngle", desAngle, "curAngle",curAngle, "laneposition", ctrlPara.lanePositions[ringNum][lane_index], "ringNum", ringNum, "lane_index",lane_index)    

    Message_Played, messageId, toPlayMessageId = playMessage(myGps_z, ctrlPara, toPlayMessageId, ringNum)

    if toPlayMessageId == 'Msg1':
        mTimer += 1
        if joystick.getPressedButton() == SwitchMode:
            isOnramp_takeover = True
            Participant_Presses_button = 2
        if mTimer>=Time_Threshold/2.5:
            isOnramp_takeover = True
            
    if messageId == 'Msg1': 
        #threading.Thread(target=playsound, args=('Takeover.mp3',), daemon=True).start()
        isOnramp_takeover = True
    if messageId == 'Msg6':
        isOnbroken_takeover_req = True

    if isOnbroken_takeover_req:
        mTimer += 1
        if joystick.getPressedButton() == SwitchMode:                
            isOnbroken_takeover = True
            Participant_Presses_button = 2
            print(Participant_Presses_button)
        # if mTimer>=Time_Threshold/2 or myGps_z >=ctrlPara.pos_cadTohd[toCtrlTLIndex[0]]: #Previous
        if mTimer>=Time_Threshold or myGps_z >=ctrlPara.pos_cadTohd[toCtrlTLIndex[0]]:
            print ("the controller is switching to human-driven mode")
            isOnbroken_takeover = True

    #update traffic light status
    # if(toCtrlTLIndex[1]==0 and myGps_z >= TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]]-ctrlPara.DeltaZ_TL_onramp)\
    # or(toCtrlTLIndex[1]==1 and myGps_z>=TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]]-ctrlPara.DeltaZ_TL_stopngo)\
    # or(toCtrlTLIndex[1]==2 and myGps_z>=TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]]-ctrlPara.DeltaZ_TL_takeover)\
    # or(toCtrlTLIndex[1]==3 and myGps_z>=TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]]-ctrlPara.DeltaZ_TL_offramp):
    if(toCtrlTLIndex[1]==2 and myGps_z>=TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]]-ctrlPara.DeltaZ_TL_takeover):
        print ("myGps_z:",myGps_z, "TL_Pos",TL_Pos[toCtrlTLIndex[0]][toCtrlTLIndex[1]])
        state_field = myTrafficLight.getField("state") if myTrafficLight is not None else None
        if state_field is not None:
            state_field.setSFString("green")
        else:
            print("[auto_ringroad_driver] WARNING: cannot set green state for", TL_Name[toCtrlTLIndex[0]][toCtrlTLIndex[1]])
        temp = toCtrlTLIndex[0]*ctrlPara.numTrafficLightRoad+toCtrlTLIndex[1]+1
        toCtrlTLIndex[0] = int(temp/ctrlPara.numTrafficLightRoad)
        toCtrlTLIndex[1] = int(temp%ctrlPara.numTrafficLightRoad)
        myTrafficLight = driver.getFromDef(TL_Name[toCtrlTLIndex[0]][toCtrlTLIndex[1]])



############# only record the trajectory after simulation time > 0.5sec ###############################
    if round(driver.getTime(),2)>=0.5:
        pvInfo.append(round(driver.getTime(),2))
        pvInfo.append(int(Main_road==True))
        pvInfo.append('Road ID')
        if Main_road:
            Lane_ID = "RR"+str(ringNum+1)+"_Lane" + str(lane_index+1);
            pvInfo.append(Lane_ID)
        else:
            pvInfo.append('N/A')
        
        pvInfo.append( round(myGps_x,4))
        pvInfo.append(  round(myGps_z-ctrlPara.gpsZOffset,4))
        pvInfo.append(round(myCurSpeed,4))
        
        #pvInfo.append(id)
        pvInfo.append(int(Message_Played == True))   
        pvInfo.append(messageId)
        ##Participant_Controller
        pvInfo.append('Auto-driving')

        pvInfo.append(Participant_Presses_button)
        pvInfo.append(Participant_Indicates)
        pvInfo.append(Participant_Indicate_back)
        pvInfo.append(int(Participant_Brakes==True))
#        pvInfo.append('Participant_is_hit_from_behind')
#        pvInfo.append('Participant_crashes_its_leader')
        
        Participants_TrafficLightAhead_ID =TL_Name[toCtrlTLIndex[0]][toCtrlTLIndex[1]] 
#        pvInfo.append(Participants_TrafficLightAhead_ID)
        
        Participants_TrafficLightAhead_State = myTrafficLight.getField("state").getSFString()
        pvInfo.append(Participants_TrafficLightAhead_State)     

        
        temp0 = toCtrlTLIndex[0]*ctrlPara.numTrafficLightRoad+toCtrlTLIndex[1]-1
        if temp0 <0:
            pvInfo.append('N/A')
            pvInfo.append('N/A')
        else:
            afterCtrlTLIndex[0] = int(temp0/ctrlPara.numTrafficLightRoad)
            afterCtrlTLIndex[1] = int(temp0%ctrlPara.numTrafficLightRoad)
        
            Participants_TrafficLightBehind_ID = TL_Name[afterCtrlTLIndex[0]][afterCtrlTLIndex[1]]
            pvInfo.append(Participants_TrafficLightBehind_ID)
            
            myTrafficLight0 = driver.getFromDef(TL_Name[afterCtrlTLIndex[0]][afterCtrlTLIndex[1]])
        
            Participants_TrafficLightBehind_State = myTrafficLight0.getField("state").getSFString()
            pvInfo.append(Participants_TrafficLightBehind_State)

        if preVeh:
            pvInfo.append(gap2pred)
            pvInfo.append(speedDiff)
        else:
            pvInfo.append('N/A')
            pvInfo.append('N/A')            
        pvInfo.append(curAngle)
        pvInfo.append(joystick.getAxisValue(0)/65536)
        pvInfo.append(joystick.getAxisValue(2)/65535)

        
        
        

        # fp.write(','.join(str(line) for line in pvInfo))
        # fp.write('\n')
        csvwriter.writerow(pvInfo) 
        
        if isOnbroken_takeover or isOnramp_takeover:
        
            #print(pvInfo, file=fp)    
            fp.close()

            controllerName.setSFString("racing_wheel_com")

    #print("Steering Throttle Brake is",joystick.getAxisValue(0),",",joystick.getAxisValue(1),",",joystick.getAxisValue(2))
        
    pvInfo = []
    numStep += 1