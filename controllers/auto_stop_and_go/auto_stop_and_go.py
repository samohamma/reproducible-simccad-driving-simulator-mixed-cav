
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
import struct
import time
import os
import numpy as np
import math
import threading
from multiprocessing import Lock
import csv
import sys 
sys.path.append("..") 
from ContrlPara import ContrlPara
import fcntlock as fcntl
#from auto_backup_0.auto_backup_0 import global_lock
#from auto_ringroad_driver.auto_ringroad_driver import global_lock

def get_latest_file(dir):
    '''search latest file'''
    file_lists = os.listdir(dir)
    if not file_lists:
        return 0
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


def getSpeedProfile():


    return newSpeed

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

def speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,lane_index):
    if preVeh and gap2pred< ctrlPara.myMaxGap:  #Added: A new condition for spacing
        desHeadwayTime = myHeadwayTime
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 10
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*ctrlPara.cutinDuration)*(myHeadwayTime-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)\
            
#        gap = gap2pred - ctrlPara.myMinGap # Previous
        gap = gap2pred - ctrlPara.MinGapJam[lane_index-1] #Revised
        
        if gap2pred< ctrlPara.SpacingCr[lane_index-1]: #Revised        
            optSpeed = max(0, min(ctrlPara.SpeedMaxFD[lane_index-1],gap/desHeadwayTime))        
        else:
            optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime))    
        #optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime)) #Previous

        #Adde
        
        #optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime))
        
                
        Lambda1=ctrlPara.Lambda1
        Lambda2=ctrlPara.Lambda2
        Lambda1=1/ctrlPara.HeadwaySetting[lane_index-1]
        Lambda2=0.8*Lambda1
        
#        accel = (optSpeed-speed)/ctrlPara.T_adpt + ctrlPara.Lamda*(predSpeed-speed) #Previous
        #Added AVDM instead of FVDM
        SpeedDiff=(predSpeed-speed) #Added
        DecelTerm=Lambda1*np.heaviside(-SpeedDiff,0) #Added # Corresponding to a deceleration situation
        AccelTerm=Lambda2*np.heaviside(SpeedDiff,0)  #Added # Corresponding to acceleration situatio

        #Note that only one of the terms DecelTerm and AccelTerm would be active at once:
        RelaxAccel=(optSpeed-speed)/ctrlPara.T_adpt #Added
        AnicAccel=(DecelTerm+AccelTerm)*SpeedDiff #Added
        accel = (optSpeed-speed)/ctrlPara.T_adpt + (DecelTerm+AccelTerm)*SpeedDiff #Revised  AVDM

#        accel = (optSpeed-speed)/ctrlPara.T_adpt + ctrlPara.Lamda*(predSpeed-speed) #Previous

        if accel>ctrlPara.maxAccel:
            accel = ctrlPara.maxAccel
        if accel<ctrlPara.minAccel:
            accel = ctrlPara.minAccel
        
        newSpeed = speed + accel*time_step/1000
        #print ( "newSpeed", newSpeed, "accel", accel, "optSpeed", optSpeed)
        #print ( "DesiredSpeed", ctrlPara.myDesSpeed[ringNum], "MaxSpeed", ctrlPara.myMaxSpeed)
        # print("My Gap is",gap)
        # print("My Optimal Speed is",optSpeed, "my Speed is",speed)
        # print("My Optimal Speed is",optSpeed, "my Speed is",speed)
        # print("My Relaxation acceleration is",RelaxAccel)
        # print("My Anicipation acceleration is",AnicAccel)

      
    else:
        optSpeed =ctrlPara.myMaxSpeed 
    
        accel_relax = (optSpeed-speed)/ctrlPara.T_adpt  #Revised  AVDM   
        newSpeed = speed + min(accel_relax,ctrlPara.maxAccel)*time_step/1000 #Revised  AVDM  
        newSpeed=min(newSpeed, ctrlPara.myMaxSpeed) #Revised    
        newSpeed = speedSpeedControl(speed, vErr, ctrlPara.mySpeedControlGain) #Previous 
    
    
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
    #print("speedDiff", speedDiff, "vErr", vErr)
        

    if time_gap > 3:
         newSpeed = speedSpeedControl(speed, vErr, ctrlPara.mySpeedControlGain)
         newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,lane_index)

         CACC_ControlMode[0] = 0
    elif time_gap <= 2:
        newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel,lane_change_action,time_step, myHeadwayTime,ctrlPara)
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara,lane_index)
        CACC_ControlMode[0] = 1
    else:
        if CACC_ControlMode[0]==0:
            newSpeed = speedSpeedControl(speed, vErr,ctrlPara.mySpeedControlGain)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara,lane_index)
            
        else:
            newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action,time_step, myHeadwayTime, ctrlPara)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara,lane_index)


        
    return newSpeed


def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))





time.sleep(2)




###import system parameters

ctrlPara = ContrlPara()
paraFilename = '../Controller_input_param.csv'
#read csv, and split on "," the line
P_File = open(paraFilename, "r")
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadVehInfo(paraFile)
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
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.load_Msg_PosPara(paraFile)
P_File.seek(0)




driver = Driver()
veh = driver.getSelf()
id = veh.getField("name").getSFString()
controllerName = veh.getField("controller")
print("current controller is ",controllerName.getSFString())
myPosition = veh.getField("translation")


### save results to file
fileDate = time.strftime("%Y%m%d")
fileTime = time.strftime("%H%M%S")
fileDir = '../../Extracted data/SimCCAD_CAVs/'
recFile = get_latest_file(fileDir)



if recFile == 0 or time.time() -os.path.getmtime(recFile)>10:
    createFile = fileDir + 'Run-Data-' + fileDate + '-Time-' + fileTime  + '-CAV' +'.csv'
    fp = open(createFile, 'a', newline = '')
    
    fields = ['Local_time', 'Road_ID', 'Lane_ID', 'AV_Leader_ID','AV_follower_ID', 'AV_ID',\
              'AV_Position_X', 'AV_Position_Z', 'AV_Speed','Spacing_Gap','TTC', 'AV_Brake','BrakeLightOn', 'AV_LC_Coop_status','Participant_CutsIn']    
    csvwriter = csv.writer(fp)
    fcntl.lock(fp, fcntl.LOCK_EX)
    csvwriter.writerow(fields)
    fp.flush()
    os.fsync(fp)
    fcntl.unlock(fp)

 
else:
    fp = open(recFile, 'a', newline = '')
    csvwriter = csv.writer(fp)
    #print ('global_lock id is ', global_lock)

    
    
    

time_step = int(driver.getBasicTimeStep()) 

radar = driver.getRadar("radar")
radar.enable(time_step)

#print("radar", radar)

gps = driver.getGPS("gps")
gps.enable(time_step)

recv = driver.getReceiver("receiver")
recv.enable(time_step)

driver.step()


#######initialize status

apply_PID.integral = 0
apply_PID.previousDiff = None
## define cut_in state
cutIn_action = ('CUT_NO', 'CUT_IN', 'CUT_OUT')
CACC_ControlMode=[0]
get_filtered_speed.previousSpeeds = []
#ring road number 
ringNum = 0
cutIn_on = False                
cutinHeadwayTime0 = 0
numStep=0
lane_index = 0 
#obtain original position of the first ringroad.
isOriginPos = True
myOriginPos_x = 0
myOriginPos_z = 0
myOriginSeq = 0
myPreSpeed = 0


## set initial traffic light with red
trafficLight = driver.getFromDef("TL_img_StnGo" + str(ringNum))
tlPosition=trafficLight.getField("translation").getSFVec3f()[2]
print("traffic light location is ", tlPosition)
isTLinitialized =False
## define logic if participant is allowed to enter the main road. (in case the first vehicle overtakes the participant)
isReadyToEnter = False

avInfoStngo = []
av_brake_act = 0
av_reaction_pv = 0

#### stop and go vehicle state
Veh_StnGo_speedMode = 'NORMAL'
Veh_StnGo_turnGreen = False
Veh_StnGo_readtoCount = False
#Veh_StnGo_stop = False
Veh_StnGo_Timer = 0




speedFile = './StopNGo_trajectories.csv'
#read csv, and split on "," the line
S_File = open(speedFile, "r")


## Speed profile for each vehicle on each lane to be read as follows:

## speedProfile(speedProfile[,0]==RRX_LaneX_ID[ringNum][lane_index-1])
##
##
##
##


speedProfile = csv.reader(S_File, delimiter=",")
listSpeed = list(speedProfile)

### only support up to 6 lanes
Veh_StnGo_profile_offset = [1, 1, 1, 1, 1,1]
for i in range (1, len(listSpeed)):
    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane1[ringNum]:
        Veh_StnGo_profile_offset[0] = i
        break
for i in range (1, len(listSpeed)):
    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane2[ringNum]:
        Veh_StnGo_profile_offset[1] = i
        break
for i in range (1, len(listSpeed)):
    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane3[ringNum]:
        Veh_StnGo_profile_offset[2] = i
        break
for i in range (1, len(listSpeed)):
    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane4[ringNum]:
        Veh_StnGo_profile_offset[3] = i
        break
        


print ("speed_profile_offset is :", Veh_StnGo_profile_offset[0],",",  Veh_StnGo_profile_offset[1],",", Veh_StnGo_profile_offset[2],",", Veh_StnGo_profile_offset[3])


StnGo_originPos = [[[ctrlPara.RR1_Stop_nGoVeh_Lane1_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane1_trans_z],[ctrlPara.RR1_Stop_nGoVeh_Lane2_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane2_trans_z],[ctrlPara.RR1_Stop_nGoVeh_Lane3_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane3_trans_z],[ctrlPara.RR1_Stop_nGoVeh_Lane4_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane4_trans_z]],\
                   [[ctrlPara.RR2_Stop_nGoVeh_Lane1_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane1_trans_z],[ctrlPara.RR2_Stop_nGoVeh_Lane2_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane2_trans_z],[ctrlPara.RR2_Stop_nGoVeh_Lane3_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane3_trans_z],[ctrlPara.RR2_Stop_nGoVeh_Lane4_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane4_trans_z]],\
                   [[ctrlPara.RR3_Stop_nGoVeh_Lane1_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane1_trans_z],[ctrlPara.RR3_Stop_nGoVeh_Lane2_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane2_trans_z],[ctrlPara.RR3_Stop_nGoVeh_Lane3_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane3_trans_z],[ctrlPara.RR3_Stop_nGoVeh_Lane4_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane4_trans_z]]]


Veh_name=driver.getSelf().getField("name").getSFString() 
myGps_x = gps.getValues()[0]
myGps_z = gps.getValues()[2]



# global_lock.acquire()
# global_lock.acquire()
# global_lock.acquire()

# print ('state of global_lock is', global_lock.locked())
# time.sleep(4)
# global_lock.release()
# print ('state of global_lock is', global_lock.locked())

# global_lock.acquire()
# global_lock.acquire()
#Added:____________________________________________________
TTC_Max=100000.9801;
BrakeLightOn=False         
TTC=TTC_Max
#avInfo = [] #Previous:
avInfo = [] #Previous
Added_row_data=list()
speedDiff=0

Lane1LeaderPrepLC=False
Lane1LeaderPrepLCTimer=0
TimeStepBudgetLC=0
#____________________________________________________________

while driver.step() != -1:
    TTC=TTC_Max   

    if not Veh_StnGo_turnGreen:
        if trafficLight.getField("state").getSFString() == 'green':
            Veh_StnGo_speedMode = 'NGSIM'
            #print("NGSIM: myGps_z",gps.getValues()[2])
            Veh_StnGo_readtoCount = True
            Veh_StnGo_turnGreen = True
            
       
            
        else:
        
            myCurSpeed=0
            gap2pred=ctrlPara.myOriginGap            
            avInfoStngo.append( round(driver.getTime(),4))
            avInfoStngo.append( 'RR'+str(ringNum+1))
            avInfoStngo.append( lane_index)
            avInfoStngo.append( 'leaderIdStngo2')
            avInfoStngo.append('followerId')
            avInfoStngo.append( Veh_name)
            avInfoStngo.append( round(myGps_x,4))
            avInfoStngo.append( round(myGps_z -ctrlPara.gpsZOffset,4))
            avInfoStngo.append( round(myCurSpeed,4))
            avInfoStngo.append( round(gap2pred,4))
            avInfoStngo.append( round(TTC,4))        
            avInfoStngo.append(av_brake_act)
            avInfoStngo.append(BrakeLightOn)
            avInfoStngo.append(av_reaction_pv)
            avInfoStngo.append(cutIn_on)    

            #print('locked is',global_lock.locked())
           
            #global_lock.acquire()
            fcntl.lock(fp, fcntl.LOCK_EX)
            csvwriter.writerow(avInfoStngo)
            #fp.flush()
            #os.fsync(fp)
            fcntl.unlock(fp)
            #global_lock.release()
            avInfoStngo = []                 
        
            if myCurSpeed<=0.01:
                driver.setBrakeIntensity(0.001)
                BrakeLightOn=True   

            numStep += 1
            continue
    
   
    #Added: Add more waiting for the first lane
    myGps_x = gps.getValues()[0]
    Which_lane = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)
    
    if Which_lane==1: 
        Waiting_time=1+ctrlPara.DeltT_TL_stopngo
    elif Which_lane==2:
        Waiting_time=4+ctrlPara.DeltT_TL_stopngo
    elif Which_lane==3:
        Waiting_time=2+ctrlPara.DeltT_TL_stopngo
    elif Which_lane==4:
        Waiting_time=1.5+ctrlPara.DeltT_TL_stopngo
                   
    if Veh_StnGo_readtoCount:
        if Veh_StnGo_Timer*time_step/1000<Waiting_time:
            Veh_StnGo_Timer += 1
            
            myCurSpeed=0
            gap2pred=ctrlPara.myOriginGap

            avInfoStngo.append( round(driver.getTime(),4))
            avInfoStngo.append( 'RR'+str(ringNum+1))
            avInfoStngo.append( lane_index)
            avInfoStngo.append( 'leaderIdStngo2')
            avInfoStngo.append('followerId')
            avInfoStngo.append( Veh_name)
            avInfoStngo.append( round(myGps_x,4))
            avInfoStngo.append( round(myGps_z -ctrlPara.gpsZOffset,4))
            avInfoStngo.append( round(myCurSpeed,4))
            avInfoStngo.append( round(gap2pred,4))
            avInfoStngo.append( round(TTC,4))        
            avInfoStngo.append(av_brake_act)
            avInfoStngo.append(BrakeLightOn)
            avInfoStngo.append(av_reaction_pv)
            avInfoStngo.append(cutIn_on)    
           
            #global_lock.acquire()
            fcntl.lock(fp, fcntl.LOCK_EX)
            csvwriter.writerow(avInfoStngo)
            #fp.flush()
            #os.fsync(fp)
            fcntl.unlock(fp)
            #global_lock.release()
            avInfoStngo = []                    
    
            if myCurSpeed<=0.01:
                driver.setBrakeIntensity(0.001)
                BrakeLightOn=True                     
            
            numStep += 1
            continue 
        else:
            Veh_StnGo_Timer = 0
            Veh_StnGo_readtoCount = False
            

    
    #if Veh_StnGo_speedMode == 'NGSIM' and gps.getValues()[2]>= ctrlPara.RRX_PosTLImgnry_TurnRed[ringNum]:
    if Veh_StnGo_speedMode == 'NGSIM' and trafficLight.getField("state").getSFString() == 'red':
        Veh_StnGo_speedMode = 'NORMAL'
        
    myCurSpeed = gps.getSpeed()
    if myCurSpeed<200 and myCurSpeed>=0:
        #here myGps_z is the GPS postion of vehicle, different from vehicle position in Webots. GPS offset is needed.
        myGps_x = gps.getValues()[0]
        myGps_z = gps.getValues()[2]
        lane_index = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)
        
        if isOriginPos:
            myOriginPos_x = myGps_x
            myOriginPos_z = myGps_z -ctrlPara.gpsZOffset
            #obtain the sequence of CAV.
            myOriginSeq = int(round((myOriginPos_z - ctrlPara.RR1_Stop_nGoVeh_Lane1_trans_z)/ctrlPara.myOriginGap))
            #print("myOriginSeq",myOriginSeq)
            isOriginPos = False



        #threading.Thread(target=playsound, args=('Auto.mp3',), daemon=True).start() #AutO
            
        #decide which lane the vehicle is located in.
        #singleGap = myCurSpeed*ctrlPara.myHeadwayTime[ringNum][lane_index] + ctrlPara.myMinGap #Previous 
        singleGap = myCurSpeed*ctrlPara.HeadwaySetting[lane_index-1] + ctrlPara.MinGapJam[lane_index-1] #Revised       
        preVeh, gap2pred, speedDiff = getLeaderInfo(radar, singleGap, myCurSpeed, ctrlPara)
        #print("preVeh", preVeh,"gap2pred", gap2pred, "speedDiff",speedDiff,"lane_index", lane_index, "myOriginSeq",myOriginSeq )
        gap2pred = gap2pred - 0.5*ctrlPara.vehLength
        predSpeed = speedDiff + myCurSpeed
        myAccel = (myCurSpeed-myPreSpeed)/(time_step/1000)
        if myAccel<0:
            av_brake_act = 1
        else:
            av_brake_act = 0
            
        myPreSpeed = myCurSpeed


        lane_change_action = cutIn_action[0]
        # check the message and implement cut in response
        if recv.getQueueLength()>0:
            message=recv.getData()
            dataList=struct.unpack(b"4s4d",message)
            #str=''.join(dataList)
            recv.nextPacket()

            if cutIn_on and (message.find(b"LCLL")>=0 or message.find(b"LCRL")>=0):
                cutIn_on = False                
                av_reaction_pv =0
                print("Lane change finished at position", myGps_z)

            ### judge if received the first LCL message in the target area
            if (not cutIn_on) and (message.find(b"LCLF")>=0 or message.find(b"LCLN")>=0) and dataList[4]<=(myGps_z+gap2pred+1.5*ctrlPara.vehLength)\
            and ((dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myfastSpdGap) or (dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myMinRespGap and dataList[1]-myCurSpeed<2) )\
            and dataList[2]>=ctrlPara.lanePositions[ringNum][lane_index]-0.9*ctrlPara.laneWidth and dataList[2]<=ctrlPara.lanePositions[ringNum][lane_index]-0.4*ctrlPara.laneWidth:
                cutIn_on = True
                cutinHeadwayTime0 = (dataList[4]-myGps_z-ctrlPara.vehLength)/myCurSpeed
                av_reaction_pv = 1
                print("Receive message of LC to left at position", myGps_z, ", position of participant vehicle is ", dataList[4])

            
            if cutIn_on and message.find(b"LCL")>=0:
                if dataList[2]<ctrlPara.lanePositions[ringNum][lane_index]+0.5*ctrlPara.laneWidth:
                    ## CUT-IN: HD vehicle at the right side of egovehicle, and wants to do left lane-change. additional 0.5laneWidth is to avoid over-lane-change.
                    ## The hd vehicle is the leader vehicle of egovehicle
                    lane_change_action = cutIn_action[1]
                    preVeh = True
                    gap2pred = dataList[4]-myGps_z-ctrlPara.vehLength
                    speedDiff = dataList[1]-myCurSpeed
                    predSpeed = dataList[1]
                else:
                    ## CUT-OUT: HD vehicle has left the current lane and is doing left lane-change, cut_in action finished at the current lane.    
                    lane_change_action = cutIn_action[2]
                    av_reaction_pv =0
                    cutIn_on = False  
            

            ### judge if received the first LCR message in the target area
            if (not cutIn_on) and (message.find(b"LCRF")>=0 or message.find(b"LCRN")>=0) and dataList[4]<=(myGps_z+gap2pred+1.5*ctrlPara.vehLength) \
            and ((dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myfastSpdGap) or (dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myMinRespGap  and dataList[1]-myCurSpeed<2 ))\
            and dataList[2]>=ctrlPara.lanePositions[ringNum][lane_index]+0.4*ctrlPara.laneWidth and dataList[2]<=ctrlPara.lanePositions[ringNum][lane_index]+0.9*ctrlPara.laneWidth:
                cutIn_on = True
                cutinHeadwayTime0 = (dataList[4]-myGps_z-ctrlPara.vehLength)/myCurSpeed
                av_reaction_pv = 1
                print("Receive message of LC to Right at position", myGps_z, ", position of participant vehicle is ", dataList[4])

            if cutIn_on and message.find(b"LCR")>=0:
                if dataList[2]>ctrlPara.lanePositions[ringNum][lane_index]-0.5*ctrlPara.laneWidth:
                    ## CUT-IN: HD vehicle at the right side of egovehicle, and wants to do left lane-change. additional 0.5laneWidth is to avoid over-lane-change.
                    ## The hd vehicle is the leader vehicle of egovehicle
                    lane_change_action = cutIn_action[1]
                    preVeh = True
                    gap2pred = dataList[4]-myGps_z-ctrlPara.vehLength
                    speedDiff = dataList[1]-myCurSpeed
                    predSpeed = dataList[1]
                else:
                    ## CUT-OUT: HD vehicle  has left the current lane and is doing right lane-change, cut_in action finished at the current lane.    
                    lane_change_action = cutIn_action[2]
                    av_reaction_pv =0
                    cutIn_on = False  

            #dataList[4]=0
            #dataList[3]=0
            #dataList[2]=0
            #dataList[1]=0
            
        if Veh_StnGo_speedMode == 'NGSIM':
            newSpeed = float(listSpeed[Veh_StnGo_profile_offset[lane_index-1]][2])            
            #print('NGSIM speed, ', newSpeed)
            Veh_StnGo_profile_offset[lane_index-1] += 1
        else:
        
   #newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, ctrlPara.myHeadwayTime[ringNum][lane_index], ctrlPara) #Previous
            newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, ctrlPara.HeadwaySetting[lane_index-1], ctrlPara)  #Revised            # desired speed of leader is constant.
            if not preVeh and myOriginSeq ==ctrlPara.Num_row_vehs_stopngo-1:
                newSpeed = ctrlPara.myDesSpeed[ringNum]
        
       
               
                
        if newSpeed-myCurSpeed<-0.01 or myCurSpeed<=0.01:
            driver.setBrakeIntensity(0.001)
            BrakeLightOn=True
        if newSpeed-myCurSpeed>=0:
            driver.setBrakeIntensity(0)            
            BrakeLightOn=False      
        # desired speed of leader is constant.
            #if myOriginSeq ==ctrlPara.Num_row_vehs_stopngo-1:
                #newSpeed = ctrlPara.myDesSpeed[ringNum]
                #implement desired longitudinal speed.
        #Added: Extract TTC    
        if speedDiff<0:
            TTC=abs(gap2pred/speedDiff)
        else:
            TTC=TTC_Max
            
        #if myGps_z <= ctrlPara.zoffsetRoadEnd[ringNum]+ctrlPara.gpsZOffset+ctrlPara.mySteadyGap[ringNum][ctrlPara.numLane-1]*(ctrlPara.vehNumber_Lane-1)-(ctrlPara.vehNumber_Lane-myOriginSeq-1)*ctrlPara.mySteadyGap[ringNum][lane_index] and ringNum <=ctrlPara.numRingRoad-1:
        if myGps_z <= ctrlPara.zoffsetRoadEnd[ringNum]+ctrlPara.gpsZOffset and ringNum <=ctrlPara.numRingRoad-1:

            driver.setCruisingSpeed(newSpeed*3.6)
        
        # calculate desired angle 
        curAngle=round(driver.getSteeringAngle(), 3)-veh.getField("rotation").getSFRotation()[3]
        desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index], curAngle, ctrlPara)
        #print("current angle is ",curAngle,"desired angle is ", desAngle,"myGps_x is ", round(myGps_x, 2),"myPosition_Z is ", round(myGps_z, 2),  "lane postion is ", lanePositions[lane_index])
        if abs(desAngle-curAngle)>0.00001:
            #print("current angle is ",curAngle,"desired angle is ", desAngle,"myGps_x is ", round(myGps_x, 2),"myPosition_Z is ", round(myGps_z, 2),  "lane postion is ", ctrlPara.lanePositions[ringNum][lane_index])
            driver.setSteeringAngle(desAngle)
        
        #if myGps_z > ctrlPara.zoffsetRoadEnd[ringNum]+ctrlPara.gpsZOffset+ctrlPara.mySteadyGap[ringNum][ctrlPara.numLane-1]*(ctrlPara.vehNumber_Lane-1)-(ctrlPara.vehNumber_Lane-myOriginSeq-1)*ctrlPara.mySteadyGap[ringNum][lane_index] and ringNum <ctrlPara.numRingRoad-1: #\
        if myGps_z > ctrlPara.zoffsetRoadEnd[ringNum]+ctrlPara.gpsZOffset and ringNum <ctrlPara.numRingRoad-1: #\

        #and myOriginSeq == 0 and lane_index == numLane-1 and ringNum <2:
            driver.setCruisingSpeed(0)
            if myCurSpeed <= 0.1:
                ringNum += 1
                myPosition.setSFVec3f([StnGo_originPos[ringNum][lane_index-1][0], 0.51, StnGo_originPos[ringNum][lane_index-1][2]+myOriginSeq*ctrlPara.myOriginGap])
                trafficLight = driver.getFromDef("TL_img_StnGo" + str(ringNum))
                tlPosition = trafficLight.getField("translation").getSFVec3f()[2]

                for i in range (1, len(listSpeed)):
                    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane1[ringNum]:
                        Veh_StnGo_profile_offset[0] = i
                        break
                for i in range (1, len(listSpeed)):
                    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane2[ringNum]:
                        Veh_StnGo_profile_offset[1] = i
                        break
                for i in range (1, len(listSpeed)):
                    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane3[ringNum]:
                        Veh_StnGo_profile_offset[2] = i
                        break
                for i in range (1, len(listSpeed)):
                    if listSpeed[i][0] == ctrlPara.Veh_stpngo_Lane4[ringNum]:
                        Veh_StnGo_profile_offset[3] = i
                        break
        

                Veh_StnGo_turnGreen = False
                Veh_StnGo_readtoCount = False                
                #Veh_StnGo_stop = False

        
        # lane_index = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)
        # if myGps_z<ctrlPara.Pos_Msg3[ringNum] and Veh_StnGo_speedMode!= 'NGSIM':
            # if lane_index==1 and Veh_name=='CAV_StopnGo_row1_Lane1':             
                # Lane1LeaderPrepLC=True
            # else:    
                
                # Lane1LeaderPrepLC=False
    
        # if Lane1LeaderPrepLC:
            # TimeStepBudgetLC+=1
            # TimeBudgetLC=TimeStepBudgetLC*time_step/1000
            # print("myVehicle name is:",Veh_name,"My GPS Z is:",myGps_z,"time Budget is:",TimeBudgetLC)
            # print("lane index is:",lane_index)

            # curAngle=round(driver.getSteeringAngle(), 3)-veh.getField("rotation").getSFRotation()[3]
            # #desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index], curAngle, ctrlPara)
            
            
            # curAngle=round(driver.getSteeringAngle(), 3)-veh.getField("rotation").getSFRotation()[3]
            
            # print("curAngle is", curAngle)
      
            # lane_index = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)
            # if lane_index!=1:
                # desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index], curAngle, ctrlPara)
            
            # desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index+1], curAngle, ctrlPara)
            
            # if abs(desAngle-curAngle)>0.00001:
                # #if not Lane1LeaderPrepLC:
                # #print("current angle is ",curAngle,"desired angle is ", desAngle,"myGps_x is ", round(myGps_x, 2),"myPosition_Z is ", round(myGps_z, 2),  "lane postion is ", ctrlPara.lanePositions[ringNum][lane_index])
                # driver.setSteeringAngle(desAngle)            
            # else:
                # Lane1LeaderPrepLC=False        
            # if TimeBudgetLC<4:
                # newSpeed=27
                # # driver.setCruisingSpeed(newSpeed*3.6)
            # else:
   
            
            # if myGps_z>=ctrlPara.Pos_Msg3[ringNum]:
                # Lane1LeaderPrepLC=False

        # avInfoStngo.append( round(driver.getTime(),2))
        # Lane_ID = 'RR'+str(ringNum+1)+'_Lane' + str(lane_index+1);
        # avInfoStngo.append( 'roadId')
        # avInfoStngo.append( Lane_ID)
        # avInfoStngo.append( 'AV_Leader_ID')
        # avInfoStngo.append('AV_follower_ID')
        # AV_Leader_position = round(gap2pred + ctrlPara.vehLength + myGps_z-ctrlPara.gpsZOffset,4)
        # avInfoStngo.append( AV_Leader_position)
        # Controller = controllerName.getSFString()
        # avInfoStngo.append(Controller)
        # avInfoStngo.append('AV_Follower_position')
        # AV_ID = id
        # avInfoStngo.append(AV_ID)
        
        # AV_Position_X = round(myGps_x,4)
        # avInfoStngo.append(AV_Position_X)
        # AV_Position_Z = round(myGps_z-ctrlPara.gpsZOffset,4)
        # avInfoStngo.append(AV_Position_Z)
        # AV_Speed = round(myCurSpeed,4)
        # avInfoStngo.append(AV_Speed)
        
        # AV_Brake = av_brake_act
        # avInfoStngo.append(AV_Brake)     
        # AV_LC_Coop_status = av_reaction_pv
        # avInfoStngo.append(AV_LC_Coop_status)
        # av_reaction_pv = 0
        
        avInfoStngo.append( round(driver.getTime(),4))
        avInfoStngo.append( 'RR'+str(ringNum+1))
        avInfoStngo.append( lane_index)
        avInfoStngo.append( 'leaderIdStngo2')
        avInfoStngo.append('followerId')
        avInfoStngo.append( Veh_name)
        avInfoStngo.append( round(myGps_x,4))
        avInfoStngo.append( round(myGps_z -ctrlPara.gpsZOffset,4))
        avInfoStngo.append( round(myCurSpeed,4))
        avInfoStngo.append( round(gap2pred,4))
        avInfoStngo.append( round(TTC,4))        
        avInfoStngo.append(av_brake_act)
        avInfoStngo.append(BrakeLightOn)
        avInfoStngo.append(av_reaction_pv)
        avInfoStngo.append(cutIn_on)        
      
        #global_lock.acquire()
        fcntl.lock(fp, fcntl.LOCK_EX)
        csvwriter.writerow(avInfoStngo)
        #fp.flush()
        #os.fsync(fp)
        fcntl.unlock(fp)
        #global_lock.release()
        avInfoStngo = []               


    
    numStep += 1    
   
    #print("position", gps.getValues()[2],"gap", gap, "speed difference", speedDiff)

