class ContrlPara:
    def __init__(self):
        ### define vehicle parameters
        self.vehLength = 4.9
        self.gpsZOffset= 1
        self.myMaxSpeed = 23
        self.myMaxGap=300;
        self.maxAccel = 4
        self.minAccel= -8        
        ###################  define road geometric parameters ######################################
        self.numRingRoad=4
        self.numLane=4
        self.laneWidth=3.65
        self.xoffsetRoad = [-3500, -3398.6, -3297.2, -3195.8]
        self.zoffsetRoad = [-18000, -9007.6, -15.2, 8977.2] 
        #self.startLength = 1000
        #self.mergeLength = 500☻
        #self.Stop_and_Go_length = 100
        #self.hdriveLength = 1500
        #self.cadriveLength = 4000
        #self.takeoverLength = 2000
        #self.divergeLength = 500

        #define waiting time interval for switching mode
        self.switchWaitingTime = 20000
        
        #define headway time and steady state of traffic flow
        self.myDesSpeed = [22.2222, 19.4444, 16.6667]
        self.mySpeedLimit = [25,25,25]
        self.myHeadwayTime = [[1.9,1.6,1.3,1,1],[1.9,1.6,1.3,1,1],[1.9,1.6,1.3,1,1]]
        self.myMinGap = 2
        self.myOriginGap = 30
        
        # display update period
        self.infoDispTime = 200
        
        #define number of vehicle in single lane
        self.vehNumber_Lane =12


        ## load vehicle's original position
        self.Num_row_vehs_backup = 9
        self.Num_row_vehs_surr = 2
        self.Num_row_vehs_stopngo  = 1
        
        self.RR1_Backup_Lane1_trans_x = -3494.6
        self.RR1_Backup_Lane1_trans_z = -18000
        self.RR1_Backup_Lane2_trans_x = -3498.2
        self.RR1_Backup_Lane2_trans_z = -18000
        self.RR1_Backup_Lane3_trans_x = -3501.8
        self.RR1_Backup_Lane3_trans_z = -18000
        self.RR1_Backup_Lane4_trans_x = -3505.4
        self.RR1_Backup_Lane4_trans_z = -18000
        self.RR2_Backup_Lane1_trans_x = -3506.005776
        self.RR2_Backup_Lane1_trans_z = -12181.31168
        self.RR2_Backup_Lane2_trans_x = -3509.605776
        self.RR2_Backup_Lane2_trans_z = -12181.31168
        self.RR2_Backup_Lane3_trans_x = -3513.205776
        self.RR2_Backup_Lane3_trans_z = -12181.31168
        self.RR2_Backup_Lane4_trans_x = -3516.805776
        self.RR2_Backup_Lane4_trans_z = -12181.31168
        self.RR3_Backup_Lane1_trans_x = -3517.411553
        self.RR3_Backup_Lane1_trans_z = -6362.623367
        self.RR3_Backup_Lane2_trans_x = -3521.011553
        self.RR3_Backup_Lane2_trans_z = -6362.623367
        self.RR3_Backup_Lane3_trans_x = -3524.611553
        self.RR3_Backup_Lane3_trans_z = -6362.623367
        self.RR3_Backup_Lane4_trans_x = -3528.211553
        self.RR3_Backup_Lane4_trans_z = -6362.623367
        self.RR4_Backup_Lane1_trans_x = -3517.411553
        self.RR4_Backup_Lane1_trans_z = -6362.623367
        self.RR4_Backup_Lane2_trans_x = -3521.011553
        self.RR4_Backup_Lane2_trans_z = -6362.623367
        self.RR4_Backup_Lane3_trans_x = -3524.611553
        self.RR4_Backup_Lane3_trans_z = -6362.623367
        self.RR4_Backup_Lane4_trans_x = -3528.211553
        self.RR4_Backup_Lane4_trans_z = -6362.623367        
        self.RR1_SurrVeh_Lane1_trans_x = -3494.6
        self.RR1_SurrVeh_Lane1_trans_z = -17200
        self.RR1_SurrVeh_Lane2_trans_x = -3498.2
        self.RR1_SurrVeh_Lane2_trans_z = -17200
        self.RR1_SurrVeh_Lane3_trans_x = -3501.8
        self.RR1_SurrVeh_Lane3_trans_z = -17200
        self.RR1_SurrVeh_Lane4_trans_x = -3505.4
        self.RR1_SurrVeh_Lane4_trans_z = -17200
        self.RR2_SurrVeh_Lane1_trans_x = -3506.005776
        self.RR2_SurrVeh_Lane1_trans_z = -11381.31168
        self.RR2_SurrVeh_Lane2_trans_x = -3509.605776
        self.RR2_SurrVeh_Lane2_trans_z = -11381.31168
        self.RR2_SurrVeh_Lane3_trans_x = -3513.205776
        self.RR2_SurrVeh_Lane3_trans_z = -11381.31168
        self.RR2_SurrVeh_Lane4_trans_x = -3516.805776
        self.RR2_SurrVeh_Lane4_trans_z = -11381.31168
        self.RR3_SurrVeh_Lane1_trans_x = -3517.411553
        self.RR3_SurrVeh_Lane1_trans_z = -5562.623367
        self.RR3_SurrVeh_Lane2_trans_x = -3521.011553
        self.RR3_SurrVeh_Lane2_trans_z = -5562.623367
        self.RR3_SurrVeh_Lane3_trans_x = -3524.611553
        self.RR3_SurrVeh_Lane3_trans_z = -5562.623367
        self.RR3_SurrVeh_Lane4_trans_x = -3528.211553
        self.RR3_SurrVeh_Lane4_trans_z = -5562.623367
        self.RR1_Stop_nGoVeh_Lane1_trans_x = -3494.6
        self.RR1_Stop_nGoVeh_Lane1_trans_z = -16200
        self.RR1_Stop_nGoVeh_Lane2_trans_x = -3498.2
        self.RR1_Stop_nGoVeh_Lane2_trans_z = -16200
        self.RR1_Stop_nGoVeh_Lane3_trans_x = -3501.8
        self.RR1_Stop_nGoVeh_Lane3_trans_z = -16200
        self.RR1_Stop_nGoVeh_Lane4_trans_x = -3505.4
        self.RR1_Stop_nGoVeh_Lane4_trans_z = -16200
        self.RR2_Stop_nGoVeh_Lane1_trans_x = -3506.005776
        self.RR2_Stop_nGoVeh_Lane1_trans_z = -10381.31168
        self.RR2_Stop_nGoVeh_Lane2_trans_x = -3509.605776
        self.RR2_Stop_nGoVeh_Lane2_trans_z = -10381.31168
        self.RR2_Stop_nGoVeh_Lane3_trans_x = -3513.205776
        self.RR2_Stop_nGoVeh_Lane3_trans_z = -10381.31168
        self.RR2_Stop_nGoVeh_Lane4_trans_x = -3516.805776
        self.RR2_Stop_nGoVeh_Lane4_trans_z = -10381.31168
        self.RR3_Stop_nGoVeh_Lane1_trans_x = -3517.411553
        self.RR3_Stop_nGoVeh_Lane1_trans_z = -4562.623367
        self.RR3_Stop_nGoVeh_Lane2_trans_x = -3521.011553
        self.RR3_Stop_nGoVeh_Lane2_trans_z = -4562.623367
        self.RR3_Stop_nGoVeh_Lane3_trans_x = -3524.611553
        self.RR3_Stop_nGoVeh_Lane3_trans_z = -4562.623367
        self.RR3_Stop_nGoVeh_Lane4_trans_x = -3528.211553
        self.RR3_Stop_nGoVeh_Lane4_trans_z = -4562.623367
        self.RR1_BrokenVeh_trans_x = -3505.4
        self.RR1_BrokenVeh_trans_z = -13400
        self.RR2_BrokenVeh_trans_x = -3495.205776
        self.RR2_BrokenVeh_trans_z = -7581.311684
        self.RR3_BrokenVeh_trans_x = -3506.611553
        self.RR3_BrokenVeh_trans_z = -1762.623367

        self.DeltaZ_Msg1 = 200
        self.DeltaZ_Msg2 = 500
        self.DeltaZ_Msg3 = 200
        self.DeltaZ_Msg4 = 0
        self.DeltaZ_Msg5a = 250
        self.DeltaZ_Msg5b = 250
        self.DeltaZ_Msg6 = 200
        self.DeltaZ_Msg7 = 500
        self.DeltaZ_Msg8 = 50
        self.RR_1_NDRT = 0
        self.RR_2_NDRT = 0
        self.RR_3_NDRT = 1
        self.RRX_NDRT = [0, 1, 1]


        #CAV HeadWaySetting Based on  Constructing a fundamental diagram for traffic flow with automated vehicles: 
        #ethodology and demonstrationX Shi, X Li - Transportation Research Part B: Methodological, 2021 - Elsevier
        #Numerical details below are extracted from Table II (see page 8 of the paper)         
        self.Qmax=[1850,1500,2250,2900]; #Veh/hr
        self.Cjam=[28.4,20,47.2,61.1]; #Shockwave speed: (km/hr)
        self.Rhojam=[86.11,90,74.96,80.77] #Veh/km
        self.MinGapJam=[6.7131,6.2111,8.4404,7.4808] # Calculated as: 1000/self.Rhojam-self.vehLength # Intervehicular spacing corresponding to the jam condition
        self.RhoCr=[20.9692,15.0000,27.2905,33.3068]   #Critical density (veh/km) calculated as: (self.Rhojam-self.Qmax/(self.Cjam)) # Veh/km
        self.SpacingCr=[47.6890,66.6667,36.6428,30.0239]  #Corresponding Spacing to the Critical density (veh/km) calculated as: 1000/(self.RhoCr) # Veh/km        
        self.HeadwaySetting=[1.4721,2.0000,1.0175, 0.7295] # (s) Corresponding headway setting from the trinagular FD: calculated as: (1000/self.Rhojam)/(self.Cjam/3.6) 
        self.SpeedMaxFD=[24.5068,27.7778,22.9017,24.1859]#(m/s) Corresponding maximum speedfrom the trinagular FD: calculated as: (self.Qmax/self.Rhocr)/3.6

      

        #load road transation_Z
        self.RRX_translation_z = [-13400,-5981.311684, -162.623367]
       
        #load traffic light position
        self.RRX_TLonrmp_trans_x = [-3492.4,-3503.805776, -3515.211553]
        self.RRX_TLonrmp_trans_z = [-17000,-11181.31168,-5362.623367]

        self.RRX_TLImgnry_StnGo_trans_x = [-3500,-3511.405776, -3522.811553]
        self.RRX_TLImgnry_StnGo_trans_z = [-16200,-10381.31168,-4562.623367 ]

        self.RRX_TLImgnry_TakeOver_trans_x = [-3500,-3511.405776,-3522.811553]
        self.RRX_TLImgnry_TakeOver_trans_z = [-13400,-5981.311684, -162.623367]

        self.RRX_TLoffrmp_trans_x = [0] * self.numRingRoad #Added
        self.RRX_TLoffrmp_trans_z = [0] * self.numRingRoad #Added
        
        self.numTrafficLightRoad=4; #Added
        
        ## load time settings
        self.DeltT_TL_backup = 2
        self.DeltT_TL_surr = 2
        self.DeltT_TL_stopngo = 2
        self.DeltT_TL_broken = 2
        
        self.DeltaZ_TL_onramp = 5        
        self.DeltaZ_TL_stopngo = 200        
        self.DeltaZ_TL_takeover = 200     
        self.DeltaZ_TL_offramp = 5              
        ### time (second) ahead of traffic light turn green for participant to drive
        self.aheadTL_Green = 7        
        self.Pos_Msg22=0

################## Stop and go traffic light turns red####################
        self.RRX_PosTLImgnry_TurnRed = [0] * self.numRingRoad

        self.RRX_PosTLImgnry_TurnRed[0]=-16500
        self.RRX_PosTLImgnry_TurnRed[1]=-3081.631641
        self.RRX_PosTLImgnry_TurnRed[2]=10336.73672

        self.RRX_VehLateralPos_x=[0] * self.numRingRoad





###################  define lane-change parameters ####################
        self.myMinRespGap = 6
        self.myfastSpdGap = 16
        #defines the slope of the cut-in transition function
        self.Ktg = 0.05
        #cut-in transition duration 4 second
        self.cutinDuration = 2

##########define lateral PID control ################
        self.LaneChg_PID_P_S = 0.005
        self.LaneChg_PID_I_S = 0.0000002
        self.LaneChg_PID_D_S = 0.125
        self.LaneChg_PID_K_S = 0
        
        self.LaneChg_PID_P = 0.05
        self.LaneChg_PID_I = 0.000015
        self.LaneChg_PID_D = 0.25        
        self.LaneChg_PID_K = -0.3


#################### define controller (CACC and FVDM)parameters   #######################################

        self.myDesSpeedGain = 1.2
        self.mySpeedControlGain=-0.4
        self.myGapControlGainGap = 0.45
        self.myGapControlGainGapDot = 0.0125
        self.myCollisionAvoidanceGainGap = 0.45
        self.myCollisionAvoidanceGainGapDot = 0.05
        self.myGapClosingControlGainGap = 0.005
        self.myGapClosingControlGainGapDot = 0.05
        self.T_adpt = 3
        self.Lamda = 0.65 
        self.maxDetectRange = 77 
        
        #Assymetric FVDM:
        
        self.Lambda1 = 1         
        self.Lambda2 = 0.5 
#################### define message play bool   #######################################        
        self.PlaySlowTrafficAhead_Msg = [1, 0, 0]
        self.PlayRedTrafflicLight_Msg = [1, 0, 0]



    # call loadVehInfo first 
    def IsMsgPlay(self, paraFile):

        for row in paraFile:
            if row[0] == 'RR1_PlaySlowTrafficAhead_Msg':
                self.PlaySlowTrafficAhead_Msg[0] = float(row[1])
            elif row[0] == 'RR2_PlaySlowTrafficAhead_Msg':   
                self.PlaySlowTrafficAhead_Msg[1] = float(row[1])                
            elif row[0] == 'RR3_PlaySlowTrafficAhead_Msg':   
                self.PlaySlowTrafficAhead_Msg[2] = float(row[1])
            elif row[0] == 'RR1_PlayRedTrafflicLight_Msg':   
                self.PlayRedTrafflicLight_Msg[0] = float(row[1])
            elif row[0] == 'RR2_PlayRedTrafflicLight_Msg':   
                self.PlayRedTrafflicLight_Msg[1] = float(row[1])
            elif row[0] == 'RR3_PlayRedTrafflicLight_Msg':   
                self.PlayRedTrafflicLight_Msg[2] = float(row[1])




    # call loadVehInfo first 
    def loadVehInfo(self, paraFile):

        for row in paraFile:
            if row[0] == 'vehLength':
                self.vehLength = float(row[1])
            elif row[0] == 'gpsZOffset':   
                self.gpsZOffset = float(row[1])                
            elif row[0] == 'myMaxSpeed':   
                self.myMaxSpeed = float(row[1])
            elif row[0] == 'myMaxGap':   
                self.myMaxGap = float(row[1])                
            elif row[0] == 'maxAccel':               
                self.maxAccel = float(row[1])
            elif row[0] == 'minAccel':   
                self.minAccel = float(row[1])
              
       
    def loadGeoPara(self, paraFile):

        
        
        self.Veh_stpngo_Lane1=[0] * self.numRingRoad
        self.Veh_stpngo_Lane2=[0] * self.numRingRoad
        self.Veh_stpngo_Lane3=[0] * self.numRingRoad
        self.Veh_stpngo_Lane4=[0] * self.numRingRoad       
        self.pos_hdTocad = [0] * self.numRingRoad        
        self.pos_cadTohd = [0] * self.numRingRoad        
        self.zoffsetRoadEnd = [0] * self.numRingRoad
        
        for row in paraFile:
            if row[0] == 'numRingRoad':
                self.numRingRoad = int(row[1])
            elif row[0] == 'numTrafficLightRoad':   
                self.numTrafficLightRoad = int(row[1])
            elif row[0] == 'numberOfLanes':   
                self.numLane = int(row[1])    
            elif row[0] == 'laneWidth':   
                self.laneWidth = float(row[1])
            elif row[0] == 'RR1_Length':   
                self.RR1_Length = float(row[1])                
            #elif row[0] == 'startLength':   
               #self.startLength = int(row[1])
            #elif row[0] == 'mergeLength':   
                #self.mergeLength = int(row[1])
            #elif row[0] == 'Stop_and_Go_length':
                #self.Stop_and_Go_length = int(row[1]) 
            #elif row[0] == 'hdriveLength':   
                #self.hdriveLength = int(row[1])
            #elif row[0] == 'cadriveLength':   
                #self.cadriveLength = int(row[1])
            #elif row[0] == 'takeoverLength':   
                #self.takeoverLength = int(row[1])
            #elif row[0] == 'divergeLength':   
                #self.divergeLength = int(row[1])
            elif row[0] == 'numVehicle_Lane':   
                self.vehNumber_Lane = int(row[1])
            elif row[0] == 'OriginGap':   
                self.myOriginGap = float(row[1])
            elif row[0] == 'minGap':   
                self.myMinGap = float(row[1])
            elif row[0] == 'switchWaitingTime':   
                self.switchWaitingTime = int(row[1])
            elif row[0] == 'infoDispTime':   
                self.infoDispTime = int(row[1])
            elif row[0] == 'RR1_translation_x':   
                self.xoffsetRoad[0] = float(row[1])
            elif row[0] == 'RR1_translation_z':   
                self.zoffsetRoad[0] = float(row[1])
            elif row[0] == 'RR2_translation_x':   
                self.xoffsetRoad[1] = float(row[1])
            elif row[0] == 'RR2_translation_z':   
                self.zoffsetRoad[1] = float(row[1])
            elif row[0] == 'RR3_translation_x':   
                self.xoffsetRoad[2] = float(row[1])
            elif row[0] == 'RR3_translation_z':   
                self.zoffsetRoad[2] = float(row[1])
            elif row[0] == 'RR4_translation_x':   
                self.xoffsetRoad[3] = float(row[1])
            elif row[0] == 'RR4_translation_z':   
                self.zoffsetRoad[3] = float(row[1])
                
            elif row[0] == 'DesiredSpeed_RR1':   
                self.myDesSpeed[0] = float(row[1])
            elif row[0] == 'DesiredSpeed_RR2':   
                self.myDesSpeed[1] = float(row[1])
            elif row[0] == 'DesiredSpeed_RR3':   
                self.myDesSpeed[2] = float(row[1])



            elif row[0] == 'SpeedLimit_RR1':   
                self.mySpeedLimit[0] = float(row[1])
            elif row[0] == 'SpeedLimit_RR2':   
                self.mySpeedLimit[1] = float(row[1])
            elif row[0] == 'SpeedLimit_RR3':   
                self.mySpeedLimit[2] = float(row[1])


                
            elif row[0] == 'HeadwayRR1_Lane_1':   
                self.myHeadwayTime[0][0] = float(row[1])
            elif row[0] == 'HeadwayRR1_Lane_2':   
                self.myHeadwayTime[0][1] = float(row[1])
            elif row[0] == 'HeadwayRR1_Lane_3':   
                self.myHeadwayTime[0][2] = float(row[1])
            elif row[0] == 'HeadwayRR1_Lane_4':   
                self.myHeadwayTime[0][3] = float(row[1])
            elif row[0] == 'HeadwayRR2_Lane_1':   
                self.myHeadwayTime[1][0] = float(row[1])
            elif row[0] == 'HeadwayRR2_Lane_2':   
                self.myHeadwayTime[1][1] = float(row[1])
            elif row[0] == 'HeadwayRR2_Lane_3':   
                self.myHeadwayTime[1][2] = float(row[1])
            elif row[0] == 'HeadwayRR2_Lane_4':   
                self.myHeadwayTime[1][3] = float(row[1])
            elif row[0] == 'HeadwayRR3_Lane_1':   
                self.myHeadwayTime[2][0] = float(row[1])
            elif row[0] == 'HeadwayRR3_Lane_2':   
                self.myHeadwayTime[2][1] = float(row[1])
            elif row[0] == 'HeadwayRR3_Lane_3':   
                self.myHeadwayTime[2][2] = float(row[1])
            elif row[0] == 'HeadwayRR3_Lane_4':   
                self.myHeadwayTime[2][3] = float(row[1])
            elif row[0] == 'HeadwayRR3_Lane_1':   
                self.myHeadwayTime[2][0] = float(row[1])
            elif row[0] == 'HeadwayRR4_Lane_2':   
                self.myHeadwayTime[2][1] = float(row[1])
            elif row[0] == 'HeadwayRR4_Lane_3':   
                self.myHeadwayTime[2][2] = float(row[1])
            elif row[0] == 'HeadwayRR4_Lane_4':   
                self.myHeadwayTime[2][3] = float(row[1])

                

          
            # if row[0] == 'RR1_PosMsg0':
                # self.Pos_Msg22 = float(row[1])
                # self.Pos_Msg0= float(row[1])  
            # elif row[0] == 'RR1_PosMsg1':
                # self.Pos_Msg1[0] = float(row[1])                
            # elif row[0] == 'RR2_PosMsg1':
                # self.Pos_Msg1[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg1':
                # self.Pos_Msg1[2] = float(row[1])                

            # if row[0] == 'RR1_PosMsg2':
                # self.Pos_Msg2[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg2':
                # self.Pos_Msg2[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg2':
                # self.Pos_Msg2[2] = float(row[1])                  

            # if row[0] == 'RR1_PosMsg3':
                # self.Pos_Msg3[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg3':
                # self.Pos_Msg3[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg3':
                # self.Pos_Msg3[2] = float(row[1])       

            # if row[0] == 'RR1_PosMsg4':
                # self.Pos_Msg4[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg4':
                # self.Pos_Msg4[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg4':
                # self.Pos_Msg4[2] = float(row[1])  

            # if row[0] == 'RR1_PosMsg5a':
                # self.Pos_Msg5a[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg5a':
                # self.Pos_Msg5a[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg5a':
                # self.Pos_Msg5a[2] = float(row[1])  
                
            # if row[0] == 'RR1_PosMsg5b':
                # self.Pos_Msg5b[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg5b':
                # self.Pos_Msg5b[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg5b':
                # self.Pos_Msg5b[2] = float(row[1])  
                
            # if row[0] == 'RR1_PosMsg6':
                # self.Pos_Msg6[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg6':
                # self.Pos_Msg6[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg6':
                # self.Pos_Msg6[2] = float(row[1])  

            # if row[0] == 'RR1_PosMsg7':
                # self.Pos_Msg7[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg7':
                # self.Pos_Msg7[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg7':
                # self.Pos_Msg7[2] = float(row[1]) 

            # if row[0] == 'RR1_PosMsg8':
                # self.Pos_Msg8[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg8':
                # self.Pos_Msg8[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg8':
                # self.Pos_Msg8[2] = float(row[1])

            # if row[0] == 'RR1_PosMsg9':
                # self.Pos_Msg9[0] = float(row[1])
            # elif row[0] == 'RR2_PosMsg9':
                # self.Pos_Msg9[1] = float(row[1])                
            # elif row[0] == 'RR3_PosMsg9':
                # self.Pos_Msg9[2] = float(row[1])    
            if row[0] == 'RR1_Lane1_ID':
                self.Veh_stpngo_Lane1[0] = float(row[1])
            elif row[0] == 'RR2_Lane1_ID':
                self.Veh_stpngo_Lane1[1] = float(row[1])                
            elif row[0] == 'RR3_Lane1_ID':
                self.Veh_stpngo_Lane1[2] = float(row[1]) 
                
            if row[0] == 'RR1_Lane2_ID':
                self.Veh_stpngo_Lane2[0] = float(row[1])
            elif row[0] == 'RR2_Lane2_ID':
                self.Veh_stpngo_Lane2[1] = float(row[1])                
            elif row[0] == 'RR3_Lane2_ID':
                self.Veh_stpngo_Lane2[2] = float(row[1]) 
            
            if row[0] == 'RR1_Lane3_ID':
                self.Veh_stpngo_Lane3[0] = float(row[1])
            elif row[0] == 'RR2_Lane3_ID':
                self.Veh_stpngo_Lane3[1] = float(row[1])                
            elif row[0] == 'RR3_Lane3_ID':
                self.Veh_stpngo_Lane3[2] = float(row[1]) 

            if row[0] == 'RR1_Lane4_ID':
                self.Veh_stpngo_Lane4[0] = float(row[1])
            elif row[0] == 'RR2_Lane4_ID':
                self.Veh_stpngo_Lane4[1] = float(row[1])                
            elif row[0] == 'RR3_Lane4_ID':
                self.Veh_stpngo_Lane4[2] = float(row[1])                 
                

            elif row[0] == 'RR1_pos_hdTocad':
                self.pos_hdTocad[0] = float(row[1])
            elif row[0] == 'RR2_pos_hdTocad':
                self.pos_hdTocad[1] = float(row[1])                
            elif row[0] == 'RR3_pos_hdTocad':
                self.pos_hdTocad[2] = float(row[1])                 


            elif row[0] == 'RR1_pos_cadTohd':
                self.pos_cadTohd[0] = float(row[1])
            elif row[0] == 'RR2_pos_cadTohd':
                self.pos_cadTohd[1] = float(row[1])                
            elif row[0] == 'RR3_pos_cadTohd':
                self.pos_cadTohd[2] = float(row[1])                 


            elif row[0] == 'RR1_translation_end_z':
                self.zoffsetRoadEnd[0]=  float(row[1])
            elif row[0] == 'RR2_translation_end_z':
                self.zoffsetRoadEnd[1]=  float(row[1])
            elif row[0] == 'RR3_translation_end_z':
                self.zoffsetRoadEnd[2]=  float(row[1]) 




                
                


        #self.ringRoadLength = self.startLength+self.mergeLength+self.Stop_and_Go_length+self.hdriveLength+self.cadriveLength+self.takeoverLength+self.divergeLength

    


        #self.pos_stop_and_go = [0] * self.numRingRoad        
        # self.Pos_Msg2=[0] * self.numRingRoad
        #for i in range(0,self.numRingRoad):
            #self.pos_stop_and_go[i] = self.zoffsetRoad[i] + self.startLength+self.mergeLength
            # self.Pos_Msg2[i] = self.pos_stop_and_go[i]+self.DZ_Msg2


        # self.pos_hdTocad = [0] * self.numRingRoad
        # for i in range(0,self.numRingRoad):
            # self.pos_hdTocad[i] = self.zoffsetRoad[i] + self.startLength+self.mergeLength+self.Stop_and_Go_length+self.hdriveLength
          
        # self.pos_cadTohd = [0] * self.numRingRoad
        # for i in range(0,self.numRingRoad):
            # self.pos_cadTohd[i] = self.zoffsetRoad[i] + self.startLength+self.mergeLength+self.Stop_and_Go_length+self.hdriveLength+self.cadriveLength



        #self.pos_diverge = [0] * self.numRingRoad
        #for i in range(0,self.numRingRoad):
            #self.pos_diverge[i] = self.zoffsetRoad[i] + self.startLength+self.mergeLength+self.Stop_and_Go_length+self.hdriveLength+self.cadriveLength+self.takeoverLength

        ### only the first three ringroad is set 
        self.mySteadyGap = [[0 for j in range(self.numLane)] for i in range(self.numRingRoad-1)]
        for i in range(0,self.numRingRoad-1):
            for j in range(0,self.numLane):
                self.mySteadyGap[i][j] = self.myDesSpeed[i]*self.myHeadwayTime[i][j] + self.vehLength + self.myMinGap


    def loadVehPosPara(self, paraFile):
        for row in paraFile:

            if row[0] == 'Num_row_vehs_backup':
                self.Num_row_vehs_backup = float(row[1])
            elif row[0] == 'Num_row_vehs_surr':
                self.Num_row_vehs_surr = float(row[1])
            elif row[0] == 'Num_row_vehs_stopngo':
                self.Num_row_vehs_stopngo = float(row[1])
            elif row[0] == 'RR1_Backup_Lane1_trans_x':
                self.RR1_Backup_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR1_Backup_Lane1_trans_z':   
                self.RR1_Backup_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR1_Backup_Lane2_trans_x':
                self.RR1_Backup_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR1_Backup_Lane2_trans_z':   
                self.RR1_Backup_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR1_Backup_Lane3_trans_x':
                self.RR1_Backup_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR1_Backup_Lane3_trans_z':   
                self.RR1_Backup_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR1_Backup_Lane4_trans_x':
                self.RR1_Backup_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR1_Backup_Lane4_trans_z':   
                self.RR1_Backup_Lane4_trans_z = float(row[1])
            elif row[0] == 'RR2_Backup_Lane1_trans_x':
                self.RR2_Backup_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR2_Backup_Lane1_trans_z':   
                self.RR2_Backup_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR2_Backup_Lane2_trans_x':
                self.RR2_Backup_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR2_Backup_Lane2_trans_z':   
                self.RR2_Backup_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR2_Backup_Lane3_trans_x':
                self.RR2_Backup_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR2_Backup_Lane3_trans_z':   
                self.RR2_Backup_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR2_Backup_Lane4_trans_x':
                self.RR2_Backup_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR2_Backup_Lane4_trans_z':   
                self.RR2_Backup_Lane4_trans_z = float(row[1])
            elif row[0] == 'RR3_Backup_Lane1_trans_x':
                self.RR3_Backup_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR3_Backup_Lane1_trans_z':   
                self.RR3_Backup_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR3_Backup_Lane2_trans_x':
                self.RR3_Backup_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR3_Backup_Lane2_trans_z':   
                self.RR3_Backup_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR3_Backup_Lane3_trans_x':
                self.RR3_Backup_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR3_Backup_Lane3_trans_z':   
                self.RR3_Backup_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR3_Backup_Lane4_trans_x':
                self.RR3_Backup_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR3_Backup_Lane4_trans_z':   
                self.RR3_Backup_Lane4_trans_z = float(row[1])

            elif row[0] == 'RR1_SurrVeh_Lane1_trans_x':   
                self.RR1_SurrVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane1_trans_z':   
                self.RR1_SurrVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane2_trans_x':   
                self.RR1_SurrVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane2_trans_z':   
                self.RR1_SurrVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane3_trans_x':   
                self.RR1_SurrVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane3_trans_z':   
                self.RR1_SurrVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane4_trans_x':   
                self.RR1_SurrVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane4_trans_z':   
                self.RR1_SurrVeh_Lane4_trans_z = float(row[1])     

            elif row[0] == 'RR2_SurrVeh_Lane1_trans_x':   
                self.RR2_SurrVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane1_trans_z':   
                self.RR2_SurrVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane2_trans_x':   
                self.RR2_SurrVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane2_trans_z':   
                self.RR2_SurrVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane3_trans_x':   
                self.RR2_SurrVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane3_trans_z':   
                self.RR2_SurrVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane4_trans_x':   
                self.RR2_SurrVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane4_trans_z':   
                self.RR2_SurrVeh_Lane4_trans_z = float(row[1])

            elif row[0] == 'RR3_SurrVeh_Lane1_trans_x':   
                self.RR3_SurrVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane1_trans_z':   
                self.RR3_SurrVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane2_trans_x':   
                self.RR3_SurrVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane2_trans_z':   
                self.RR3_SurrVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane3_trans_x':   
                self.RR3_SurrVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane3_trans_z':   
                self.RR3_SurrVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane4_trans_x':   
                self.RR3_SurrVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane4_trans_z':   
                self.RR3_SurrVeh_Lane4_trans_z = float(row[1])


            elif row[0] == 'RR1_SurrVeh_Lane1_trans_x':   
                self.RR1_SurrVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane1_trans_z':   
                self.RR1_SurrVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane2_trans_x':   
                self.RR1_SurrVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane2_trans_z':   
                self.RR1_SurrVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane3_trans_x':   
                self.RR1_SurrVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane3_trans_z':   
                self.RR1_SurrVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane4_trans_x':   
                self.RR1_SurrVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR1_SurrVeh_Lane4_trans_z':   
                self.RR1_SurrVeh_Lane4_trans_z = float(row[1])     

            elif row[0] == 'RR2_SurrVeh_Lane1_trans_x':   
                self.RR2_SurrVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane1_trans_z':   
                self.RR2_SurrVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane2_trans_x':   
                self.RR2_SurrVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane2_trans_z':   
                self.RR2_SurrVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane3_trans_x':   
                self.RR2_SurrVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane3_trans_z':   
                self.RR2_SurrVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane4_trans_x':   
                self.RR2_SurrVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR2_SurrVeh_Lane4_trans_z':   
                self.RR2_SurrVeh_Lane4_trans_z = float(row[1])

            elif row[0] == 'RR3_SurrVeh_Lane1_trans_x':   
                self.RR3_SurrVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane1_trans_z':   
                self.RR3_SurrVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane2_trans_x':   
                self.RR3_SurrVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane2_trans_z':   
                self.RR3_SurrVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane3_trans_x':   
                self.RR3_SurrVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane3_trans_z':   
                self.RR3_SurrVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane4_trans_x':   
                self.RR3_SurrVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR3_SurrVeh_Lane4_trans_z':   
                self.RR3_SurrVeh_Lane4_trans_z = float(row[1])

           

            elif row[0] == 'RR1_Stop_nGoVeh_Lane1_trans_x':
                self.RR1_Stop_nGoVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane1_trans_z':   
                self.RR1_Stop_nGoVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane2_trans_x':
                self.RR1_Stop_nGoVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane2_trans_z':   
                self.RR1_Stop_nGoVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane3_trans_x':
                self.RR1_Stop_nGoVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane3_trans_z':   
                self.RR1_Stop_nGoVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane4_trans_x':
                self.RR1_Stop_nGoVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR1_Stop_nGoVeh_Lane4_trans_z':   
                self.RR1_Stop_nGoVeh_Lane4_trans_z = float(row[1])

            elif row[0] == 'RR2_Stop_nGoVeh_Lane1_trans_x':
                self.RR2_Stop_nGoVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane1_trans_z':   
                self.RR2_Stop_nGoVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane2_trans_x':
                self.RR2_Stop_nGoVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane2_trans_z':   
                self.RR2_Stop_nGoVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane3_trans_x':
                self.RR2_Stop_nGoVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane3_trans_z':   
                self.RR2_Stop_nGoVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane4_trans_x':
                self.RR2_Stop_nGoVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR2_Stop_nGoVeh_Lane4_trans_z':   
                self.RR2_Stop_nGoVeh_Lane4_trans_z = float(row[1])

            elif row[0] == 'RR3_Stop_nGoVeh_Lane1_trans_x':
                self.RR3_Stop_nGoVeh_Lane1_trans_x = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane1_trans_z':   
                self.RR3_Stop_nGoVeh_Lane1_trans_z = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane2_trans_x':
                self.RR3_Stop_nGoVeh_Lane2_trans_x = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane2_trans_z':   
                self.RR3_Stop_nGoVeh_Lane2_trans_z = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane3_trans_x':
                self.RR3_Stop_nGoVeh_Lane3_trans_x = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane3_trans_z':   
                self.RR3_Stop_nGoVeh_Lane3_trans_z = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane4_trans_x':
                self.RR3_Stop_nGoVeh_Lane4_trans_x = float(row[1])
            elif row[0] == 'RR3_Stop_nGoVeh_Lane4_trans_z':   
                self.RR3_Stop_nGoVeh_Lane4_trans_z = float(row[1])



            elif row[0] == 'RR1_BrokenVeh_trans_x':
                self.RR1_BrokenVeh_trans_x = float(row[1])
            elif row[0] == 'RR1_BrokenVeh_trans_z':   
                self.RR1_BrokenVeh_trans_z = float(row[1])
            elif row[0] == 'RR2_BrokenVeh_trans_x':
                self.RR2_BrokenVeh_trans_x = float(row[1])
            elif row[0] == 'RR2_BrokenVeh_trans_z':   
                self.RR2_BrokenVeh_trans_z = float(row[1])
            elif row[0] == 'RR3_BrokenVeh_trans_x':
                self.RR3_BrokenVeh_trans_x = float(row[1])
            elif row[0] == 'RR3_BrokenVeh_trans_z':   
                self.RR3_BrokenVeh_trans_z = float(row[1])


        self.xoffsetLane = [0] * self.numRingRoad
        for i in range(0,self.numRingRoad):
            self.xoffsetLane[i] = self.xoffsetRoad[i]-self.laneWidth*self.numLane/2


        self.lanePositions=[[0 for j in range(self.numLane)] for i in range(self.numRingRoad)]
        for i  in range(0,self.numRingRoad):
            for j  in range(0,self.numLane):
                self.lanePositions[i][j]=self.xoffsetLane[i]+(2*j+1)*self.laneWidth/2
        
                
                        
                
    def load_Msg_PosPara(self, paraFile):
        self.Pos_Msg0 = 0
        self.Pos_Msg1 = [0] * self.numRingRoad
        self.Pos_Msg2 = [0] * self.numRingRoad
        self.Pos_Msg2b = [0] * self.numRingRoad
        self.Pos_Msg3 = [0] * self.numRingRoad
        self.Pos_Msg4 = [0] * self.numRingRoad
        self.Pos_Msg5a = [0] * self.numRingRoad
        self.Pos_Msg5b = [0] * self.numRingRoad
        self.Pos_Msg6 = [0] * self.numRingRoad
        self.Pos_Msg7 = [0] * self.numRingRoad
        self.Pos_Msg8 = [0] * self.numRingRoad
        self.Pos_Msg9 = [0] * self.numRingRoad
        
        for row in paraFile:  
            if row[0] == 'RR1_PosMsg0':
                self.Pos_Msg0= float(row[1])  
            elif row[0] == 'RR1_PosMsg1':
                self.Pos_Msg1[0] = float(row[1])                
            elif row[0] == 'RR2_PosMsg1':
                self.Pos_Msg1[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg1':
                self.Pos_Msg1[2] = float(row[1])                

            if row[0] == 'RR1_PosMsg2':
                self.Pos_Msg2[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg2':
                self.Pos_Msg2[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg2':
                self.Pos_Msg2[2] = float(row[1])                  

            if row[0] == 'RR1_PosMsg2b':
                self.Pos_Msg2b[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg2b':
                self.Pos_Msg2b[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg2b':
                self.Pos_Msg2b[2] = float(row[1])                  


            if row[0] == 'RR1_PosMsg3':
                self.Pos_Msg3[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg3':
                self.Pos_Msg3[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg3':
                self.Pos_Msg3[2] = float(row[1])       

            if row[0] == 'RR1_PosMsg4':
                self.Pos_Msg4[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg4':
                self.Pos_Msg4[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg4':
                self.Pos_Msg4[2] = float(row[1])  

            if row[0] == 'RR1_PosMsg5a':
                self.Pos_Msg5a[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg5a':
                self.Pos_Msg5a[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg5a':
                self.Pos_Msg5a[2] = float(row[1])  
                
            if row[0] == 'RR1_PosMsg5b':
                self.Pos_Msg5b[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg5b':
                self.Pos_Msg5b[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg5b':
                self.Pos_Msg5b[2] = float(row[1])  
                
            if row[0] == 'RR1_PosMsg6':
                self.Pos_Msg6[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg6':
                self.Pos_Msg6[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg6':
                self.Pos_Msg6[2] = float(row[1])  

            if row[0] == 'RR1_PosMsg7':
                self.Pos_Msg7[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg7':
                self.Pos_Msg7[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg7':
                self.Pos_Msg7[2] = float(row[1]) 

            if row[0] == 'RR1_PosMsg8':
                self.Pos_Msg8[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg8':
                self.Pos_Msg8[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg8':
                self.Pos_Msg8[2] = float(row[1])

            if row[0] == 'RR1_PosMsg9':
                self.Pos_Msg9[0] = float(row[1])
            elif row[0] == 'RR2_PosMsg9':
                self.Pos_Msg9[1] = float(row[1])                
            elif row[0] == 'RR3_PosMsg9':
                self.Pos_Msg9[2] = float(row[1])    

                
               
    def load_TL_PosPara(self, paraFile):

        for row in paraFile:
         
            if row[0] == 'RR1_translation_z':
                self.RRX_translation_z[0] = float(row[1])
            elif row[0] == 'RR2_translation_z':   
                self.RRX_translation_z[1] = float(row[1])
            elif row[0] == 'RR3_translation_z':   
                self.RRX_translation_z[2] = float(row[1])
            elif row[0] == 'RR4_translation_z':   
                self.RRX_translation_z[3] = float(row[1])



         
            if row[0] == 'RR1_TLonrmp_trans_x':
                self.RRX_TLonrmp_trans_x[0] = float(row[1])
            elif row[0] == 'RR1_TLonrmp_trans_z':   
                self.RRX_TLonrmp_trans_z[0] = float(row[1])
            elif row[0] == 'RR2_TLonrmp_trans_x':   
                self.RRX_TLonrmp_trans_x[1] = float(row[1])
            elif row[0] == 'RR2_TLonrmp_trans_z':   
                self.RRX_TLonrmp_trans_z[1] = float(row[1])
            elif row[0] == 'RR3_TLonrmp_trans_x':   
                self.RRX_TLonrmp_trans_x[2] = float(row[1])
            elif row[0] == 'RR3_TLonrmp_trans_z':   
                self.RRX_TLonrmp_trans_z[2] = float(row[1])

                
            elif row[0] == 'RR1_TLImgnry_StnGo_trans_x':   
                self.RRX_TLImgnry_StnGo_trans_x[0] = float(row[1])
            elif row[0] == 'RR1_TLImgnry_StnGo_trans_z':   
                self.RRX_TLImgnry_StnGo_trans_z[0] = float(row[1])
            elif row[0] == 'RR2_TLImgnry_StnGo_trans_x':   
                self.RRX_TLImgnry_StnGo_trans_x[1] = float(row[1])
            elif row[0] == 'RR2_TLImgnry_StnGo_trans_z':   
                self.RRX_TLImgnry_StnGo_trans_z[1] = float(row[1])
            elif row[0] == 'RR3_TLImgnry_StnGo_trans_x':   
                self.RRX_TLImgnry_StnGo_trans_x[2] = float(row[1])
            elif row[0] == 'RR3_TLImgnry_StnGo_trans_z':
                self.RRX_TLImgnry_StnGo_trans_z[2] = float(row[1]) 

               
            elif row[0] == 'RR1_TLImgnry_TakeOver_trans_x':   
                self.RRX_TLImgnry_TakeOver_trans_x[0] = float(row[1])
            elif row[0] == 'RR1_TLImgnry_TakeOver_trans_z':   
                self.RRX_TLImgnry_TakeOver_trans_z[0] = float(row[1])
            elif row[0] == 'RR2_TLImgnry_TakeOver_trans_x':   
                self.RRX_TLImgnry_TakeOver_trans_x[1] = float(row[1])
            elif row[0] == 'RR2_TLImgnry_TakeOver_trans_z':   
                self.RRX_TLImgnry_TakeOver_trans_z[1] = float(row[1])
            elif row[0] == 'RR3_TLImgnry_TakeOver_trans_x':   
                self.RRX_TLImgnry_TakeOver_trans_x[2] = float(row[1])
            elif row[0] == 'RR3_TLImgnry_TakeOver_trans_z':   
                self.RRX_TLImgnry_TakeOver_trans_z[2] = float(row[1])
             
            #Added  Traffic lights at the end of each road               
            elif row[0] == 'RR1_TLoffrmp_trans_x':   
                self.RRX_TLoffrmp_trans_x[0] = float(row[1])
            elif row[0] == 'RR1_TLoffrmp_trans_z':   
                self.RRX_TLoffrmp_trans_z[0] = float(row[1]) 
            elif row[0] == 'RR2_TLoffrmp_trans_x':   
                self.RRX_TLoffrmp_trans_x[1] = float(row[1])
            elif row[0] == 'RR2_TLoffrmp_trans_z':   
                self.RRX_TLoffrmp_trans_z[1] = float(row[1])
            elif row[0] == 'RR3_TLoffrmp_trans_x':   
                self.RRX_TLoffrmp_trans_x[2] = float(row[1])
            elif row[0] == 'RR3_TLoffrmp_trans_x':   
                self.RRX_TLoffrmp_trans_z[2] = float(row[1])
  
            elif row[0] == 'RR1_PosTLImgnry_TurnRed':   
                self.RRX_PosTLImgnry_TurnRed[0] = float(row[1])
            elif row[0] == 'RR2_PosTLImgnry_TurnRed':   
                self.RRX_PosTLImgnry_TurnRed[1] = float(row[1])
            elif row[0] == 'RR3_PosTLImgnry_TurnRed':   
                self.RRX_PosTLImgnry_TurnRed[2] = float(row[1])    
                
            elif row[0] =='RR1_NDRT':
                self.RRX_NDRT[0] = int(row[1])
            elif row[0] =='RR2_NDRT':
                self.RRX_NDRT[1] = int(row[1])
            elif row[0] =='RR3_NDRT':
                self.RRX_NDRT[2] = int(row[1])
 
    def loadTimeSyn(self, paraFile):


        for row in paraFile:
            if row[0] == 'DeltT_TL_surr':
                self.DeltT_TL_surr = float(row[1])
              
            elif row[0] == 'DeltT_TL_backup':
                self.DeltT_TL_backup = float(row[1])
            elif row[0] == 'DeltT_TL_broken':
                self.DeltT_TL_broken = float(row[1])
            elif row[0] == 'DeltT_TL_stopngo':
                self.DeltT_TL_stopngo = float(row[1])
            elif row[0] == 'DeltaZ_TL_onramp':   
                self.DeltaZ_TL_onramp = float(row[1])
            elif row[0] == 'DeltaZ_TL_stopngo':   
                self.DeltaZ_TL_stopngo = float(row[1])
            elif row[0] == 'DeltaZ_TL_takeover':   
                self.DeltaZ_TL_takeover = float(row[1])
            elif row[0] == 'DeltaZ_TL_offramp':   
                self.DeltaZ_TL_offramp = float(row[1])                
            elif row[0] == 'aheadTL_Green':
                self.aheadTL_Green = float(row[1])

                
                
            
            elif row[0] =='DeltaZ_Msg1':
                self.DeltaZ_Msg1 = int(row[1])                
            elif row[0] =='DeltaZ_Msg2':
                self.DeltaZ_Msg2 = int(row[1])
            elif row[0] =='DeltaZ_Msg3':
                self.DeltaZ_Msg3 = int(row[1])
            elif row[0] =='DeltaZ_Msg4':
                self.DeltaZ_Msg4 = int(row[1])
            elif row[0] =='DeltaZ_Msg5a':
                self.DeltaZ_Msg5a = int(row[1])
            elif row[0] =='DeltaZ_Msg5b':
                self.DeltaZ_Msg5b = int(row[1])
            elif row[0] =='DeltaZ_Msg6':
                self.DeltaZ_Msg6 = int(row[1])
            elif row[0] =='DeltaZ_Msg7':
                self.DeltaZ_Msg7 = int(row[1])
            elif row[0] =='DeltaZ_Msg8':
                self.DeltaZ_Msg8 = int(row[1])
        
            elif row[0] =='DeltaZ_Msg1':
                self.DeltaZ_Msg1 = int(row[1])                
            elif row[0] =='DeltaZ_Msg2':
                self.DeltaZ_Msg2 = int(row[1])
            elif row[0] =='DeltaZ_Msg3':
                self.DeltaZ_Msg3 = int(row[1])
            elif row[0] =='DeltaZ_Msg4':
                self.DeltaZ_Msg4 = int(row[1])
            elif row[0] =='DeltaZ_Msg5a':
                self.DeltaZ_Msg5a = int(row[1])
            elif row[0] =='DeltaZ_Msg5b':
                self.DeltaZ_Msg5b = int(row[1])
            elif row[0] =='DeltaZ_Msg6':
                self.DeltaZ_Msg6 = int(row[1])
            elif row[0] =='DeltaZ_Msg7':
                self.DeltaZ_Msg7 = int(row[1])
            elif row[0] =='DeltaZ_Msg8':
                self.DeltaZ_Msg8 = int(row[1])      
                
 #       self.RRX_NDRT = [self.RR1_NDRT, self.RR2_NDRT, self.RR3_NDRT]
        #for i in range(0,self.numRingRoad):
#            self.Pos_Msg2[i] = self.zoffsetRoad[i] + self.startLength+self.mergeLength+self.Stop_and_Go_length-self.DeltaZ_Msg2;
    def loadCtrlPara(self, paraFile):

        for row in paraFile:
            #define lateral control parameters
            if row[0] == 'LaneChg_PID_P':
                self.LaneChg_PID_P = float(row[1])
            elif row[0] == 'LaneChg_PID_I':   
                self.LaneChg_PID_I = float(row[1])
            elif row[0] == 'LaneChg_PID_D':   
                self.LaneChg_PID_D = float(row[1])
            elif row[0] == 'LaneChg_PID_K':   
                self.LaneChg_PID_K = float(row[1])

            if row[0] == 'LaneChg_PID_P_S':
                self.LaneChg_PID_P_S = float(row[1])
            elif row[0] == 'LaneChg_PID_I_S':   
                self.LaneChg_PID_I_S = float(row[1])
            elif row[0] == 'LaneChg_PID_D_S':   
                self.LaneChg_PID_D_S = float(row[1])
            elif row[0] == 'LaneChg_PID_K_S':   
                self.LaneChg_PID_K_S = float(row[1])

                
            #define lane change parameters.
            elif row[0] == 'myMinRespGap':   
                self.myMinRespGap = float(row[1])
            elif row[0] == 'myfastSpdGap':   
                self.myfastSpdGap = float(row[1])
            #defines the slope of the cut-in transition function    
            elif row[0] == 'Ktg':   
                self.Ktg = float(row[1])
            #cut-in transition duration 4 second
            elif row[0] == 'cutinDuration':   
                self.cutinDuration = float(row[1])
                
                

            #define controller (CACC and FVDM)parameters
            elif row[0] == 'myDesSpeedGain':   
                self.myDesSpeedGain = float(row[1])
            elif row[0] == 'mySpeedControlGain':   
                self.mySpeedControlGain = float(row[1])
            elif row[0] == 'myGapControlGainGap':   
                self.myGapControlGainGap = float(row[1])
            elif row[0] == 'myGapControlGainGapDot':   
                self.myGapControlGainGapDot = float(row[1])            
            elif row[0] == 'myCollisionAvoidanceGainGap':   
                self.myCollisionAvoidanceGainGap = float(row[1])            
            elif row[0] == 'myCollisionAvoidanceGainGapDot':   
                self.myCollisionAvoidanceGainGapDot = float(row[1])       
            elif row[0] == 'myGapClosingControlGainGap':   
                self.myGapClosingControlGainGap = float(row[1])
            elif row[0] == 'myGapClosingControlGainGapDot':   
                self.myGapClosingControlGainGapDot = float(row[1])
            elif row[0] == 'T_adpt':   
                self.T_adpt = float(row[1])
            elif row[0] == 'Lamda':   
                self.Lambda = float(row[1])  
            elif row[0] == 'Lambda1':   
                self.Lambda1 = float(row[1])   
            elif row[0] == 'Lambda2':   
                self.Lambda2 = float(row[1])                   
            elif row[0] == 'maxDetectRange':   
                self.maxDetectRange = float(row[1])     