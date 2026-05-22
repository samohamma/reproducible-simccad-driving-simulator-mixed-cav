#################### define display information  #######################################
RampDefaultIcon_file='../icon/RampDefault.png' #Added
MainDefaultIcon_file = '../icon/MainDefault.png' #Added
MergeIcon_file = '../icon/Merge.png'
MoveToRightIcon_file = '../icon/MoveToRight.png'
DivergeIcon_file = '../icon/Diverge.png'
#DefaultIcon_file = '../icon/Default.png' #Previous


#DefaultIcon = display.imageLoad(DefaultIcon_file) #Revised: No longer needed

RampDefaultIcon=display.imageLoad(RampDefaultIcon_file) #Added
MainDefaultIcon=display.imageLoad(MainDefaultIcon_file) #Added 
MergeIcon = display.imageLoad(MergeIcon_file)
MoveToRightIcon = display.imageLoad(MoveToRightIcon_file)
DivergeIcon = display.imageLoad(DivergeIcon_file)

#display.imagePaste(DefaultIcon, 0, 0, False) #Revised: Changed to:
display.imagePaste(RampDefaultIcon, 0, 0, False)


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
            
    elif not Main_road: # The vehicle is not driving in the in the main freeway     
        if myGps_z>= ctrlPara.RRX_TLonrmp_trans_z[ringNum]-ctrlPara.DeltaZ_TL_onramp and myGps_z < ctrlPara.RRX_TLonrmp_trans_z[ringNum]:
            display.imagePaste(MergeIcon, 0, 0, False) # merge condition
        else:
            display.imagePaste(RampDefaultIcon, 0, 0, False) # This should be deafualt.      
        
    #display.drawText("{:.0f}".format(ctrlPara.mySpeedLimit[ringNum]*3.6)+' Km/h', 290, 50)
    display.setColor(0xFF0000)    
    display.drawText("{:.0f}".format(ctrlPara.mySpeedLimit[ringNum]*3.6), 15, 20)    
    

    #display.drawText("{:.1f}".format(speed*3.6)+' Km/h', 290, 220)
    display.setColor(0xABA7AA)
    display.drawText("{:.1f}".format(speed*3.6), 130, 125)