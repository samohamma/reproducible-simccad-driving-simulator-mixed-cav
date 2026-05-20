// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOYSTICK_INTERFACE__HPP
#define JOYSTICK_INTERFACE__HPP

#include <map>
#include <webots/Emitter.hpp>
#include <webots/Radar.hpp>
#include <webots/GPS.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

namespace webots {
  class Driver;
  class Joystick;
  class Radar;
  class GPS;
  class Emitter;
  class Display;
  class ImageRef;
  class Node;
  class Supervisor;
  
}  // namespace webots


struct LCMessage
{
   char cmd[3];
   char isfirst;
   double speed;
   double position[3];
};


struct ControlPara
{
	int numRingRoad;
	int startLength;
	int mergeLength;
	int stop_and_go_length;
	int hdriveLength;
	int hdriveLength_pre;
	int cadriveLength;
	int takeoverLength;
	int divergeLength;


	// define waiting time interval for switching mode
	int switchWaitingTime;



	double zoffsetRoad[4];  

	double pos_ramp_entry[4];

	double pos_diverge_entry[4];

	double pos_hdTocad[4];
				
	double pos_cadTohd[4];  

    double pos_stop_and_go[4];
	
	double pos_diverge[4];
		
	//double myDesSpeed[3];

	//double myHeadwayTime[3][4];

	//double myMinGap;
	//double myOriginGap;
		
	int infoDispTime;



/*		
	int vehNumber_Lane;

	int RowNumber_Backup;
	int RowNumber_SurrVeh;
	int RowNumber_Stop_nGoVeh;

	double RR1_Backup_Lane1_trans_x;
	double RR1_Backup_Lane1_trans_z;
	double RR1_Backup_Lane2_trans_x;
	double RR1_Backup_Lane2_trans_z;
	double RR1_Backup_Lane3_trans_x;
	double RR1_Backup_Lane3_trans_z;
	double RR1_Backup_Lane4_trans_x;
	double RR1_Backup_Lane4_trans_z;
	double RR2_Backup_Lane1_trans_x;
	double RR2_Backup_Lane1_trans_z;
	double RR2_Backup_Lane2_trans_x;
	double RR2_Backup_Lane2_trans_z;
	double RR2_Backup_Lane3_trans_x;
	double RR2_Backup_Lane3_trans_z;
	double RR2_Backup_Lane4_trans_x;
	double RR2_Backup_Lane4_trans_z;
	double RR3_Backup_Lane1_trans_x;
	double RR3_Backup_Lane1_trans_z;
	double RR3_Backup_Lane2_trans_x;
	double RR3_Backup_Lane2_trans_z;
	double RR3_Backup_Lane3_trans_x;
	double RR3_Backup_Lane3_trans_z;
	double RR3_Backup_Lane4_trans_x;
	double RR3_Backup_Lane4_trans_z;
	double RR1_SurrVeh_Lane1_trans_x;
	double RR1_SurrVeh_Lane1_trans_z;
	double RR1_SurrVeh_Lane2_trans_x;
	double RR1_SurrVeh_Lane2_trans_z;
	double RR1_SurrVeh_Lane3_trans_x;
	double RR1_SurrVeh_Lane3_trans_z;
	double RR1_SurrVeh_Lane4_trans_x;
	double RR1_SurrVeh_Lane4_trans_z;
	double RR2_SurrVeh_Lane1_trans_x;
	double RR2_SurrVeh_Lane1_trans_z;
	double RR2_SurrVeh_Lane2_trans_x;
	double RR2_SurrVeh_Lane2_trans_z;
	double RR2_SurrVeh_Lane3_trans_x;
	double RR2_SurrVeh_Lane3_trans_z;
	double RR2_SurrVeh_Lane4_trans_x;
	double RR2_SurrVeh_Lane4_trans_z;
	double RR3_SurrVeh_Lane1_trans_x;
	double RR3_SurrVeh_Lane1_trans_z;
	double RR3_SurrVeh_Lane2_trans_x;
	double RR3_SurrVeh_Lane2_trans_z;
	double RR3_SurrVeh_Lane3_trans_x;
	double RR3_SurrVeh_Lane3_trans_z;
	double RR3_SurrVeh_Lane4_trans_x;
	double RR3_SurrVeh_Lane4_trans_z;
	double RR1_Stop_nGoVeh_Lane1_trans_x;
	double RR1_Stop_nGoVeh_Lane1_trans_z;
	double RR1_Stop_nGoVeh_Lane2_trans_x;
	double RR1_Stop_nGoVeh_Lane2_trans_z;
	double RR1_Stop_nGoVeh_Lane3_trans_x;
	double RR1_Stop_nGoVeh_Lane3_trans_z;
	double RR1_Stop_nGoVeh_Lane4_trans_x;
	double RR1_Stop_nGoVeh_Lane4_trans_z;
	double RR2_Stop_nGoVeh_Lane1_trans_x;
	double RR2_Stop_nGoVeh_Lane1_trans_z;
	double RR2_Stop_nGoVeh_Lane2_trans_x;
	double RR2_Stop_nGoVeh_Lane2_trans_z;
	double RR2_Stop_nGoVeh_Lane3_trans_x;
	double RR2_Stop_nGoVeh_Lane3_trans_z;
	double RR2_Stop_nGoVeh_Lane4_trans_x;
	double RR2_Stop_nGoVeh_Lane4_trans_z;
	double RR3_Stop_nGoVeh_Lane1_trans_x;
	double RR3_Stop_nGoVeh_Lane1_trans_z;
	double RR3_Stop_nGoVeh_Lane2_trans_x;
	double RR3_Stop_nGoVeh_Lane2_trans_z;
	double RR3_Stop_nGoVeh_Lane3_trans_x;
	double RR3_Stop_nGoVeh_Lane3_trans_z;
	double RR3_Stop_nGoVeh_Lane4_trans_x;
	double RR3_Stop_nGoVeh_Lane4_trans_z;
	double RR1_BrokenVeh_trans_x;
	double RR1_BrokenVeh_trans_z;
	double RR2_BrokenVeh_trans_x;
	double RR2_BrokenVeh_trans_z;
	double RR3_BrokenVeh_trans_x;
	double RR3_BrokenVeh_trans_z;

	int L_Msg1;
	int L_Msg2;
	int L_Msg3;
	int L_Msg4;
	int L_Msg5;
	int L_Msg6;
	int L_Msg7;
	int L_Msg8;
	int RR_1_NDRT;
	int RR_2_NDRT;
	int RR_3_NDRT;
*/

			
	//load traffic light position
	//double RR1_TLonrmp_trans_x;
	double RR1_TLonrmp_trans_z;
	//double RR2_TLonrmp_trans_x;
	double RR2_TLonrmp_trans_z;
	//double RR3_TLonrmp_trans_x;
	double RR3_TLonrmp_trans_z;
	//double RR1_TLImgnry_StnGo_trans_x;
	double RR1_TLImgnry_StnGo_trans_z;
	//double RR2_TLImgnry_StnGo_trans_x;
	double RR2_TLImgnry_StnGo_trans_z;
	//double RR3_TLImgnry_StnGo_trans_x;
	double RR3_TLImgnry_StnGo_trans_z;
	//double RR1_TLImgnry_TakeOver_trans_x;
	double RR1_TLImgnry_TakeOver_trans_z;
	//double RR2_TLImgnry_TakeOver_trans_x;
	double RR2_TLImgnry_TakeOver_trans_z;
	//double RR3_TLImgnry_TakeOver_trans_x;
	double RR3_TLImgnry_TakeOver_trans_z;

	// load time settings
	//int T_start_surr;
	//int T_start_takeover;	
	//int T_start_backup;	
	//int aheadTL_Green;    
	
	int DeltaZ_TL_onramp;
	int DeltaZ_TL_takeover;
	int DeltaZ_TL_stopngo;
	
	int DeltaZ_Msg1;
	int DeltaZ_Msg2;
	int DeltaZ_Msg3;
	int DeltaZ_Msg6;
	int DeltaZ_Msg7;
	int DeltaZ_Msg8;

    






  
  
  
};







//class JoystickInterface: public Supervisor {
class JoystickInterface {
public:
  explicit JoystickInterface(webots::Driver *driver);
  JoystickInterface(webots::Driver *driver, const char *configFile);
  virtual ~JoystickInterface() {}

  bool step();

private:
  static void fatal(const std::string &txt);
  static bool fileExists(const std::string &name);
  static double convertFeedback(int raw, int minimum, int maximum);

  void init(webots::Driver *driver, const char *configFile);

  void initCtrlPara(const char *configFile);
  
  double myTLzPos[3][3];
  int toCtrlTLIndex[2];
  
  int mGear;
  webots::Driver::WiperMode mWiperMode;

  webots::Driver *mDriver;
  webots::Joystick *mJoystick;
  webots::Radar *mRadar;
  webots::GPS *mGps;
  webots::Emitter *mEmitter;
  webots::Display *mDisplay;
  webots::ImageRef *mImg;
  webots::Field *mcontrollerName;
  webots::Node *TL_onramp;
  webots::Node *TL_Imgnry_StnGo;
  webots::Node *TL_Imgnry_TakeOver;
  webots::Node *myTrafficLight;
  
  
  std::ofstream moutfile;
  
  LCMessage mLcMessage;
  
  ControlPara mControlPara;


  int ringNum;
  string messageId;
  string toPlayMessageId;
  string nextMessageId;
	
  const LCMessage* mptrMessage;
  
  std::map<const std::string, int> mAxesMap;
  std::map<const std::string, int> mAxesBoundsMap;
  std::map<const std::string, int> mButtonsMap;
  std::map<const std::string, double> mGainMap;
};

#endif  // JOYSTICK_INTERFACE__HPP
