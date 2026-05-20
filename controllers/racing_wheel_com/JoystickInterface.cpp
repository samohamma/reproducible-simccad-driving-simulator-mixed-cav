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

#include <QtCore/QSettings>
#include <webots/Joystick.hpp>
#include <webots/vehicle/Driver.hpp>

#include<Windows.h>
#include<mmsystem.h>
//#pragma comment(lib,"WINMM.LIB")

#include "JoystickInterface.hpp"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <string> 
#include <iomanip>  
#include <vector>




#define NAXIS 3
#define NAXISBOUNDS 2
#define NBUTTONS 14
#define NGAINS 2
#define NUMMESSAGE 10


// define 
#define TurningLeftIcon_file "../icon/left-arrow.png"
#define TurningRightIcon_file  "../icon/right-arrow.png"
#define StayonIcon_file  "../icon/up-arrow.png"

using namespace webots;
using namespace std;



 

static const char *TL_Name[3][3] = {{"TL_onrmp0", "TL_img_StnGo0", "TL_img_takeOver0"}, 
									{"TL_onrmp1", "TL_img_StnGo1", "TL_img_takeOver1"}, 
									{"TL_onrmp2", "TL_img_StnGo2", "TL_img_takeOver2"}};



//static const char *msgNames[NUMMESSAGE] = {"Msg0", "Msg1", "Msg2", "Msg3", "Msg4", "Msg5a", "Msg5b", "Msg6", "Msg7", "Msg8"};


static const char *axesNames[NAXIS] = {"Steering", "Throttle", "Brake"};

static const char *axesBoundNames[NAXISBOUNDS] = {"min", "max"};

static const char *buttonNames[NBUTTONS] = {"SwitchMode", "RightWarning", "LeftWarning",    "NextGear",          "PreviousGear", "FirstGear",
                                            "SecondGear",   "ThirdGear",      "FourthGear",        "FifthGear",    "SixthGear",
                                            "ReverseGear",  "NextWiperMode ", "PreviousWiperMode "};

static const char *gainNames[NGAINS] = {"AutoCentering", "Resistance"};

#ifdef _WIN32
static string platformExtension = "windows";
#elif defined(__APPLE__)
static string platformExtension = "mac";
#elif defined(__linux__)
static string platformExtension = "linux";
#else
#error Unsupported OS
#endif

void JoystickInterface::fatal(const string &txt) {
  cerr << txt << endl;
  exit(-1);
}

bool JoystickInterface::fileExists(const string &name) {
  ifstream f(name.c_str());
  return f.good();
}

double JoystickInterface::convertFeedback(int raw, int minimum, int maximum) {
  if (maximum == minimum)
    fatal("Prevent division by 0.");
  return std::max(0.0, std::min(1.0, ((double)(raw - minimum)) / ((double)(maximum - minimum))));
}

void JoystickInterface::init(webots::Driver *driver, const char *configFile) {
  mDriver = driver;
  
  mGear = 1;

  
  mWiperMode = Driver::DOWN;
  mDriver->setGear(mGear);
  mDriver->setWiperMode(mWiperMode);

  if (!mJoystick) {
    mJoystick = driver->getJoystick();
    mJoystick->enable(mDriver->getBasicTimeStep());
    driver->step();
  }

  if (mJoystick->isConnected())
    cout << "'" << mJoystick->getModel() << "' detected (the following configuration file is used: '" << configFile << "')."
         << endl;

cout <<"time step is "<< mDriver->getBasicTimeStep()<<"ms."<<endl;

  QSettings settings(configFile, QSettings::IniFormat);
  settings.setIniCodec("UTF-8");
  for (int i = 0; i < NAXIS; ++i)
    mAxesMap[axesNames[i]] = settings.value(QString("Axis/") + QString(axesNames[i]), -1).toInt();
  for (int i = 0; i < NAXIS; ++i) {
    for (int j = 0; j < NAXISBOUNDS; ++j) {
      string boundName = string() + axesBoundNames[j] + axesNames[i];
      mAxesBoundsMap[boundName] = settings.value(QString("AxisBounds/") + QString::fromStdString(boundName), 0).toInt();
    }
  }
  for (int i = 0; i < NBUTTONS; ++i)
    mButtonsMap[buttonNames[i]] = settings.value(QString("Buttons/") + QString(buttonNames[i]), -1).toInt();
  for (int i = 0; i < NGAINS; ++i)
    mGainMap[gainNames[i]] = settings.value(QString("Gains/") + QString(gainNames[i]), 0.0).toDouble();

  if (mJoystick->isConnected())
    mJoystick->setForceAxis(mAxesMap["Steering"]);





}

void JoystickInterface::initCtrlPara (const char *configFile){

  mControlPara.numRingRoad = 4;
  mControlPara.startLength =1000;
  mControlPara.mergeLength =500;
  mControlPara.stop_and_go_length = 100;
  mControlPara.hdriveLength =1500;
  mControlPara.hdriveLength_pre =1000;
  mControlPara.cadriveLength =4000;
  mControlPara.takeoverLength =2000;
  mControlPara.divergeLength =500;

// define waiting time interval for switching mode
  mControlPara.switchWaitingTime =20000;

  mControlPara.zoffsetRoad[0] = -18000;
  mControlPara.zoffsetRoad[1] = -9007.6;
  mControlPara.zoffsetRoad[2] = -15.2;
  mControlPara.zoffsetRoad[3] = 8977.2;


  mControlPara.infoDispTime = 200;

  	//double RR1_TLonrmp_trans_x;
	mControlPara.RR1_TLonrmp_trans_z = -17000;
	//double RR2_TLonrmp_trans_x;
	mControlPara.RR2_TLonrmp_trans_z = -11181.31168;
	//double RR3_TLonrmp_trans_x;
	mControlPara.RR3_TLonrmp_trans_z = -5362.623367;
	//double RR1_TLImgnry_StnGo_trans_x;
	mControlPara.RR1_TLImgnry_StnGo_trans_z = -16200;
	//double RR2_TLImgnry_StnGo_trans_x;
	mControlPara.RR2_TLImgnry_StnGo_trans_z = -10381.31168;
	//double RR3_TLImgnry_StnGo_trans_x;
	mControlPara.RR3_TLImgnry_StnGo_trans_z = -4562.623367;
	//double RR1_TLImgnry_TakeOver_trans_x;
	mControlPara.RR1_TLImgnry_TakeOver_trans_z = -13100;
	//double RR2_TLImgnry_TakeOver_trans_x;
	mControlPara.RR2_TLImgnry_TakeOver_trans_z = -5981.311684;
	//double RR3_TLImgnry_TakeOver_trans_x;
	mControlPara.RR3_TLImgnry_TakeOver_trans_z = -162.623367;

	// load time settings
	//mControlPara.T_start_surr = 4;
	mControlPara.DeltaZ_TL_onramp = 5;
	mControlPara.DeltaZ_TL_stopngo = 200;
	mControlPara.DeltaZ_TL_takeover = 200;	
	//time (second) ahead of traffic light turn green for participant to drive
	//mControlPara.aheadTL_Green = 7; 

    mControlPara.DeltaZ_Msg1 = 300;
    mControlPara.DeltaZ_Msg2 = 150;
    mControlPara.DeltaZ_Msg3 = 200;
    mControlPara.DeltaZ_Msg6 = 300;
    mControlPara.DeltaZ_Msg7 = 50;
    mControlPara.DeltaZ_Msg8 = 50;
	
	
	
	

  vector<vector<string>> content;
  vector<string> row;
  string line, word;
 
  fstream file (configFile, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
 			stringstream str(line);
 			while(getline(str, word, ','))
				row.push_back(word);
			content.push_back(row);
		}
	}
	else
		cout<<"Could not open the file\n";
 


	for(int i=0;i<(int)content.size();i++)
	{
		if (content[i][0] == "startLength")
		{
			mControlPara.startLength = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "mergeLength")
		{ 
			mControlPara.mergeLength = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "Stop_and_Go_length")
		{ 
			mControlPara.stop_and_go_length = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "hdriveLength")
		{ 
			mControlPara.hdriveLength = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "cadriveLength")
		{ 
			mControlPara.cadriveLength = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "takeoverLength")
		{ 
			mControlPara.takeoverLength = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "divergeLength")
		{ 
			mControlPara.divergeLength = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "switchWaitingTime")
		{ 
			mControlPara.switchWaitingTime = atoi(content[i][1].c_str()); 
		}
		else if (content[i][0] == "infoDispTime")
		{ 
			mControlPara.infoDispTime = atoi(content[i][1].c_str()); 
		}		
		else if (content[i][0] == "RR1_translation_z")
		{ 
			mControlPara.zoffsetRoad[0] = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR2_translation_z")
		{ 
			mControlPara.zoffsetRoad[1] = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR3_translation_z")
		{ 
			mControlPara.zoffsetRoad[2] = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR4_translation_z")
		{ 
			mControlPara.zoffsetRoad[3] = strtod(content[i][1].c_str(),NULL); 
		}



		else if (content[i][0] == "RR1_TLonrmp_trans_z")
		{ 
			mControlPara.RR1_TLonrmp_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR2_TLonrmp_trans_z")
		{ 
			mControlPara.RR2_TLonrmp_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR3_TLonrmp_trans_z")
		{ 
			mControlPara.RR3_TLonrmp_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR1_TLImgnry_StnGo_trans_z")
		{ 
			mControlPara.RR1_TLImgnry_StnGo_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR2_TLImgnry_StnGo_trans_z")
		{ 
			mControlPara.RR2_TLImgnry_StnGo_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR3_TLImgnry_StnGo_trans_z")
		{ 
			mControlPara.RR3_TLImgnry_StnGo_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR1_TLImgnry_TakeOver_trans_z")
		{ 
			mControlPara.RR1_TLImgnry_TakeOver_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR2_TLImgnry_TakeOver_trans_z")
		{ 
			mControlPara.RR2_TLImgnry_TakeOver_trans_z = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "RR3_TLImgnry_TakeOver_trans_z")
		{ 
			mControlPara.RR3_TLImgnry_TakeOver_trans_z = strtod(content[i][1].c_str(),NULL); 
		}

		else if (content[i][0] == "DeltaZ_TL_stopngo")
		{ 
			mControlPara.DeltaZ_TL_stopngo = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_TL_takeover")
		{ 
			mControlPara.DeltaZ_TL_takeover = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_Msg1")
		{ 
			mControlPara.DeltaZ_Msg1 = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_Msg2")
		{ 
			mControlPara.DeltaZ_Msg2 = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_Msg3")
		{ 
			mControlPara.DeltaZ_Msg3 = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_Msg6")
		{ 
			mControlPara.DeltaZ_Msg6 = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_Msg7")
		{ 
			mControlPara.DeltaZ_Msg7 = strtod(content[i][1].c_str(),NULL); 
		}
		else if (content[i][0] == "DeltaZ_Msg8")
		{ 
			mControlPara.DeltaZ_Msg8 = strtod(content[i][1].c_str(),NULL); 
		}
		
		// else if (content[i][0] == "T_start_surr")
		// { 
			// mControlPara.T_start_surr = strtod(content[i][1].c_str(),NULL); 
		// }
		// else if (content[i][0] == "aheadTL_Green")
		// { 
			// mControlPara.aheadTL_Green = strtod(content[i][1].c_str(),NULL); 
		// }






	
	}
	for (int i=0;i<mControlPara.numRingRoad;i++){
		mControlPara.pos_ramp_entry[i] = mControlPara.zoffsetRoad[i]+mControlPara.startLength;
		mControlPara.pos_diverge_entry[i] = mControlPara.zoffsetRoad[i]+mControlPara.startLength+ mControlPara.mergeLength + mControlPara.stop_and_go_length +mControlPara.hdriveLength +mControlPara.cadriveLength+ mControlPara.takeoverLength+mControlPara.divergeLength;
		mControlPara.pos_stop_and_go[i] = mControlPara.zoffsetRoad[i] + mControlPara.startLength+mControlPara.mergeLength;
		mControlPara.pos_hdTocad[i] = mControlPara.zoffsetRoad[i]+mControlPara.startLength+ mControlPara.mergeLength + mControlPara.stop_and_go_length +mControlPara.hdriveLength;
		mControlPara.pos_cadTohd[i] = mControlPara.zoffsetRoad[i]+mControlPara.startLength + mControlPara.mergeLength + mControlPara.stop_and_go_length + mControlPara.hdriveLength + mControlPara.cadriveLength;
		mControlPara.pos_diverge[i] = mControlPara.zoffsetRoad[i]+mControlPara.startLength + mControlPara.mergeLength + mControlPara.stop_and_go_length + mControlPara.hdriveLength + mControlPara.cadriveLength+mControlPara.takeoverLength;
	}
}

JoystickInterface::JoystickInterface(webots::Driver *driver) {
  // 1. find out "joystick_configuration_file" associated to the connected joystick
  string configurationFile;
  string configControlParaFile;
  mJoystick = driver->getJoystick();
  mJoystick->enable(driver->getBasicTimeStep());

  //initialize parameter
  configControlParaFile = "../Controller_input_param.csv";
  initCtrlPara(configControlParaFile.c_str());
  
  //reset hazard flash to false
  /*
  cout <<"current hazard state is "<<driver->getHazardFlashers()<<endl;
  driver->setHazardFlashers(false);
  cout <<"after setting current hazard state is "<<driver->getHazardFlashers()<<endl;
  driver->setIndicator((Driver::INDICATOR_RIGHT));
  while (driver->getTime()<5)
	continue;
  */
  
  // get Radar, GPS and Emitter
  mptrMessage = &mLcMessage;
  mRadar = driver->getRadar("radar");
  mRadar->enable(driver->getBasicTimeStep());
  mGps = driver->getGPS("gps");
  mGps->enable(driver->getBasicTimeStep());
  mEmitter = driver->getEmitter("emitter");

  mDisplay = driver->getDisplay("display");
  mDisplay->setColor(0xFF0000);
  mDisplay->setFont("Arial", 20, true);


  // get controller name
  Node *veh = driver->getSelf();
  if (!veh)
    // robot might be NULL if the controller is about to quit
    exit(1);
  //obtain current controller's name 	
  mcontrollerName = veh->getField("controller");
  cout << "controller Name: " << mcontrollerName->getSFString() << endl;
  
  //create recording file
  //id = veh->getId();

  time_t timep;
  char time_text[256] = {0};
  time(&timep);
  strftime( time_text, sizeof(time_text), "%Y%m%d-%H%M%S",localtime(&timep));
  string recvfile = to_string(veh->getId()) + '-'+string(time_text) + ".csv";
  moutfile.open(recvfile, ios::app);
  
  driver->step();
  
//obtain gps information and record 
  double gpsSpeed=mGps->getSpeed();
  double gpsPosition[3]={mGps->getValues()[0],mGps->getValues()[1],mGps->getValues()[2]};
  
// select which traffic light should be controlled by participant vehicle
  // myTLzPos[3][3] = {{mControlPara.RR1_TLonrmp_trans_z, mControlPara.RR1_TLImgnry_StnGo_trans_z, mControlPara.RR1_TLImgnry_TakeOver_trans_z},
					// {mControlPara.RR2_TLonrmp_trans_z, mControlPara.RR2_TLImgnry_StnGo_trans_z, mControlPara.RR2_TLImgnry_TakeOver_trans_z},
					// {mControlPara.RR3_TLonrmp_trans_z, mControlPara.RR3_TLImgnry_StnGo_trans_z, mControlPara.RR3_TLImgnry_TakeOver_trans_z}};
					
  myTLzPos[0][0] = mControlPara.RR1_TLonrmp_trans_z;
  myTLzPos[0][1] = mControlPara.RR1_TLImgnry_StnGo_trans_z;
  myTLzPos[0][2] = mControlPara.RR1_TLImgnry_TakeOver_trans_z;
  myTLzPos[1][0] = mControlPara.RR2_TLonrmp_trans_z;
  myTLzPos[1][1] = mControlPara.RR2_TLImgnry_StnGo_trans_z;
  myTLzPos[1][2] = mControlPara.RR2_TLImgnry_TakeOver_trans_z;
  myTLzPos[2][0] = mControlPara.RR3_TLonrmp_trans_z; 
  myTLzPos[2][1] = mControlPara.RR3_TLImgnry_StnGo_trans_z;
  myTLzPos[2][2] = mControlPara.RR3_TLImgnry_TakeOver_trans_z;					
					
	for(int i =0;i<3;i++)
	  for(int j =0;j<3;j++)
		  cout<<myTLzPos[i][j]<<endl;	
					
  // toCtrlTLIndex[x][y]: x is the number of ringroad, y is the traffic light type
  toCtrlTLIndex[0]=3;
  toCtrlTLIndex[1]=3;

  for(int i =0;i<3;i++){
	  for(int j =0;j<3;j++){
		if  (gpsPosition[2]<myTLzPos[i][j]){
			cout<<"i = "<< i<<"j = "<<j<<endl;
			myTrafficLight = driver->getFromDef(TL_Name[i][j]);
			if (myTrafficLight->getField("state")->getSFString() == "red"){
				toCtrlTLIndex[0]=i;
				toCtrlTLIndex[1]=j;
				goto outer;
			
			}
		}
	  }

  }
  outer: 
  cout << "toCtrlTLIndex  " << toCtrlTLIndex[0]<<toCtrlTLIndex[1] << "gps z"<<gpsPosition[2]<<endl;
  
  moutfile<<fixed << std::setprecision(4)<< mDriver->getTime()<<','<< gpsPosition[0]<<','<< gpsPosition[1]<<','<< gpsPosition[2]<<','<< gpsSpeed <<endl;

  

  
  
  
  
/*    
  cout << "send LACL"<< endl;
    double gpsSpeed=mGps->getSpeed();
    double gpsPostion[4]={mGps->getValues()[0],mGps->getValues()[1],mGps->getValues()[2],mGps->getValues()[3]};
    mLcMessage.cmd[0]='L';
    mLcMessage.cmd[1]='A';
    mLcMessage.cmd[2]='C';
    mLcMessage.cmd[3]='L';
    mLcMessage.speed = gpsSpeed;
    memcpy(mLcMessage.postion, gpsPostion, sizeof(double)*3);
    mEmitter->send(mptrMessage, sizeof(struct LCMessage));
*/

  if (!mJoystick->isConnected())
    return;

  string model = mJoystick->getModel();
  if (model.find("G29") != std::string::npos)
    configurationFile = "config_logitech_g29.ini";
  else if (model.find("G27") != std::string::npos)
    configurationFile = "config_logitech_g27.ini";
  else if (model.find("FANATEC CSL Elite Wheel Base") != std::string::npos)
    configurationFile = "config_fanatec_csl_elite_wheel_base.ini";
  else
    fatal("'" + model + "' not supported please provide a custom configuration file in argument.");

  // 2. look for platform specific configuration file:
  string platformConfigurationFile = configurationFile;
  platformConfigurationFile.erase(platformConfigurationFile.end() - 4, platformConfigurationFile.end());
  platformConfigurationFile += "." + platformExtension + ".ini";
  if (fileExists(platformConfigurationFile))
    configurationFile = platformConfigurationFile;





// initialize the messageId to be played frist.
ringNum = int((gpsPosition[2]-mControlPara.zoffsetRoad[0])/(mControlPara.zoffsetRoad[1]-mControlPara.zoffsetRoad[0]));

if ((gpsPosition[2]> mControlPara.zoffsetRoad[ringNum])&&(gpsPosition[2]< myTLzPos[ringNum][0])){
	
	nextMessageId = "Msg1";
	toPlayMessageId = nextMessageId;
}
if ((gpsPosition[2]>= = mControlPara.pos_diverge[ringNum]-DeltaZ_Msg7)&&(gpsPosition[2]< mControlPara.pos_diverge[ringNum])){
//if ((gpsPosition[2]>= myTLzPos[ringNum][2]- mControlPara.DeltaZ_Msg6)&&(gpsPosition[2]< myTLzPos[ringNum][2])){
	nextMessageId = "Msg7";
	toPlayMessageId = nextMessageId;	
}

  cout << nextMessageId<< endl;
  init(driver, configurationFile.c_str());
  

  
  
}



JoystickInterface::JoystickInterface(webots::Driver *driver, const char *configFile) {
  mJoystick = NULL;
  if (!fileExists(configFile))
    fatal("File '" + string(configFile) + "' does not exist.");
    
  init(driver, configFile);
}





bool JoystickInterface::step() {
  if (!mJoystick->isConnected())
    return false;

  // useful debug code: display joystick state
  /*
  cout << "axes:" << endl;
  for (int i = 0; i < mJoystick->getNumberOfAxes(); ++i)
    cout << "- axe " << i << " " << mJoystick->getAxisValue(i) << endl;
  cout << "povs:" << endl;
  for (int i = 0; i < mJoystick->getNumberOfPovs(); ++i)
    cout << "- pov " << i << " " << mJoystick->getPovValue(i) << endl;
  int b = mJoystick->getPressedButton();
  cout << "buttons:" << endl;
  while (b >= 0) {
    cout << b << " ";
    b = mJoystick->getPressedButton();
  }
  cout << endl << endl;
  */

  // update steering, throttle, and brake based on axes value

  // raw data
  int steeringFeedback = mJoystick->getAxisValue(mAxesMap["Steering"]);
  int throttleFeedback = mJoystick->getAxisValue(mAxesMap["Throttle"]);
  int brakeFeedback = mJoystick->getAxisValue(mAxesMap["Brake"]);

  // bounded scaled data [0, 1]
  double steeringAngle = convertFeedback(steeringFeedback, mAxesBoundsMap["minSteering"], mAxesBoundsMap["maxSteering"]);
  double throttle = convertFeedback(throttleFeedback, mAxesBoundsMap["minThrottle"], mAxesBoundsMap["maxThrottle"]);
  double brake = convertFeedback(brakeFeedback, mAxesBoundsMap["minBrake"], mAxesBoundsMap["maxBrake"]);
  // useful debug code: display the resulting scaled data before sending it to the driver library
  // cout << "steering:" << steeringAngle << " throttle:" << throttle << " brake:" << brake << endl;

  // to automobile API
  mDriver->setSteeringAngle(steeringAngle - 0.5);  // convert to [-0.5, 0.5] radian range
  mDriver->setThrottle(throttle);
  mDriver->setBrakeIntensity(brake);

  // update gear and indicator based on buttons state
  int button = mJoystick->getPressedButton();
  int gear = mGear;
  webots::Driver::WiperMode wiperMode = mWiperMode;
  static bool wasSwitchingToNextGear = false;
  static bool wasSwitchingToPreviousGear = false;
  bool isSwitchingToNextGear = false;
  bool isSwitchingToPreviousGear = false;
  static bool wasLeftBlinkerOn = false;
  static bool wasRightBlinkerOn = false;
  bool isLeftBlinkerOn = false;
  bool isRightBlinkerOn = false;
  static bool wasSwitchingToNextWiperMode = false;
  static bool wasSwitchingToPreviousWiperMode = false;
  bool isSwitchingToNextWiperMode = false;
  bool isSwitchingToPreviousWiperMode = false;
  bool isReqSwitchingToCAD = false;
  bool isSwitchToCAD = false;
  string playCmd;

  static bool isTurningLeftIconLoaded = false;
  static bool isTurningRightIconLoaded = false;
  static bool isStayOnIconLoaded = false;
  std::stringstream strDisp;
	
  static int mTimer = 0;
  static int msgTimer = 0;
  static int time_step = mDriver->getBasicTimeStep();
  static int Time_Threshold = mControlPara.switchWaitingTime/mDriver->getBasicTimeStep();
  //static bool isSwitchReminded = false;

  //obtain gps information and record 
  double gpsSpeed=mGps->getSpeed();
  double gpsPosition[3]={mGps->getValues()[0],mGps->getValues()[1],mGps->getValues()[2]};
  
  //decide which ring road number is based on the current location of participant. 
  //int ringNum = int((gpsPosition[2]-mControlPara.zoffsetRoad[0])/(mControlPara.zoffsetRoad[1]-mControlPara.zoffsetRoad[0]));
  //vector<vector<string>> messageId;
  
  //obtain gap2pred and speedDiff from radar
  /*
  int numVeh =mRadar->getNumberOfTargets();
  double gap2pred=100;
  double speedDiff=30;
    if (numVeh>0){
      const RadarTarget *preObj=mRadar->getTargets();
      for ( int i = 0; i < numVeh; i++){
        if ((preObj[i].distance<gap2pred)&&(abs(preObj[i].azimuth) < 0.0001)){
          gap2pred = preObj[i].distance;
          speedDiff=preObj[i].speed;
		}
	  }
	}
	*/
  // record current vehicle's state
  //print(round(driver.getTime(),4),',', round(myPosition, 4),',' , round(myCurSpeed,4),',',  round(myAccel,4),',', round(gap2pred,4),',', round(speedDiff,4), 
  moutfile<<fixed << std::setprecision(4)<< mDriver->getTime()<<','<< gpsPosition[0]<<','<< gpsPosition[1]<<','<< gpsPosition[2]<<','<< gpsSpeed <<endl;



  while (button >= 0) {
    if (button == mButtonsMap["SwitchMode"]){
        isReqSwitchingToCAD = true;
    }else if (button == mButtonsMap["NextGear"]) {
      if (!wasSwitchingToNextGear)
        gear += 1;
      isSwitchingToNextGear = true;
    } else if (button == mButtonsMap["PreviousGear"]) {
      if (!wasSwitchingToPreviousGear)
        gear -= 1;
      isSwitchingToPreviousGear = true;
    } else if (button == mButtonsMap["FirstGear"])
      gear = 1;
    else if (button == mButtonsMap["SecondGear"])
      gear = 2;
    else if (button == mButtonsMap["ThirdGear"])
      gear = 3;
    else if (button == mButtonsMap["FourthGear"])
      gear = 4;
    else if (button == mButtonsMap["FifthGear"])
      gear = 5;
    else if (button == mButtonsMap["SixthGear"])
      gear = 6;
    else if (button == mButtonsMap["ReverseGear"])
      gear = -1;
    else if (button == mButtonsMap["RightWarning"]) {
	  // the original judgement	
      //if (!wasRightBlinkerOn)  // not pressed previous step
        //mDriver->getIndicator() == Driver::INDICATOR_RIGHT ? mDriver->setIndicator(Driver::INDICATOR_OFF) :
                                                             //mDriver->setIndicator(Driver::INDICATOR_RIGHT);
      //if (mDriver->getIndicator() == Driver::INDICATOR_RIGHT){
		//mLcMessage.isfirst = 'F';
		//cout << "Participant intends to Turn Right " << endl;
	  //}															 
															 
      // new judgement, if second press of the indicator, then send the last message to CAVs.
      if (!wasRightBlinkerOn){  // not pressed previous step
        if (mDriver->getIndicator() == Driver::INDICATOR_RIGHT){
	      mDriver->setIndicator(Driver::INDICATOR_OFF);
		  mLcMessage.isfirst = 'L';
		  cout << "Participant has Turned Right " << endl;		  
		}else{
		  mDriver->setIndicator(Driver::INDICATOR_RIGHT);
		  mLcMessage.isfirst = 'F';
		  cout << "Participant intends to Turn Right " << endl;
		}
	  }
	  

	  isRightBlinkerOn = true;
    } else if (button == mButtonsMap["LeftWarning"]) {
      // the original judgement	
      //if (!wasLeftBlinkerOn)  // not pressed previous step
        //mDriver->getIndicator() == Driver::INDICATOR_LEFT ? mDriver->setIndicator(Driver::INDICATOR_OFF) :
                                                            //mDriver->setIndicator(Driver::INDICATOR_LEFT);
      //if (mDriver->getIndicator() == Driver::INDICATOR_LEFT){
		//mLcMessage.isfirst = 'F';
		//cout<< "Participant intends to Turn Right "<< endl;

	  //}

      // new judgement, if second press of the indicator, then send the last message to CAVs.
      if (!wasLeftBlinkerOn){  // not pressed previous step
        if (mDriver->getIndicator() == Driver::INDICATOR_LEFT){
	      mDriver->setIndicator(Driver::INDICATOR_OFF);
		  mLcMessage.isfirst = 'L';
		  cout << "Participant has Turned Left " << endl;		  
		}else{
		  mDriver->setIndicator(Driver::INDICATOR_LEFT);
		  mLcMessage.isfirst = 'F';
		  cout << "Participant intends to Turn Left " << endl;
		}
	  }




      isLeftBlinkerOn = true;
    } else if (button == mButtonsMap["NextWiperMode "] || wiperMode < Driver::FAST) {
    //cout << "NextWiperMode button number is: " << button<< endl;
      if (!wasSwitchingToNextWiperMode)
        wiperMode = webots::Driver::WiperMode(wiperMode + 1);
      isSwitchingToNextWiperMode = true;
    } else if (button == mButtonsMap["PreviousWiperMode "] || wiperMode > Driver::DOWN) {
    //cout << "PreviousWiperMode button number is: " << button << endl;
      if (!wasSwitchingToPreviousWiperMode)
        wiperMode = webots::Driver::WiperMode(wiperMode - 1);
      isSwitchingToPreviousWiperMode = true;
    } 
    
    cout << "button number is: " << button<< endl;
    button = mJoystick->getPressedButton();

  }
  

  
  
	// send the LACR/LACL message and gps via V2V communication
  if((mDriver->getIndicator() == Driver::INDICATOR_RIGHT) || (mDriver->getIndicator() == Driver::INDICATOR_LEFT) ||((mDriver->getIndicator() == Driver::INDICATOR_OFF )&& (mLcMessage.isfirst == 'L') ) ){
    mLcMessage.cmd[0]='L';
    mLcMessage.cmd[1]='C';
    if(mDriver->getIndicator() == Driver::INDICATOR_RIGHT){
      mLcMessage.cmd[2]='R';
	}
    else{
      mLcMessage.cmd[2]='L';
	}
	mLcMessage.speed = gpsSpeed;
    memcpy(mLcMessage.position, gpsPosition, sizeof(double)*3);
    mEmitter->send(mptrMessage, sizeof(struct LCMessage));
	mLcMessage.isfirst = 'N';
  }
  
  
  wasSwitchingToNextGear = isSwitchingToNextGear;
  wasSwitchingToPreviousGear = isSwitchingToPreviousGear;
  wasLeftBlinkerOn = isLeftBlinkerOn;
  wasRightBlinkerOn = isRightBlinkerOn;
  wasSwitchingToNextWiperMode = isSwitchingToNextWiperMode;
  wasSwitchingToPreviousWiperMode = isSwitchingToPreviousWiperMode;

  gear = std::max(-1, std::min(mDriver->getGearNumber(), gear));
  if (gear != mGear) {
    mGear = gear;
    cout << "gear: " << mGear << endl;
    mDriver->setGear(mGear);
  }

  if (wiperMode != mWiperMode) {
    mWiperMode = wiperMode;
    mDriver->setWiperMode(mWiperMode);
  }

  // update resistance and auto-centering gain based on speed
  static const double maxSpeed = 60.0;  // speed from which the max gain is applied
  double speed = mDriver->getCurrentSpeed();
  if (mGainMap["AutoCentering"] > 0.0)
    mJoystick->setAutoCenteringGain(speed > maxSpeed ? mGainMap["AutoCentering"] :
                                                       mGainMap["AutoCentering"] * speed / maxSpeed);
  if (mGainMap["Resistance"] > 0.0)
    mJoystick->setResistanceGain(speed > maxSpeed ? 0.0 : mGainMap["Resistance"] * (1.0 - speed / maxSpeed));



  // display on screen information
  // decide the initial picture to install
  
  if((!isTurningLeftIconLoaded)&&(((gpsPosition[2]>=mControlPara.RR1_TLonrmp_trans_z) && (gpsPosition[2]<mControlPara.RR1_TLonrmp_trans_z+50))|| \
     ((gpsPosition[2]>=mControlPara.RR2_TLonrmp_trans_z) && (gpsPosition[2]<mControlPara.RR2_TLonrmp_trans_z+50))|| \
	 ((gpsPosition[2]>=mControlPara.RR3_TLonrmp_trans_z) && (gpsPosition[2]<mControlPara.RR3_TLonrmp_trans_z+50)))){
	
       mImg = mDisplay->imageLoad(TurningLeftIcon_file);
	   isTurningLeftIconLoaded =true;
	   
	 }else if ((!isTurningRightIconLoaded)&&(((gpsPosition[2]>=mControlPara.pos_cadTohd[0]) && (gpsPosition[2]<mControlPara.pos_diverge_entry[0]))|| \
     ((gpsPosition[2]>=mControlPara.pos_cadTohd[1]) && (gpsPosition[2]<mControlPara.pos_diverge_entry[1]))|| \
     ((gpsPosition[2]>=mControlPara.pos_cadTohd[2]) && (gpsPosition[2]<mControlPara.pos_diverge_entry[2])))){
       mImg = mDisplay->imageLoad(TurningLeftIcon_file);
	   isTurningRightIconLoaded = true;
	 }
	 else {
	  if(!isStayOnIconLoaded){
       mImg = mDisplay->imageLoad(StayonIcon_file);
	   isStayOnIconLoaded = true;
	  }
	 }
	 
  if(mTimer%(mControlPara.infoDispTime/time_step)==0){
    mDisplay->imagePaste(mImg, 0, 0, false);
    strDisp << std::fixed << std::setprecision(1) << gpsSpeed*3.6;
    mDisplay->drawText(strDisp.str()+" Km/h", 150, 30);
  }
  
  
  
 // play message sound 
    messageId = "";
    if  (toPlayMessageId == "Msg1" && gpsPosition[2] >= myTLzPos[ringNum][0]-mControlPara.DeltaZ_Msg1){
        messageId = "Msg1";
        nextMessageId = "Msg2";
		cout<<nextMessageId<<endl;
	}
    if  (toPlayMessageId == "Msg2" && gpsPosition[2] >= mControlPara.pos_stop_and_go[ringNum]-mControlPara.DeltaZ_Msg2){
        messageId = "Msg2";
        nextMessageId = "Msg3";
		cout<<nextMessageId<<endl;
	}        
    if  (toPlayMessageId == "Msg3" && gpsPosition[2] >= mControlPara.pos_hdTocad[ringNum]-mControlPara.DeltaZ_Msg3){
        messageId = "Msg3";
        nextMessageId = "Msg4";
		cout<<nextMessageId<<endl;        
	}
     
    if  (toPlayMessageId == "Msg7" && gpsPosition[2] >= mControlPara.pos_cadTohd[ringNum] - mControlPara.DeltaZ_Msg7){
        messageId = "Msg7";
        nextMessageId = "Msg8";
		cout<<nextMessageId<<endl;        
	}        
    if  (toPlayMessageId == "Msg8" && gpsPosition[2] >= mControlPara.pos_diverge[ringNum] + mControlPara.DeltaZ_Msg8){
        messageId = "Msg8";
        nextMessageId = "Msg1";
		//next message should play in the next ringroad.
	    ringNum = min(ringNum+1,2);
		
		cout<<nextMessageId<<endl;        
	}
    if (messageId != ""){
      playCmd = "../soundPlay/"+ messageId +".wav";
      PlaySound(playCmd.c_str(), NULL, SND_ASYNC);  
//      playCmd = "open ../soundPlay/"+ messageId +".mp3 alias mysong";
//      mciSendString(playCmd.c_str(), NULL, 0, NULL);
//	  playCmd ="play mysong wait";
//      mciSendString(playCmd.c_str(), NULL, 0, NULL);
//	  playCmd ="close mysong";
//      mciSendString(playCmd.c_str(), NULL, 0, NULL);   	  
	}	

	  
    if (nextMessageId == "Msg4"){
      msgTimer++;
      if (isReqSwitchingToCAD || (msgTimer>=Time_Threshold)||(gpsPosition[2] >= mControlPara.pos_hdTocad[ringNum]))
        isSwitchToCAD = true;
	}
    // switch to automated driving if button is pressed or after waiting time
    if(isSwitchToCAD){
      //pvInfo.append(2)
     // print(pvInfo, file=fp)   
	  
      moutfile.close();
      mcontrollerName->setSFString("auto-ringroad-driver");             
    }
	else{
		
	        ;//pvInfo.append(0)## button id is 0
            //print(pvInfo, file=fp)    	
		
	}
	
    toPlayMessageId = nextMessageId;

  
  
  
  
  
  
  
  
  
  //update traffic light status
   if(((toCtrlTLIndex[1]==0)&&(gpsPosition[2]>=myTLzPos[toCtrlTLIndex[0]][0]-mControlPara.DeltaZ_TL_onramp))
	||((toCtrlTLIndex[1]==1)&&(gpsPosition[2]>=myTLzPos[toCtrlTLIndex[0]][1]-mControlPara.DeltaZ_TL_stopngo))
    ||((toCtrlTLIndex[1]==2)&&(gpsPosition[2]>=myTLzPos[toCtrlTLIndex[0]][2]-mControlPara.DeltaZ_TL_takeover))){
		

		myTrafficLight->getField("controller")->setSFString("traffic_light_green");
		myTrafficLight->getField("controllerArgs")->setSFString("0.1 2000 g");
		//move to the next traffic light
		int temp = toCtrlTLIndex[0]*3+toCtrlTLIndex[1]+1;
		toCtrlTLIndex[0] = temp/3;
		toCtrlTLIndex[1] = temp%3;
		myTrafficLight = mDriver->getFromDef(TL_Name[toCtrlTLIndex[0]][toCtrlTLIndex[1]]);
        cout << "toCtrlTLIndex  " << toCtrlTLIndex[0]<<toCtrlTLIndex[1]<<TL_Name[toCtrlTLIndex[0]][toCtrlTLIndex[1]] << myTrafficLight->getField("state")->getSFString()<< endl;
	
  } 
  
 
  
  mTimer++;
  
  return true;
}
