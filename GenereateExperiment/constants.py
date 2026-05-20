# -*- coding: utf-8 -*-
import math
PI = math.pi

# geometry
LANE_W=3.6; RAMP_W=7.2
L_START=1400.0; R=500.0; DELTA=0.3
L_LS=150.0; L_AL=350.0; L_DIV_AL=300.0; L_DIV_LS=300.0; L_EXIT=1000.0
CLEAR=6.8; BORDER_W=1.6
BETA = -PI/2

# TL fractions (defaults match 12,300m baseline)
STNGO_FRAC_DEFAULT         = 1300.0/12300.0
TAKEOVER_BACK_FRAC_DEFAULT = 2800.0/12300.0
OFF_AFTER_EXIT = 250.0

# world scaffolding
HEADER = """#VRML_SIM R2019b utf8
WorldInfo {
  info [
    "Autonomous Vehicle Simulation"
    "The vehicle based on the Car PROTO is modelled with realistic physics properties."
  ]
  title "City"
  ERP 0.6
  basicTimeStep 50
  lineScale 1
  contactProperties [
    ContactProperties { coulombFriction [ 8 ] softCFM 1e-05 bumpSound "" rollSound "" slideSound "" }
    ContactProperties { material2 "CitroenCZeroWheels" coulombFriction [ 8 ] softCFM 1e-05 bumpSound "" rollSound "" slideSound "" }
    ContactProperties { material2 "ToyotaPriusWheels"  coulombFriction [ 8 ] softCFM 1e-05 bumpSound "" rollSound "" slideSound "" }
    ContactProperties { material2 "LincolnMKZWheels"   coulombFriction [ 8 ] softCFM 1e-05 bumpSound "" rollSound "" slideSound "" }
    ContactProperties { material2 "RangeRoverSportSVRWheels" coulombFriction [ 8 ] softCFM 1e-05 bumpSound "" rollSound "" slideSound "" }
    ContactProperties { material2 "TruckWheels"        coulombFriction [ 8 ] softCFM 1e-05 bumpSound "" rollSound "" slideSound "" }
  ]
}
TexturedBackground { texture "noon_building_overcast" }
TexturedBackgroundLight { texture "noon_building_overcast" castShadows FALSE }
"""

TRAILER = """HighwayPole {
  translation -3785.32 0.2 12245.9
  rotation 0 1 0 1.5708
  name "Experiment_Ends"
  stand 2
  height 5
  thickness 0.35
  curveRadius 0.5
  rightHorizontalSigns [ HighwaySign { translation 0 0 1.55 height 2.5 length 8 texture [ "textures/Finish.jpg" ] } ]
  rightVerticalSigns []
}
"""
