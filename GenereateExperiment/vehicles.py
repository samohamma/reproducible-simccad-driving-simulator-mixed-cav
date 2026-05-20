# -*- coding: utf-8 -*-
import random
from .geometry import lane_x_n, onramp_local_start, world_from_local
from .constants import *
from .lights import emit_tlights

def _rand_color(rng): return f"{rng.random():.5f} {rng.random():.5f} {rng.random():.5f}"


def _controller_lane_to_geometry_lane(lane, n_lanes):
    """Map controller lane IDs to Webots geometry lane IDs.

    The original Round 01 experiment and Controller_input_param.csv use
    surrounding-lane IDs 1..4 with x positions:
        Lane1=-3503.6, Lane2=-3500.0, Lane3=-3496.4, Lane4=-3492.8
    for the 5-lane baseline. In the Webots road geometry these correspond
    to geometry lanes 2..5, because geometry lane 1 is the outer/unused lane.

    A previous generator version mirrored the lane index with _phys_lane(),
    placing vehicles named Lane1 in the physical Lane4 position. That made the
    generated benchmark worlds non-comparable with the Round 01 reference and
    could force controllers to correct inconsistent lane positions.
    """
    lane = int(lane)
    return max(1, min(int(n_lanes), lane + 1))


def _bmw(type_name,name,x,z,color_rgb):
    ctrl = {"surr":"auto_surrounding_merge","sg":"auto_stop_and_go","broken":"auto_broken"}[type_name]
    return f'''BmwX5 {{
  translation {x:.4f} 1.4 {z:.4f}
  color {color_rgb}
  name "{name}"
  controller "{ctrl}"
  supervisor TRUE
  sensorsSlotFront [ Radar {{ minRange 2 maxRange 300 horizontalFieldOfView 0.5 verticalFieldOfView 0.002 }} ]
  sensorsSlotTop [ GPS {{}} Receiver {{}} Emitter {{}} ]
  interior FALSE
  dynamicSpeedDisplay FALSE
  indicatorLevers FALSE
  completeInterior FALSE
}}'''

def _ego_participant(name, Sx, Sz, n_lanes, L_ramp):
    f, l = onramp_local_start(n_lanes, L_ramp)
    x, z = world_from_local(Sx, Sz, f, l)

    return f'''DEF EGO_PARTICIPANT BmwX5Au {{
  translation {x:.4f} 1.38 {z:.4f}
  rotation 0 1 0 {-DELTA:.4f}
  color 0.1 0.11 0.1
  name "{name}"
  controller "auto_ringroad_driver"
  supervisor TRUE
  sensorsSlotFront [
    Mirror {{
      translation 1.01417 0.531 1.72733
      rotation 0.7006521232906121 0.6883611211278197 0.18773803303541975 0.3
      supervisor TRUE
      fieldOfView 1.57
      width 0.15
      height 0.1
      frameThickness 0.00025
    }}
    Mirror {{
      translation -1.01177 0.525008 1.73931
      rotation -0.16395703340977408 0.978412199372542 -0.12580802563609275 -0.4
      supervisor TRUE
      fieldOfView 1.57
      width 0.2
      height 0.1
      frameThickness 0.00025
    }}
    Radar {{
      minRange 2
      maxRange 80
      horizontalFieldOfView 0.05
      verticalFieldOfView 0.002
    }}
  ]
  sensorsSlotTop [
    GPS {{ }}
    Receiver {{ }}
    Emitter {{ }}
  ]
  sensorsSlotCenter [
    Display {{
      width 387
      height 228
    }}
  ]
  completeInterior FALSE
  innerWindowTransparency 1
}}'''

def emit_vehicles(prefix,Sx,Sz,*,L_main,n_lanes,L_ramp,
                  surr_counts_by_lane, surr_first_gap, surr_spacing,
                  stngo_row_spacing=30.0, stngo_lanes=(1,2,3,4),
                  broken_lane=1, broken_offset=30.0, seed=None,
                  include_participant=True):
    rng = random.Random(seed)
    _, tlpos = emit_tlights(prefix,Sx,Sz,L_main=L_main)
    x_on,z_on   = tlpos['onrmp']
    _,   z_stn  = tlpos['stngo']
    _,   z_tko  = tlpos['takeover']

    nodes = []
    if include_participant:
        nodes.append(_ego_participant("veh-driver",Sx,Sz,n_lanes,L_ramp))

    # Surrounding flow: lanes 1..(n_lanes-1) (keep rightmost empty)
    for lane in range(1, n_lanes):
        count = int(surr_counts_by_lane.get(lane, 0))
        if count <= 0: 
            continue
        phys = _controller_lane_to_geometry_lane(lane, n_lanes)
        x_lane = lane_x_n(Sx, n_lanes, phys)
        for r in range(1, count+1):
            z = z_on - (surr_first_gap + (r-1)*surr_spacing)
            nodes.append(_bmw("surr", f"CAV_Surr_row{r}_Lane{lane}", x_lane, z, _rand_color(rng)))

    # Stop-and-go (avoid rightmost lane)
    for lane in [l for l in stngo_lanes if 1 <= l <= (n_lanes-1)]:
        phys = _controller_lane_to_geometry_lane(lane, n_lanes)
        x_lane = lane_x_n(Sx, n_lanes, phys)
        nodes.append(_bmw("sg", f"CAV_StopnGo_row2_Lane{lane}", x_lane, z_stn,                     _rand_color(rng)))
        nodes.append(_bmw("sg", f"CAV_StopnGo_row1_Lane{lane}", x_lane, z_stn + stngo_row_spacing, _rand_color(rng)))

    # Broken vehicle (avoid rightmost)
    bl = max(1, min(broken_lane, n_lanes-1))
    phys_bl = _controller_lane_to_geometry_lane(bl, n_lanes)
    x_b = lane_x_n(Sx, n_lanes, phys_bl); z_b = z_tko + broken_offset
    nodes.append(_bmw("broken", f"Broken_Surr_row0_Lane{bl}", x_b, z_b, _rand_color(rng)))

    return "\n".join(nodes)
