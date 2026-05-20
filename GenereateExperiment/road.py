# -*- coding: utf-8 -*-
import math
from .constants import *
from .geometry import _rot, _v, _indent, world_from_local, onramp_local_start

def emit_one_road(prefix, Sx, Sz, *, n_lanes, L_mid, L_ramp, L_main, with_transition=True):
    a_main = +PI/2; a_merge=-PI/2; a_onramp=+PI/2-DELTA
    offset = (n_lanes*LANE_W)/2 + CLEAR
    lat_curve_left = -(R + offset)

    f_LS = L_START + L_LS
    f_AL = f_LS + L_AL
    f_MAIN = f_AL

    f_on, l_on = onramp_local_start(n_lanes, L_ramp)
    main_w = n_lanes*LANE_W
    merge_sep_w = (n_lanes+2)*LANE_W

    parts = []

    parts.append(f"""DEF {prefix}_CAVsStartExperiment StraightRoadSegment {{
  {_rot(a_main)}
  name "{prefix}_Sec1_StartCAVs"
  width {main_w:.1f}
  numberOfLanes {n_lanes}
  roadBorderHeight 0
  length {L_START:.0f}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_DriveInOnRamp StraightRoadSegment {{
  {_v(f_on,0,l_on)}
  {_rot(a_onramp)}
  name "{prefix}_Onramp_Str"
  width {RAMP_W:.1f}
  lines [ RoadLine {{ type "none" }} ]
  roadBorderHeight 0
  length {L_ramp:.0f}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_DriveInOnRamp CurvedRoadSegment {{
  {_v(L_START,0,lat_curve_left)}
  {_rot(-PI/2)}
  name "{prefix}_Onramp_Crv"
  width {RAMP_W:.1f}
  roadBorderHeight 0
  roadBorderWidth [ {BORDER_W:.1f} ]
  curvatureRadius {R:.0f}
  totalAngle {DELTA}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_MergeInHighway LaneSeparation {{
  {_v(f_LS,0,-9.0)}
  {_rot(a_merge)}
  name "{prefix}_Merge_LS"
  width {merge_sep_w:.1f}
  length {L_LS:.0f}
  numberOfLanes {n_lanes + 2}
  newLaneLeft FALSE
  roadBorderHeight 0
  roadBorderWidth {BORDER_W:.1f}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_MergeInHighway AddLanesRoadSegment {{
  {_v(f_AL,0,0.0)}
  {_rot(a_merge)}
  name "{prefix}_Merge_AL"
  width {main_w:.1f}
  length {L_AL:.0f}
  numberOfLanes {n_lanes}
  newLaneLeft FALSE
  roadBorderHeight 0
  roadBorderWidth {BORDER_W:.1f}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_Participant_OnFreeway StraightRoadSegment {{
  {_v(f_MAIN,0,0.0)}
  {_rot(a_main)}
  name "{prefix}_Main"
  width {main_w:.1f}
  numberOfLanes {n_lanes}
  lines [ RoadLine {{ type "double" }} ]
  roadBorderHeight 0
  length {L_main:.0f}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    f_DivAL = f_MAIN + L_main
    f_DivLS = f_DivAL + L_DIV_AL
    f_Exit  = f_DivLS + L_DIV_LS

    parts.append(f"""DEF {prefix}_DivergeFromHighway AddLanesRoadSegment {{
  {_v(f_DivAL,0,0.0)}
  {_rot(a_main)}
  name "{prefix}_Div_AL"
  width {main_w:.1f}
  length {L_DIV_AL:.0f}
  numberOfLanes {n_lanes}
  roadBorderHeight 0
  roadBorderWidth {BORDER_W:.1f}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_DivergeFromHighway LaneSeparation {{
  {_v(f_DivLS,0,-9.0)}
  {_rot(a_main)}
  name "{prefix}_Div_LS"
  width {merge_sep_w:.1f}
  length {L_DIV_LS:.0f}
  numberOfLanes {n_lanes + 2}
  roadBorderHeight 0
  roadBorderWidth {BORDER_W:.1f}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    parts.append(f"""DEF {prefix}_CAVsExitFreeway StraightRoadSegment {{
  {_v(f_Exit,0,0.0)}
  {_rot(a_main)}
  name "{prefix}_Exit"
  width {main_w:.1f}
  numberOfLanes {n_lanes}
  roadBorderHeight 0
  length {L_EXIT:.0f}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    if with_transition:
        parts.append(f"""DEF {prefix}_OffRamp_Sec1 CurvedRoadSegment {{
  {_v(f_Exit,0,lat_curve_left)}
  {_rot(-PI/2 + DELTA)}
  name "{prefix}_Trans_Sec1"
  width {RAMP_W:.1f}
  lines [ RoadLine {{ type "none" }} ]
  roadBorderHeight 0
  roadBorderWidth [ {BORDER_W:.1f} ]
  curvatureRadius {R:.0f}
  totalAngle {DELTA}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

        c=math.cos(DELTA); s=math.sin(DELTA)
        offset = (n_lanes*LANE_W)/2 + CLEAR
        lat_right_center = R - (offset + 2*R*(1 - c))
        lat_parallel     = -(offset + 2*R*(1 - c))
        fwd_right_center = f_Exit + 2*R*s
        fwd_right_center_2 = fwd_right_center + L_mid

        parts.append(f"""DEF {prefix}_OffRamp_Sec2 CurvedRoadSegment {{
  {_v(fwd_right_center,0,lat_right_center)}
  {_rot(+PI/2 + DELTA)}
  name "{prefix}_Trans_Sec2"
  width {RAMP_W:.1f}
  lines [ RoadLine {{ type "none" }} ]
  roadBorderHeight 0
  roadBorderWidth [ {BORDER_W:.1f} ]
  curvatureRadius {R:.0f}
  totalAngle {DELTA}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

        parts.append(f"""DEF {prefix}_OffRamp_Sec3 StraightRoadSegment {{
  {_v(fwd_right_center,0,lat_parallel)}
  {_rot(+PI/2)}
  name "{prefix}_Trans_Sec3"
  width {RAMP_W:.1f}
  lines [ RoadLine {{ type "none" }} ]
  roadBorderHeight 0
  length {L_mid:.0f}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

        parts.append(f"""DEF {prefix}_OffRamp_Sec4 CurvedRoadSegment {{
  {_v(fwd_right_center_2,0,lat_right_center)}
  {_rot(+PI/2)}
  name "{prefix}_Trans_Sec4"
  width {RAMP_W:.1f}
  lines [ RoadLine {{ type "none" }} ]
  roadBorderHeight 0
  roadBorderWidth [ {BORDER_W:.1f} ]
  curvatureRadius {R:.0f}
  totalAngle {DELTA}
  endLine [ "" ]
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

        f_end = fwd_right_center_2 + R*math.sin(DELTA)
        l_end = lat_right_center  - R*math.cos(DELTA)
        Tx, Tz = world_from_local(Sx, Sz, f_end, l_end)
    else:
        Tx, Tz = world_from_local(Sx, Sz, f_Exit, 0.0)

    children = "\n\n".join(parts)
    solid = f"""DEF {prefix} Solid {{
  translation {Sx:.4f} 1 {Sz:.4f}
  {_rot(BETA)}
  children [
{_indent(children, 4)}
  ]
  name "{prefix}"
}}"""
    return solid, (Tx, Tz)
