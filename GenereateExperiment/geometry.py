# -*- coding: utf-8 -*-
import math
from .constants import *

def _rot(a): return f"rotation 0 1 0 {a:.4f}"
def _v(fwd,y,lat): return f"translation {fwd:.4f} {y:.0f} {lat:.4f}"
def _indent(s, n=4): return "\n".join((" " * n) + line if line else line for line in s.splitlines())

def world_from_local(Sx, Sz, fwd, lat):  # Solid yaw = -pi/2
    return Sx - lat, Sz + fwd

def lane_x_n(Sx, n_lanes, lane):  # lane 1 = leftmost
    return Sx + (lane - (n_lanes+1)/2.0)*LANE_W

def _phys_lane(lane, n_lanes):  # mirror logical index to geometry index
    return n_lanes - lane + 1

def onramp_local_start(n_lanes, L_ramp):
    f = L_START - (R*math.sin(DELTA) + L_ramp*math.cos(DELTA))
    l = -((n_lanes*LANE_W)/2 + CLEAR + L_ramp*math.sin(DELTA) + R*(1-math.cos(DELTA)))
    return f, l

def f_main_head():  # start of mainline after merge
    return L_START + L_LS + L_AL

def f_exit_anchor(L_main):  # start of exit straight
    return f_main_head() + L_main + L_DIV_AL + L_DIV_LS
