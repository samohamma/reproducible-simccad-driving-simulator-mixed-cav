# -*- coding: utf-8 -*-
from .constants import *
from .geometry import f_main_head

def emit_edge_trees(prefix,Sx,Sz,*,n_lanes, start_len=None, spacing=8.0,
                    alt_step=1.7, tree_margin=None):
    if start_len is None: start_len = 0.9*L_START
    if tree_margin is None: tree_margin = max(0.0, CLEAR - 3.05)
    half = (n_lanes*LANE_W)/2.0
    off_base = half + BORDER_W + tree_margin
    xs = (Sx + off_base, Sx + off_base + alt_step)
    N = int(max(0.0,start_len)//spacing)+1
    lines=[]
    idx=1
    for i in range(N):
        z = Sz + i*spacing
        x = xs[i%2]
        lines.append(f'''SimpleTree {{
  translation {x:.2f} 1 {z:.2f}
  name "{prefix}_TreesStart_{idx}"
}}'''); idx+=1
    return "\n".join(lines)

def emit_speed_limit_series(prefix,Sx,Sz,*,L_main,n_lanes,
                            side_clear=0.0, step=300.0,
                            start_frac=-0.25, end_frac=1.05,
                            image="textures/signs/us/speed_limit_100.jpg"):
    half = (n_lanes*LANE_W)/2.0
    left_x  = Sx - (half + BORDER_W + side_clear)
    right_x = Sx + (half + BORDER_W + side_clear)

    f0 = f_main_head()
    z0 = Sz + max(0.0, f0 + start_frac*L_main)
    z1 = Sz + (f0 + end_frac*L_main)
    if z1 < z0: z0, z1 = z1, z0

    lines=[]; k=0; z=z0
    while z <= z1 + 1e-6:
        for x,side in ((right_x,"R"),(left_x,"L")):
            lines.append(f'''SpeedLimitSign {{
  translation {x:.2f} 1 {z:.2f}
  name "{prefix}_Speed_{k}_{side}"
  signBoards [ SpeedLimitPanel {{ translation 0 0 -0.023 signImage [ "{image}" ] }} ]
}}''')
        k+=1; z += step
    return "\n".join(lines)

def emit_curve_cautions(prefix,Sx,Sz,*,L_ramp, n_lanes, yaw=-DELTA, count=6, spacing=22.0, side_margin=2.0):
    on_dfwd = L_START - 110.0
    start_dfwd = max(0.0, on_dfwd - (count-1)*spacing)
    half = (n_lanes*LANE_W)/2.0
    x_right = Sx + (half + BORDER_W + side_margin)
    lines=[]
    for i in range(count):
        fwd = start_dfwd + i*spacing
        z = Sz + fwd
        lines.append(f'''CautionSign {{
  translation {x_right:.2f} 1 {z:.2f}
  rotation 0 1 0 {yaw:.4f}
  name "{prefix}_caution_curve_{i+1}"
  signBoards [ CautionPanel {{ translation 0 -0.17 0 signImage [ "textures/signs/us/left_curve.jpg" ] }} ]
}}''')
    return "\n".join(lines)

def emit_trees_and_signs(prefix,Sx,Sz,*,n_lanes,L_main,L_ramp,
                         stngo_frac=STNGO_FRAC_DEFAULT, tko_back_frac=TAKEOVER_BACK_FRAC_DEFAULT,
                         tree_margin=None, side_clear_signs=0.0):
    trees = emit_edge_trees(prefix,Sx,Sz,n_lanes=n_lanes, tree_margin=tree_margin)
    signs = emit_speed_limit_series(prefix,Sx,Sz,L_main=L_main,n_lanes=n_lanes, side_clear=side_clear_signs)
    cauts = emit_curve_cautions(prefix,Sx,Sz,L_ramp=L_ramp,n_lanes=n_lanes)
    return "\n\n".join([trees, signs, cauts])
