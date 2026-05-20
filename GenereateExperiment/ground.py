# -*- coding: utf-8 -*-
from .constants import *
from .geometry import onramp_local_start, world_from_local, f_exit_anchor

def ground_from_network(orig_x, orig_z, Sx_last, Sz_last, first_cfg, last_L_main, pad_x=100.0, pad_z=100.0):
    f_on1, l_on1 = onramp_local_start(first_cfg["n_lanes"], first_cfg["L_ramp"])
    x_on1, _ = world_from_local(orig_x, orig_z, f_on1, l_on1)
    x_last_center = Sx_last
    x_min, x_max = sorted((x_on1, x_last_center))
    size_x = (x_max - x_min) + 2*pad_x
    cx = 0.5*(x_min + x_max)

    f_Exit0 = f_exit_anchor(last_L_main)
    f_end_total = f_Exit0 + L_EXIT
    z_min, z_max = sorted((orig_z, Sz_last + f_end_total))
    size_z = (z_max - z_min) + 2*pad_z
    cz = 0.5*(z_min + z_max)

    return f"""DEF GROUND Solid {{
  translation {cx:.4f} 1 {cz:.4f}
  boundingObject DEF GROUND_PLANE Plane {{
    size {size_x:.0f} {size_z:.0f}
  }}
  locked TRUE
  translationStep 1
}}
DEF GROUND_SHAPE Solid {{
  translation {cx:.4f} -0.02 {cz:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 0.8 0.8
        baseColorMap ImageTexture {{ url [ "textures/ground_grass.jpg" ] }}
        roughness 0.5
        metalness 0
        textureTransform TextureTransform {{ scale 500 500 }}
      }}
      geometry DEF GROUND_PLANE Plane {{ size {size_x:.0f} {size_z:.0f} }}
      castShadows FALSE
      isPickable FALSE
    }}
  ]
  name "Ground"
  boundingObject USE GROUND_PLANE
  locked TRUE
}}
Viewpoint {{
  orientation 0.0015718133004660956 0.9991629344422414 0.04087737564616572 2.947055053885618
  position -3166.5994807068887 2.4267137017217006 -17703.63038887528
  near 1
  follow "veh-driver"
  followOrientation TRUE
  followSmoothness 0
}}
"""
