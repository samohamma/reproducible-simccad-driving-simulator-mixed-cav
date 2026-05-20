# -*- coding: utf-8 -*-
import re
from .constants import *
from .geometry import f_main_head, f_exit_anchor

def _legacy_tl_suffix(prefix):
    """Return the zero-based traffic-light suffix used by racing_wheel_com.

    The generated road prefixes are RR1, RR2, RR3, ... while the compiled
    racing_wheel_com controller expects traffic-light DEF names:
        TL_onrmp0, TL_img_StnGo0, TL_img_takeOver0, ...
        TL_onrmp1, TL_img_StnGo1, TL_img_takeOver1, ...
    Therefore RR1 -> 0, RR2 -> 1, etc.
    """
    match = re.search(r"(\d+)$", str(prefix))
    if match:
        return str(int(match.group(1)) - 1)
    return str(prefix)

def emit_tlights(prefix, Sx, Sz, *, L_main,
                 onrmp_dfwd=None, onrmp_dlat=+33.0, onrmp_yaw=-DELTA,
                 stngo_frac=STNGO_FRAC_DEFAULT,
                 takeover_back_frac=TAKEOVER_BACK_FRAC_DEFAULT,
                 off_after_exit=OFF_AFTER_EXIT, off_dlat=-10.0, off_yaw=-DELTA):
    if onrmp_dfwd is None:
        onrmp_dfwd = L_START - 110.0

    stngo_dfwd    = f_main_head() + stngo_frac * L_main
    takeover_dfwd = f_main_head() + (1.0 - takeover_back_frac) * L_main
    off_dfwd      = f_exit_anchor(L_main) + L_EXIT + off_after_exit

    x_on , z_on  = Sx + onrmp_dlat, Sz + onrmp_dfwd
    x_stn, z_stn = Sx,               Sz + stngo_dfwd
    x_tko, z_tko = Sx,               Sz + takeover_dfwd
    x_off, z_off = Sx + off_dlat,    Sz + off_dfwd

    suffix = _legacy_tl_suffix(prefix)
    tl_onrmp = f"TL_onrmp{suffix}"
    tl_stngo = f"TL_img_StnGo{suffix}"
    tl_takeover = f"TL_img_takeOver{suffix}"
    tl_offrmp = f"TL_offrmp{suffix}"

    nodes = []
    nodes.append(f"""DEF {tl_onrmp} GenericTrafficLight {{
  translation {x_on:.4f} 0.30 {z_on:.4f}
  rotation 0 1 0 {onrmp_yaw:.4f}
  name "{tl_onrmp}"
  state "red"
}}""")
    nodes.append(f"""DEF {tl_stngo} GenericTrafficLight {{
  translation {x_stn:.4f} -5 {z_stn:.4f}
  name "{tl_stngo}"
  state "red"
}}""")
    nodes.append(f"""DEF {tl_takeover} GenericTrafficLight {{
  translation {x_tko:.4f} -5 {z_tko:.4f}
  name "{tl_takeover}"
  state "red"
}}""")
    nodes.append(f"""DEF {tl_offrmp} GenericTrafficLight {{
  translation {x_off:.4f} 0.50 {z_off:.4f}
  rotation 0 1 0 {off_yaw:.4f}
  name "{tl_offrmp}"
  state "red"
}}""")
    return "\n".join(nodes), dict(onrmp=(x_on,z_on), stngo=(x_stn,z_stn), takeover=(x_tko,z_tko), offrmp=(x_off,z_off))
