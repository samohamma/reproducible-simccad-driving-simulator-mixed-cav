# -*- coding: utf-8 -*-
from .constants import HEADER, TRAILER
from .geometry import onramp_local_start
from .road import emit_one_road
from .lights import emit_tlights
from .scenery import emit_trees_and_signs
from .ground import ground_from_network
from .vehicles import emit_vehicles
from .profiler import emit_performance_profiler_node

def make_roads(N,
               n_lanes=5,
               L_mid=700.0,
               L_ramp=1000.0,
               L_main=12300.0,
               stngo_frac=0.10569,
               tko_back_frac=0.22764,
               surr_counts_rr1=None,
               surr_first_gap=158.3,
               surr_spacing=30.0,
               stngo_lanes=(1,2,3,4),
               broken_lane=1):
    if surr_counts_rr1 is None:
        surr_counts_rr1 = {1:12,2:10,3:6,4:1,5:0}

    def _expand(val, N):
        if isinstance(val,(list,tuple)):
            if len(val)!=N: raise ValueError(f"Iterable length {len(val)} != N={N}")
            return list(val)
        return [val]*N

    cfgs=[]
    Lr_list = _expand(L_ramp,N)
    Lm_list = _expand(L_main,N)
    Ld_list = _expand(L_mid,N)
    sfrac   = _expand(stngo_frac,N)
    tfrac   = _expand(tko_back_frac,N)

    for i in range(N):
        cfgs.append({
            "prefix": f"RR{i+1}",
            "n_lanes": n_lanes,
            "L_mid":   float(Ld_list[i]),
            "L_ramp":  float(Lr_list[i]),
            "L_main":  float(Lm_list[i]),
            "stngo_frac": float(sfrac[i]),
            "tko_back_frac": float(tfrac[i]),
            "traffic": {
                "surr_counts_by_lane": surr_counts_rr1 if i==0 else {},
                "surr_first_gap": surr_first_gap, "surr_spacing": surr_spacing,
                "stngo_lanes": stngo_lanes, "broken_lane": broken_lane
            }
        })
    return cfgs

def build_world(orig_x=-3500.0, orig_z=-18000.0,
                road_cfgs=None,
                include_trees=True, include_signs=True,
                seed=42, pad_x=100.0, pad_z=100.0,
                include_performance_profiler=False,
                profiler_config=None):
    if not road_cfgs:
        raise ValueError("Provide road_cfgs")

    solids = []; tl_all = []; deco_all = []; vnodes_list = []; profiler_nodes = []
    Sx, Sz = orig_x, orig_z
    origins = [(Sx,Sz)]

    for i, cfg in enumerate(road_cfgs, start=1):
        prefix = cfg["prefix"]
        solid, (Tx, Tz) = emit_one_road(prefix, Sx, Sz,
                                        n_lanes=cfg["n_lanes"],
                                        L_mid=cfg["L_mid"],
                                        L_ramp=cfg["L_ramp"],
                                        L_main=cfg["L_main"],
                                        with_transition=(i < len(road_cfgs)))
        solids.append(solid)

        tl_nodes, _ = emit_tlights(prefix, Sx, Sz, L_main=cfg["L_main"],
                                   stngo_frac=cfg["stngo_frac"],
                                   takeover_back_frac=cfg["tko_back_frac"])
        tl_all.append(tl_nodes)

        if include_trees or include_signs:
            deco_all.append(emit_trees_and_signs(prefix,Sx,Sz,
                                                 n_lanes=cfg["n_lanes"],
                                                 L_main=cfg["L_main"],
                                                 L_ramp=cfg["L_ramp"]))

        if i < len(road_cfgs):
            n_next  = road_cfgs[i]["n_lanes"]
            Lr_next = road_cfgs[i]["L_ramp"]
            f_on, l_on = onramp_local_start(n_next, Lr_next)
            Sx = Tx + l_on
            Sz = Tz - f_on
            origins.append((Sx,Sz))

    ground = ground_from_network(orig_x, orig_z, Sx, Sz,
                                 first_cfg=road_cfgs[0],
                                 last_L_main=road_cfgs[-1]["L_main"],
                                 pad_x=pad_x, pad_z=pad_z)

    if include_performance_profiler:
        pcfg = profiler_config or {}
        profiler_nodes.append(emit_performance_profiler_node(**pcfg))

    cfg1 = road_cfgs[0]; traf = cfg1["traffic"]
    vnodes_list.append(
        emit_vehicles(cfg1["prefix"], origins[0][0], origins[0][1],
                      L_main=cfg1["L_main"], n_lanes=cfg1["n_lanes"], L_ramp=cfg1["L_ramp"],
                      surr_counts_by_lane=traf["surr_counts_by_lane"],
                      surr_first_gap=traf["surr_first_gap"], surr_spacing=traf["surr_spacing"],
                      stngo_lanes=traf["stngo_lanes"], broken_lane=traf["broken_lane"], seed=seed)
    )

    return (
        HEADER + "\n" + ground + "\n\n"
        + "\n\n".join(solids) + "\n\n"
        + "\n\n".join(tl_all) + "\n\n"
        + (("\n\n".join(deco_all) + "\n\n") if ((include_trees or include_signs) and deco_all) else "")
        + "\n\n".join(vnodes_list) + "\n\n"
        + (("\n\n".join(profiler_nodes) + "\n\n") if profiler_nodes else "")
        + TRAILER
    )


