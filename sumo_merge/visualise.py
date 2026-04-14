#!/usr/bin/env python3
"""
Visualise the 16 merge-scenario networks and one animated traffic run.

Topology: 3-lane main road → ramp joins as 4th lane → 4-lane accel section
          → lane drop back to 3 lanes (taper).

Outputs (written to sumo_merge/results/):
  network_grid.png   – 4×4 panel showing every network, zoomed to the
                       accel / lane-drop zone
  traffic_anim.gif   – animated traffic flow for one chosen scenario
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import numpy as np
import sys
import sumolib

sys.path.insert(0, str(Path(__file__).parent))
from generate_network import ROAD_LENGTH, RAMP_JOIN_X

RESULTS           = Path(__file__).parent / "results"
MERGE_DISTANCES   = [100, 200, 300, 400]
CURVATURE_OFFSETS = [10,  20,  30,  40]

# ── colour palette ────────────────────────────────────────────────────────────
C_MAIN   = "#2c7bb6"
C_ACCEL  = "#74add1"   # 4-lane acceleration section
C_RAMP   = "#d7191c"
C_TAPER  = "#1a9641"   # taper / lane-drop node
C_MAIN_V  = "#4488cc"
C_MERGE_V = "#ee5533"


# ═══════════════════════════════════════════════════════════════════════════════
# 1.  NETWORK GRID
# ═══════════════════════════════════════════════════════════════════════════════

def _draw_net(ax, net, taper_x, title):
    """Draw one network, zoomed to the full accel section + some context."""
    for edge in net.getEdges():
        shape = edge.getShape()
        xs = [p[0] for p in shape]
        ys = [p[1] for p in shape]
        eid = edge.getID()
        if eid == "ramp":
            color, lw, zorder = C_RAMP,  2.0, 4
        elif eid == "main_accel":
            color, lw, zorder = C_ACCEL, 3.5, 2
        elif eid in ("main_before", "main_after"):
            color, lw, zorder = C_MAIN,  3.0, 1
        else:
            color, lw, zorder = "#aaaaaa", 1.0, 0
        ax.plot(xs, ys, color=color, lw=lw, zorder=zorder, solid_capstyle="round")

    # taper (lane-drop) marker
    ax.scatter([taper_x], [0], color=C_TAPER, s=55, zorder=6)
    # ramp join marker
    ax.scatter([RAMP_JOIN_X], [0], color="#ff7f00", s=35, zorder=6)

    ax.set_title(title, fontsize=7, pad=2)
    ax.set_aspect("equal")
    ax.axis("off")

    # Zoom: from 100 m before ramp join to 100 m after taper
    ax.set_xlim(RAMP_JOIN_X - 100, taper_x + 100)
    ax.set_ylim(-55, 25)


def build_network_grid():
    fig, axes = plt.subplots(4, 4, figsize=(15, 10),
                             gridspec_kw={"wspace": 0.06, "hspace": 0.28})

    for row, md in enumerate(MERGE_DISTANCES):
        for col, co in enumerate(CURVATURE_OFFSETS):
            sim_id   = f"md{md:03d}_co{co:02d}"
            net_path = RESULTS / sim_id / "merge.net.xml"
            ax       = axes[row][col]
            net      = sumolib.net.readNet(str(net_path), withInternal=False)
            taper_x  = RAMP_JOIN_X + md
            _draw_net(ax, net, taper_x, f"merge={md} m  curve={co} m")

    legend_handles = [
        Line2D([0],[0], color=C_MAIN,  lw=2.5, label="3-lane main road"),
        Line2D([0],[0], color=C_ACCEL, lw=2.5, label="4-lane accel section"),
        Line2D([0],[0], color=C_RAMP,  lw=2,   label="on-ramp"),
        Line2D([0],[0], marker="o", color="w", markerfacecolor="#ff7f00",
               markersize=7, label="ramp joins (merge_begin)"),
        Line2D([0],[0], marker="o", color="w", markerfacecolor=C_TAPER,
               markersize=7, label="lane drop (taper)"),
    ]
    fig.legend(handles=legend_handles, loc="lower center", ncol=5,
               fontsize=7.5, frameon=True, bbox_to_anchor=(0.5, 0.005))

    fig.suptitle(
        "SUMO 3→4→3 merge: all 16 networks  "
        "(rows = merge_distance 100–400 m,  cols = curvature_offset 10–40 m)",
        fontsize=10, y=0.99, fontweight="bold")

    out = RESULTS / "network_grid.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved → {out}")
    return out


# ═══════════════════════════════════════════════════════════════════════════════
# 2.  TRAFFIC ANIMATION
# ═══════════════════════════════════════════════════════════════════════════════

FCD_SIM_ID      = "md200_co20"
FCD_XML         = RESULTS / FCD_SIM_ID / "fcd.xml"
ANIM_DURATION_S = 120
EVERY_N_STEPS   = 5        # sub-sample: 1 frame per 0.5 sim-s
ANIM_FPS        = 20


def _run_fcd_sim():
    sim_dir = RESULTS / FCD_SIM_ID
    cfg_src = sim_dir / "merge.sumocfg"
    cfg_fcd = sim_dir / "merge_fcd.sumocfg"
    tree = ET.parse(str(cfg_src))
    root = tree.getroot()
    out_el = root.find("output")
    if out_el is None:
        out_el = ET.SubElement(root, "output")
    fcd_el = out_el.find("fcd-output")
    if fcd_el is None:
        fcd_el = ET.SubElement(out_el, "fcd-output")
    fcd_el.set("value", "fcd.xml")
    time_el = root.find("time")
    if time_el is not None:
        end_el = time_el.find("end")
        if end_el is None:
            end_el = ET.SubElement(time_el, "end")
        end_el.set("value", str(ANIM_DURATION_S))
    ET.indent(tree, space="  ")
    tree.write(str(cfg_fcd), encoding="unicode", xml_declaration=True)
    subprocess.run(
        ["sumo", "-c", str(cfg_fcd), "--no-step-log",
         "--no-warnings", "--duration-log.disable"],
        cwd=str(sim_dir), check=True, capture_output=True,
    )
    print(f"FCD written → {FCD_XML}")


def _parse_fcd(fcd_path):
    frames = {}
    for ts in ET.parse(str(fcd_path)).getroot().findall("timestep"):
        t = float(ts.get("time"))
        frames[t] = [
            (float(v.get("x")), float(v.get("y")),
             float(v.get("speed")), v.get("lane", ""))
            for v in ts.findall("vehicle")
        ]
    return frames


def build_traffic_animation():
    if not FCD_XML.exists():
        _run_fcd_sim()
    else:
        print(f"FCD already exists: {FCD_XML}")

    md = 200
    sim_dir  = RESULTS / FCD_SIM_ID
    net_path = sim_dir / "merge.net.xml"
    net      = sumolib.net.readNet(str(net_path), withInternal=False)
    taper_x  = RAMP_JOIN_X + md

    frames = _parse_fcd(FCD_XML)
    times  = sorted(frames.keys())[::EVERY_N_STEPS]

    fig, ax = plt.subplots(figsize=(14, 4.5))
    ax.set_facecolor("#f5f5f5")
    fig.patch.set_facecolor("#f5f5f5")

    for edge in net.getEdges():
        shape = edge.getShape()
        xs = [p[0] for p in shape]
        ys = [p[1] for p in shape]
        eid = edge.getID()
        if eid == "ramp":
            color, lw, zo = C_RAMP,  1.8, 3
        elif eid == "main_accel":
            color, lw, zo = C_ACCEL, 4.0, 2
        elif eid in ("main_before", "main_after"):
            color, lw, zo = C_MAIN,  3.5, 1
        else:
            color, lw, zo = "#bbbbbb", 1.0, 0
        ax.plot(xs, ys, color=color, lw=lw, zorder=zo,
                solid_capstyle="round", alpha=0.85)

    ax.scatter([taper_x],    [0], color=C_TAPER,   s=60, zorder=7)
    ax.scatter([RAMP_JOIN_X],[0], color="#ff7f00",  s=45, zorder=7)

    sc_main  = ax.scatter([], [], s=45, color=C_MAIN_V,  zorder=8, alpha=0.9)
    sc_merge = ax.scatter([], [], s=45, color=C_MERGE_V, zorder=9, alpha=0.95)
    time_txt = ax.text(0.01, 0.95, "", transform=ax.transAxes,
                       fontsize=9, va="top", fontfamily="monospace")

    # Zoom to the interesting section
    ax.set_xlim(RAMP_JOIN_X - 150, taper_x + 300)
    ax.set_ylim(-55, 25)
    ax.set_aspect("equal")
    ax.axis("off")

    legend_handles = [
        Line2D([0],[0], color=C_MAIN,  lw=3,   label="3-lane main road"),
        Line2D([0],[0], color=C_ACCEL, lw=3,   label="4-lane accel section"),
        Line2D([0],[0], color=C_RAMP,  lw=2,   label="on-ramp"),
        Line2D([0],[0], marker="o", color="w", markerfacecolor="#ff7f00",
               markersize=7, label="ramp joins"),
        Line2D([0],[0], marker="o", color="w", markerfacecolor=C_TAPER,
               markersize=7, label="lane drop"),
        Line2D([0],[0], marker="o", color="w", markerfacecolor=C_MAIN_V,
               markersize=7, label="main vehicles"),
        Line2D([0],[0], marker="o", color="w", markerfacecolor=C_MERGE_V,
               markersize=7, label="ramp vehicles"),
    ]
    ax.legend(handles=legend_handles, loc="upper right", fontsize=7,
              framealpha=0.8, ncol=2)
    ax.set_title(
        f"Traffic flow – merge_dist={md} m, curvature_offset=20 m  "
        f"(IDM + LC2013 defaults,  main 3600 veh/h, ramp 600 veh/h)",
        fontsize=9, pad=4)

    def _update(fi):
        t    = times[fi]
        vehs = frames.get(t, [])
        mx, my, rx, ry = [], [], [], []
        for (x, y, spd, lane) in vehs:
            if "ramp" in lane:
                rx.append(x); ry.append(y)
            else:
                mx.append(x); my.append(y)
        sc_main.set_offsets(np.c_[mx, my] if mx else np.empty((0, 2)))
        sc_merge.set_offsets(np.c_[rx, ry] if rx else np.empty((0, 2)))
        time_txt.set_text(f"t = {t:6.1f} s   vehicles: {len(vehs):3d}")
        return sc_main, sc_merge, time_txt

    anim = animation.FuncAnimation(fig, _update, frames=len(times),
                                   interval=1000/ANIM_FPS, blit=True)
    out = RESULTS / "traffic_anim.gif"
    print(f"Saving animation ({len(times)} frames) …")
    anim.save(str(out), writer=animation.PillowWriter(fps=ANIM_FPS), dpi=120)
    plt.close(fig)
    print(f"Saved → {out}")
    return out


# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import sys
    build_network_grid()
    build_traffic_animation()
