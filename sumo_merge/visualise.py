#!/usr/bin/env python3
"""
Visualise the 16 merge-scenario networks and one animated traffic run.

Outputs (written to sumo_merge/results/):
  network_grid.png    – 4×4 panel showing every network geometry
  traffic_anim.gif    – animated traffic flow for one chosen scenario
"""

import os
import sys
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from itertools import product

import matplotlib
matplotlib.use("Agg")          # no display needed
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import numpy as np
import sumolib

RESULTS  = Path(__file__).parent / "results"
MERGE_DISTANCES   = [100, 200, 300, 400]
CURVATURE_OFFSETS = [10,  20,  30,  40]
ROAD_LENGTH       = 1000


# ── colour palette ────────────────────────────────────────────────────────────
C_MAIN  = "#2c7bb6"   # main road
C_RAMP  = "#d7191c"   # merge ramp
C_JOIN  = "#1a9641"   # merge junction dot
C_MAIN_V  = "#4488cc" # main-road vehicle
C_MERGE_V = "#ee5533" # merge-ramp vehicle (identified by departure edge)


# ═══════════════════════════════════════════════════════════════════════════════
# 1.  NETWORK GRID
# ═══════════════════════════════════════════════════════════════════════════════

def _draw_net(ax, net, merge_join_x, title, zoom=True):
    """Draw a single sumolib network onto an existing Axes.

    zoom=True crops to ±300 m around the merge junction for clarity.
    """
    for edge in net.getEdges():
        shape = edge.getShape()
        xs = [p[0] for p in shape]
        ys = [p[1] for p in shape]
        eid = edge.getID()
        if eid == "merge_lane":
            color, lw, zorder = C_RAMP, 2.2, 3
        elif eid in ("main_before", "main_after"):
            color, lw, zorder = C_MAIN, 3.0, 2
        else:
            color, lw, zorder = "#999999", 1.0, 1
        ax.plot(xs, ys, color=color, lw=lw, zorder=zorder, solid_capstyle="round")

    # junction dot
    ax.scatter([merge_join_x], [0], color=C_JOIN, s=55, zorder=5)

    if zoom:
        pad_x, pad_y = 80, 35
        ax.set_xlim(merge_join_x - 280, merge_join_x + pad_x)
        ax.set_ylim(-pad_y, pad_y)

    ax.set_title(title, fontsize=7, pad=2)
    ax.set_aspect("equal")
    ax.axis("off")


def build_network_grid():
    fig, axes = plt.subplots(
        4, 4,
        figsize=(14, 10),
        gridspec_kw={"wspace": 0.08, "hspace": 0.25},
    )

    for row, md in enumerate(MERGE_DISTANCES):
        for col, co in enumerate(CURVATURE_OFFSETS):
            sim_id  = f"md{md:03d}_co{co:02d}"
            net_path = RESULTS / sim_id / "merge.net.xml"
            ax = axes[row][col]

            net = sumolib.net.readNet(str(net_path), withInternal=False)
            merge_join_x = ROAD_LENGTH - md
            title = f"merge={md} m  curve={co} m"
            _draw_net(ax, net, merge_join_x, title)

    # shared legend
    legend_handles = [
        Line2D([0], [0], color=C_MAIN,  lw=2, label="3-lane main road"),
        Line2D([0], [0], color=C_RAMP,  lw=2, label="merge ramp"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=C_JOIN,
               markersize=6, label="zipper junction"),
    ]
    fig.legend(handles=legend_handles, loc="lower center", ncol=3,
               fontsize=8, frameon=True, bbox_to_anchor=(0.5, 0.01))

    # row / column labels
    for row, md in enumerate(MERGE_DISTANCES):
        axes[row][0].set_ylabel(f"merge\n{md} m", fontsize=7,
                                rotation=0, labelpad=38, va="center")
    for col, co in enumerate(CURVATURE_OFFSETS):
        axes[0][col].set_title(
            f"curve {co} m\n" + axes[0][col].get_title(), fontsize=7, pad=3
        )
        # remove duplicate title
        axes[0][col].set_title(f"curve={co} m / merge={MERGE_DISTANCES[0]} m",
                               fontsize=7, pad=3)

    fig.suptitle("SUMO merge-onto-3-lane-road: all 16 network geometries",
                 fontsize=11, y=0.98, fontweight="bold")

    out = RESULTS / "network_grid.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved → {out}")
    return out


# ═══════════════════════════════════════════════════════════════════════════════
# 2.  TRAFFIC ANIMATION
# ═══════════════════════════════════════════════════════════════════════════════

FCD_SIM_ID = "md200_co20"          # scenario to animate
FCD_XML    = RESULTS / FCD_SIM_ID / "fcd.xml"
ANIM_DURATION_S = 120              # first N sim-seconds to animate
STEP_LENGTH     = 0.1
ANIM_FPS        = 20
EVERY_N_STEPS   = 5                # sub-sample: 1 frame per 0.5 sim-s


def _run_fcd_sim():
    """Re-run the chosen scenario with FCD output enabled."""
    sim_dir = RESULTS / FCD_SIM_ID
    # patch the sumocfg to add fcd-output
    cfg_src = sim_dir / "merge.sumocfg"
    cfg_fcd = sim_dir / "merge_fcd.sumocfg"
    tree = ET.parse(str(cfg_src))
    root = tree.getroot()

    # add / replace <output> section
    output_el = root.find("output")
    if output_el is None:
        output_el = ET.SubElement(root, "output")
    fcd_el = output_el.find("fcd-output")
    if fcd_el is None:
        fcd_el = ET.SubElement(output_el, "fcd-output")
    fcd_el.set("value", "fcd.xml")

    # shorten sim to ANIM_DURATION_S
    time_el = root.find("time")
    if time_el is not None:
        end_el = time_el.find("end")
        if end_el is None:
            end_el = ET.SubElement(time_el, "end")
        end_el.set("value", str(ANIM_DURATION_S))

    ET.indent(tree, space="  ")
    tree.write(str(cfg_fcd), encoding="unicode", xml_declaration=True)

    cmd = ["sumo", "-c", str(cfg_fcd),
           "--no-step-log", "--no-warnings", "--duration-log.disable"]
    print(f"Running FCD sim ({ANIM_DURATION_S} s) …")
    subprocess.run(cmd, cwd=str(sim_dir), check=True,
                   capture_output=True)
    print(f"FCD written → {FCD_XML}")


def _parse_fcd(fcd_path):
    """Parse FCD XML → dict {timestep: [(x,y,speed,edge), ...]}"""
    frames = {}
    tree = ET.parse(str(fcd_path))
    for ts in tree.getroot().findall("timestep"):
        t = float(ts.get("time"))
        vehs = []
        for v in ts.findall("vehicle"):
            vehs.append((
                float(v.get("x")),
                float(v.get("y")),
                float(v.get("speed")),
                v.get("lane", ""),
            ))
        frames[t] = vehs
    return frames


def build_traffic_animation():
    # 1. run sim if FCD not yet produced
    if not FCD_XML.exists():
        _run_fcd_sim()
    else:
        print(f"FCD already exists: {FCD_XML}")

    # 2. load network
    sim_dir  = RESULTS / FCD_SIM_ID
    net_path = sim_dir / "merge.net.xml"
    net = sumolib.net.readNet(str(net_path), withInternal=False)
    md  = 200
    merge_join_x = ROAD_LENGTH - md

    # 3. parse FCD
    frames = _parse_fcd(FCD_XML)
    times  = sorted(frames.keys())
    # sub-sample
    times  = times[::EVERY_N_STEPS]

    # 4. build figure
    fig, ax = plt.subplots(figsize=(13, 4))
    ax.set_facecolor("#f0f0f0")
    fig.patch.set_facecolor("#f0f0f0")

    # draw static network
    for edge in net.getEdges():
        shape  = edge.getShape()
        xs = [p[0] for p in shape]
        ys = [p[1] for p in shape]
        eid = edge.getID()
        if eid == "merge_lane":
            color, lw, zorder = C_RAMP,  1.6, 2
        elif eid in ("main_before", "main_after"):
            color, lw, zorder = C_MAIN, 2.8, 1
        else:
            color, lw, zorder = "#aaaaaa", 1.0, 0
        ax.plot(xs, ys, color=color, lw=lw, zorder=zorder,
                solid_capstyle="round", alpha=0.85)

    ax.scatter([merge_join_x], [0], color=C_JOIN, s=50, zorder=6, label="zipper junction")

    # vehicle scatter – will be updated each frame
    sc_main  = ax.scatter([], [], s=40, color=C_MAIN_V,  zorder=7, alpha=0.88,
                          label="main-road vehicles")
    sc_merge = ax.scatter([], [], s=40, color=C_MERGE_V, zorder=8, alpha=0.92,
                          label="ramp vehicles")
    time_txt = ax.text(0.01, 0.96, "", transform=ax.transAxes,
                       fontsize=9, va="top", fontfamily="monospace")

    # Zoom to merge zone: 350 m before → 150 m after the junction
    ax.set_xlim(merge_join_x - 350, merge_join_x + 150)
    ax.set_ylim(-35, 35)

    ax.set_aspect("equal")
    ax.axis("off")

    legend_handles = [
        Line2D([0], [0], color=C_MAIN,   lw=2.5,  label="3-lane main road"),
        Line2D([0], [0], color=C_RAMP,   lw=2,    label="merge ramp"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=C_JOIN,
               markersize=7, label="zipper junction"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=C_MAIN_V,
               markersize=7, label="main-road vehicles"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=C_MERGE_V,
               markersize=7, label="ramp vehicles"),
    ]
    ax.legend(handles=legend_handles, loc="upper right", fontsize=7,
              framealpha=0.7, ncol=1)
    ax.set_title(
        f"Traffic flow – merge={md} m, curve_offset=20 m  "
        f"(IDM + LC2013 defaults,  main 3600 veh/h, ramp 600 veh/h)",
        fontsize=9, pad=4
    )

    def _update(frame_idx):
        t = times[frame_idx]
        vehs = frames.get(t, [])
        mx, my, rx, ry = [], [], [], []
        for (x, y, spd, lane) in vehs:
            if "merge_lane" in lane:
                rx.append(x); ry.append(y)
            else:
                mx.append(x); my.append(y)
        sc_main.set_offsets(np.c_[mx, my] if mx else np.empty((0, 2)))
        sc_merge.set_offsets(np.c_[rx, ry] if rx else np.empty((0, 2)))
        time_txt.set_text(f"t = {t:6.1f} s   vehicles: {len(vehs):3d}")
        return sc_main, sc_merge, time_txt

    anim = animation.FuncAnimation(
        fig, _update,
        frames=len(times),
        interval=1000 / ANIM_FPS,
        blit=True,
    )

    out = RESULTS / "traffic_anim.gif"
    print(f"Saving animation ({len(times)} frames) …")
    anim.save(
        str(out),
        writer=animation.PillowWriter(fps=ANIM_FPS),
        dpi=120,
    )
    plt.close(fig)
    print(f"Saved → {out}")
    return out


# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    build_network_grid()
    build_traffic_animation()
    print("\nDone.  Output files:")
    print(f"  {RESULTS}/network_grid.png")
    print(f"  {RESULTS}/traffic_anim.gif")
