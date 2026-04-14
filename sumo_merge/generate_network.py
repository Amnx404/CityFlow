#!/usr/bin/env python3
"""
Generate SUMO network for a 3-lane road with a proper on-ramp / acceleration lane.

Topology
--------

  ramp_start(0, -offset)
       |
       | [ramp – 1 lane, Bézier curve]
       |
  merge_begin(RAMP_JOIN_X, 0)
      /
     /
(0,0)--[main_before, 3L]-->merge_begin--[main_accel, 4L, merge_dist m]-->taper--[main_after, 3L]-->(ROAD_LENGTH, 0)

At merge_begin the ramp becomes lane 0 (rightmost) of the 4-lane
acceleration section.  At the taper node the 4-lane road drops back to
3 lanes (lane-drop / zipper): lane 0 and lane 1 both feed into lane 0
of main_after, so IDM + LC2013 vehicles in lane 0 start merging left
well before the taper thanks to lcStrategic.

Parameters
----------
merge_distance  : length (m) of the 4-lane acceleration section
                  (how far before the lane drop the ramp lane "gives in")
curvature_offset: lateral distance (m) of the ramp start below the main
                  road centre line – larger value → more curved approach
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

# ── Layout constants ──────────────────────────────────────────────────────────
ROAD_LENGTH  = 1800        # total road length (m)
RAMP_JOIN_X  = 500         # x-coordinate where ramp meets the main road
NUM_MAIN_LANES = 3
MAIN_SPEED   = "33.33"     # ~120 km/h
RAMP_SPEED   = "27.78"     # ~100 km/h


def _indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad
    if not level:
        elem.tail = "\n"


def _bezier_cubic(p0, p1, p2, p3, n=16):
    pts = []
    for i in range(1, n):
        t = i / n
        u = 1 - t
        x = u**3*p0[0] + 3*u**2*t*p1[0] + 3*u*t**2*p2[0] + t**3*p3[0]
        y = u**3*p0[1] + 3*u**2*t*p1[1] + 3*u*t**2*p2[1] + t**3*p3[1]
        pts.append((round(x, 3), round(y, 3)))
    return pts


def _ramp_shape(curvature_offset: float) -> str:
    """
    Bézier shape from ramp_start (0, -offset) to merge_begin (RAMP_JOIN_X, 0).

    Control points keep the ramp parallel to the main road for 60 % of the
    horizontal run, then curve smoothly into the junction.
    """
    p0 = (0.0, -curvature_offset)
    p3 = (float(RAMP_JOIN_X), 0.0)
    p1 = (0.60 * RAMP_JOIN_X, -curvature_offset)   # stays offset
    p2 = (0.85 * RAMP_JOIN_X, -curvature_offset * 0.1)  # tangential arrival
    interior = _bezier_cubic(p0, p1, p2, p3, n=18)
    all_pts = [p0] + interior + [p3]
    return " ".join(f"{x},{y}" for x, y in all_pts)


def write_nodes(out_dir: Path, merge_distance: float, curvature_offset: float):
    taper_x = RAMP_JOIN_X + merge_distance
    nodes = ET.Element("nodes")
    defs = [
        ("main_start",  "0",            "0",                    "priority"),
        ("ramp_start",  "0",            str(-curvature_offset), "priority"),
        ("merge_begin", str(RAMP_JOIN_X), "0",                  "priority"),
        ("taper",       str(taper_x),   "0",                    "zipper"),
        ("main_end",    str(ROAD_LENGTH), "0",                  "priority"),
    ]
    for nid, x, y, ntype in defs:
        ET.SubElement(nodes, "node", id=nid, x=x, y=y, type=ntype)
    _indent(nodes)
    path = out_dir / "merge.nod.xml"
    ET.ElementTree(nodes).write(str(path), encoding="unicode", xml_declaration=True)
    return path


def write_edges(out_dir: Path, merge_distance: float, curvature_offset: float):
    taper_x = RAMP_JOIN_X + merge_distance
    edges = ET.Element("edges")

    # 3-lane main road before the ramp joins
    ET.SubElement(edges, "edge", attrib={
        "id": "main_before", "from": "main_start", "to": "merge_begin",
        "numLanes": str(NUM_MAIN_LANES), "speed": MAIN_SPEED, "priority": "2",
    })
    # Curved on-ramp (1 lane)
    ET.SubElement(edges, "edge", attrib={
        "id": "ramp", "from": "ramp_start", "to": "merge_begin",
        "numLanes": "1", "speed": RAMP_SPEED, "priority": "1",
        "shape": _ramp_shape(curvature_offset),
    })
    # 4-lane acceleration section
    ET.SubElement(edges, "edge", attrib={
        "id": "main_accel", "from": "merge_begin", "to": "taper",
        "numLanes": str(NUM_MAIN_LANES + 1), "speed": MAIN_SPEED, "priority": "2",
    })
    # 3-lane main road after the lane drop
    ET.SubElement(edges, "edge", attrib={
        "id": "main_after", "from": "taper", "to": "main_end",
        "numLanes": str(NUM_MAIN_LANES), "speed": MAIN_SPEED, "priority": "2",
    })

    _indent(edges)
    path = out_dir / "merge.edg.xml"
    ET.ElementTree(edges).write(str(path), encoding="unicode", xml_declaration=True)
    return path


def write_connections(out_dir: Path):
    connections = ET.Element("connections")

    # ── merge_begin: 3-lane main + 1-lane ramp → 4-lane accel ────────────────
    # Main road lanes shift right (ramp occupies lane 0 = rightmost)
    for i in range(NUM_MAIN_LANES):
        ET.SubElement(connections, "connection", attrib={
            "from": "main_before", "to": "main_accel",
            "fromLane": str(i), "toLane": str(i + 1),
        })
    # Ramp becomes the rightmost (acceleration) lane
    ET.SubElement(connections, "connection", attrib={
        "from": "ramp", "to": "main_accel",
        "fromLane": "0", "toLane": "0",
    })

    # ── taper: 4-lane accel → 3-lane main (lane drop / zipper) ───────────────
    # Lane 0 (accel/ramp lane) and lane 1 both feed into lane 0 of main_after
    # LC2013 lcStrategic will cause lane-0 vehicles to merge left proactively
    ET.SubElement(connections, "connection", attrib={
        "from": "main_accel", "to": "main_after",
        "fromLane": "0", "toLane": "0", "type": "zipper",
    })
    ET.SubElement(connections, "connection", attrib={
        "from": "main_accel", "to": "main_after",
        "fromLane": "1", "toLane": "0", "type": "zipper",
    })
    for i in range(2, NUM_MAIN_LANES + 1):          # lanes 2, 3 → 1, 2
        ET.SubElement(connections, "connection", attrib={
            "from": "main_accel", "to": "main_after",
            "fromLane": str(i), "toLane": str(i - 1),
        })

    _indent(connections)
    path = out_dir / "merge.con.xml"
    ET.ElementTree(connections).write(str(path), encoding="unicode", xml_declaration=True)
    return path


def build_net(out_dir, nod, edg, con):
    net_path = out_dir / "merge.net.xml"
    result = subprocess.run([
        "netconvert",
        "--node-files", str(nod),
        "--edge-files", str(edg),
        "--connection-files", str(con),
        "--output-file", str(net_path),
        "--no-warnings",
        "--junctions.join",
        "--default.lanewidth", "3.2",
    ], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"netconvert failed:\n{result.stderr}\n{result.stdout}")
    return net_path


def generate_network(merge_distance: float, curvature_offset: float,
                     out_dir: Path) -> Path:
    """
    Build the SUMO .net.xml for one parameter combination.

    merge_distance  : length (m) of the 4-lane acceleration section
    curvature_offset: lateral start offset (m) of the on-ramp
    out_dir         : directory to write files into
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    nod = write_nodes(out_dir, merge_distance, curvature_offset)
    edg = write_edges(out_dir, merge_distance, curvature_offset)
    con = write_connections(out_dir)
    return build_net(out_dir, nod, edg, con)


if __name__ == "__main__":
    import sys
    md = float(sys.argv[1]) if len(sys.argv) > 1 else 200
    co = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    net = generate_network(md, co, Path(f"/tmp/test_md{int(md)}_co{int(co)}"))
    print(f"Network written to {net}")
