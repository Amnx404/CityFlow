#!/usr/bin/env python3
"""
Generate SUMO network XML files for a 3-lane road with a merging lane.

Parameters
----------
merge_distance : float
    Distance (m) from the merge junction to the road end.
    Controls how far before the end the zipper lane gives in.
curvature_offset : float
    Lateral distance (m) at which the merge lane starts away from the main road.
    Larger value = tighter/more curved approach road.

Network layout
--------------
   merge_start(0, -offset)
        |
        | merge_lane (1 lane, curved)
        |
main_start(0,0) ---main_before(3 lanes)--- merge_join(X,0) ---main_after(3 lanes)--- main_end(1000,0)
"""

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROAD_LENGTH = 1000       # total length of the main road (m)
NUM_MAIN_LANES = 3
MAIN_SPEED = "33.33"     # ~120 km/h
MERGE_SPEED = "27.78"    # ~100 km/h on ramp


def _indent(elem, level=0):
    """Add pretty-print indentation to an ElementTree element."""
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


def bezier_cubic(p0, p1, p2, p3, n=14):
    """Sample a cubic bezier curve, returning n-1 interior points (excluding endpoints)."""
    pts = []
    for i in range(1, n):
        t = i / n
        u = 1.0 - t
        x = u**3*p0[0] + 3*u**2*t*p1[0] + 3*u*t**2*p2[0] + t**3*p3[0]
        y = u**3*p0[1] + 3*u**2*t*p1[1] + 3*u*t**2*p2[1] + t**3*p3[1]
        pts.append((round(x, 3), round(y, 3)))
    return pts


def merge_lane_shape(merge_join_x: float, curvature_offset: float) -> str:
    """
    Build the SUMO edge shape string for the merge ramp.

    The ramp starts at (0, -curvature_offset) and joins the main road at
    (merge_join_x, 0).  The bezier control points create an S-curve:
      - P1 stays at the lateral offset for 60% of the horizontal run
        (vehicle travels alongside the main road before curving in)
      - P2 arrives tangentially (nearly horizontal) at the merge point

    Larger curvature_offset means the approach road is more curved.
    """
    p0 = (0.0, -curvature_offset)
    p3 = (merge_join_x, 0.0)

    # Control point 1: keep the lane at the offset level for 60% of the run
    p1 = (0.60 * merge_join_x, -curvature_offset)
    # Control point 2: approach tangentially – still 20% offset, 15% before junction
    p2 = (0.85 * merge_join_x, -curvature_offset * 0.15)

    interior = bezier_cubic(p0, p1, p2, p3, n=16)

    all_pts = [p0] + interior + [p3]
    return " ".join(f"{x},{y}" for x, y in all_pts)


def write_nodes(out_dir: Path, merge_join_x: float, curvature_offset: float):
    nodes = ET.Element("nodes")
    defs = [
        ("main_start",  "0",                "0",                    "priority"),
        ("merge_start", "0",                str(-curvature_offset), "priority"),
        ("merge_join",  str(merge_join_x),  "0",                    "zipper"),
        ("main_end",    str(ROAD_LENGTH),   "0",                    "priority"),
    ]
    for nid, x, y, ntype in defs:
        ET.SubElement(nodes, "node", id=nid, x=x, y=y, type=ntype)

    _indent(nodes)
    path = out_dir / "merge.nod.xml"
    ET.ElementTree(nodes).write(str(path), encoding="unicode", xml_declaration=True)
    return path


def write_edges(out_dir: Path, merge_join_x: float, curvature_offset: float):
    edges = ET.Element("edges")

    # Main road before merge (3 lanes, straight)
    ET.SubElement(edges, "edge", attrib={
        "id": "main_before",
        "from": "main_start",
        "to": "merge_join",
        "numLanes": str(NUM_MAIN_LANES),
        "speed": MAIN_SPEED,
        "priority": "2",
    })

    # Main road after merge (3 lanes, straight)
    ET.SubElement(edges, "edge", attrib={
        "id": "main_after",
        "from": "merge_join",
        "to": "main_end",
        "numLanes": str(NUM_MAIN_LANES),
        "speed": MAIN_SPEED,
        "priority": "2",
    })

    # Merge ramp (1 lane, curved)
    shape = merge_lane_shape(merge_join_x, curvature_offset)
    ET.SubElement(edges, "edge", attrib={
        "id": "merge_lane",
        "from": "merge_start",
        "to": "merge_join",
        "numLanes": "1",
        "speed": MERGE_SPEED,
        "priority": "1",
        "shape": shape,
    })

    _indent(edges)
    path = out_dir / "merge.edg.xml"
    ET.ElementTree(edges).write(str(path), encoding="unicode", xml_declaration=True)
    return path


def write_connections(out_dir: Path):
    connections = ET.Element("connections")

    # Through lanes on main road
    for i in range(NUM_MAIN_LANES):
        ET.SubElement(connections, "connection", attrib={
            "from": "main_before",
            "to": "main_after",
            "fromLane": str(i),
            "toLane": str(i),
        })

    # Merge ramp zippers into rightmost lane (lane 0)
    ET.SubElement(connections, "connection", attrib={
        "from": "merge_lane",
        "to": "main_after",
        "fromLane": "0",
        "toLane": "0",
        "type": "zipper",
    })

    _indent(connections)
    path = out_dir / "merge.con.xml"
    ET.ElementTree(connections).write(str(path), encoding="unicode", xml_declaration=True)
    return path


def build_net(out_dir: Path, nod: Path, edg: Path, con: Path) -> Path:
    net_path = out_dir / "merge.net.xml"
    cmd = [
        "netconvert",
        "--node-files", str(nod),
        "--edge-files", str(edg),
        "--connection-files", str(con),
        "--output-file", str(net_path),
        "--no-warnings",
        "--junctions.join",
        "--default.lanewidth", "3.2",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"netconvert failed:\n{result.stderr}\n{result.stdout}")
    return net_path


def generate_network(merge_distance: float, curvature_offset: float, out_dir: Path) -> Path:
    """
    Full pipeline: write XML sources -> run netconvert -> return .net.xml path.

    Parameters
    ----------
    merge_distance : float
        Metres from merge junction to road end.
    curvature_offset : float
        Lateral offset (m) of the merge lane start from the main road centre.
    out_dir : Path
        Directory to write all output files into.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    merge_join_x = ROAD_LENGTH - merge_distance

    nod = write_nodes(out_dir, merge_join_x, curvature_offset)
    edg = write_edges(out_dir, merge_join_x, curvature_offset)
    con = write_connections(out_dir)
    net = build_net(out_dir, nod, edg, con)
    return net


if __name__ == "__main__":
    import sys
    md = float(sys.argv[1]) if len(sys.argv) > 1 else 200
    co = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    net = generate_network(md, co, Path(f"/tmp/test_md{int(md)}_co{int(co)}"))
    print(f"Network written to {net}")
