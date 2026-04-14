#!/usr/bin/env python3
"""
Generate SUMO route/flow file for the merge scenario.

Vehicle type uses SUMO default IDM parameters:
  accel=2.6, decel=4.5, sigma=0.5, tau=1.0, minGap=2.5
Lane change uses default LC2013 model.
"""

from pathlib import Path


# ── Simulation timing ────────────────────────────────────────────────────────
SIM_DURATION = 600          # seconds

# ── Flow rates ────────────────────────────────────────────────────────────────
# Total main-road vehicles/hour split evenly across NUM_MAIN_LANES=3 lanes
FLOW_MAIN_TOTAL_VPH = 3600   # 1 200 veh/h per lane  →  level-of-service C/D
FLOW_MERGE_VPH      = 600    # on-ramp demand

# ── IDM defaults (SUMO 1.x) ──────────────────────────────────────────────────
IDM = dict(
    carFollowModel="IDM",
    accel="2.6",          # m/s²
    decel="4.5",          # m/s²
    emergencyDecel="9.0", # m/s²
    sigma="0.5",          # driver imperfection
    tau="1.0",            # desired time headway (s)
    minGap="2.5",         # minimum gap (m)
    maxSpeed="33.33",     # ~120 km/h
    length="5.0",
    width="1.8",
)

# ── Lane-change defaults (LC2013) ─────────────────────────────────────────────
LC = dict(
    laneChangeModel="LC2013",
    lcStrategic="1.0",
    lcCooperative="1.0",
    lcSpeedGain="1.0",
    lcKeepRight="1.0",
)


def _vtype_attrs() -> str:
    attrs = {**IDM, **LC}
    return " ".join(f'{k}="{v}"' for k, v in attrs.items())


TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8"?>
<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">

    <!-- Single vehicle type: IDM + LC2013 with SUMO defaults -->
    <vType id="car_idm" {vtype_attrs}/>

    <!-- Main road flow  (3 lanes, random departure lane) -->
    <flow id="main_flow"
          type="car_idm"
          from="main_before"
          to="main_after"
          begin="0"
          end="{sim_duration}"
          vehsPerHour="{main_vph}"
          departLane="random"
          departSpeed="max"
          color="0,0,200"/>

    <!-- On-ramp / merge flow -->
    <flow id="merge_flow"
          type="car_idm"
          from="merge_lane"
          to="main_after"
          begin="0"
          end="{sim_duration}"
          vehsPerHour="{merge_vph}"
          departLane="0"
          departSpeed="max"
          color="200,0,0"/>

</routes>
"""


def generate_routes(out_dir: Path) -> Path:
    """Write merge.rou.xml into *out_dir* and return the path."""
    out_dir.mkdir(parents=True, exist_ok=True)
    rou_path = out_dir / "merge.rou.xml"

    content = TEMPLATE.format(
        vtype_attrs=_vtype_attrs(),
        sim_duration=SIM_DURATION,
        main_vph=FLOW_MAIN_TOTAL_VPH,
        merge_vph=FLOW_MERGE_VPH,
    )
    rou_path.write_text(content)
    return rou_path


if __name__ == "__main__":
    import sys
    d = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/routes_test")
    p = generate_routes(d)
    print(f"Routes written to {p}")
