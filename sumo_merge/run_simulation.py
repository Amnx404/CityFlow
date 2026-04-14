#!/usr/bin/env python3
"""
SUMO merge-scenario runner.

Writes a .sumocfg, runs the simulation through traci, and returns a dict of
safety metrics.  Each run is fully self-contained so the 16 combinations can
be executed in sequence without port collisions.

Safety metrics collected
------------------------
min_ttc_s          : minimum time-to-collision [s] observed anywhere in the
                     merge zone (lower = more dangerous; -1 if never measured)
mean_ttc_s         : mean TTC over all merge-zone observations
drac_max           : maximum Deceleration Rate to Avoid Crash [m/s²]
drac_mean          : mean DRAC across all merge-zone pairs
hard_brake_events  : vehicle-steps with deceleration > HARD_BRAKE_THR
near_miss_events   : vehicle-steps with TTC < TTC_WARN_THRESHOLD
collision_count    : SUMO-detected collisions
vehicles_completed : vehicles that reached the end node
avg_speed_ms       : mean speed [m/s] of vehicles inside the merge zone
"""

import os
import sys
from pathlib import Path

import traci
import traci.constants as tc


# ── Safety thresholds ─────────────────────────────────────────────────────────
HARD_BRAKE_THR   = 4.0   # m/s²   deceleration magnitude that counts as hard braking
TTC_WARN_THR     = 1.5   # s      TTC below this = near-miss
TTC_UPPER_BOUND  = 60.0  # s      ignore pairs with TTC > this (not really approaching)

# ── Merge-zone window ─────────────────────────────────────────────────────────
ZONE_UPSTREAM    = 250   # m before the merge junction
ZONE_DOWNSTREAM  = 150   # m after the merge junction

# ── Simulation step size ──────────────────────────────────────────────────────
STEP_LENGTH = 0.1        # seconds


def write_sumocfg(net_path: Path, rou_path: Path, out_dir: Path, sim_id: str) -> Path:
    cfg_path = out_dir / "merge.sumocfg"
    cfg = f"""\
<?xml version="1.0" encoding="UTF-8"?>
<configuration>
  <input>
    <net-file  value="{net_path.name}"/>
    <route-files value="{rou_path.name}"/>
  </input>
  <time>
    <begin value="0"/>
    <end   value="600"/>
    <step-length value="{STEP_LENGTH}"/>
  </time>
  <processing>
    <collision.action       value="warn"/>
    <collision.mingap-factor value="0"/>
    <lanechange.duration    value="3"/>
  </processing>
  <random>
    <seed value="42"/>
  </random>
</configuration>
"""
    cfg_path.write_text(cfg)
    return cfg_path


# ── Welford online statistics ─────────────────────────────────────────────────

class OnlineStat:
    """Compute mean and min incrementally (Welford)."""
    def __init__(self):
        self.n   = 0
        self.mean = 0.0
        self.min  = float("inf")

    def update(self, x: float):
        self.n += 1
        self.min = min(self.min, x)
        delta = x - self.mean
        self.mean += delta / self.n

    @property
    def valid(self):
        return self.n > 0


# ── Core runner ───────────────────────────────────────────────────────────────

def run_simulation(cfg_path: Path, merge_join_x: float) -> dict:
    """
    Start sumo via traci, step through the full simulation, collect metrics.

    Parameters
    ----------
    cfg_path      : path to the .sumocfg file
    merge_join_x  : x-coordinate of the zipper junction (m)

    Returns
    -------
    dict of safety metrics (see module docstring)
    """
    zone_lo = max(0.0, merge_join_x - ZONE_UPSTREAM)
    zone_hi = merge_join_x + ZONE_DOWNSTREAM

    sumo_cmd = [
        "sumo",
        "-c", str(cfg_path),
        "--no-step-log",
        "--no-warnings",
        "--duration-log.disable",
    ]

    traci.start(sumo_cmd)

    ttc_stat  = OnlineStat()
    drac_stat = OnlineStat()
    hard_brake_events  = 0
    near_miss_events   = 0
    collision_count    = 0
    vehicles_completed = 0
    speed_stat         = OnlineStat()

    # Subscribe to simulation-level collision counter
    # (updated each step)
    try:
        end_time = int(600 / STEP_LENGTH)
        for _step in range(end_time):
            traci.simulationStep()

            # Accumulate completed vehicles and collisions
            vehicles_completed += traci.simulation.getArrivedNumber()
            collision_count    += traci.simulation.getCollidingVehiclesNumber()

            for vid in traci.vehicle.getIDList():
                try:
                    pos = traci.vehicle.getPosition(vid)
                    vx  = pos[0]

                    # Only analyse vehicles inside the merge zone
                    if not (zone_lo <= vx <= zone_hi):
                        continue

                    spd  = traci.vehicle.getSpeed(vid)
                    accel = traci.vehicle.getAcceleration(vid)

                    speed_stat.update(spd)

                    # Hard braking
                    if accel < -HARD_BRAKE_THR:
                        hard_brake_events += 1

                    # TTC / DRAC toward leader
                    leader_info = traci.vehicle.getLeader(vid, 150.0)
                    if leader_info is not None:
                        leader_id, gap = leader_info
                        if gap > 0.01:
                            try:
                                lspd = traci.vehicle.getSpeed(leader_id)
                            except traci.exceptions.TraCIException:
                                continue

                            rel_spd = spd - lspd   # positive when closing

                            if rel_spd > 0.1:       # only when approaching
                                ttc  = gap / rel_spd
                                drac = (rel_spd ** 2) / (2.0 * gap)

                                if ttc < TTC_UPPER_BOUND:
                                    ttc_stat.update(ttc)
                                    drac_stat.update(drac)

                                    if ttc < TTC_WARN_THR:
                                        near_miss_events += 1

                except traci.exceptions.TraCIException:
                    continue

    finally:
        traci.close()

    return {
        "min_ttc_s":         round(ttc_stat.min  if ttc_stat.valid  else -1.0, 4),
        "mean_ttc_s":        round(ttc_stat.mean if ttc_stat.valid  else -1.0, 4),
        "drac_max":          round(drac_stat.min  if drac_stat.valid else -1.0, 4),  # max DRAC = min TTC companion
        "drac_mean":         round(drac_stat.mean if drac_stat.valid else -1.0, 4),
        "hard_brake_events": hard_brake_events,
        "near_miss_events":  near_miss_events,
        "collision_count":   collision_count,
        "vehicles_completed": vehicles_completed,
        "avg_speed_ms":      round(speed_stat.mean if speed_stat.valid else -1.0, 3),
    }


if __name__ == "__main__":
    # Quick single-run test
    from generate_network import generate_network
    from generate_routes  import generate_routes

    md  = float(sys.argv[1]) if len(sys.argv) > 1 else 200
    co  = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    out = Path(f"/tmp/sumo_test_md{int(md)}_co{int(co)}")

    net  = generate_network(md, co, out)
    rou  = generate_routes(out)
    cfg  = write_sumocfg(net, rou, out, "test")
    from generate_network import ROAD_LENGTH
    mj   = ROAD_LENGTH - md

    m = run_simulation(cfg, mj)
    print("Metrics:", m)
