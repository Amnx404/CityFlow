#!/usr/bin/env python3
"""
Run all 4×4 = 16 merge-scenario combinations and write safety_metrics.csv.

Parameter grids
---------------
merge_distance  (m) : [100, 200, 300, 400]
    Distance from the zipper merge junction to the end of the road.
    Smaller value → the zipper lane "gives in" closer to the road end.

curvature_offset (m) : [10, 20, 30, 40]
    Lateral distance of the merge ramp start from the main road.
    Larger value → tighter / more-curved approach geometry.

Vehicle model  : IDM  with SUMO defaults
Lane-change    : LC2013 with SUMO defaults
"""

import csv
import itertools
import sys
import time
import traceback
from pathlib import Path

# ── Ensure local modules are importable ───────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))

from generate_network import generate_network, ROAD_LENGTH
from generate_routes  import generate_routes
from run_simulation   import run_simulation, write_sumocfg


# ── Parameter grids ───────────────────────────────────────────────────────────
MERGE_DISTANCES   = [100, 200, 300, 400]   # metres
CURVATURE_OFFSETS = [10,  20,  30,  40]    # metres

BASE_DIR = Path(__file__).parent / "results"


# ── Pretty-print helpers ──────────────────────────────────────────────────────

HEADER_FMT = (
    f"{'ID':<22} {'merge_dist_m':>12} {'curve_off_m':>11} "
    f"{'min_ttc_s':>10} {'mean_ttc_s':>10} "
    f"{'drac_max':>9} {'drac_mean':>9} "
    f"{'hard_brk':>9} {'near_miss':>9} "
    f"{'collisions':>10} {'completed':>9} {'avg_spd_ms':>10}"
)
ROW_FMT = (
    "{id:<22} {merge_distance_m:>12} {curvature_offset_m:>11} "
    "{min_ttc_s:>10.4f} {mean_ttc_s:>10.4f} "
    "{drac_max:>9.4f} {drac_mean:>9.4f} "
    "{hard_brake_events:>9} {near_miss_events:>9} "
    "{collision_count:>10} {vehicles_completed:>9} {avg_speed_ms:>10.3f}"
)
SEP = "-" * len(HEADER_FMT)


def run_all():
    BASE_DIR.mkdir(parents=True, exist_ok=True)
    results = []

    combos = list(itertools.product(MERGE_DISTANCES, CURVATURE_OFFSETS))
    total  = len(combos)

    print(f"\nRunning {total} combinations  (merge_distances × curvature_offsets)\n")
    print(HEADER_FMT)
    print(SEP)

    for idx, (md, co) in enumerate(combos, 1):
        sim_id  = f"md{md:03d}_co{co:02d}"
        out_dir = BASE_DIR / sim_id

        try:
            # 1. Generate network
            net = generate_network(md, co, out_dir)

            # 2. Generate routes (identical across combos but kept local)
            rou = generate_routes(out_dir)

            # 3. Write SUMO config
            cfg = write_sumocfg(net, rou, out_dir, sim_id)

            # 4. Run and collect metrics
            merge_join_x = ROAD_LENGTH - md
            metrics = run_simulation(cfg, merge_join_x)

            row = {
                "id":                  sim_id,
                "merge_distance_m":    md,
                "curvature_offset_m":  co,
                **metrics,
            }

        except Exception as exc:
            print(f"\n[ERROR] {sim_id}: {exc}")
            traceback.print_exc()
            row = {
                "id":                  sim_id,
                "merge_distance_m":    md,
                "curvature_offset_m":  co,
                "min_ttc_s":           -1,
                "mean_ttc_s":          -1,
                "drac_max":            -1,
                "drac_mean":           -1,
                "hard_brake_events":   -1,
                "near_miss_events":    -1,
                "collision_count":     -1,
                "vehicles_completed":  -1,
                "avg_speed_ms":        -1,
            }

        results.append(row)
        print(ROW_FMT.format(**row))

    # ── Write CSV ─────────────────────────────────────────────────────────────
    csv_path = BASE_DIR / "safety_metrics.csv"
    if results:
        fieldnames = list(results[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    print(SEP)
    print(f"\nResults saved → {csv_path}\n")

    # ── Summary statistics ────────────────────────────────────────────────────
    valid = [r for r in results if r["min_ttc_s"] >= 0]
    if valid:
        print("=== AGGREGATE SUMMARY ===")
        for key in ("min_ttc_s", "drac_max", "hard_brake_events",
                    "near_miss_events", "collision_count", "vehicles_completed"):
            vals = [r[key] for r in valid]
            print(f"  {key:<24}  min={min(vals):<10.3g}  max={max(vals):<10.3g}  "
                  f"mean={sum(vals)/len(vals):.3g}")

    return results


if __name__ == "__main__":
    t0 = time.time()
    run_all()
    print(f"Total wall-clock time: {time.time()-t0:.1f}s")
