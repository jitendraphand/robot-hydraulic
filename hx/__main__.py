"""HX-1 command line.

    python -m hx loop          # full recursive design loop (sim -> size -> CAD -> masses ...)
    python -m hx sim           # run simulation scenarios, write demand
    python -m hx size          # size cylinders from the stored demand
    python -m hx cad           # build STLs, plates, print plan, BOM, mass properties
    python -m hx asm           # assembly interference check over all poses
    python -m hx render        # render assembly images to docs/img
    python -m hx gen           # generate firmware config header + CAN protocol + wiring docs
    python -m hx sil           # hydraulic software-in-the-loop tests (needs firmware lib)
    python -m hx docs          # regenerate docs/00_design_summary.md
    python -m hx qa            # run the full recursive QA tree, write qa/report.md
    python -m hx all           # gen + cad + asm + render + qa (no re-simulation)
"""
from __future__ import annotations

import argparse
import json
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser(prog="hx", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["loop", "sim", "size", "cad", "asm", "render", "gen", "sil", "docs", "qa", "all"])
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--fast", action="store_true", help="QA: skip slow simulation re-runs")
    a = ap.parse_args(argv)
    from .config import RobotConfig, save_derived

    t0 = time.time()
    cfg = RobotConfig()
    if a.cmd == "loop":
        from .design import loop
        r = loop(cfg, max_iter=a.iters)
        print(json.dumps({k: v for k, v in r.items() if k != "history"}, indent=1))
    elif a.cmd == "sim":
        from .design import envelope, run_sim, hpu_check, save_sim, save_traces
        demand, summ, series = run_sim(cfg)
        save_derived("demand", envelope(demand))
        save_sim(summ)
        save_traces(cfg, series)
        if cfg.derived("sizing"):
            save_derived("hpu", hpu_check(cfg, series, cfg.derived("sizing")))
    elif a.cmd == "size":
        from .hydraulics import size_all
        dem = cfg.derived("demand")
        save_derived("sizing", size_all(cfg, dem))
    elif a.cmd == "cad":
        from .cad.build import build_all
        build_all(cfg)
    elif a.cmd == "asm":
        from .cad.assembly import run_interference
        r = run_interference(cfg)
        bad = {k: v for k, v in r.items() if v["hits"] or v["cyl_fit_fail"]}
        print(f"{len(r)} poses checked, {len(bad)} with problems")
        sys.exit(1 if bad else 0)
    elif a.cmd == "render":
        from .tools_render import render_all
        render_all(cfg)
    elif a.cmd == "gen":
        from .codegen import generate_all
        generate_all(cfg)
    elif a.cmd == "sil":
        from .sim.sil import run_all as sil_all
        r = sil_all(cfg)
        save_derived("sil", r)
        print(json.dumps({k: v.get("pass") for k, v in r.items() if isinstance(v, dict)}, indent=1))
    elif a.cmd == "docs":
        from .report import write
        write(cfg)
    elif a.cmd == "qa":
        from .qa.run import run_qa
        ok = run_qa(cfg, fast=a.fast)
        sys.exit(0 if ok else 1)
    elif a.cmd == "all":
        from .codegen import generate_all
        from .cad.build import build_all
        from .tools_render import render_all
        from .qa.run import run_qa
        from .report import write
        generate_all(cfg)
        build_all(cfg)
        render_all(RobotConfig())
        write(RobotConfig())
        ok = run_qa(RobotConfig(), fast=a.fast)
        sys.exit(0 if ok else 1)
    print(f"[hx] {a.cmd} done in {time.time() - t0:.1f} s")


if __name__ == "__main__":
    main()
