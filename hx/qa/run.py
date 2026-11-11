"""Run the full recursive QA (design tree + physical build tree) and write reports."""
from __future__ import annotations

import datetime as dt
import json

from ..config import ROOT, RobotConfig, config_hash
from .build import evaluate
from .checks import build_tree
from .framework import ICON, to_json, to_markdown


def run_qa(cfg: RobotConfig, fast: bool = False, log=print) -> bool:
    root = build_tree(cfg)
    ctx = {"cfg": cfg, "root": root, "fast": fast}
    log(f"▸ {root.name}  (robot.yaml {config_hash()})")
    design = root.run(ctx, log)
    build = evaluate(cfg)
    log(f"▸ {build.name}")
    bstat = build.run(ctx, log)
    now = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    hdr = (f"# HX-1 QA report\n\nGenerated {now} by `python -m hx qa` for robot.yaml `{config_hash()}`.\n\n"
           "Status of a node = worst of its own checks and all children (recursive roll-up). "
           "Design QA gates the repository; the build QA tracks the physical build from "
           "`qa/build_log.yaml`.\n\n# Design QA")
    md = to_markdown(root, hdr) + "\n" + to_markdown(build, "# Physical build QA")
    (ROOT / "qa").mkdir(exist_ok=True)
    (ROOT / "qa" / "report.md").write_text(md)
    (ROOT / "qa" / "report.json").write_text(json.dumps({"config": config_hash(), "design": to_json(root),
                                                          "build": to_json(build)}, indent=1, default=str))
    n = list(root.all_checks())
    counts = {s: sum(1 for c in n if c.result.status == s) for s in ("PASS", "WARN", "FAIL")}
    log(f"\nDesign QA: {ICON[design]} {design}  ({counts['PASS']} pass, {counts['WARN']} warn, {counts['FAIL']} fail)"
        f"   Build QA: {ICON[bstat]} {bstat}   -> qa/report.md")
    return design in ("PASS", "WARN") and bstat != "FAIL"
