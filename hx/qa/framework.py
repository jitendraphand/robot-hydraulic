"""Recursive QA framework.

A QA *node* owns checks and child nodes.  A node's status is the worst of its
own checks and all of its children (recursively), so a subsystem can only
pass when every component beneath it passes.  Checks carry requirement IDs;
traceability (every requirement verified by >= 1 check) is checked too.
"""
from __future__ import annotations

import time
import traceback
from dataclasses import dataclass, field
from typing import Callable

ORDER = {"PASS": 0, "SKIP": 1, "PENDING": 2, "WARN": 3, "FAIL": 4}
ICON = {"PASS": "✅", "SKIP": "⏭️", "PENDING": "⏳", "WARN": "⚠️", "FAIL": "❌"}


@dataclass
class Result:
    status: str
    detail: str = ""
    metrics: dict = field(default_factory=dict)


def ok(detail="", **m):
    return Result("PASS", detail, m)


def warn(detail="", **m):
    return Result("WARN", detail, m)


def fail(detail="", **m):
    return Result("FAIL", detail, m)


def check_bool(cond: bool, detail_ok: str, detail_bad: str, **m) -> Result:
    return Result("PASS" if cond else "FAIL", detail_ok if cond else detail_bad, m)


@dataclass
class Check:
    id: str
    title: str
    fn: Callable[[dict], Result]
    reqs: list[str] = field(default_factory=list)
    result: Result | None = None
    seconds: float = 0.0


@dataclass
class Node:
    name: str
    checks: list[Check] = field(default_factory=list)
    children: list["Node"] = field(default_factory=list)
    status: str = "SKIP"

    def add(self, *items):
        for it in items:
            (self.children if isinstance(it, Node) else self.checks).append(it)
        return self

    def run(self, ctx: dict, log=print, depth: int = 0) -> str:
        worst = "PASS"
        for c in self.checks:
            t0 = time.time()
            try:
                c.result = c.fn(ctx)
            except Exception as e:      # a crashing check is a failing check
                c.result = fail(f"exception: {e!r}", trace=traceback.format_exc()[-800:])
            c.seconds = time.time() - t0
            if ORDER[c.result.status] > ORDER[worst]:
                worst = c.result.status
            log(f"{'  ' * (depth + 1)}{ICON[c.result.status]} [{c.id}] {c.title}: {c.result.detail}")
        for ch in self.children:
            log(f"{'  ' * (depth + 1)}▸ {ch.name}")
            s = ch.run(ctx, log, depth + 1)
            if ORDER[s] > ORDER[worst]:
                worst = s
        self.status = worst if (self.checks or self.children) else "SKIP"
        return self.status

    def walk(self):
        yield self
        for ch in self.children:
            yield from ch.walk()

    def all_checks(self):
        for n in self.walk():
            yield from n.checks


def to_markdown(root: Node, header: str = "") -> str:
    L = [header, "", f"**Overall: {ICON[root.status]} {root.status}**", "", "## Tree", ""]

    def rec(n: Node, d: int):
        L.append(f"{'  ' * d}- {ICON[n.status]} **{n.name}**")
        for c in n.checks:
            r = c.result
            reqs = f" `{', '.join(c.reqs)}`" if c.reqs else ""
            L.append(f"{'  ' * (d + 1)}- {ICON[r.status]} `{c.id}` {c.title}{reqs} — {r.detail}")
        for ch in n.children:
            rec(ch, d + 1)
    rec(root, 0)
    return "\n".join(L) + "\n"


def to_json(root: Node) -> dict:
    def rec(n: Node):
        return {"name": n.name, "status": n.status,
                "checks": [{"id": c.id, "title": c.title, "reqs": c.reqs, "status": c.result.status,
                            "detail": c.result.detail, "seconds": round(c.seconds, 3),
                            "metrics": {k: v for k, v in c.result.metrics.items() if k != "trace"}}
                           for c in n.checks],
                "children": [rec(c) for c in n.children]}
    return rec(root)
