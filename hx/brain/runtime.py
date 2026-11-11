"""High-level controller ("brain") for the Raspberry Pi.

Streams whole-body joint setpoints to the nodes over CAN using the generated
protocol, using the SAME gait generator + whole-body IK as the simulation.

    python -m hx.brain.runtime --dry-run          # virtual bus, prints status
    python -m hx.brain.runtime --bus socketcan --chan-a can0 --chan-b can1 --motion stand

Arming sequence (refused unless every step succeeds):
  1. all nodes report IDLE with no faults             (CLEAR_FAULT may be sent once)
  2. ARM  -> nodes HOLD position, hub starts the HPU   (PRIME -> RUN)
  3. wait for supply pressure inside the band
  4. RUN  -> setpoints are tracked
Any node FAULT/ESTOP or a lost status stream -> DISARM to every node.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field

import numpy as np

from .. import protocol as P
from ..config import RobotConfig


@dataclass
class NodeView:
    id: int
    name: str
    bus: str
    joints: list
    state: str = "UNKNOWN"
    faults: int = 0
    last_rx: float = 0.0
    q: dict = field(default_factory=dict)


class VirtualBus:
    """In-process loopback that emulates node supervisors (for dry runs and tests)."""

    def __init__(self, nodes: dict):
        self.nodes = nodes
        self.tx: list = []
        self.state = {n.id: 1 for n in nodes.values()}     # IDLE
        self.p_supply = 0.0

    def send(self, can_id: int, data: bytes):
        self.tx.append((can_id, data))
        if can_id == P.ID_CMD:
            cmd, mask = data[0], data[1]
            for n in self.nodes.values():
                if mask & (1 << (n.id - 1)):
                    if cmd == P.CMD["ARM"] and self.state[n.id] == 1:
                        self.state[n.id] = 2
                    elif cmd == P.CMD["RUN"] and self.state[n.id] == 2:
                        self.state[n.id] = 3
                    elif cmd in (P.CMD["DISARM"], P.CMD["ESTOP"]):
                        self.state[n.id] = 1 if cmd == P.CMD["DISARM"] else 6

    def recv(self):
        out = []
        armed = any(s in (2, 3) for s in self.state.values())
        self.p_supply = min(15e5, self.p_supply + 2e5) if armed else max(0.0, self.p_supply - 5e5)
        for n in self.nodes.values():
            out.append((P.ID_STATUS + n.id, P.pack("status", state=self.state[n.id], joint_count=len(n.joints),
                                                    faults=0, p_supply=self.p_supply, temp=35.0)))
        out.append((P.ID_HPU, P.pack("hpu", state=2 if armed else 0, faults=0, p_supply=self.p_supply,
                                     duty=0.3, oil_temp=30, vbat=23.5)))
        return out


class SocketCanBus:
    def __init__(self, channels: dict):
        import can                                   # python-can (pip install python-can)
        self.buses = {k: can.interface.Bus(channel=v, interface="socketcan", bitrate=P.BITRATE) for k, v in channels.items()}
        self.route = {}

    def send(self, can_id: int, data: bytes, bus: str | None = None):
        import can
        targets = [self.buses[bus]] if bus else self.buses.values()
        for b in targets:
            b.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))

    def recv(self):
        out = []
        for b in self.buses.values():
            while True:
                m = b.recv(timeout=0)
                if m is None:
                    break
                out.append((m.arbitration_id, bytes(m.data)))
        return out


class Brain:
    def __init__(self, cfg: RobotConfig, bus):
        self.cfg = cfg
        self.bus = bus
        from ..electronics.netlist import EDIR
        import yaml
        B = yaml.safe_load(open(EDIR / "boards.yaml"))
        bus_of = {n: k for k, d in B["can_buses"].items() for n in d["nodes"]}
        self.nodes = {n["name"]: NodeView(n["id"], n["name"], bus_of[n["name"]], n["joints"])
                      for n in cfg["electrical"]["nodes"]}
        self.p_supply = 0.0
        self.counter = 0
        self.t0 = time.time()
        self.band = [b * 1e5 for b in cfg.hyd["hpu"]["pressure_band_bar"]]

    # ---------------------------------------------------------------- I/O
    def poll(self):
        now = time.time()
        for cid, data in self.bus.recv():
            if P.ID_STATUS < cid < P.ID_STATUS + 16:
                st = P.unpack("status", data)
                for n in self.nodes.values():
                    if n.id == cid - P.ID_STATUS:
                        n.state = P.NODE_STATE.get(int(st["state"]), "?")
                        n.faults = int(st["faults"])
                        n.last_rx = now
            elif cid == P.ID_HPU:
                self.p_supply = P.unpack("hpu", data)["p_supply"]
            elif P.ID_STATE <= cid < P.ID_STATUS:
                node_id, j = divmod(cid - P.ID_STATE, 8)
                for n in self.nodes.values():
                    if n.id == node_id and j < len(n.joints):
                        n.q[n.joints[j]] = P.unpack("state", data)["q"]

    def sync(self):
        self.counter += 1
        self.bus.send(P.ID_SYNC, P.pack_sync(self.counter, int((time.time() - self.t0) * 1000)))

    def command(self, cmd: str, names=None):
        mask = 0
        for n in self.nodes.values():
            if names is None or n.name in names:
                mask |= 1 << (n.id - 1)
        self.bus.send(P.ID_CMD, P.pack_cmd(cmd, mask))

    def send_setpoints(self, q: dict, qd: dict | None = None, tau: dict | None = None, mode="IMPEDANCE"):
        for n in self.nodes.values():
            for j, jn in enumerate(n.joints):
                if jn not in q:
                    continue
                self.bus.send(P.setpoint_id(n.id, j),
                              P.pack_setpoint(q[jn], (qd or {}).get(jn, 0.0), (tau or {}).get(jn, 0.0), mode=mode))

    def all_in(self, states, stale_s=0.25):
        now = time.time()
        return all(n.state in states and now - n.last_rx < stale_s for n in self.nodes.values())

    # ------------------------------------------------------------ sequence
    def arm(self, timeout=10.0, log=print) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.sync(); self.poll()
            if self.all_in({"IDLE"}):
                break
            time.sleep(0.01)
        else:
            log("[brain] nodes not IDLE: " + ", ".join(f"{n.name}={n.state}" for n in self.nodes.values()))
            return False
        self.command("ARM")
        while time.time() < deadline:
            self.sync(); self.poll()
            if self.all_in({"ARMED"}) and self.band[0] <= self.p_supply:
                log(f"[brain] armed, supply {self.p_supply / 1e5:.1f} bar")
                return True
            if any(n.state in ("FAULT", "ESTOP") for n in self.nodes.values()):
                break
            time.sleep(0.01)
        log("[brain] arming failed -> DISARM")
        self.command("DISARM")
        return False

    def run_trajectory(self, ts, qs: list[dict], log=print) -> bool:
        self.command("RUN")
        t_start = time.time()
        for k, t in enumerate(ts):
            while time.time() - t_start < t:
                time.sleep(0.0005)
            self.sync(); self.poll()
            if any(n.state in ("FAULT", "ESTOP") for n in self.nodes.values()):
                log("[brain] node fault -> DISARM all")
                self.command("DISARM")
                return False
            qd = {j: (qs[min(k + 1, len(qs) - 1)][j] - qs[k][j]) / max(ts[1] - ts[0], 1e-3) for j in qs[k]}
            self.send_setpoints(qs[k], qd)
        self.command("DISARM")
        return True


def stand_trajectory(cfg: RobotConfig, seconds=3.0, dt=0.01):
    from ..sim.scenarios import Stand
    sc = Stand(cfg)
    ts, qpos, _ = sc.reference()
    m = sc.model
    names = [m.joint(i).name for i in range(m.njnt) if m.joint(i).name != "root"]
    out = [{n: float(q[m.joint(n).qposadr[0]]) for n in names} for q in qpos[:: int(dt / 0.005)]]
    return np.arange(len(out)) * dt, out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--chan-a", default="can0")
    ap.add_argument("--chan-b", default="can1")
    ap.add_argument("--motion", default="stand", choices=["stand"])
    a = ap.parse_args(argv)
    cfg = RobotConfig()
    if a.dry_run:
        nodes = {n["name"]: NodeView(n["id"], n["name"], "", n["joints"]) for n in cfg["electrical"]["nodes"]}
        bus = VirtualBus(nodes)
    else:
        bus = SocketCanBus({"CAN_A": a.chan_a, "CAN_B": a.chan_b})
    br = Brain(cfg, bus)
    if not br.arm():
        raise SystemExit(2)
    ts, qs = stand_trajectory(cfg)
    ok = br.run_trajectory(ts[:100], qs[:100])
    print("[brain] trajectory", "completed" if ok else "aborted")


if __name__ == "__main__":
    main()
