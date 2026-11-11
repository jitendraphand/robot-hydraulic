"""Electronics: netlist expansion, ERC, power budget, harness checks, docs + pins.h.

Board pin maps (electronics/boards.yaml) and the component library
(electronics/parts.yaml) are expanded into explicit component/pin/net lists.
The ERC then checks connectivity, voltage compatibility, ESP32-S3 pin rules,
driver conflicts and decoupling.  Results feed the QA tree.
"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from ..config import ROOT, RobotConfig, save_derived

EDIR = ROOT / "electronics"
GEN = EDIR / "generated"

# --------------------------------------------------------------- ESP32-S3
ESP_FORBIDDEN = {26: "SPI flash", 27: "SPI flash", 28: "SPI flash", 29: "SPI flash", 30: "SPI flash",
                 31: "SPI flash", 32: "SPI flash", 19: "USB D-", 20: "USB D+", 43: "UART0 TX (console)",
                 44: "UART0 RX (console)"}
ESP_STRAPPING = {0: "BOOT strap", 3: "JTAG strap", 45: "VDD_SPI strap", 46: "ROM log strap"}
ESP_RGB_LED = {38: "DevKitC-1 v1.1 RGB LED", 48: "DevKitC-1 v1.0 RGB LED"}
ESP_ADC1 = set(range(1, 11))
ESP_VALID = set(range(0, 22)) | set(range(26, 49))
LEDC_CHANNELS = 8


def esp32s3_part():
    pins = {f"GPIO{g}": {"kind": "io", "vmax": 3.6} for g in sorted(ESP_VALID)}
    pins.update({"5V": {"kind": "pwr_in", "vmin": 4.75, "vmax": 5.5}, "3V3": {"kind": "pwr_out", "v": 3.3},
                 "GND": {"kind": "gnd"}})
    return {"desc": "ESP32-S3-DevKitC-1 (WROOM-1-N8)", "ic": True, "pins": pins}


@dataclass
class Netlist:
    board: str
    parts: dict = field(default_factory=dict)       # ref -> part type
    lib: dict = field(default_factory=dict)
    nets: dict = field(default_factory=lambda: defaultdict(list))
    roles: dict = field(default_factory=dict)        # net -> role (adc, pwm, ...)
    pinmap: dict = field(default_factory=dict)       # function -> gpio

    def add(self, ref, ptype):
        self.parts[ref] = ptype

    def connect(self, net, *pins):
        for p in pins:
            self.nets[net].append(p)

    def pin_info(self, pin):
        ref, name = pin.split(".", 1)
        part = self.lib[self.parts[ref]]
        return part["pins"].get(name)


def _lib():
    lib = yaml.safe_load(open(EDIR / "parts.yaml"))
    lib["ESP32S3"] = esp32s3_part()
    return lib


def build_limb(b: dict) -> Netlist:
    n = Netlist("limb", lib=_lib())
    P = b["pins"]
    n.pinmap = P
    for ref, t in (("U1", "ESP32S3"), ("U2", "SN65HVD230"), ("U3", "MCP3208"), ("U4", "MCP3208"),
                   ("J1", "POWER_IN"), ("J2", "JST_CAN"), ("J3", "JST_CAN"), ("JP1", "TERM_120"),
                   ("OK1", "OPTO_ESTOP"), ("R1", "RES_PULLUP"), ("D1", "LED"),
                   ("C1", "CAP"), ("C2", "CAP"), ("C3", "CAP"), ("C4", "CAP")):
        n.add(ref, t)
    g = lambda x: f"U1.GPIO{x}"
    n.connect("+5V", "J1.V5", "U1.5V")
    n.connect("+3V3", "U1.3V3", "U2.VCC", "U3.VDD", "U3.VREF", "U4.VDD", "U4.VREF", "R1.A", "C1.A", "C2.A", "C3.A", "C4.A")
    n.connect("GND", "J1.GND", "U1.GND", "U2.GND", "U2.RS", "U3.AGND", "U3.DGND", "U4.AGND", "U4.DGND", "J2.GND", "J3.GND",
              "OK1.CATHODE", "OK1.EMIT", "D1.K", "C1.B", "C2.B", "C3.B", "C4.B")
    n.connect("CAN_TX", g(P["CAN_TX"]), "U2.D")
    n.connect("CAN_RX", g(P["CAN_RX"]), "U2.R")
    n.connect("CANH", "U2.CANH", "J2.CANH", "J3.CANH", "JP1.A")
    n.connect("CANL", "U2.CANL", "J2.CANL", "J3.CANL", "JP1.B")
    n.connect("CAN_5V", "J2.V5", "J3.V5")
    n.connect("SPI_SCLK", g(P["SPI_SCLK"]), "U3.CLK", "U4.CLK")
    n.connect("SPI_MOSI", g(P["SPI_MOSI"]), "U3.DIN", "U4.DIN")
    n.connect("SPI_MISO", g(P["SPI_MISO"]), "U3.DOUT", "U4.DOUT")
    for i, cs in enumerate(P["ADC_CS"]):
        n.connect(f"ADC_CS{i}", g(cs), f"U{3 + i}.CS")
    for i, cs in enumerate(P["ENC_CS"]):
        ref = f"ENC{i}"
        n.add(ref, "AS5048A_BOARD")
        n.connect(f"ENC_CS{i}", g(cs), f"{ref}.CSN")
        n.connect("SPI_SCLK", f"{ref}.CLK")
        n.connect("SPI_MOSI", f"{ref}.MOSI")
        n.connect("SPI_MISO", f"{ref}.MISO")
        n.connect("+3V3", f"{ref}.VDD3V")
        n.connect("GND", f"{ref}.GND")
    for i in range(b["adc_channels"] - 4):          # 12 pressure channels (2 per hydraulic joint x 6)
        ps, dv = f"PS{i}", f"DIV{i}"
        n.add(ps, "PRESSURE_SENSOR")
        n.add(dv, "DIVIDER")
        n.connect("+5V", f"{ps}.VCC")
        n.connect("GND", f"{ps}.GND", f"{dv}.GND")
        n.connect(f"P_RAW{i}", f"{ps}.OUT", f"{dv}.IN")
        n.connect(f"P_ADC{i}", f"{dv}.OUT", f"U{3 + i // 8}.CH{i % 8}")
    for i, gp in enumerate(P["PWM"]):
        ref = f"SRV{i}"
        n.add(ref, "SERVO")
        n.connect(f"PWM{i}", g(gp), f"{ref}.SIG")
        n.roles[f"PWM{i}"] = "pwm"
        n.connect("V_VALVE" if i < 5 else "V_SERVO", f"{ref}.V")
        n.connect("GND", f"{ref}.GND")
    n.connect("V_VALVE", "J1.VVALVE")
    n.connect("V_SERVO", "J1.VSERVO")
    n.connect("V_ACT_SENSE", "J1.VBAT_SENSE", "OK1.ANODE")
    n.connect("ESTOP_SENSE", "OK1.COLL", "R1.B", g(P["ESTOP_SENSE"]))
    n.connect("LED", g(P["LED"]), "D1.A")
    return n


def build_hub(b: dict) -> Netlist:
    n = Netlist("hub", lib=_lib())
    P = b["pins"]
    n.pinmap = P
    for ref, t in (("U1", "ESP32S3"), ("U2", "SN65HVD230"), ("U3", "MCP3208"), ("U4", "IMU_BNO085"),
                   ("U5", "CURRENT_SENSOR"), ("J1", "POWER_IN"), ("J2", "JST_CAN"), ("J3", "JST_CAN"),
                   ("JP1", "TERM_120"), ("OK1", "OPTO_ESTOP"), ("R1", "RES_PULLUP"), ("R2", "RES_PULLUP"),
                   ("R3", "RES_PULLUP"), ("R4", "RES_PULLUP"), ("D1", "LED"), ("ESC1", "ESC"),
                   ("Q1", "MOSFET_LOWSIDE"), ("Q2", "MOSFET_LOWSIDE"), ("SW1", "LEVEL_SWITCH"),
                   ("RT1", "NTC_DIVIDER"), ("RV1", "VBAT_DIVIDER"), ("SRV0", "SERVO"), ("SRV1", "SERVO"),
                   ("C1", "CAP"), ("C2", "CAP"), ("C3", "CAP"), ("C4", "CAP")):
        n.add(ref, t)
    g = lambda x: f"U1.GPIO{x}"
    n.connect("+5V", "J1.V5", "U1.5V")
    n.connect("+3V3", "U1.3V3", "U2.VCC", "U3.VDD", "U3.VREF", "U4.VIN", "U5.VCC", "R1.A", "R2.A", "R3.A", "R4.A",
              "RT1.VTOP", "C1.A", "C2.A", "C3.A", "C4.A")
    n.connect("GND", "J1.GND", "U1.GND", "U2.GND", "U2.RS", "U3.AGND", "U3.DGND", "U4.GND", "U5.GND", "J2.GND", "J3.GND",
              "OK1.CATHODE", "OK1.EMIT", "D1.K", "ESC1.GND", "Q1.SOURCE", "Q2.SOURCE", "SW1.B", "RT1.GND", "RV1.GND",
              "SRV0.GND", "SRV1.GND", "C1.B", "C2.B", "C3.B", "C4.B")
    n.connect("CAN_TX", g(P["CAN_TX"]), "U2.D")
    n.connect("CAN_RX", g(P["CAN_RX"]), "U2.R")
    n.connect("CANH", "U2.CANH", "J2.CANH", "J3.CANH", "JP1.A")
    n.connect("CANL", "U2.CANL", "J2.CANL", "J3.CANL", "JP1.B")
    n.connect("CAN_5V", "J2.V5", "J3.V5")
    n.connect("SPI_SCLK", g(P["SPI_SCLK"]), "U3.CLK")
    n.connect("SPI_MOSI", g(P["SPI_MOSI"]), "U3.DIN")
    n.connect("SPI_MISO", g(P["SPI_MISO"]), "U3.DOUT")
    n.connect("ADC_CS0", g(P["ADC_CS"][0]), "U3.CS")
    for i in range(3):
        ps, dv = f"PS{i}", f"DIV{i}"
        n.add(ps, "PRESSURE_SENSOR"); n.add(dv, "DIVIDER")
        n.connect("+5V", f"{ps}.VCC"); n.connect("GND", f"{ps}.GND", f"{dv}.GND")
        n.connect(f"P_RAW{i}", f"{ps}.OUT", f"{dv}.IN")
        n.connect(f"P_ADC{i}", f"{dv}.OUT", f"U3.CH{i}")
    n.connect("I2C_SDA", g(P["I2C_SDA"]), "U4.SDA", "R2.B")
    n.connect("I2C_SCL", g(P["I2C_SCL"]), "U4.SCL", "R3.B")
    n.connect("IMU_INT", g(P["IMU_INT"]), "U4.INT")
    n.connect("IMU_RST", g(P["IMU_RST"]), "U4.RST")
    n.connect("ESC_PWM", g(P["ESC_PWM"]), "ESC1.SIG"); n.roles["ESC_PWM"] = "pwm"
    n.connect("DUMP_GATE", g(P["DUMP_VALVE"]), "Q1.GATE")
    n.connect("DUMP_COIL", "Q1.DRAIN")          # to 24 V coil (off-board)
    n.connect("FAN_GATE", g(P["FAN_PWM"]), "Q2.GATE"); n.roles["FAN_GATE"] = "pwm"
    n.connect("FAN_COIL", "Q2.DRAIN")
    for i, gp in enumerate(P["NECK_PWM"]):
        n.connect(f"NECK_PWM{i}", g(gp), f"SRV{i}.SIG"); n.roles[f"NECK_PWM{i}"] = "pwm"
        n.connect("V_SERVO", f"SRV{i}.V")
    n.connect("V_SERVO", "J1.VSERVO")
    n.connect("VBAT", "J1.VBAT_SENSE", "RV1.IN", "OK1.ANODE")
    n.connect("VBAT_ADC", "RV1.OUT", g(P["VBAT_ADC"])); n.roles["VBAT_ADC"] = "adc"
    n.connect("IBAT_ADC", "U5.OUT", g(P["IBAT_ADC"])); n.roles["IBAT_ADC"] = "adc"
    n.connect("NTC_ADC", "RT1.OUT", g(P["NTC_ADC"])); n.roles["NTC_ADC"] = "adc"
    n.connect("LEVEL_SW", "SW1.A", "R4.B", g(P["LEVEL_SW"]))
    n.connect("ESTOP_SENSE", "OK1.COLL", "R1.B", g(P["ESTOP_SENSE"]))
    n.connect("LED", g(P["LED"]), "D1.A")
    return n


# --------------------------------------------------------------------- ERC
def net_voltage(n: Netlist) -> dict:
    """Max voltage on every net from sources, propagated through dividers."""
    v = {}
    pull = {}
    for net, pins in n.nets.items():
        for p in pins:
            info = n.pin_info(p) or {}
            if info.get("kind") in ("pwr_out", "out"):
                v[net] = max(v.get(net, 0.0), info["v"])
            if info.get("kind") == "gnd":
                v[net] = max(v.get(net, 0.0), 0.0)
    for _ in range(3):
        for ref, t in n.parts.items():
            part = n.lib[t]
            if "ratio" in part:
                ni = next((k for k, ps in n.nets.items() if f"{ref}.IN" in ps), None)
                no = next((k for k, ps in n.nets.items() if f"{ref}.OUT" in ps), None)
                if ni in v and no:
                    v[no] = max(v.get(no, 0.0), v[ni] * part["ratio"])
            if t in ("RES_PULLUP",):
                na = next((k for k, ps in n.nets.items() if f"{ref}.A" in ps), None)
                nb = next((k for k, ps in n.nets.items() if f"{ref}.B" in ps), None)
                if na in v and nb:
                    pull[nb] = v[na]
                    v[nb] = max(v.get(nb, 0.0), v[na])
            if t == "NTC_DIVIDER":
                na = next((k for k, ps in n.nets.items() if f"{ref}.VTOP" in ps), None)
                no = next((k for k, ps in n.nets.items() if f"{ref}.OUT" in ps), None)
                if na in v and no:
                    v[no] = max(v.get(no, 0.0), v[na])
    # external off-board loads on known rails
    for net, val in (("V_VALVE", 6.0), ("V_SERVO", 7.4), ("VBAT", 25.2), ("V_ACT_SENSE", 25.2),
                     ("DUMP_COIL", 25.2), ("FAN_COIL", 25.2), ("CAN_5V", 5.0), ("CANH", 5.0), ("CANL", 5.0)):
        if net in n.nets:
            v[net] = max(v.get(net, 0.0), val)
    return v


def erc(n: Netlist) -> list[dict]:
    issues = []

    def add(sev, msg):
        issues.append({"board": n.board, "severity": sev, "msg": msg})

    used = defaultdict(list)
    for net, pins in n.nets.items():
        if len(pins) < 2 and net not in ("DUMP_COIL", "FAN_COIL"):
            add("error", f"net {net} has a single connection {pins}")
        for p in pins:
            used[p].append(net)
            if n.pin_info(p) is None:
                add("error", f"unknown pin {p} on net {net}")
    for p, nets in used.items():
        if len(set(nets)) > 1:
            add("error", f"pin {p} on multiple nets {sorted(set(nets))}")
    V = net_voltage(n)
    for net, pins in n.nets.items():
        drivers = []
        for p in pins:
            info = n.pin_info(p) or {}
            k = info.get("kind")
            vn = V.get(net)
            if k == "pwr_in" and vn is not None and not (info["vmin"] <= vn <= info["vmax"]):
                add("error", f"{p} supply {vn} V outside {info['vmin']}-{info['vmax']} V (net {net})")
            if k in ("in", "io", "analog_in", "od") and vn is not None and vn > info.get("vmax", 99):
                add("error", f"{p} sees {vn:.2f} V > abs max {info['vmax']} V (net {net})")
            if k in ("out",):
                drivers.append(p)
            if k == "in" and "vih" in info:
                srcs = [n.pin_info(q) for q in pins if (n.pin_info(q) or {}).get("kind") in ("io", "out")]
                if srcs and max(s.get("vmax", s.get("v", 0)) for s in srcs) < info["vih"]:
                    add("error", f"{p} logic high {info['vih']} V not reached")
        if len(drivers) > 1 and net not in ("SPI_MISO",):
            add("error", f"net {net} has multiple push-pull drivers {drivers}")
    # ESP32-S3 pin rules
    gpios = {}
    for p, nets in used.items():
        if p.startswith("U1.GPIO"):
            gp = int(p[7:])
            gpios[gp] = nets[0]
            if gp not in ESP_VALID:
                add("error", f"GPIO{gp} does not exist")
            if gp in ESP_FORBIDDEN:
                add("error", f"GPIO{gp} reserved ({ESP_FORBIDDEN[gp]}) used for {nets[0]}")
            if gp in ESP_STRAPPING:
                add("warning", f"GPIO{gp} is a strapping pin ({ESP_STRAPPING[gp]}) used for {nets[0]}")
            if gp in ESP_RGB_LED:
                add("warning", f"GPIO{gp} drives the onboard {ESP_RGB_LED[gp]} ({nets[0]})")
            if n.roles.get(nets[0]) == "adc" and gp not in ESP_ADC1:
                add("error", f"{nets[0]} on GPIO{gp}: internal ADC requires ADC1 (GPIO1-10)")
    npwm = sum(1 for net in n.nets if n.roles.get(net) == "pwm")
    if npwm > LEDC_CHANNELS:
        add("error", f"{npwm} PWM outputs > {LEDC_CHANNELS} LEDC channels")
    # decoupling: one 100 nF per IC on its supply net (count caps on +3V3 vs ICs on +3V3)
    ics = [r for r, t in n.parts.items() if n.lib[t].get("ic") and not t.startswith("AS5048") and t != "ESP32S3"]
    caps = [r for r, t in n.parts.items() if t == "CAP"]
    if len(caps) < len(ics):
        add("error", f"{len(ics)} ICs but only {len(caps)} decoupling capacitors")
    # SPI chip selects unique
    cs = [k for k in n.nets if "_CS" in k]
    if len(set(gpios[g_] for g_ in gpios if gpios[g_] in cs)) != len(cs):
        add("error", "chip-select nets not unique")
    return issues


# ------------------------------------------------------------ power budget
def power_budget(cfg: RobotConfig, B: dict) -> dict:
    pw = B["power"]
    out = {"rails": {}, "issues": []}
    regs = {r["feeds"]: (k, r) for k, r in pw["regulators"].items()}
    batt_w_nom = batt_w_peak = 0.0
    for rail, loads in pw["loads"].items():
        nom = sum(q * a for _, q, a, _ in loads)
        peak = sum(q * p for _, q, _, p in loads) * pw["simultaneity"].get(rail, 1.0)
        v = cfg["electrical"]["rails"][rail]["v"]
        e = {"nominal_a": nom, "peak_a": peak, "v": v}
        if rail in regs:
            name, r = regs[rail]
            e.update(regulator=name, imax=r["imax"], margin=r["imax"] / max(peak, 1e-9))
            if r["imax"] < 1.2 * peak:
                out["issues"].append(f"{rail}: regulator {name} {r['imax']} A < 1.2 x peak {peak:.1f} A")
            if r["estop"] != cfg["electrical"]["rails"][rail]["estop_switched"]:
                out["issues"].append(f"{rail}: e-stop switching mismatch between robot.yaml and boards.yaml")
            batt_w_nom += nom * v / r["eff"]
            batt_w_peak += peak * v / r["eff"]
        else:
            batt_w_nom += nom * v
            batt_w_peak += peak * v
        out["rails"][rail] = e
    bt = pw["battery"]
    i_peak = batt_w_peak / bt["v_min"]
    i_nom = batt_w_nom / bt["v_nom"]
    out.update(battery_nominal_w=batt_w_nom, battery_peak_w=batt_w_peak, battery_peak_a=i_peak,
               battery_max_a=bt["capacity_ah"] * bt["c_rating"],
               runtime_min=60 * 0.8 * bt["capacity_ah"] / max(i_nom, 1e-9))
    if i_peak > 0.5 * out["battery_max_a"]:
        out["issues"].append(f"battery peak {i_peak:.0f} A > 50 % of C-rating")
    if pw["main_fuse_a"] < 1.1 * i_peak:
        out["issues"].append(f"main fuse {pw['main_fuse_a']} A < 1.1 x peak {i_peak:.0f} A")
    return out


def harness_check(B: dict) -> list[str]:
    amp = {int(k): v for k, v in B["awg_ampacity"].items()}
    issues = []
    for w in B["harness"]:
        cap = amp[w["awg"]]
        if w["i"] > 0.8 * cap:
            issues.append(f"{w['id']}: design current {w['i']} A > 80 % of AWG{w['awg']} ampacity {cap} A")
        if w["fuse"] is not None:
            if w["fuse"] > cap:
                issues.append(f"{w['id']}: fuse {w['fuse']} A does not protect AWG{w['awg']} ({cap} A)")
            if w["fuse"] < 1.25 * w["i"]:
                issues.append(f"{w['id']}: fuse {w['fuse']} A < 1.25 x design current {w['i']} A (nuisance trips)")
    return issues


def can_bus_load(cfg: RobotConfig, B: dict) -> dict:
    bits = 135   # 8-byte standard frame incl. worst-case stuffing + IFS
    rates = B["can_rates_hz"]
    nodes = {n["name"]: n for n in cfg["electrical"]["nodes"]}
    out = {}
    for bus, d in B["can_buses"].items():
        fps = rates["sync"]
        for nn in d["nodes"]:
            nj = len(nodes[nn]["joints"])
            fps += nj * (rates["setpoint"] + rates["state"]) + rates["status"]
            if nodes[nn]["kind"] == "hub":
                fps += rates["hpu"] + rates["imu"]
        out[bus] = {"frames_per_s": fps, "load": fps * bits / cfg["control"]["can_bitrate"]}
    return out


# ------------------------------------------------------------------ outputs
def pins_h(B: dict) -> str:
    L = ["// GENERATED by `python -m hx gen` from electronics/boards.yaml - do not edit.", "#pragma once",
         "#include <stdint.h>", "", "namespace hx { namespace pins {"]
    for bname, b in B["boards"].items():
        L.append(f"namespace {bname} {{")
        for k, v in b["pins"].items():
            if isinstance(v, list):
                L.append(f"constexpr uint8_t {k}[] = {{{', '.join(map(str, v))}}};")
                L.append(f"constexpr uint8_t {k}_COUNT = {len(v)};")
            else:
                L.append(f"constexpr uint8_t {k} = {v};")
        L.append(f"}}  // namespace {bname}")
    L += ["} }  // namespace hx::pins", ""]
    return "\n".join(L)


def netlist_md(n: Netlist) -> str:
    L = [f"# {n.board} board netlist", "", "Generated from `electronics/boards.yaml` + `electronics/parts.yaml`.", "",
         "## Components", "", "| ref | part | description |", "|---|---|---|"]
    for r, t in n.parts.items():
        L.append(f"| {r} | {t} | {n.lib[t]['desc']} |")
    V = net_voltage(n)
    L += ["", "## Nets", "", "| net | max V | connections |", "|---|---|---|"]
    for net, pins in sorted(n.nets.items()):
        L.append(f"| {net} | {V.get(net, '')} | {', '.join(pins)} |")
    L += ["", "## MCU pin map", "", "| function | GPIO |", "|---|---|"]
    for k, v in n.pinmap.items():
        L.append(f"| {k} | {v} |")
    return "\n".join(L) + "\n"


def wiring_svg(cfg: RobotConfig, B: dict) -> str:
    """System block diagram (power + CAN) as a standalone SVG."""
    W, H = 1100, 640
    boxes = {
        "BATTERY": (40, 40, "2x 6S LiPo 10 Ah"), "MAIN_FUSE": (40, 150, "Main fuse 60 A"),
        "ESTOP": (40, 260, "E-stop chain + contactor"), "PDB_ACT": (40, 380, "Actuator PDB (e-stop switched)"),
        "PDB_LOGIC": (300, 150, "Logic PDB (always on)"), "ESC": (40, 500, "ESC + BLDC pump"),
        "B74": (300, 380, "Buck 7.4 V 30 A (servos)"), "B6": (300, 260, "Buck 6 V 15 A (valve servos)"),
        "B5": (560, 150, "Buck 5 V 6 A (logic)"), "B5PI": (560, 40, "Buck 5.1 V 6 A (brain)"),
        "PI": (820, 40, "Brain: Raspberry Pi 5"), "LEGL": (820, 200, "Node leg_L"), "LEGR": (820, 270, "Node leg_R"),
        "ARML": (820, 380, "Node arm_L"), "ARMR": (820, 450, "Node arm_R"), "HUB": (820, 520, "Hub node (HPU/IMU)"),
        "DUMP": (300, 500, "Dump valve (NO)"),
    }
    L = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="Helvetica,Arial" font-size="12">',
         '<rect width="100%" height="100%" fill="#ffffff"/>',
         '<text x="20" y="22" font-size="16" font-weight="bold">HX-1 power &amp; communication architecture</text>']

    def box(k):
        x, y, t = boxes[k]
        return f'<rect x="{x}" y="{y}" width="220" height="44" rx="6" fill="#eef1f6" stroke="#334"/>' \
               f'<text x="{x + 110}" y="{y + 27}" text-anchor="middle">{t}</text>'

    def line(a, b, col, label="", dash=False):
        xa, ya, _ = boxes[a]
        xb, yb, _ = boxes[b]
        x1, y1 = xa + 220 if xb > xa + 100 else xa + 110, ya + 22 if xb > xa + 100 else ya + 44
        x2, y2 = xb if xb > xa + 100 else xb + 110, yb + 22 if xb > xa + 100 else yb
        d = ' stroke-dasharray="6,4"' if dash else ""
        s = f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{col}" stroke-width="2.5"{d}/>'
        if label:
            s += f'<text x="{(x1 + x2) / 2 + 4}" y="{(y1 + y2) / 2 - 4}" fill="{col}" font-size="11">{label}</text>'
        return s
    for a, b, c, l in (("BATTERY", "MAIN_FUSE", "#c22", "AWG8"), ("MAIN_FUSE", "ESTOP", "#c22", "AWG8"),
                       ("ESTOP", "PDB_ACT", "#c22", "AWG8"), ("PDB_ACT", "ESC", "#c22", "AWG12"),
                       ("MAIN_FUSE", "PDB_LOGIC", "#d70", "AWG16"), ("PDB_ACT", "B74", "#c22", "AWG14"),
                       ("PDB_LOGIC", "B6", "#d70", "AWG16"), ("PDB_LOGIC", "B5", "#d70", "AWG18"),
                       ("PDB_LOGIC", "B5PI", "#d70", "AWG18"), ("B5PI", "PI", "#d70", "5.1 V"),
                       ("PDB_ACT", "DUMP", "#c22", "24 V")):
        L.append(line(a, b, c, l))
    for n_ in ("LEGL", "LEGR", "ARML", "ARMR", "HUB"):
        L.append(line("B5", n_, "#d70", "", dash=True))
    L.append('<path d="M 1045 62 L 1075 62 L 1075 292 L 1045 292" fill="none" stroke="#06c" stroke-width="3"/>')
    L.append('<text x="1050" y="180" fill="#06c" transform="rotate(90 1050 180)">CAN_A 1 Mbit</text>')
    L.append('<path d="M 1045 70 L 1090 70 L 1090 542 L 1045 542" fill="none" stroke="#08a" stroke-width="3"/>')
    L.append('<text x="1098" y="400" fill="#08a" transform="rotate(90 1098 400)">CAN_B 1 Mbit</text>')
    for k in boxes:
        L.append(box(k))
    L.append('<text x="40" y="610" fill="#555">Red: e-stop switched actuator power. Orange: always-on logic power '
             '(nodes stay alive to centre valve spools and report). Blue: CAN buses with 120 R terminations at both ends.</text>')
    L.append("</svg>")
    return "\n".join(L)


def generate_outputs(cfg: RobotConfig, log=print) -> dict:
    B = yaml.safe_load(open(EDIR / "boards.yaml"))
    GEN.mkdir(parents=True, exist_ok=True)
    boards = {"limb": build_limb(B["boards"]["limb"]), "hub": build_hub(B["boards"]["hub"])}
    issues = []
    for k, n in boards.items():
        issues += erc(n)
        (GEN / f"{k}_netlist.md").write_text(netlist_md(n))
    pb = power_budget(cfg, B)
    hz = harness_check(B)
    bus = can_bus_load(cfg, B)
    inc = ROOT / "firmware" / "platform" / "esp32s3" / "include"
    inc.mkdir(parents=True, exist_ok=True)
    (inc / "pins.h").write_text(pins_h(B))
    (GEN / "system_wiring.svg").write_text(wiring_svg(cfg, B))
    write_docs(cfg, B, pb, hz, bus, issues)
    res = {"erc": issues, "power": pb, "harness": hz, "can": bus,
           "counts": {k: {"parts": len(n.parts), "nets": len(n.nets)} for k, n in boards.items()}}
    save_derived("electronics", res)
    nerr = sum(1 for i in issues if i["severity"] == "error")
    log(f"  [elec] ERC: {nerr} errors, {len(issues) - nerr} warnings; power issues {len(pb['issues'])}; "
        f"harness issues {len(hz)}; CAN load " + ", ".join(f"{k} {v['load'] * 100:.0f}%" for k, v in bus.items()))
    return res


def write_docs(cfg, B, pb, hz, bus, issues):
    L = ["# Power budget, harness and CAN load", "", "Generated by `python -m hx gen`.", "", "## Rails", "",
         "| rail | V | nominal A | peak A | regulator | rating A | margin |", "|---|---|---|---|---|---|---|"]
    for r, e in pb["rails"].items():
        L.append(f"| {r} | {e['v']} | {e['nominal_a']:.2f} | {e['peak_a']:.2f} | {e.get('regulator', 'battery')} | "
                 f"{e.get('imax', '-')} | {e.get('margin', float('nan')):.2f} |")
    L += ["", f"Battery: nominal {pb['battery_nominal_w']:.0f} W, peak {pb['battery_peak_w']:.0f} W "
          f"({pb['battery_peak_a']:.0f} A of {pb['battery_max_a']:.0f} A rated); estimated runtime at nominal load "
          f"**{pb['runtime_min']:.0f} min** (80 % depth of discharge).", "", "## Harness", "",
          "| id | from | to | AWG | length m | connector | fuse A | design A |", "|---|---|---|---|---|---|---|---|"]
    for w in B["harness"]:
        L.append(f"| {w['id']} | {w['from']} | {w['to']} | {w['awg']} | {w['len']} | {w['conn']} | {w['fuse'] or '-'} | {w['i']} |")
    L += ["", "## CAN bus load", "", "| bus | frames/s | load |", "|---|---|---|"]
    for k, v in bus.items():
        L.append(f"| {k} | {v['frames_per_s']} | {v['load'] * 100:.1f} % |")
    L += ["", "## ERC findings", ""]
    L += [f"- **{i['severity']}** ({i['board']}): {i['msg']}" for i in issues] or ["- none"]
    L += ["", "## Power / harness findings", ""]
    L += [f"- {x}" for x in pb["issues"] + hz] or ["- none"]
    (GEN / "power_harness.md").write_text("\n".join(L) + "\n")
