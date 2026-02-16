"""WaveGo motion debug utilities.

This script sends HTTP GET requests to the robot web server to control servo
angles for a single leg. It targets the endpoint:
  http://<wavego_ip>/control?var=sangle&val=<servo_id>&cmd=<angle>

Leg/servo mapping follows components/control/leg.h.
"""

from __future__ import annotations

import argparse
import time
from typing import Dict, Optional

import requests


LEG_SERVO_MAP: Dict[str, Dict[str, int]] = {
    "A": {"FORE": 8, "BACK": 9, "WAVE": 10},
    "B": {"WAVE": 13, "FORE": 14, "BACK": 15},
    "C": {"FORE": 7, "BACK": 6, "WAVE": 5},
    "D": {"WAVE": 2, "FORE": 1, "BACK": 0},
}


def _clamp_angle(angle_deg: float) -> float:
    if angle_deg < 0.0:
        return 0.0
    if angle_deg > 180.0:
        return 180.0
    return float(angle_deg)


def set_servo_angle(
    wavego_ip: str,
    servo_id: int,
    angle_deg: float,
    *,
    port: int = 80,
    timeout_s: float = 2.0,
) -> None:
    """Set a single servo to an angle in degrees."""

    angle_deg = _clamp_angle(angle_deg)
    url = f"http://{wavego_ip}:{port}/control"
    params = {"var": "sangle", "val": str(int(servo_id)), "cmd": str(angle_deg)}

    resp = requests.get(url, params=params, timeout=timeout_s)
    resp.raise_for_status()


def set_leg_angles(
    wavego_ip: str,
    leg: str,
    *,
    fore: Optional[float] = None,
    back: Optional[float] = None,
    wave: Optional[float] = None,
    port: int = 80,
    timeout_s: float = 2.0,
    inter_cmd_delay_s: float = 0.03,
) -> None:
    """Set one leg's three servos (FORE/BACK/WAVE).

    Any of fore/back/wave can be omitted (None) to keep it unchanged.
    Commands are sent sequentially.
    """

    leg = leg.upper().strip()
    if leg not in LEG_SERVO_MAP:
        raise ValueError(f"Unknown leg '{leg}'. Choose from {sorted(LEG_SERVO_MAP.keys())}")

    mapping = LEG_SERVO_MAP[leg]
    plan = [("FORE", fore), ("BACK", back), ("WAVE", wave)]

    for joint, angle in plan:
        if angle is None:
            continue
        set_servo_angle(
            wavego_ip,
            mapping[joint],
            angle,
            port=port,
            timeout_s=timeout_s,
        )
        if inter_cmd_delay_s > 0:
            time.sleep(inter_cmd_delay_s)


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="WaveGo single-leg servo angle controller")
    p.add_argument("--ip", required=True, help="WaveGo IP address (e.g. 192.168.4.1)")
    p.add_argument("--port", type=int, default=80, help="HTTP port (default: 80)")
    p.add_argument("--leg", required=True, choices=sorted(LEG_SERVO_MAP.keys()), help="Leg ID")
    p.add_argument("--fore", type=float, default=None, help="FORE angle (deg)")
    p.add_argument("--back", type=float, default=None, help="BACK angle (deg)")
    p.add_argument("--wave", type=float, default=None, help="WAVE angle (deg)")
    p.add_argument("--timeout", type=float, default=2.0, help="HTTP timeout seconds")
    p.add_argument("--delay", type=float, default=0.03, help="Delay between commands seconds")
    return p


def main() -> int:
    args = _build_arg_parser().parse_args()
    set_leg_angles(
        args.ip,
        args.leg,
        fore=args.fore,
        back=args.back,
        wave=args.wave,
        port=args.port,
        timeout_s=args.timeout,
        inter_cmd_delay_s=args.delay,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
