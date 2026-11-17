#!/usr/bin/env python3
"""Network preflight and optional idle Colima recovery before the UR driver starts.

Without --repair-stale-colima-network, all checks are read-only.
Do not open UR's RTDE / reverse sockets as a connectivity test: probing those
ports can disturb an existing client. Check the route and Moonraker's read-only
HTTP endpoint instead. Driver readiness is verified separately after launch.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
from pathlib import Path
import platform
import re
import signal
import subprocess
import sys
import urllib.request
from urllib.parse import urljoin, urlsplit


CONTAINER = "ur3e-ros2-real"
PORTS = (50001, 50002, 50003, 50004)


class ContainerTransportError(RuntimeError):
    """The container could not establish the read-only Moonraker connection."""


def safe_moonraker_text(value: object, limit: int) -> str:
    """Keep a useful, single-line diagnostic without echoing arbitrary response text."""
    if not isinstance(value, str):
        return ""
    # Remove terminal control sequences before filtering printable characters.
    value = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", value)
    value = "".join(char if char.isalnum() or char in " .,:;!?()'_-" else " " for char in value)
    value = " ".join(value.split())
    if len(value) > limit:
        return value[:limit - 1].rstrip() + "…"
    return value


def command(arguments: list[str], description: str, *, timeout: float = 10) -> str:
    result = subprocess.run(arguments, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        # Don't echo arbitrary Docker environment values or URL credentials.
        error = (
            ContainerTransportError
            if description == "Container connection to Moonraker /printer/info"
            and result.returncode in (7, 28)
            else RuntimeError
        )
        raise error(f"{description} failed (exit {result.returncode}).")
    return result.stdout


def preflight(robot_ip: str = "", moonraker_url: str = "") -> None:
    info = json.loads(command(["docker", "inspect", CONTAINER], "Container inspection"))[0]
    labels = info.get("Config", {}).get("Labels", {}) or {}
    if (labels.get("com.docker.compose.project") != "ur3e-real"
            or labels.get("com.docker.compose.service") != "ros2"):
        raise RuntimeError("Container does not belong to the UR printing project; refusing to use it.")
    if not info.get("State", {}).get("Running"):
        raise RuntimeError("The ROS container is not running. Check the earlier Docker startup error.")
    network = info.get("NetworkSettings", {})
    attached = network.get("Networks", {}) or {}
    if not any(endpoint.get("IPAddress") for endpoint in attached.values()):
        raise RuntimeError("The ROS container has no attached network. Restart this project's ROS container; do not press Play.")
    publications = network.get("Ports", {}) or {}
    missing = [str(port) for port in PORTS if not any(
        binding.get("HostPort") == str(port)
        for binding in (publications.get(f"{port}/tcp") or [])
    )]
    if missing:
        raise RuntimeError(f"Docker did not publish UR ports: {', '.join(missing)}. Do not press Play.")

    if robot_ip:
        robot_ip = str(ipaddress.IPv4Address(robot_ip))
        route = command(
            ["docker", "exec", "-u", "ubuntu", CONTAINER, "ip", "route", "get", robot_ip],
            "Container route to UR",
        )
        if " dev " not in route or " dev lo " in f" {route} ":
            raise RuntimeError("Container route to UR does not use an external interface.")
    if moonraker_url:
        parsed = urlsplit(moonraker_url)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise RuntimeError("Invalid Moonraker URL.")
        output = command(
            ["docker", "exec", "-u", "ubuntu", CONTAINER,
             "curl", "--noproxy", "*", "--fail", "--silent", "--show-error",
             "--connect-timeout", "3", "--max-time", "5",
             urljoin(moonraker_url.rstrip("/") + "/", "printer/info")],
            "Container connection to Moonraker /printer/info",
        )
        result = json.loads(output).get("result", {})
        if result.get("state") != "ready":
            state = safe_moonraker_text(result.get("state"), 32) or "unknown"
            message = safe_moonraker_text(result.get("state_message"), 180)
            detail = f" (state: {state}" + (f"; reason: {message}" if message else "") + ")"
            raise RuntimeError(f"Moonraker is reachable but Klipper is not ready{detail}.")
    print("Container network preflight passed (network, published ports" +
          (", UR route" if robot_ip else "") +
          (", Moonraker ready" if moonraker_url else "") + ").")


def colima_context() -> str:
    """Validate the effective Docker endpoint before touching a Colima VM."""
    if platform.system() != "Darwin":
        raise RuntimeError("Automatic VM network recovery is available only for macOS Colima.")
    if os.environ.get("COLIMA_PROFILE", "default") not in {"", "default"}:
        raise RuntimeError("Automatic network recovery requires the default Colima profile.")
    endpoint = command(
        ["docker", "context", "inspect", "--format", "{{.Endpoints.docker.Host}}"],
        "Docker endpoint inspection",
    ).strip()
    expected = "unix://" + str(Path.home() / ".colima/default/docker.sock")
    override = os.environ.get("DOCKER_HOST", "")
    if endpoint != expected or (override and override != expected):
        raise RuntimeError("Refusing network recovery: Docker is not using the default Colima endpoint.")
    context = command(["docker", "context", "show"], "Docker context inspection").strip()
    if not context or "\n" in context:
        raise RuntimeError("Cannot preserve the current Docker context; network recovery was not attempted.")
    return context


def require_idle_colima(*, allow_no_container: bool = False) -> None:
    """Shared workload guard for network recovery and runtime setup restarts."""
    running = command(
        ["docker", "ps", "--format", "{{.Names}}"], "Running container inspection",
    ).splitlines()
    if running != [CONTAINER] and not (allow_no_container and not running):
        raise RuntimeError(
            "VM network recovery requires only this project's idle ROS container; "
            "other containers must be stopped first."
        )
    if running:
        labels = command(
            ["docker", "inspect", "--format",
             '{{index .Config.Labels "com.docker.compose.project"}} {{index .Config.Labels "com.docker.compose.service"}}',
             CONTAINER], "ROS container ownership inspection",
        ).strip()
        if labels != "ur3e-real ros2":
            raise RuntimeError("Refusing to restart a VM containing an unrecognized ROS container.")
        processes = command(
            # Docker needs a PID column to map host processes to this container.
            ["docker", "top", CONTAINER, "-eo", "pid,args"], "ROS process inspection",
        )
        if re.search(r"\bros2\s+launch\s+(?:ur_robot_driver|ur_moveit_config)\b", processes) or re.search(
            r"(?<![\w-])(?:ur_ros2_control_node|move_group|klipper_bridge|"
            r"gcode_print_executor\.py|run_gcode_print_execution\.py|ur3e_cartesian_xyz_moveit)(?=\s|$)",
            processes,
        ):
            raise RuntimeError(
                "ROS control or execution processes are still running. Stop them before repairing the VM network."
            )
    host_processes = command(["ps", "-axo", "command="], "Host execution process inspection")
    if re.search(r"(?:run_gcode_print_execution|gcode_print_executor)\.py(?=\s|$)", host_processes):
        raise RuntimeError("A print executor is still running; VM network recovery was not attempted.")


def recover_colima_network(moonraker_url: str) -> None:
    """Repair a stale default-Colima network only while its workloads are idle.

    A stopped container retains the VM's broken forwarding state. Repeated
    compose up/stop cannot repair it. Never restart an active control stack or
    another project's containers as a side effect of this read-only preflight.
    """
    context = colima_context()

    url = urljoin(moonraker_url.rstrip("/") + "/", "printer/info")
    # Bypass proxies exactly as the in-container curl does. A real device fault
    # or an unreachable host must remain a failed preflight, not a VM restart.
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(url, timeout=5) as response:
            host_info = json.loads(response.read(256_001))
        if host_info.get("result", {}).get("state") != "ready":
            raise ValueError("Klipper is not ready")
    except (OSError, ValueError, AttributeError) as exc:
        raise RuntimeError(
            "Mac cannot verify that Klipper is ready; VM network recovery was not attempted."
        ) from exc

    vm = subprocess.run(
        ["colima", "ssh", "--", "curl", "--noproxy", "*", "--silent", "--show-error",
         "--connect-timeout", "3", "--max-time", "5", "--output", "/dev/null",
         "--write-out", "%{exitcode}", url],
        capture_output=True, text=True, timeout=12,
    )
    if vm.returncode == 0 or vm.stdout.strip() not in {"7", "28"}:
        raise RuntimeError(
            "The Colima VM transport failure was not confirmed; automatic restart was not attempted."
        )

    require_idle_colima()

    print(
        "Mac can reach Klipper but Colima cannot. No control workloads are running; "
        "repairing the Colima network once…", flush=True,
    )
    # Preserve the existing profile, mounts and Docker context. Do not recurse
    # into the launch script or clear any execution/stop evidence.
    try:
        command(["colima", "stop"], "Colima network recovery stop", timeout=60)
        command(["colima", "start"], "Colima network recovery start", timeout=120)
    finally:
        # Colima stop may select Docker's default context, and saved autoActivate
        # settings can prevent start from restoring it. Also restore on failure:
        # context use is local metadata and does not need a running Docker VM.
        command(["docker", "context", "use", context], "Docker context restoration")
    command(["docker", "start", CONTAINER], "ROS container restart", timeout=30)


def ensure_preflight(
    robot_ip: str, moonraker_url: str, *, repair_stale_colima: bool = False,
) -> None:
    try:
        preflight(robot_ip, moonraker_url)
    except ContainerTransportError as original:
        if not repair_stale_colima:
            raise
        try:
            recover_colima_network(moonraker_url)
        except RuntimeError as exc:
            raise RuntimeError(f"{original} {exc}") from exc
        # Exactly one recovery attempt. Every original check still has to pass.
        preflight(robot_ip, moonraker_url)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="")
    parser.add_argument("--moonraker-url", default="")
    parser.add_argument("--repair-stale-colima-network", action="store_true")
    parser.add_argument("--check-idle-colima", action="store_true")
    args = parser.parse_args()
    try:
        if args.check_idle_colima:
            colima_context()
            require_idle_colima(allow_no_container=True)
            print("Colima workload inspection passed; no control processes are running.")
            return 0
        ensure_preflight(
            args.robot_ip, args.moonraker_url,
            repair_stale_colima=args.repair_stale_colima_network,
        )
    except (RuntimeError, OSError, subprocess.TimeoutExpired, ValueError, KeyError, IndexError, TypeError) as exc:
        print(f"ERROR: ROS container network preflight: {exc}", file=sys.stderr)
        return 69
    return 0


if __name__ == "__main__":
    # A GUI cancellation signals the whole launcher process group. Unwind the
    # recovery finally block so even an interrupted VM start restores context.
    def terminate_cleanly(signum: int, _frame: object) -> None:
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, terminate_cleanly)
    signal.signal(signal.SIGINT, terminate_cleanly)
    raise SystemExit(main())
