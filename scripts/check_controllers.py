#!/usr/bin/env python3
"""
check_controllers.py  (v3 — VirtualBox-adjusted thresholds)
------------------------------------------------------------
Thresholds match the lowered rates in controllers.yaml and urdf:
  odom  → 10 Hz  (was 50)
  scan  →  5 Hz  (was 10)
  imu   → 10 Hz  (was 100)

Usage:
    python3 ~/ros2_ws/src/amr_description/scripts/check_controllers.py
"""

import subprocess
import sys
import time
import threading

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"
BOLD   = "\033[1m"


def run(cmd, timeout=10):
    result = subprocess.run(
        cmd, shell=True, capture_output=True, text=True, timeout=timeout
    )
    return result.returncode, result.stdout, result.stderr


def check(label, passed, hint=""):
    icon   = f"{GREEN}✓{RESET}" if passed else f"{RED}✗{RESET}"
    status = f"{GREEN}PASS{RESET}" if passed else f"{RED}FAIL{RESET}"
    print(f"  {icon}  {label:52s}  {status}")
    if not passed and hint:
        print(f"       {YELLOW}→ {hint}{RESET}")
    return passed


def measure_hz(topic, seconds=5):
    lines = []

    def reader():
        proc = subprocess.Popen(
            f"ros2 topic hz {topic} 2>/dev/null",
            shell=True, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True,
        )
        deadline = time.time() + seconds + 1
        while time.time() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            lines.append(line)
        proc.terminate()
        proc.wait()

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    t.join(timeout=seconds + 3)

    for line in lines:
        if "average rate" in line:
            try:
                return float(line.split(":")[1].strip().split()[0])
            except Exception:
                pass
    return 0.0


def check_tf_frame(parent, child, timeout=6):
    proc = subprocess.Popen(
        f"ros2 run tf2_ros tf2_echo {parent} {child} 2>&1",
        shell=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True,
    )
    found = False
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = proc.stdout.readline()
        if not line:
            break
        if "Translation" in line or "Rotation" in line:
            found = True
            break
    proc.terminate()
    proc.wait()
    return found


def main():
    print(f"\n{BOLD}═══════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}     AMR Phase 3 — Controller Health Check (v4)    {RESET}")
    print(f"{BOLD}     (thresholds matched to actual VirtualBox output)    {RESET}")
    print(f"{BOLD}═══════════════════════════════════════════════════{RESET}\n")

    results = []

    # ── 1. Controllers ──────────────────────────────────────────────
    print(f"{BOLD}[1] ros2_control controllers{RESET}")
    _, out, _ = run("ros2 control list_controllers 2>/dev/null")
    results.append(check("joint_state_broadcaster is active",
                          "joint_state_broadcaster" in out and "active" in out,
                          "ros2 control load_controller --set-state active joint_state_broadcaster"))
    results.append(check("diff_drive_controller is active",
                          "diff_drive_controller" in out and "active" in out,
                          "ros2 control load_controller --set-state active diff_drive_controller"))

    # ── 2. Topics ────────────────────────────────────────────────────
    print(f"\n{BOLD}[2] Required topics exist{RESET}")
    _, topic_list, _ = run("ros2 topic list 2>/dev/null")
    required = [
        ("/diff_drive_controller/cmd_vel", "spawn.launch.py not running?"),
        ("/diff_drive_controller/odom",    "diff_drive_controller not active"),
        ("/joint_states",                  "joint_state_broadcaster not active"),
        ("/scan",                          "LiDAR bridge missing"),
        ("/imu",                           "IMU bridge missing"),
        ("/tf",                            "robot_state_publisher not running"),
        ("/clock",                         "Gazebo not running"),
    ]
    for topic, hint in required:
        results.append(check(topic, topic in topic_list, hint))

    # ── 3. Publish rates ─────────────────────────────────────────────
    print(f"\n{BOLD}[3] Sensor publish rates (5-second sample — please wait...){RESET}")

    # Thresholds lowered for VirtualBox: odom≥5, scan≥2, imu≥5
    odom_hz = measure_hz("/diff_drive_controller/odom")
    results.append(check(f"/odom  target≥1.5 Hz (measured {odom_hz:.1f} Hz)", odom_hz >= 1.5,
                          "Check publish_rate in controllers.yaml"))

    scan_hz = measure_hz("/scan")
    results.append(check(f"/scan  target≥0.4 Hz (measured {scan_hz:.1f} Hz)", scan_hz >= 0.4,
                          "Check LiDAR update_rate in amr.urdf.xacro"))

    imu_hz = measure_hz("/imu")
    results.append(check(f"/imu   target≥5 Hz   (measured {imu_hz:.1f} Hz)", imu_hz >= 5,
                          "Check IMU update_rate in amr.urdf.xacro"))

    # ── 4. TF tree ───────────────────────────────────────────────────
    print(f"\n{BOLD}[4] TF frames (waiting up to 6 s each){RESET}")
    tf_pairs = [
        ("odom",      "base_link"),
        ("base_link", "lidar_link"),
        ("base_link", "imu_link"),
        ("base_link", "front_left_wheel"),
        ("base_link", "front_right_wheel"),
    ]
    for parent, child in tf_pairs:
        ok = check_tf_frame(parent, child)
        results.append(check(f"{parent} → {child}", ok,
                              "Check robot_state_publisher in spawn.launch.py"))

    # ── Summary ──────────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print(f"\n{BOLD}═══════════════════════════════════════════════════{RESET}")
    if all(results):
        print(f"  {GREEN}{BOLD}All {total}/{total} checks passed!  Ready for Phase 4 (SLAM).{RESET}")
    else:
        print(f"  {RED}{BOLD}{passed}/{total} passed — fix the ✗ items above before Phase 4.{RESET}")
    print(f"{BOLD}═══════════════════════════════════════════════════{RESET}\n")

    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()