#!/usr/bin/env python3
"""Randomized robustness study: many runs, each with a different red obstacle.

Every run starts a FRESH headless Gazebo + the full stack with one random red box
(size, distance, sideways offset) and one of several cruise speeds. The expected
outcome is decided from geometry BEFORE the run:

  in_path  box overlaps the robot's body           -> robot must STOP, without contact
  clear    box is well beside the swept path        -> robot must PASS without stopping
  edge     box near the path edge (ambiguous zone)  -> either is acceptable, reported only

During each approach the monocular gap estimate is also compared with the true gap
from odometry, giving perception accuracy across box sizes and distances.

  ros2 run vision_safety_bringup robustness_study.py                # 30 runs, ~20 min
  ros2 run vision_safety_bringup robustness_study.py --runs 5       # quick check
  ros2 run vision_safety_bringup robustness_study.py --seed 7 --out ~/robustness

Outputs (default /tmp/vision_safety_robustness): runs.csv, summary.md, summary.json,
stop_gaps.png, perception_error.png, and one log file per run.
"""

import argparse
import csv
import json
import math
import os
import random
import re
import signal
import statistics
import subprocess
import sys
import time

# ----------------------------------------------------------------------------
# Geometry of the robot, world and configuration (must match the repo's files)
# ----------------------------------------------------------------------------
ROBOT_FRONT_X = 0.15      # front edge ahead of base_footprint (vision_bot.urdf.xacro)
ROBOT_HALF_WIDTH = 0.14   # half width including wheels
PATH_HALF_WIDTH = 0.24    # zone.path_half_width_m (obstacle_detector.yaml)
STOP_GAP = 0.60           # zone.stop_gap_m
CORRIDOR_INNER = 0.95     # usable half width between the walls (walls at +-1.0)

IN_PATH_MAX_INNER = 0.10  # box inner edge closer than this: clearly in the path
CLEAR_MIN_INNER = 0.33    # inner edge farther than this: clearly beside the path

# Distractors already in corridor.sdf that random boxes must not overlap
BLUE_BOX = (1.85, 2.15, -0.90, -0.60)   # x_min, x_max, y_min, y_max

SAFE_FINAL_GAP = 0.20     # an in_path stop closer than this counts as unsafe
CONTACT_GAP = 0.02


# ----------------------------------------------------------------------------
# Pure functions (unit-tested in test/test_robustness_logic.py)
# ----------------------------------------------------------------------------
def inner_edge(lateral, size):
    """Smallest |lateral| of any point of the box: 0 if it straddles the centre line."""
    return max(0.0, abs(lateral) - size / 2.0)


def classify(lateral, size):
    edge = inner_edge(lateral, size)
    if edge <= IN_PATH_MAX_INNER:
        return 'in_path'
    if edge >= CLEAR_MIN_INNER:
        return 'clear'
    return 'edge'


def _overlaps_blue_box(x_min, x_max, y_min, y_max, margin=0.10):
    bx0, bx1, by0, by1 = BLUE_BOX
    return not (x_max + margin < bx0 or x_min - margin > bx1 or
                y_max + margin < by0 or y_min - margin > by1)


def sample_scenario(rng, speeds, weights=(0.5, 0.3, 0.2)):
    """Random box and speed. Category is chosen first so the mix is controlled."""
    while True:
        category = rng.choices(['in_path', 'clear', 'edge'], weights=weights)[0]
        size = round(rng.uniform(0.10, 0.40), 3)
        half = size / 2.0
        initial_gap = round(rng.uniform(1.5, 3.5), 3)   # robot front -> box front face
        side = rng.choice([-1.0, 1.0])
        if category == 'in_path':
            lateral = rng.uniform(-(IN_PATH_MAX_INNER + half), IN_PATH_MAX_INNER + half)
        elif category == 'clear':
            hi = CORRIDOR_INNER - size
            if hi < CLEAR_MIN_INNER:
                continue
            lateral = side * (half + rng.uniform(CLEAR_MIN_INNER, hi))
        else:
            lateral = side * (half + rng.uniform(IN_PATH_MAX_INNER + 0.01, CLEAR_MIN_INNER - 0.01))
        lateral = round(lateral, 3)
        if abs(lateral) + half > CORRIDOR_INNER:
            continue
        front_x = ROBOT_FRONT_X + initial_gap
        if _overlaps_blue_box(front_x, front_x + size, lateral - half, lateral + half):
            continue
        if classify(lateral, size) != category:   # rounding pushed it across a boundary
            continue
        return {
            'category': category,
            'size_m': size,
            'lateral_m': lateral,
            'inner_edge_m': round(inner_edge(lateral, size), 3),
            'initial_gap_m': initial_gap,
            'box_front_x': round(front_x, 4),
            'box_center_x': round(front_x + half, 4),
            'speed_mps': rng.choice(list(speeds)),
        }


def make_world(template_sdf, scenario):
    """corridor.sdf with the fixed red box replaced by the scenario's box."""
    pattern = re.compile(r'\s*<model name="red_obstacle">.*?</model>', re.S)
    if not pattern.search(template_sdf):
        raise ValueError('template world has no red_obstacle model')
    s = scenario['size_m']
    box = f'''
    <model name="red_obstacle">
      <static>true</static>
      <pose>{scenario['box_center_x']} {scenario['lateral_m']} {s / 2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{s} {s} {s}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{s} {s} {s}</size></box></geometry>
          <material>
            <ambient>0.8 0.0 0.0 1</ambient>
            <diffuse>0.9 0.0 0.0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>'''
    return pattern.sub(box, template_sdf, count=1)


def true_gap(box_front_x, robot_x):
    return box_front_x - (robot_x + ROBOT_FRONT_X)


# SafetyStatus state values (kept here so the pure functions need no ROS import)
STATE_NAMES = {0: 'INIT', 1: 'CLEAR', 2: 'STOP_OBSTACLE', 3: 'STOP_STALE'}


def diagnose(c):
    """Name the first broken link in clock -> detector -> safety -> robot.

    c: counters from a run that did not finish: clock, detections, status,
    last_state, cmd_raw, cmd_out, last_cmd_linear, moved_m.
    """
    if c['clock'] == 0:
        return 'no /clock: simulation time never reached ROS (bridge?)'
    if c['detections'] == 0:
        return 'no /perception/obstacle messages'
    if c['status'] == 0:
        return 'safety_monitor silent: no /safety/status (node down or its timer never fired)'
    state = STATE_NAMES.get(c['last_state'], str(c['last_state']))
    if c['last_state'] == 0:
        return 'safety_monitor stuck in INIT: it never received a detection'
    if c['last_state'] == 3:
        return 'safety_monitor in STOP_STALE: perception too old when it arrived'
    if c['cmd_raw'] == 0:
        return f'no cmd_vel_raw: cruise_driver not publishing (state {state})'
    if c['cmd_out'] == 0 or c['last_cmd_linear'] == 0.0:
        return f'safety_monitor commanding zero (state {state})'
    if c['moved_m'] < 0.05:
        return 'non-zero cmd_vel but the robot did not move (bridge or DiffDrive)'
    return f'robot moving ({c["moved_m"]:.2f} m) but the scenario did not finish in time'


def find_gz_servers(proc_root='/proc'):
    """All running 'gz sim' processes we can see: list of (pid, GZ_PARTITION, cmdline)."""
    found = []
    for name in os.listdir(proc_root):
        if not name.isdigit():
            continue
        base = os.path.join(proc_root, name)
        try:
            with open(os.path.join(base, 'cmdline'), 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
            if 'gz sim' not in cmd:
                continue
            with open(os.path.join(base, 'environ'), 'rb') as f:
                env = dict(e.split('=', 1) for e in f.read().decode(errors='replace').split('\0')
                           if '=' in e)
        except OSError:
            continue  # exited meanwhile, or not ours to read
        found.append((int(name), env.get('GZ_PARTITION', ''), cmd))
    return found


def is_test_leftover(partition):
    """Gazebo partitions created by this study or by the scenario tests."""
    return partition.startswith('vs_robust_') or partition == 'vision_safety_test'


def kill_test_leftovers(proc_root='/proc', kill=os.kill):
    """SIGKILL orphaned Gazebo servers from earlier test runs. Returns their pids."""
    killed = []
    for pid, partition, _cmd in find_gz_servers(proc_root):
        if is_test_leftover(partition):
            try:
                kill(pid, signal.SIGKILL)
                killed.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
    return killed


def evaluate(scenario, outcome, final_gap, stopped_ever):
    """Did the robot do the right thing? Returns (passed, note)."""
    cat = scenario['category']
    if outcome in ('no_perception', 'no_clock', 'timeout', 'launch_failed', 'launch_exited'):
        return None, f'infrastructure: {outcome}'
    if cat == 'in_path':
        if outcome != 'stopped':
            return False, 'did not stop for an obstacle in its path'
        if final_gap is not None and final_gap < CONTACT_GAP:
            return False, 'contact with the obstacle'
        if final_gap is not None and final_gap < SAFE_FINAL_GAP:
            return False, f'stopped too close ({final_gap:.2f} m)'
        return True, 'stopped safely'
    if cat == 'clear':
        if stopped_ever or outcome != 'passed':
            return False, 'false stop for an obstacle beside the path'
        return True, 'passed without stopping'
    # edge: physically clear of the robot body, so either decision is acceptable
    return True, 'stopped (edge)' if outcome == 'stopped' else 'passed (edge)'


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    k = (len(v) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def summarize(runs, errors):
    """Aggregate per-run rows and perception error samples into a summary dict."""
    judged = [r for r in runs if r['passed'] in (True, False)]
    by_cat = {}
    for cat in ('in_path', 'clear', 'edge'):
        rows = [r for r in judged if r['category'] == cat]
        by_cat[cat] = {
            'runs': len(rows),
            'correct': sum(1 for r in rows if r['passed']),
            'stopped': sum(1 for r in rows if r['outcome'] == 'stopped'),
        }
    by_speed = {}
    for r in judged:
        if r['category'] == 'in_path' and r['outcome'] == 'stopped' and r['final_gap_m'] is not None:
            by_speed.setdefault(r['speed_mps'], []).append(r)
    speed_stats = {}
    for v, rows in sorted(by_speed.items()):
        gaps = [r['final_gap_m'] for r in rows]
        travel = [r['travel_after_trigger_m'] for r in rows if r['travel_after_trigger_m'] is not None]
        speed_stats[str(v)] = {
            'stops': len(rows),
            'final_gap_min': round(min(gaps), 3),
            'final_gap_median': round(statistics.median(gaps), 3),
            'final_gap_max': round(max(gaps), 3),
            'travel_after_trigger_median': round(statistics.median(travel), 3) if travel else None,
        }
    abs_err = [abs(e['error_m']) for e in errors]
    bins = [(0.3, 0.8), (0.8, 1.5), (1.5, 3.0)]
    err_bins = {}
    for lo, hi in bins:
        sel = [abs(e['error_m']) for e in errors if lo <= e['true_gap_m'] < hi]
        err_bins[f'{lo}-{hi} m'] = {
            'samples': len(sel),
            'median_abs_cm': round(100 * statistics.median(sel), 1) if sel else None,
            'p95_abs_cm': round(100 * percentile(sel, 0.95), 1) if sel else None,
        }
    return {
        'runs_retried': sum(1 for r in runs if r.get('attempts', 1) > 1),
        'runs_total': len(runs),
        'runs_judged': len(judged),
        'infrastructure_failures': len(runs) - len(judged),
        'correct_total': sum(1 for r in judged if r['passed']),
        'by_category': by_cat,
        'stops_by_speed': speed_stats,
        'perception_error': {
            'samples': len(errors),
            'median_abs_cm': round(100 * statistics.median(abs_err), 1) if abs_err else None,
            'p95_abs_cm': round(100 * percentile(abs_err, 0.95), 1) if abs_err else None,
            'by_distance': err_bins,
        },
        'failures': [{'run': r['run'], 'category': r['category'], 'note': r['note'],
                      'size_m': r['size_m'], 'lateral_m': r['lateral_m'],
                      'speed_mps': r['speed_mps']}
                     for r in judged if not r['passed']],
    }


def summary_markdown(s, args):
    lines = [
        '# Robustness study',
        '',
        f'{s["runs_judged"]} randomized runs (seed {args.seed}), fresh headless Gazebo per run. '
        f'Box size 0.10-0.40 m, start distance 1.5-3.5 m, cruise speeds {args.speeds} m/s, '
        f'stop gap {STOP_GAP} m.',
        '',
        f'**Correct decisions: {s["correct_total"]} / {s["runs_judged"]}**'
        + (f' ({s["infrastructure_failures"]} runs excluded for infrastructure problems)'
           if s['infrastructure_failures'] else ''),
        '',
        f'Simulator restarts: {s.get("runs_retried", 0)} run(s) needed a second attempt because '
        'the simulation infrastructure failed to start (not a safety decision).'
        if s.get('runs_retried') else 'No run needed a simulator restart.',
        '',
        '| Case | Expected | Runs | Correct |',
        '|---|---|---|---|',
    ]
    labels = {'in_path': ('Box overlaps the robot', 'stop, no contact'),
              'clear': ('Box clearly beside the path', 'pass without stopping'),
              'edge': ('Box near the path edge', 'either (reported)')}
    for cat, (name, expected) in labels.items():
        c = s['by_category'][cat]
        extra = f' ({c["stopped"]} stopped)' if cat == 'edge' else ''
        lines.append(f'| {name} | {expected} | {c["runs"]} | {c["correct"]}{extra} |')
    lines += ['', '## Stopping distance by speed', '',
              '| Speed | Stops | Final gap min / median / max | Travel after decision (median) |',
              '|---|---|---|---|']
    for v, st in s['stops_by_speed'].items():
        travel = st['travel_after_trigger_median']
        lines.append(
            f'| {v} m/s | {st["stops"]} | {st["final_gap_min"]:.2f} / {st["final_gap_median"]:.2f} / '
            f'{st["final_gap_max"]:.2f} m | {"" if travel is None else f"{100 * travel:.1f} cm"} |')
    pe = s['perception_error']
    lines += ['', '## Monocular distance estimate vs ground truth', '',
              f'{pe["samples"]} samples, median absolute error '
              f'{pe["median_abs_cm"]} cm, 95th percentile {pe["p95_abs_cm"]} cm.', '',
              '| True distance | Samples | Median abs error | 95th percentile |', '|---|---|---|---|']
    for rng_label, b in pe['by_distance'].items():
        lines.append(f'| {rng_label} | {b["samples"]} | {b["median_abs_cm"]} cm | {b["p95_abs_cm"]} cm |')
    if s['failures']:
        lines += ['', '## Failures', '']
        for f in s['failures']:
            lines.append(f'- run {f["run"]} ({f["category"]}, size {f["size_m"]} m, '
                         f'lateral {f["lateral_m"]} m, {f["speed_mps"]} m/s): {f["note"]}')
    lines += ['', '![Stopping gap by speed](stop_gaps.png)', '',
              '![Monocular distance error](perception_error.png)', '']
    return '\n'.join(lines)


# ----------------------------------------------------------------------------
# Running one scenario (ROS + Gazebo)
# ----------------------------------------------------------------------------
def run_one(index, scenario, world_path, args, log_path):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from vision_safety_interfaces.msg import ObstacleDetection, SafetyStatus

    domain = 60 + index % 30
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
               GZ_PARTITION=f'vs_robust_{os.getpid()}_{index}')
    cmd = ['ros2', 'launch', 'vision_safety_bringup', 'system.launch.py',
           f'world:={world_path}', f'headless:={"true" if args.headless else "false"}',
           'rviz:=false', 'debug_view:=false', 'cruise:=true',
           f'cruise_speed:={scenario["speed_mps"]}']

    odom, status, dets = [], [], []

    def st(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node('robustness_monitor', context=ctx)
    node.create_subscription(Odometry, '/odom', lambda m: odom.append(
        (st(m.header.stamp), m.pose.pose.position.x, m.twist.twist.linear.x)), 100)
    node.create_subscription(SafetyStatus, '/safety/status', lambda m: status.append(
        (st(m.header.stamp), m.state)), 100)
    node.create_subscription(ObstacleDetection, '/perception/obstacle', lambda m: dets.append(
        (st(m.header.stamp), m.in_path, m.in_danger_zone, m.gap_m, m.bottom_clipped)), 100)

    # Diagnostics: count traffic on every link, best effort so no QoS mismatch can hide it.
    diag = {'clock': 0, 'cmd_raw': 0, 'cmd_out': 0, 'last_cmd_linear': 0.0}

    def on_cmd_out(m):
        diag['cmd_out'] += 1
        diag['last_cmd_linear'] = m.linear.x

    node.create_subscription(Clock, '/clock', lambda m: diag.__setitem__(
        'clock', diag['clock'] + 1), qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel_raw', lambda m: diag.__setitem__(
        'cmd_raw', diag['cmd_raw'] + 1), qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', on_cmd_out, qos_profile_sensor_data)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)

    log = open(log_path, 'w')
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)

    def spin_until(pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)
            if pred():
                return True
            if proc.poll() is not None:
                return False
        return False

    outcome = 'timeout'
    try:
        if not spin_until(lambda: len(dets) > 5 and len(odom) > 5, args.startup_timeout):
            outcome = 'launch_failed' if proc.poll() is not None else 'no_perception'
        elif not spin_until(lambda: diag['clock'] > 5, 10.0):
            # Without /clock the sim-time nodes cannot run; no point waiting 60 s.
            outcome = 'launch_exited' if proc.poll() is not None else 'no_clock'
        else:
            box_far_x = scenario['box_center_x'] + scenario['size_m'] / 2.0
            STOP = SafetyStatus.STATE_STOP_OBSTACLE

            def finished():
                if odom and odom[-1][1] > box_far_x + 0.3:
                    return True
                stop_times = [t for t, s in status if s == STOP]
                if stop_times and len(odom) > 30:
                    recent = [vx for t, x, vx in odom if t > stop_times[0]][-30:]
                    return len(recent) >= 30 and all(abs(vx) < 0.005 for vx in recent)
                return False

            if spin_until(finished, args.run_timeout):
                outcome = 'passed' if odom[-1][1] > box_far_x + 0.3 else 'stopped'
                if outcome == 'stopped':
                    spin_until(lambda: False, 1.0)   # confirm: no creep afterwards
            elif proc.poll() is not None:
                outcome = 'launch_exited'
    finally:
        exit_code = proc.poll()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=15)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                pass
        log.close()
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)

    # ---------- measurements ----------
    stop_times = [t for t, s in status if s == SafetyStatus.STATE_STOP_OBSTACLE]
    stopped_ever = bool(stop_times)
    final_gap = trigger_gap = travel = None
    if odom:
        ts = [o[0] for o in odom]
        xs = [o[1] for o in odom]

        def x_at(t):
            i = min(range(len(ts)), key=lambda k: abs(ts[k] - t))
            return xs[i]

        if stop_times:
            x_trig = x_at(stop_times[0])
            trigger_gap = round(true_gap(scenario['box_front_x'], x_trig), 3)
            travel = round(xs[-1] - x_trig, 3)
        final_gap = round(true_gap(scenario['box_front_x'], xs[-1]), 3)

        # Perception accuracy: every in-path estimate with a visible base, vs truth
        errors = []
        for t, in_path, _zone, gap_m, clipped in dets:
            if not in_path or clipped or gap_m is None or math.isnan(gap_m):
                continue
            g = true_gap(scenario['box_front_x'], x_at(t))
            if 0.25 < g < 3.0:
                errors.append({'run': index, 'true_gap_m': round(g, 3),
                               'error_m': round(gap_m - g, 4), 'size_m': scenario['size_m']})
    else:
        errors = []

    passed, note = evaluate(scenario, outcome, final_gap if outcome == 'stopped' else None,
                            stopped_ever)
    counters = dict(diag, detections=len(dets), status=len(status),
                    last_state=status[-1][1] if status else None,
                    moved_m=round(odom[-1][1] - odom[0][1], 3) if odom else 0.0)
    if passed is None:
        if outcome == 'launch_exited':
            note += f' (exit code {exit_code}; see the run log)'
        elif outcome == 'timeout':
            note += f' -> {diagnose(counters)}'
    row = dict(scenario, run=index, outcome=outcome, passed=passed, note=note,
               stopped_ever=stopped_ever, trigger_gap_m=trigger_gap, final_gap_m=final_gap,
               travel_after_trigger_m=travel,
               **{f'diag_{k}': v for k, v in counters.items()})
    return row, errors


# ----------------------------------------------------------------------------
# Plots
# ----------------------------------------------------------------------------
INK, INK2, MUTED, GRID, BASE, SURFACE, SERIES = (
    '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7', '#fcfcfb', '#2a78d6')


def _style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(BASE)
    ax.tick_params(colors=MUTED, labelcolor=INK2)


def make_plots(runs, errors, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available: skipping plots')
        return

    # 1. Final stopping gap per speed (in_path stops)
    stops = [r for r in runs if r['category'] == 'in_path' and r['outcome'] == 'stopped'
             and r['final_gap_m'] is not None]
    if stops:
        speeds = sorted({r['speed_mps'] for r in stops})
        fig, ax = plt.subplots(figsize=(7, 4), facecolor=SURFACE)
        _style(ax)
        rng = random.Random(0)
        for i, v in enumerate(speeds):
            ys = [r['final_gap_m'] for r in stops if r['speed_mps'] == v]
            xs = [i + rng.uniform(-0.12, 0.12) for _ in ys]
            ax.scatter(xs, ys, s=46, color=SERIES, edgecolors=SURFACE, linewidths=1.5, zorder=3)
            med = statistics.median(ys)
            ax.plot([i - 0.28, i + 0.28], [med, med], color=INK2, linewidth=1.5, zorder=2)
        ax.axhline(STOP_GAP, color=INK2, linewidth=1.2, linestyle='--', zorder=2)
        ax.text(len(speeds) - 0.55, STOP_GAP + 0.01, f'configured stop gap {STOP_GAP} m',
                color=INK2, fontsize=9, ha='right', va='bottom')
        ax.axhline(0.0, color=BASE, linewidth=1.2, zorder=2)
        ax.text(-0.45, 0.01, 'contact', color=MUTED, fontsize=9, va='bottom')
        ax.set_xticks(range(len(speeds)), [f'{v} m/s' for v in speeds])
        ax.set_xlim(-0.5, len(speeds) - 0.5)
        ax.set_ylim(-0.05, STOP_GAP + 0.15)
        ax.set_ylabel('Final distance to obstacle (m)', color=INK2)
        ax.set_title('Where the robot came to rest, by cruise speed (bar = median)',
                     color=INK, fontsize=11, loc='left')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, 'stop_gaps.png'), dpi=150, facecolor=SURFACE)
        plt.close(fig)

    # 2. Monocular distance error vs true distance
    if errors:
        fig, ax = plt.subplots(figsize=(7, 4), facecolor=SURFACE)
        _style(ax)
        ax.scatter([e['true_gap_m'] for e in errors], [100 * e['error_m'] for e in errors],
                   s=10, color=SERIES, alpha=0.45, linewidths=0, zorder=3)
        ax.axhline(0.0, color=INK2, linewidth=1.2, zorder=2)
        ax.set_xlabel('True distance robot front to obstacle (m)', color=INK2)
        ax.set_ylabel('Estimate minus truth (cm)', color=INK2)
        ax.set_title('Camera-only distance estimate error, all runs',
                     color=INK, fontsize=11, loc='left')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, 'perception_error.png'), dpi=150, facecolor=SURFACE)
        plt.close(fig)


# ----------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--runs', type=int, default=30)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--speeds', type=float, nargs='+', default=[0.2, 0.3, 0.5])
    p.add_argument('--out', default='/tmp/vision_safety_robustness')
    p.add_argument('--gui', dest='headless', action='store_false',
                   help='show Gazebo (use if the headless camera does not render)')
    p.add_argument('--startup-timeout', type=float, default=90.0)
    p.add_argument('--run-timeout', type=float, default=60.0)
    p.add_argument('--repeat', type=int, default=1,
                   help='run every selected scenario this many times (flakiness checks)')
    p.add_argument('--retries', type=int, default=1,
                   help='restart a run this many times if the INFRASTRUCTURE fails '
                        '(reported in the summary; never used for wrong decisions)')
    p.add_argument('--only', type=int, nargs='+', metavar='INDEX',
                   help='run only these scenario indices (0-based, as in run_XXX.log); '
                        'the same seed reproduces the same scenarios')
    args = p.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    template = open(os.path.join(get_package_share_directory('vision_safety_sim'),
                                 'worlds', 'corridor.sdf')).read()

    out = os.path.expanduser(args.out)
    os.makedirs(os.path.join(out, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(out, 'logs'), exist_ok=True)

    # Scenarios are always generated in the same order, so an index means the same
    # scenario in every invocation with the same seed.
    indices = sorted(set(args.only)) if args.only else list(range(args.runs))
    rng = random.Random(args.seed)
    scenarios = [sample_scenario(rng, args.speeds) for _ in range(max(indices) + 1)]

    # Other simulations compete for the GPU and CPU and distort the measurements.
    killed = kill_test_leftovers()
    if killed:
        print(f'Removed {len(killed)} orphaned Gazebo server(s) from earlier tests: {killed}')
    others = [g for g in find_gz_servers() if not is_test_leftover(g[1])]
    if others:
        print('WARNING: other Gazebo simulations are running and will slow every run:')
        for pid, part, cmd in others:
            print(f'  pid {pid} (GZ_PARTITION={part or "unset"}): {cmd[:90]}')
        print('  Close them for clean measurements.\n')

    plan = [i for i in indices for _ in range(max(1, args.repeat))]
    runs, errors = [], []
    t_start = time.monotonic()
    for n, i in enumerate(plan):
        sc = scenarios[i]
        world = os.path.join(out, 'worlds', f'run_{i:03d}.sdf')
        with open(world, 'w') as f:
            f.write(make_world(template, sc))
        print(f'[{n + 1:>3}/{len(plan)}] #{i:<3} {sc["category"]:<8} size {sc["size_m"]:.2f} m  '
              f'lateral {sc["lateral_m"]:+.2f} m  start gap {sc["initial_gap_m"]:.2f} m  '
              f'{sc["speed_mps"]} m/s ... ', end='', flush=True)
        for attempt in range(1, args.retries + 2):
            killed = kill_test_leftovers()
            if killed:
                print(f'(removed orphaned Gazebo {killed}) ', end='', flush=True)
            log = os.path.join(out, 'logs', f'run_{i:03d}_{n:03d}_try{attempt}.log')
            row, errs = run_one(i, sc, world, args, log)
            row['attempts'] = attempt
            if row['passed'] is not None or attempt > args.retries:
                break
            print(f'[infra: {row["note"]}; restarting] ', end='', flush=True)
        runs.append(row)
        errors.extend(errs)
        verdict = {True: 'OK  ', False: 'FAIL', None: 'SKIP'}[row['passed']]
        gap = f'  final gap {row["final_gap_m"]:.2f} m' if row['outcome'] == 'stopped' else ''
        print(f'{verdict} {row["note"]}{gap}')

    kill_test_leftovers()   # never leave a simulator running behind us

    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0].keys()))
        w.writeheader()
        w.writerows(runs)
    with open(os.path.join(out, 'perception_errors.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['run', 'true_gap_m', 'error_m', 'size_m'])
        w.writeheader()
        w.writerows(errors)
    summary = summarize(runs, errors)
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write(summary_markdown(summary, args))
    make_plots(runs, errors, out)

    minutes = (time.monotonic() - t_start) / 60
    print(f'\n{summary["correct_total"]}/{summary["runs_judged"]} correct '
          f'({summary["infrastructure_failures"]} infrastructure failures) in {minutes:.1f} min')
    print(f'Results: {out}/summary.md')
    return 0 if summary['correct_total'] == summary['runs_judged'] else 1


if __name__ == '__main__':
    sys.exit(main())
