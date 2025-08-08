#!/usr/bin/env python3
"""
GTFS Physics-Aware Interpolator (Siemens S200)
- Max accel: 1.3 m/s^2
- Max decel: 2.2 m/s^2
- Max speed: 60 mph (26.8224 m/s)

Generates:
- Time–distance plots with bounds
- Zoomed interval diagnostics (distance + speed stacked)
- Acceleration feasibility analysis
- Bounds validation report

Input: data/4_k_line_data_with_trip_id.csv
Outputs: outputs/plots/*.png, outputs/reports/*.txt
"""

from __future__ import annotations
import os
from datetime import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Siemens S200 physical constraints
MAX_ACCEL = 1.3  # m/s^2
MAX_DECEL = 2.2  # m/s^2
MAX_SPEED_MPH = 60
MAX_SPEED = MAX_SPEED_MPH * 0.44704  # m/s

DATA_PATH = os.path.join('data', '4_k_line_data_with_trip_id.csv')
PLOTS_DIR = os.path.join('outputs', 'plots')
REPORTS_DIR = os.path.join('outputs', 'reports')


def ensure_dirs():
    os.makedirs(PLOTS_DIR, exist_ok=True)
    os.makedirs(REPORTS_DIR, exist_ok=True)


def load_data(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    # Use full range (no 27,000 ft filter)
    df = df.copy()
    # Units
    df['dist_m'] = df['dist_from_start'] * 0.3048
    df['speed_ms'] = df['speed'] * 0.44704
    return df


def max_forward_distance(v_start: float, T: float, v_end: float) -> float:
    """Max distance achievable in time T with accel MAX_ACCEL, decel MAX_DECEL, capped at MAX_SPEED."""
    if T <= 0:
        return 0.0
    a = MAX_ACCEL
    b = MAX_DECEL
    vmax = MAX_SPEED
    t_to_vmax = max((vmax - v_start) / a, 0.0) if v_start < vmax else 0.0
    t_decel_from_vmax = max((vmax - v_end) / b, 0.0)
    if t_to_vmax + t_decel_from_vmax <= T:
        t_cruise = T - t_to_vmax - t_decel_from_vmax
        d_acc = (vmax**2 - v_start**2) / (2*a) if v_start < vmax else 0.0
        d_cruise = vmax * t_cruise
        d_dec = (vmax**2 - v_end**2) / (2*b)
        return d_acc + d_cruise + d_dec
    # No cruise, solve for v_peak
    denom = (1.0/a) + (1.0/b)
    v_peak = (T + (v_start/a) + (v_end/b)) / denom
    v_peak = min(max(v_peak, 0.0), vmax)
    t1 = max((v_peak - v_start)/a, 0.0)
    t3 = max((v_peak - v_end)/b, 0.0)
    d1 = (v_peak**2 - v_start**2)/(2*a) if v_peak >= v_start else 0.0
    d3 = (v_peak**2 - v_end**2)/(2*b) if v_peak >= v_end else 0.0
    t_left = max(T - t1 - t3, 0.0)
    return d1 + v_peak * t_left + d3


def solve_speed_limit_for_reach(s_rem: float, T_rem: float, v_end: float, tol: float = 1e-4) -> float:
    """Find the maximum allowable instantaneous speed v(t) so that
    max_forward_distance(v, T_rem, v_end) >= s_rem holds with equality (as close as possible).
    Monotone in v, so use bisection on [0, MAX_SPEED]."""
    lo, hi = 0.0, MAX_SPEED
    if T_rem <= 0:
        return 0.0
    # If even MAX_SPEED cannot cover s_rem, return MAX_SPEED to let caller clamp position later
    if max_forward_distance(hi, T_rem, v_end) < s_rem:
        return hi
    # If zero speed already covers s_rem, return 0
    if max_forward_distance(lo, T_rem, v_end) >= s_rem:
        return lo
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        s_mid = max_forward_distance(mid, T_rem, v_end)
        if s_mid >= s_rem:
            hi = mid
        else:
            lo = mid
        if hi - lo < tol:
            break
    return 0.5 * (lo + hi)


def calc_bounds(t1, d1, v1, t2, d2, v2):
    """Compute physically-consistent upper/lower distance envelopes between two points.

    Assumptions:
    - Nonnegative speeds (no reverse motion)
    - Acceleration limited to +MAX_ACCEL, deceleration limited to MAX_DECEL
    - Speed limited to MAX_SPEED

    Method:
    - Build a forward/backward-coned maximum speed envelope v_max(t)
    - Build a minimum speed envelope v_min(t)
    - Upper curve uses a scaled version of v_max that exactly matches the end distance
      (scaling preserves slope magnitudes, so constraints are respected and avoids end-corners)
    - Lower curve adds the remaining area to v_min as late as possible, clipped by v_upper,
      using bisection to satisfy the distance exactly, followed by slope capping to respect
      accel/decel limits. This avoids the sharp right-corner artifacts.

    Returns (t_rel, d_upper, d_lower) or (None, None, None) if the segment is infeasible.
    """
    dt = (t2 - t1).total_seconds()
    if dt <= 0:
        return None, None, None

    # Time grid (denser for longer intervals)
    n = max(80, int(dt * 10))
    t_rel = np.linspace(0.0, dt, n)
    dt_steps = np.diff(t_rel)
    dt_steps = np.concatenate([[dt_steps[0]], dt_steps])

    # Enforce nonnegative endpoint speeds
    v1 = max(0.0, float(v1))
    v2 = max(0.0, float(v2))

    # Forward/backward speed cone for v_max (pointwise maximum allowable speed)
    v_max_fwd = np.minimum(v1 + MAX_ACCEL * t_rel, MAX_SPEED)
    v_max_bwd = np.minimum(v2 + MAX_DECEL * (dt - t_rel), MAX_SPEED)
    v_max_env = np.minimum(v_max_fwd, v_max_bwd)

    # Minimum feasible speed envelope v_min (pointwise lower bound)
    v_min_fwd = np.maximum(v1 - MAX_DECEL * t_rel, 0.0)
    v_min_bwd = np.maximum(v2 - MAX_ACCEL * (dt - t_rel), 0.0)
    v_min_env = np.maximum(v_min_fwd, v_min_bwd)
    v_min_env = np.minimum(v_min_env, v_max_env)

    # Helper to integrate speed to distance increments (trapezoidal)
    def integrate_speed_to_distance(v_profile: np.ndarray) -> np.ndarray:
        d = np.zeros_like(v_profile)
        for i in range(1, len(v_profile)):
            d[i] = d[i-1] + 0.5 * (v_profile[i] + v_profile[i-1]) * (t_rel[i] - t_rel[i-1])
        return d

    required_delta = (d2 - d1)
    if required_delta < -1e-6:
        # Distance decreased while speeds are nonnegative – infeasible without reverse.
        return None, None, None

    # Compute maximal and minimal achievable distance under envelopes
    d_max_env = integrate_speed_to_distance(v_max_env)
    d_min_env = integrate_speed_to_distance(v_min_env)
    A_max = d_max_env[-1]
    A_min = d_min_env[-1]

    # Check feasibility window: required must be between achievable min and max
    if required_delta > A_max + 1e-6:
        # Even the most aggressive profile cannot cover enough distance in time
        return None, None, None
    if required_delta < A_min - 1e-6:
        # Even the laziest feasible profile overshoots distance
        return None, None, None

    # Upper speed: scale down v_max_env uniformly to hit the exact area (distance)
    # Scaling preserves slope magnitudes and avoids end discontinuities
    k = 1.0 if A_max == 0 else min(1.0, required_delta / A_max)
    v_upper = k * v_max_env

    # Lower speed: start from v_min_env and add late-weighted mass until total area matches
    deficit = required_delta - A_min
    if deficit < 0:
        deficit = 0.0

    # Weight function concentrated late in the interval to keep early positions low
    # w_density integrates to 1 under trapezoid rule
    p = 4  # late emphasis
    tau = (t_rel / dt) if dt > 0 else np.zeros_like(t_rel)
    w_raw = tau**p
    # Normalize to make integral(w) = 1 under trapezoid
    w_area = integrate_speed_to_distance(w_raw)[-1]
    if w_area <= 0:
        w_density = np.zeros_like(w_raw)
    else:
        w_density = w_raw / w_area

    # Capacity to add without exceeding v_upper
    cap = np.maximum(v_upper - v_min_env, 0.0)

    def fill_deficit_bisect(target_area: float) -> np.ndarray:
        if target_area <= 1e-9:
            return np.zeros_like(v_min_env)
        lo, hi = 0.0, np.max(np.where(w_density > 0, cap / (w_density + 1e-12), 0.0))
        hi = float(max(hi, 0.0))
        add = None
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            add_try = np.minimum(mid * w_density, cap)
            area = integrate_speed_to_distance(add_try)[-1]
            if area >= target_area:
                hi = mid
                add = add_try
            else:
                lo = mid
        if add is None:
            add = np.minimum(hi * w_density, cap)
        # Final small scaling to match as close as possible
        area = integrate_speed_to_distance(add)[-1]
        if area > 0:
            add *= (target_area / area)
            add = np.minimum(add, cap)
        return add

    add_profile = fill_deficit_bisect(deficit)
    v_lower = v_min_env + add_profile
    v_lower = np.minimum(v_lower, v_upper)

    # Enforce slope limits on v_lower with forward/backward passes, preserving bounds
    def slope_cap(v: np.ndarray) -> np.ndarray:
        v_c = v.copy()
        # forward (limit acceleration)
        for i in range(1, len(v_c)):
            dv_max = MAX_ACCEL * (t_rel[i] - t_rel[i-1])
            v_c[i] = min(v_c[i], v_c[i-1] + dv_max)
        # backward (limit deceleration)
        for i in range(len(v_c)-2, -1, -1):
            dv_max = MAX_DECEL * (t_rel[i+1] - t_rel[i])
            v_c[i] = min(v_c[i], v_c[i+1] + dv_max)
        # respect envelopes
        v_c = np.maximum(v_c, v_min_env)
        v_c = np.minimum(v_c, v_upper)
        return v_c

    # Iterate a couple of times to maintain area after slope capping
    for _ in range(5):
        v_lower = slope_cap(v_lower)
        area_lower = integrate_speed_to_distance(v_lower)[-1]
        gap = required_delta - area_lower
        if abs(gap) < 1e-3:
            break
        # Try to add a tiny fraction late without breaking caps
        extra = fill_deficit_bisect(max(0.0, gap))
        v_lower = np.minimum(v_lower + extra, v_upper)

    # Integrate to distance curves
    d_upper_delta = integrate_speed_to_distance(v_upper)
    d_lower_delta = integrate_speed_to_distance(v_lower)
    d_up = d1 + d_upper_delta
    d_lo = d1 + d_lower_delta

    # Enforce ordering and exact endpoints
    d_up = np.maximum(d_up, d_lo)
    d_up[0] = d1; d_lo[0] = d1
    d_up[-1] = d2; d_lo[-1] = d2

    return t_rel, d_up, d_lo


def derivatives(t: np.ndarray, x: np.ndarray):
    dt = np.gradient(t)
    dt[dt == 0] = np.finfo(float).eps
    v = np.gradient(x) / dt
    a = np.gradient(v) / dt
    return v, a


def plot_overview(df: pd.DataFrame):
    trips_out = df[df['direction'] == 'outbound']['trip_id'].unique()[:20]
    trips_in = df[df['direction'] == 'inbound']['trip_id'].unique()[:20]
    fig, axes = plt.subplots(2, 1, figsize=(18, 12))

    def plot_dir(ax, trips, cmap):
        colors = plt.cm.get_cmap(cmap)(np.linspace(0, 1, len(trips)))
        for idx, trip_id in enumerate(trips):
            td = df[df['trip_id'] == trip_id].sort_values('timestamp').copy()
            if len(td) < 2:
                continue
            start = td['timestamp'].iloc[0]
            td['tsec'] = (td['timestamp'] - start).dt.total_seconds()
            color = colors[idx]
            ax.plot(td['tsec'], td['dist_m'], color=color, alpha=0.5, linewidth=1)
            # bounds per segment
            for i in range(len(td)-1):
                t1, t2 = td.iloc[i]['timestamp'], td.iloc[i+1]['timestamp']
                d1, d2 = td.iloc[i]['dist_m'], td.iloc[i+1]['dist_m']
                v1, v2 = max(td.iloc[i]['speed_ms'], 0.0), max(td.iloc[i+1]['speed_ms'], 0.0)
                tr, up, lo = calc_bounds(t1, d1, v1, t2, d2, v2)
                if tr is None:
                    continue
                ax.fill_between(td.iloc[i]['tsec'] + tr, lo, up, color=color, alpha=0.2)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Time from trip start (s)')
        ax.set_ylabel('Distance (m)')

    plot_dir(axes[0], trips_out, 'viridis')
    axes[0].set_title('Outbound: Time–Distance with Interpolation Bounds')
    plot_dir(axes[1], trips_in, 'plasma')
    axes[1].set_title('Inbound: Time–Distance with Interpolation Bounds')

    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, 'overview_bounds.png'), dpi=300, bbox_inches='tight')
    plt.close()


def plot_zoomed_intervals(df: pd.DataFrame, per_direction: int = 6):
    out_dir = os.path.join(PLOTS_DIR, 'zoomed_intervals')
    os.makedirs(out_dir, exist_ok=True)

    def do_dir(direction: str):
        count = 0
        trips = df[df['direction'] == direction]['trip_id'].unique()
        for trip_id in trips:
            td = df[df['trip_id'] == trip_id].sort_values('timestamp').copy()
            if len(td) < 2:
                continue
            start = td['timestamp'].iloc[0]
            td['tsec'] = (td['timestamp'] - start).dt.total_seconds()
            for i in range(len(td)-1):
                t1, t2 = td.iloc[i]['timestamp'], td.iloc[i+1]['timestamp']
                d1, d2 = td.iloc[i]['dist_m'], td.iloc[i+1]['dist_m']
                v1, v2 = max(td.iloc[i]['speed_ms'], 0.0), max(td.iloc[i+1]['speed_ms'], 0.0)
                dt_sec = (t2 - t1).total_seconds()
                if not (10 <= dt_sec <= 60):
                    continue
                tr, up, lo = calc_bounds(t1, d1, v1, t2, d2, v2)
                if tr is None:
                    continue
                t_rel = td.iloc[i]['tsec'] + tr
                v_up, _ = derivatives(t_rel, up)
                v_lo, _ = derivatives(t_rel, lo)
                v_up_mph = v_up / 0.44704
                v_lo_mph = v_lo / 0.44704
                d_center = np.linspace(d1, d2, len(t_rel))
                v_center = np.gradient(d_center) / np.gradient(t_rel)

                fig, (ax_d, ax_v) = plt.subplots(2, 1, figsize=(12, 8), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
                ax_d.fill_between(t_rel, lo, up, color='lightsteelblue', alpha=0.6)
                ax_d.plot(t_rel, d_center, color='gray', linestyle='--', linewidth=1.5)
                ax_d.scatter([t_rel[0], t_rel[-1]], [d1, d2], color='k', zorder=5)
                ax_d.set_ylabel('Distance (m)')
                ax_d.set_title(f'{direction.title()} Trip {trip_id} — interval {i} (Δt={dt_sec:.1f}s)')
                ax_d.grid(True, alpha=0.3)

                ax_v.plot(t_rel, v_up_mph, color='steelblue', linewidth=2, label='Upper speed (mph)')
                ax_v.plot(t_rel, v_lo_mph, color='dodgerblue', linewidth=2, linestyle='--', label='Lower speed (mph)')
                ax_v.axhline(MAX_SPEED_MPH, color='red', linestyle=':', linewidth=1.5, label=f'Max {MAX_SPEED_MPH} mph')
                ax_v.axhline(0, color='k', linewidth=1)
                ax_v.set_xlabel('Time from trip start (s)')
                ax_v.set_ylabel('Speed (mph)')
                ax_v.grid(True, alpha=0.3)
                ax_v.legend(loc='upper left')

                plt.tight_layout()
                out_path = os.path.join(out_dir, f'{direction}_trip_{trip_id}_interval_{i}.png')
                plt.savefig(out_path, dpi=300, bbox_inches='tight')
                plt.close()
                count += 1
                if count >= per_direction:
                    return

    do_dir('outbound')
    do_dir('inbound')


def accel_feasibility(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for trip_id in df['trip_id'].unique():
        td = df[df['trip_id'] == trip_id].sort_values('timestamp').copy()
        for i in range(len(td)-1):
            t1, t2 = td.iloc[i]['timestamp'], td.iloc[i+1]['timestamp']
            dt_sec = (t2 - t1).total_seconds()
            if dt_sec <= 0:
                continue
            v1, v2 = td.iloc[i]['speed_ms'], td.iloc[i+1]['speed_ms']
            rows.append({'dt': dt_sec, 'a': (v2 - v1)/dt_sec})
    res = pd.DataFrame(rows)
    if res.empty:
        return res
    plt.figure(figsize=(10,6))
    plt.scatter(res['dt'], res['a'], s=14, alpha=0.6, color='green')
    plt.axhline(MAX_ACCEL, color='blue', linestyle='--', label=f'+{MAX_ACCEL} m/s²')
    plt.axhline(-MAX_DECEL, color='orange', linestyle='--', label=f'-{MAX_DECEL} m/s²')
    plt.xlabel('Interval (s)'); plt.ylabel('Observed acceleration (m/s²)')
    plt.title('Observed Acceleration vs Physical Limits')
    plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, 'accel_feasibility.png'), dpi=300, bbox_inches='tight')
    plt.close()
    return res


def write_summary(acc_df: pd.DataFrame):
    lines = [
        'GTFS Interpolation Summary',
        '==========================',
        f'Max accel {MAX_ACCEL} m/s^2 | Max decel {MAX_DECEL} m/s^2 | Max speed {MAX_SPEED_MPH} mph',
        '',
    ]
    if not acc_df.empty:
        lines += [
            f'Intervals analyzed: {len(acc_df)}',
            f'Mean dt: {acc_df["dt"].mean():.1f}s',
            f'Mean accel: {acc_df["a"].mean():.3f} m/s²',
            f'Max accel: {acc_df["a"].max():.3f} m/s²',
            f'Min accel: {acc_df["a"].min():.3f} m/s²',
        ]
    out = os.path.join(REPORTS_DIR, 'summary.txt')
    with open(out, 'w') as f:
        f.write('\n'.join(lines))


def main():
    ensure_dirs()
    if not os.path.exists(DATA_PATH):
        raise FileNotFoundError(f'Missing data file: {DATA_PATH}. Please place the CSV there.')
    df = load_data(DATA_PATH)
    plot_overview(df)
    plot_zoomed_intervals(df, per_direction=6)
    acc_df = accel_feasibility(df)
    write_summary(acc_df)
    print('Done. Outputs in outputs/plots and outputs/reports.')


if __name__ == '__main__':
    main()
