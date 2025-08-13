# Interpolator.py
# Full base script: lens-shaped envelopes, reverse allowed, t=0 per trip,
# inbound/outbound detection, safe plotting (no clipping).

import pandas as pd
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path
import json
import warnings
warnings.filterwarnings('ignore')
import argparse

# -------------------------- Styling / speed tweaks -------------------------- #
def set_plot_style():
    mpl.rcParams.update({
        "figure.dpi": 150,
        "savefig.dpi": 220,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.titlesize": 13,
        "axes.labelsize": 12,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.frameon": False,
        "legend.fontsize": 10,
        "font.size": 11,
        # speed / safety
        "path.simplify": True,
        "path.simplify_threshold": 1.0,
        "agg.path.chunksize": 20000,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.34,
    })

def safe_savefig(fig, path, dpi=220, pad=0.34):
    fig.canvas.draw()
    fig.savefig(path, dpi=dpi, bbox_inches='tight', pad_inches=pad, facecolor='white')

# =============================== MAIN CLASS ================================= #
class TrainTrajectoryInterpolator:
    """
    Lens-shaped feasible envelope between GPS points with accel/speed limits.
    Time for each full trip is rebased to start at 0 s. Supports brief reverse motion.
    Builds one inbound and one outbound full-trip overview.
    """

    def __init__(self, csv_file='4_k_line_data_with_trip_id.csv', output_dir='trajectory_analysis_output'):
        # Siemens S200 constraints (interpret input speeds as m/s)
        self.MAX_ACCEL_MS2 = 1.3   # m/s^2 (accelerating)
        self.MAX_DECEL_MS2 = 2.2   # m/s^2 (braking)
        self.MAX_SPEED_MPH = 60.0  # for reporting/limits (≈ 26.8224 m/s)

        # Conversions
        self.MPH_TO_FPS = 1.46667      # mph → ft/s
        self.MPH_TO_MPS = 0.44704      # mph → m/s
        self.MPS_TO_MPH = 2.2369362921 # m/s → mph
        self.MPS_TO_FPS = 3.28084      # m/s → ft/s

        # Dynamics in feet-based frame
        self.a  = self.MAX_ACCEL_MS2 * self.MPS_TO_FPS   # ft/s^2 (accelerating)
        self.b  = self.MAX_DECEL_MS2 * self.MPS_TO_FPS   # ft/s^2 (braking)
        self.V  = (self.MAX_SPEED_MPH * self.MPH_TO_FPS) # ft/s (|v| <= V)

        # Endpoint speed tolerance (m/s) for feasible envelopes and likely path
        self.endpoint_speed_tolerance_ms = 0.5
        self.endpoint_speed_tolerance_fps = self.endpoint_speed_tolerance_ms * self.MPS_TO_FPS

        # Dirs + data
        self.output_dir = Path(output_dir)
        # Clean and recreate outputs to avoid stale clutter
        try:
            if self.output_dir.exists():
                import shutil
                shutil.rmtree(self.output_dir)
        except Exception:
            pass
        self._setup_dirs()
        self._load(csv_file)

    # ------------------------------ Setup / Load ----------------------------- #
    def _setup_dirs(self):
        self.output_dir.mkdir(exist_ok=True)
        self.dirs = {
            'overview': self.output_dir / 'overview',
            'segments': self.output_dir / 'individual_segments',
            'data':     self.output_dir / 'exported_data',
            'reports':  self.output_dir / 'reports'
        }
        for d in self.dirs.values():
            d.mkdir(exist_ok=True)

    def _load(self, csv_file):
        df = pd.read_csv(csv_file)
        need = {'timestamp','dist_from_start','speed'}
        miss = need - set(df.columns)
        if miss: raise ValueError(f"CSV missing columns: {miss}")

        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df = df.sort_values('timestamp').reset_index(drop=True)
        t0 = df['timestamp'].min()
        df['time_seconds'] = (df['timestamp'] - t0).dt.total_seconds()
        # Input speed is in m/s; convert to ft/s for interpolation
        df['speed_fps']    = df['speed'] * self.MPS_TO_FPS
        self.df = df
        print(f"Loaded {len(df)} rows; {df['timestamp'].min()} → {df['timestamp'].max()}")

    # ------------------------- Forward distances (allow v<0) ----------------- #
    def _fwd_min_dist(self, v1, t):
        """
        Minimal displacement in time t from start, NO REVERSE:
        - Brake at b to 0 as quickly as possible, then wait.
        """
        if t <= 0: return 0.0
        if v1 <= 0: return 0.0
        t_stop = v1 / self.b
        if t <= t_stop:
            return v1*t - 0.5*self.b*t*t
        # stopped, then wait at zero speed
        return 0.5 * v1*v1 / self.b

    def _fwd_max_dist(self, v1, t):
        """Max displacement in time t from start: accelerate at a to +V then cruise."""
        if t <= 0: return 0.0
        t_acc = max(0.0, (self.V - v1) / self.a)
        if t <= t_acc:
            return v1*t + 0.5*self.a*t*t
        s_acc = v1*t_acc + 0.5*self.a*t_acc*t_acc
        return s_acc + self.V*(t - t_acc)

    # ------------------------ Remaining distances (to end) ------------------- #
    def _remain_min_dist(self, v2, dt):
        """
        Minimal distance you must still cover in dt to end at speed v2:
        wait as much as possible, then accelerate at a to v2.
        """
        if dt <= 0: return 0.0
        t_need = abs(v2) / self.a
        if dt >= t_need:
            return 0.5*v2*v2 / self.a
        else:
            u = max(-self.V, v2 - self.a*dt)  # initial speed needed
            return 0.5*(u + v2) * dt

    def _remain_max_dist(self, v2, dt):
        """
        Maximum distance you can still travel in dt and finish at v2:
        run at +V as long as possible, then brake at b to v2.
        """
        if dt <= 0: return 0.0
        t_dec_from_V = max(0.0, (self.V - v2) / self.b)
        if dt >= t_dec_from_V:
            s_cruise = self.V * (dt - t_dec_from_V)
            s_decel  = 0.5*(self.V + v2) * t_dec_from_V
            return s_cruise + s_decel
        else:
            v_peak = min(self.V, v2 + self.b*dt)
            return 0.5*(v_peak + v2) * dt

    # ----------------------- From-start / to-end speed bounds ---------------- #
    def _from_start_speed_bounds(self, v1, t):
        """Reachable speed interval at time t from v1 with |v|<=V and rates (a,b)."""
        # lower bound (allow negative)
        if v1 >= 0:
            t_stop = v1 / self.b
            if t <= t_stop:
                v_min = v1 - self.b*t
            else:
                v_min = max(-self.V, -self.a*(t - t_stop))
        else:
            v_min = max(-self.V, v1 - self.a*t)
        # upper bound
        v_max = min(self.V, v1 + self.a*t)
        return v_min, v_max

    def _to_end_speed_bounds_simple(self, v2, t_to_end):
        """Conservative current-speed interval that can still hit v2 in t_to_end."""
        lo = max(-self.V, v2 - self.a*t_to_end)
        hi = min( self.V, v2 + self.b*t_to_end)
        return lo, hi

    # --------------------------- Core (lens, robust) ------------------------- #
    def calculate_feasible_bounds(self, t1, t2, d1, d2, v1_fps, v2_fps, num_points=120):
        """
        Build a lens by intersecting forward/backward *position* intervals.
        Works for outbound (d2>=d1) and inbound (d2<d1) by normalizing segment
        to increasing-distance frame and transforming back. Reverse allowed.
        """
        T = t2 - t1
        if T <= 0: return None

        # 1) Normalize so distance increases
        sgn = 1.0 if d2 >= d1 else -1.0
        d1n, d2n = sgn * d1, sgn * d2
        v1n, v2n = abs(v1_fps), abs(v2_fps)

        # time grid
        t = np.linspace(t1, t2, num_points)
        tau = t - t1

        # 2) Forward distance bounds
        s_fwd_min = np.array([self._fwd_min_dist(v1n, tt) for tt in tau])
        s_fwd_max = np.array([self._fwd_max_dist(v1n, tt) for tt in tau])
        x_fwd_min = d1n + s_fwd_min
        x_fwd_max = d1n + s_fwd_max

        # 3) Backward remaining distance bounds
        rem = T - tau
        s_rem_min = np.array([self._remain_min_dist(v2n, rr) for rr in rem])
        s_rem_max = np.array([self._remain_max_dist(v2n, rr) for rr in rem])
        x_bwd_min = d2n - s_rem_max
        x_bwd_max = d2n - s_rem_min

        # 4) Lens via intersection in normalized frame
        x_lo_n = np.maximum(x_fwd_min, x_bwd_min)
        x_hi_n = np.minimum(x_fwd_max, x_bwd_max)
        bad = x_lo_n > x_hi_n
        if np.any(bad):
            mid = 0.5 * (x_lo_n + x_hi_n)
            x_lo_n[bad] = mid[bad]
            x_hi_n[bad] = mid[bad]

        # 5) Instantaneous speed bounds (with endpoint tolerance)
        tol = self.endpoint_speed_tolerance_fps
        v_from_start_min = np.maximum(0.0, (v1n - tol) - self.b * tau)
        v_from_start_max = np.minimum( self.V, (v1n + tol) + self.a * tau)
        v_to_end_min     = np.maximum(0.0, (v2n - tol) - self.a * (T - tau))
        v_to_end_max     = np.minimum( self.V, (v2n + tol) + self.b * (T - tau))
        v_lo_n = np.maximum(v_from_start_min, v_to_end_min)
        v_hi_n = np.minimum(v_from_start_max, v_to_end_max)

        # likely speed: minimal-jerk profile within [v_lo_n, v_hi_n]
        v_like_n = (1 - tau/T) * v1n + (tau/T) * v2n
        v_like_n = np.clip(v_like_n, v_lo_n, v_hi_n)
        try:
            import numpy as _np
            from scipy.optimize import minimize as _minimize
            v0 = v_like_n.copy()
            dt = float(T) / max(1, len(tau)-1)
            dt3 = (dt if dt > 0 else 1.0) ** 3

            def obj(v):
                v = _np.asarray(v)
                # jerk approx via third-order finite diff
                if len(v) < 4:
                    jcost = 0.0
                else:
                    j = (v[3:] - 3*v[2:-1] + 3*v[1:-2] - v[:-3]) / dt3
                    jcost = _np.sum(j*j)
                # endpoint soft penalties if beyond tolerance
                e0 = max(0.0, abs(v[0] - v1n) - tol)
                e1 = max(0.0, abs(v[-1] - v2n) - tol)
                pcost = 1e4*(e0*e0 + e1*e1)
                return jcost + pcost

            bounds = list(zip(v_lo_n.tolist(), v_hi_n.tolist()))
            res = _minimize(obj, v0, method='L-BFGS-B', bounds=bounds, options={'maxiter': 200})
            if res.success and _np.all(_np.isfinite(res.x)):
                v_like_n = res.x.astype(float)
            else:
                v_like_n = _np.clip(v0, v_lo_n, v_hi_n)

            # Also compute smooth lower/upper envelopes by preferring bounds but minimizing jerk
            def obj_pref(v, pref):
                v = _np.asarray(v)
                if len(v) < 4:
                    jcost = 0.0
                else:
                    j = (v[3:] - 3*v[2:-1] + 3*v[1:-2] - v[:-3]) / dt3
                    jcost = _np.sum(j*j)
                pcost = 1e-3 * _np.sum((v - pref)**2)
                return jcost + pcost

            # Lower preference towards v_lo_n
            res_lo = _minimize(lambda vv: obj_pref(vv, v_lo_n), v_lo_n, method='L-BFGS-B', bounds=bounds, options={'maxiter': 120})
            v_min_smooth_n = _np.clip(res_lo.x if res_lo.success else v_lo_n, v_lo_n, v_hi_n)
            # Upper preference towards v_hi_n
            res_hi = _minimize(lambda vv: obj_pref(vv, v_hi_n), v_hi_n, method='L-BFGS-B', bounds=bounds, options={'maxiter': 120})
            v_max_smooth_n = _np.clip(res_hi.x if res_hi.success else v_hi_n, v_lo_n, v_hi_n)
        except Exception:
            # Fallback to clipped linear if SciPy unavailable
            v_like_n = np.clip(v_like_n, v_lo_n, v_hi_n)
            v_min_smooth_n = v_lo_n.copy()
            v_max_smooth_n = v_hi_n.copy()
        # Primary: keep velocity profile (minimal jerk) and integrate to position.
        # Adjust only with a global scale k to exactly match required distance, with clipping to [v_lo_n, v_hi_n].
        v_base_n = v_like_n.copy()

        def integrate_velocity(vn: np.ndarray) -> np.ndarray:
            s = np.empty_like(tau)
            s[0] = d1n
            for j in range(1, len(tau)):
                dtj = tau[j] - tau[j-1]
                s[j] = s[j-1] + 0.5 * (vn[j-1] + vn[j]) * dtj
            return s

        def area_for_k(k: float) -> float:
            v_scaled = np.clip(k * v_base_n, v_lo_n, v_hi_n)
            s = integrate_velocity(v_scaled)
            return float(s[-1] - d1n)

        required_area = float(d2n - d1n)
        # If needed, find k via bisection so that the integrated area matches required distance
        k_lo, k_hi = 0.0, 1.0
        # Expand upper bound until area >= required or we hit a practical cap
        while area_for_k(k_hi) < required_area and k_hi < 64.0:
            k_hi *= 1.6
        # If even at large k we cannot reach the area (due to clipping), fall back to v_hi_n
        if area_for_k(k_hi) < required_area - 1e-6:
            v_like_n = v_hi_n.copy()
        else:
            for _ in range(40):
                k_mid = 0.5 * (k_lo + k_hi)
                a_mid = area_for_k(k_mid)
                if abs(a_mid - required_area) < 1e-3:
                    k_lo = k_hi = k_mid
                    break
                if a_mid < required_area:
                    k_lo = k_mid
                else:
                    k_hi = k_mid
            k_star = 0.5 * (k_lo + k_hi)
            v_like_n = np.clip(k_star * v_base_n, v_lo_n, v_hi_n)

        # Integrate final velocity to get likely position; do not clamp x_like inside lens to preserve consistency
        x_like_n = integrate_velocity(v_like_n)

        # 6) Transform back to original frame
        x_min = sgn * x_lo_n
        x_max = sgn * x_hi_n
        x_like = sgn * x_like_n
        x_min, x_max = np.minimum(x_min, x_max), np.maximum(x_min, x_max)
        
        # Enforce exact endpoint constraints
        x_min[0] = x_max[0] = x_like[0] = d1
        x_min[-1] = x_max[-1] = x_like[-1] = d2

        # Report velocity magnitude (no reverse) in mph-equivalent arrays for schema
        v_lo = np.maximum(v_lo_n, 0.0) / self.MPH_TO_FPS
        v_hi = np.maximum(v_hi_n, 0.0) / self.MPH_TO_FPS
        v_like = np.maximum(v_like_n, 0.0) / self.MPH_TO_FPS
        v_lo_smooth = np.maximum(v_min_smooth_n, 0.0) / self.MPH_TO_FPS
        v_hi_smooth = np.maximum(v_max_smooth_n, 0.0) / self.MPH_TO_FPS
        v_min = np.minimum(v_lo, v_hi)
        v_max = np.maximum(v_lo, v_hi)

        return {
            'time': t1 + tau,
            'min_speed': v_min,
            'max_speed': v_max,
            'likely_speed': v_like,
            'min_speed_smooth': v_lo_smooth,
            'max_speed_smooth': v_hi_smooth,
            'min_position': x_min,
            'max_position': x_max,
            'likely_position': x_like
        }

    def calculate_linear_velocity_segment(self, t1, t2, d1, d2, v1_fps, v2_fps, num_points=120):
        """No-bounds mode (robust + sign-aware):
        - Likely velocity is the straight line between endpoint magnitudes
        - A smooth bump (scaled by 'a') is added; we solve for 'a' so the
          integrated distance HITS the endpoint exactly (handles inbound, too)
        - Distance is the integral of the **signed** velocity (sgn by d2-d1)
        - Bands collapse to the likely curve
        Returns the same dict shape as calculate_feasible_bounds.
        """
        T = t2 - t1
        if T <= 0:
            return None
        # Preserve travel direction: sign by distance delta
        sgn = 1.0 if d2 >= d1 else -1.0
        t = np.linspace(t1, t2, num_points)
        tau = t - t1
        v1_mag = max(v1_fps, 0.0)
        v2_mag = max(v2_fps, 0.0)
        u = tau / T
        # Base: straight line between endpoints (magnitude)
        v_base_mag = v1_mag + (v2_mag - v1_mag) * u
        # Bump shape integrates to 1 over [0,1]
        g = 6.0 * u * (1.0 - u)

        def integrate_with_a(a_val: float):
            v_mag = v_base_mag + a_val * g
            v_mag = np.clip(v_mag, 0.0, self.V)  # enforce 0 <= v <= Vmax (ft/s)
            v_signed = sgn * v_mag               # **critical for inbound**
            # integrate numerically
            x = np.empty_like(tau)
            x[0] = d1
            for i in range(1, len(tau)):
                dt = tau[i] - tau[i-1]
                x[i] = x[i-1] + 0.5 * (v_signed[i-1] + v_signed[i]) * dt
            return v_mag, v_signed, x

        # Root-find a so that x_end == d2 (sign-aware & robust)
        def f(a: float) -> float:
            _, _, x = integrate_with_a(a)
            return float(x[-1] - d2)

        a_lo, a_hi = -self.V, self.V
        f_lo, f_hi = f(a_lo), f(a_hi)
        tries = 0
        # Expand bounds until we bracket a root or hit limits
        while f_lo * f_hi > 0 and tries < 40:
            # expand towards the side with larger error magnitude
            if abs(f_lo) < abs(f_hi):
                a_hi *= 1.7
                f_hi = f(a_hi)
            else:
                a_lo *= 1.7
                f_lo = f(a_lo)
            tries += 1

        # If still not bracketed, pick the best a on a coarse grid
        if f_lo * f_hi > 0:
            grid = np.linspace(min(a_lo, a_hi), max(a_lo, a_hi), 41)
            vals = [abs(f(a)) for a in grid]
            a_star = float(grid[int(np.argmin(vals))])
            v_mag, v_signed, x_like = integrate_with_a(a_star)
        else:
            # Bisection
            for _ in range(60):
                a_mid = 0.5 * (a_lo + a_hi)
                f_mid = f(a_mid)
                if abs(f_mid) <= 1e-4:
                    a_lo = a_hi = a_mid
                    break
                if f_lo * f_mid <= 0:
                    a_hi, f_hi = a_mid, f_mid
                else:
                    a_lo, f_lo = a_mid, f_mid
            a_star = 0.5 * (a_lo + a_hi)
            v_mag, v_signed, x_like = integrate_with_a(a_star)

        v_mph = np.abs(v_signed) / self.MPH_TO_FPS
        v_ms_signed = v_signed / self.MPS_TO_FPS
        return {
            'time': t1 + tau,
            'min_speed': v_mph,
            'max_speed': v_mph,
            'likely_speed': v_mph,
            'signed_velocity_ms': v_ms_signed,
            'min_position': x_like,
            'max_position': x_like,
            'likely_position': x_like
        }

    def calculate_pure_linear_segment(self, t1, t2, d1, d2, v1_fps, v2_fps, num_points=120):
        """Alternative model for side-by-side visualization:
        - Velocity connects the two endpoint speeds linearly (no bump, no cap)
        - **Signed** by travel direction (increasing distance → +, decreasing → -)
        - Distance is the plain integral of that velocity; may not hit the end dot
        """
        T = t2 - t1
        if T <= 0:
            return None
        sgn = 1.0 if d2 >= d1 else -1.0
        t = np.linspace(t1, t2, num_points)
        tau = t - t1
        v1_mag = max(v1_fps, 0.0)
        v2_mag = max(v2_fps, 0.0)
        u = tau / T
        v_like_mag = v1_mag + (v2_mag - v1_mag) * u
        v_signed = sgn * v_like_mag  # <-- sign by direction
        # integrate numerically to distance
        x_like = np.empty_like(tau)
        x_like[0] = d1
        for i in range(1, len(tau)):
            dt = tau[i] - tau[i-1]
            x_like[i] = x_like[i-1] + 0.5 * (v_signed[i-1] + v_signed[i]) * dt
        v_mph = np.abs(v_signed) / self.MPH_TO_FPS
        v_ms_signed = v_signed / self.MPS_TO_FPS
        return {
            'time': t1 + tau,
            'min_speed': v_mph,
            'max_speed': v_mph,
            'likely_speed': v_mph,
            'signed_velocity_ms': v_ms_signed,
            'min_position': x_like,
            'max_position': x_like,
            'likely_position': x_like
        }

    # ------------------------------ Plotting -------------------------------- #
    def plot_segment_detail(self, i, data, interp, tcol='t_rel', save=True, label_prefix=""):
        # Also compute alternative (pure linear) model for side-by-side comparison
        r1, r2 = data.iloc[i], data.iloc[i+1]
        alt = self.calculate_pure_linear_segment(
            r1[tcol], r2[tcol], r1['dist_from_start'], r2['dist_from_start'], r1['speed_fps'], r2['speed_fps'], num_points=120
        )

        fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
        ax1, ax2 = axes[0,0], axes[1,0]
        ax1b, ax2b = axes[0,1], axes[1,1]
        fig.set_constrained_layout_pads(w_pad=0.08, h_pad=0.10, wspace=0.12, hspace=0.15)
        fig.suptitle(
            f"{label_prefix}Segment {i+1}\n"
            f"Time: {r1['timestamp'].strftime('%H:%M:%S')} → {r2['timestamp'].strftime('%H:%M:%S')}",
            fontweight='bold'
        )

        # Distance vs time (lens) — convert ft → m for display
        min_pos_m = interp['min_position'] * 0.3048
        max_pos_m = interp['max_position'] * 0.3048
        like_pos_m = interp['likely_position'] * 0.3048
        ax1.fill_between(interp['time'], min_pos_m, max_pos_m,
                         alpha=0.28, color='lightgray', rasterized=True)
        ax1.plot(interp['time'], min_pos_m,  lw=2, label='Min feasible')
        ax1.plot(interp['time'], max_pos_m,  lw=2, label='Max feasible')
        ax1.plot(interp['time'], like_pos_m,'k--', lw=2, alpha=0.85, label='Likely')
        ax1.scatter([r1[tcol], r2[tcol]], [r1['dist_from_start']*0.3048, r2['dist_from_start']*0.3048],
                    s=80, color='crimson', label='GPS', zorder=5, rasterized=True)
        ax1.set_ylabel('Distance (m)')
        ax1.set_title('Distance vs Time (nonlinear interpolation)')
        ax1.legend(loc='upper left')
        # X-axis tailored to segment; Distance Y-axis per segment (can change)
        t0, t1 = float(interp['time'][0]), float(interp['time'][-1])
        ax1.set_xlim(t0, t1)
        ylo = min(min_pos_m.min(), r1['dist_from_start']*0.3048, r2['dist_from_start']*0.3048)
        yhi = max(max_pos_m.max(), r1['dist_from_start']*0.3048, r2['dist_from_start']*0.3048)
        dy = max(1.0, yhi - ylo)
        ax1.set_ylim(ylo - 0.05*dy, yhi + 0.05*dy)

        # Velocity vs time — plot magnitude (no negative), even if distance decreases
        if 'signed_velocity_ms' in interp:
            mag = np.abs(interp['signed_velocity_ms'])
            min_ms = max_ms = like_ms = mag
        else:
            min_ms = interp['min_speed'] * self.MPH_TO_MPS
            max_ms = interp['max_speed'] * self.MPH_TO_MPS
            like_ms = interp['likely_speed'] * self.MPH_TO_MPS
        ax2.fill_between(interp['time'], min_ms, max_ms,
                         alpha=0.28, color='lightgray', rasterized=True)
        ax2.plot(interp['time'], min_ms,  lw=2, label='Min feasible')
        ax2.plot(interp['time'], max_ms,  lw=2, label='Max feasible')
        ax2.plot(interp['time'], like_ms,'k--', lw=2, alpha=0.85, label='Likely')
        # r1['speed'] and r2['speed'] are m/s already
        ax2.scatter([r1[tcol], r2[tcol]], [r1['speed'], r2['speed']],
                    s=80, color='crimson', label='GPS', zorder=5, rasterized=True)
        ax2.axhline(self.MAX_SPEED_MPH * self.MPH_TO_MPS, ls=':', lw=1, alpha=0.7, label='Max Velocity')
        ax2.set_xlabel('Time (s)')  # trip-relative (starts at 0)
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity vs Time (nonlinear)')
        ax2.legend(loc='upper right')
        ax2.set_xlim(t0, t1)
        ax2.set_ylim(-0.5, self.MAX_SPEED_MPH * self.MPH_TO_MPS)

        # Right column: alternative model (pure linear velocity integration)
        if alt:
            # Distance
            min_pos_m_b = alt['min_position'] * 0.3048
            max_pos_m_b = alt['max_position'] * 0.3048
            like_pos_m_b = alt['likely_position'] * 0.3048
            ax1b.fill_between(alt['time'], min_pos_m_b, max_pos_m_b, alpha=0.15, color='lightgray', rasterized=True)
            ax1b.plot(alt['time'], min_pos_m_b, lw=1.2, label='Min (alt)')
            ax1b.plot(alt['time'], max_pos_m_b, lw=1.2, label='Max (alt)')
            ax1b.plot(alt['time'], like_pos_m_b, 'k--', lw=1.6, alpha=0.85, label='Likely (alt)')
            ax1b.scatter([r1[tcol], r2[tcol]], [r1['dist_from_start']*0.3048, r2['dist_from_start']*0.3048], s=40, color='crimson', label='GPS', zorder=5, rasterized=True)
            ax1b.set_ylabel('Distance (m)')
            ax1b.set_title('Distance vs Time (linear interpolation)')
            ax1b.set_xlim(t0, t1)
            ylo_b = min(min_pos_m_b.min(), r1['dist_from_start']*0.3048, r2['dist_from_start']*0.3048)
            yhi_b = max(max_pos_m_b.max(), r1['dist_from_start']*0.3048, r2['dist_from_start']*0.3048)
            dy_b = max(1.0, yhi_b - ylo_b)
            ax1b.set_ylim(ylo_b - 0.05*dy_b, yhi_b + 0.05*dy_b)

            # Velocity magnitude (alt)
            alt_mag = np.abs(alt.get('signed_velocity_ms', (alt['likely_speed'] * self.MPH_TO_MPS)))
            ax2b.fill_between(alt['time'], alt_mag, alt_mag, alpha=0.15, color='lightgray', rasterized=True)
            ax2b.plot(alt['time'], alt_mag, 'k--', lw=1.6, alpha=0.85, label='Likely (alt)')
            # Show endpoint GPS dots clearly
            ax2b.scatter([r1[tcol], r2[tcol]], [r1['speed'], r2['speed']], s=60, color='crimson', zorder=5, label='GPS')
            ax2b.axhline(self.MAX_SPEED_MPH * self.MPH_TO_MPS, ls=':', lw=1, alpha=0.7, label='Max Velocity')
            ax2b.set_xlabel('Time (s)')
            ax2b.set_ylabel('Velocity (m/s)')
            ax2b.set_title('Velocity vs Time (linear)')
            ax2b.set_xlim(t0, t1)
            ax2b.set_ylim(-0.5, self.MAX_SPEED_MPH * self.MPH_TO_MPS)

        if save:
            fn = self.dirs['segments'] / f"{label_prefix.lower()}segment_{i+1:03d}.png"
            safe_savefig(fig, fn)
            plt.close(fig)
            return fn
        else:
            plt.show()

    def plot_full_trip_overview_with_bands(self, trip_df, interps, title, filename, tcol='t_rel'):
        if not interps: return None
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), constrained_layout=True)
        fig.set_constrained_layout_pads(w_pad=0.08, h_pad=0.10, wspace=0.12, hspace=0.15)
        fig.suptitle(title, fontweight='bold')

        # Distance vs time — overlapping semi-transparent bands (darker where many)
        for p in interps:
            p_min_m = p['min_position'] * 0.3048
            p_max_m = p['max_position'] * 0.3048
            p_like_m = p['likely_position'] * 0.3048
            ax1.fill_between(p['time'], p_min_m, p_max_m,
                             alpha=0.22, color='gray', rasterized=True)
            ax1.plot(p['time'], p_like_m, '-', lw=1.1, alpha=0.80,
                     color='black', rasterized=True)
        ax1.scatter(trip_df[tcol], trip_df['dist_from_start']*0.3048, s=10, alpha=0.65,
                    label='GPS', rasterized=True)
        ax1.set_xlabel('Time (s)')  # starts at 0 for the trip
        ax1.set_ylabel('Distance (m)')
        ax1.set_title('Distance vs Time (lens bands; reverse allowed; overlap → darker)')
        ax1.legend(loc='upper left')

        # Velocity vs time — plot magnitude (no negative)
        for p in interps:
            if 'signed_velocity_ms' in p:
                mag = np.abs(p['signed_velocity_ms'])
                p_min_ms = p_max_ms = p_like_ms = mag
            else:
                p_min_ms = p['min_speed'] * self.MPH_TO_MPS
                p_max_ms = p['max_speed'] * self.MPH_TO_MPS
                p_like_ms = p['likely_speed'] * self.MPH_TO_MPS
            p_min_s_arr = p['min_speed']
            p_max_s_arr = p['max_speed']
            if 'min_speed_smooth' in p:
                p_min_s_arr = p['min_speed_smooth']
            if 'max_speed_smooth' in p:
                p_max_s_arr = p['max_speed_smooth']
            p_min_s_ms = p_min_s_arr * self.MPH_TO_MPS
            p_max_s_ms = p_max_s_arr * self.MPH_TO_MPS
            ax2.fill_between(p['time'], p_min_s_ms, p_max_s_ms,
                             alpha=0.22, color='gray', rasterized=True)
            ax2.plot(p['time'], p_like_ms, '-', lw=1.1, alpha=0.80,
                     color='black', rasterized=True)
        ax2.scatter(trip_df[tcol], trip_df['speed'], s=10, alpha=0.65,
                    label='GPS', rasterized=True)
        ax2.axhline(self.MAX_SPEED_MPH * self.MPH_TO_MPS, ls='--', lw=1, alpha=0.6, label='Max Velocity')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity vs Time (feasible bands)')
        ax2.legend(loc='upper right')

        out = self.dirs['overview'] / filename
        safe_savefig(fig, out)
        plt.close(fig)
        return out

    def plot_full_trip_overview_linear(self, trip_df, interps, title, filename, tcol='t_rel'):
        if not interps: return None
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), constrained_layout=True)
        fig.set_constrained_layout_pads(w_pad=0.08, h_pad=0.10, wspace=0.12, hspace=0.15)
        fig.suptitle(title, fontweight='bold')

        # Distance vs time (alt)
        for p in interps:
            p_min_m = p['min_position'] * 0.3048
            p_max_m = p['max_position'] * 0.3048
            p_like_m = p['likely_position'] * 0.3048
            ax1.fill_between(p['time'], p_min_m, p_max_m, alpha=0.18, color='gray', rasterized=True)
            ax1.plot(p['time'], p_like_m, '-', lw=1.1, alpha=0.85, color='black', rasterized=True)
        ax1.scatter(trip_df[tcol], trip_df['dist_from_start']*0.3048, s=10, alpha=0.65, label='GPS', rasterized=True)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance (m)')
        ax1.set_title('Distance vs Time (linear velocity integration)')

        # Velocity vs time (alt)
        for p in interps:
            if 'signed_velocity_ms' in p:
                v_ms = np.abs(p['signed_velocity_ms'])
            else:
                v_ms = p['likely_speed'] * self.MPH_TO_MPS
            ax2.plot(p['time'], v_ms, '-', lw=1.1, alpha=0.85, color='black', rasterized=True)
        try:
            ax2.scatter(trip_df[tcol], trip_df['speed'], s=10, alpha=0.65, label='GPS', rasterized=True)
        except Exception:
            pass
        ax2.axhline(self.MAX_SPEED_MPH * self.MPH_TO_MPS, ls='--', lw=1, alpha=0.6, label='Max Velocity')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity vs Time (linear)')
        ax2.set_ylim(0.0, self.MAX_SPEED_MPH * self.MPH_TO_MPS)
        ax2.legend(loc='upper right')

        out = self.dirs['overview'] / filename
        safe_savefig(fig, out)
        plt.close(fig)
        return out

    # ----------------------------- Reports / IO ------------------------------ #
    def generate_segment_report(self, i, data, interp, tcol='t_rel'):
        r1, r2 = data.iloc[i], data.iloc[i+1]
        return {
            'segment_number': i+1,
            'start_time': r1['timestamp'].isoformat(),
            'end_time':   r2['timestamp'].isoformat(),
            'duration_seconds': float(r2[tcol] - r1[tcol]),
            'start_position_ft': float(r1['dist_from_start']),
            'end_position_ft':   float(r2['dist_from_start']),
            'distance_traveled_ft': float(r2['dist_from_start'] - r1['dist_from_start']),
            'start_speed_mph': float(r1['speed'] * self.MPS_TO_MPH),
            'end_speed_mph':   float(r2['speed'] * self.MPS_TO_MPH),
            'min_possible_speed_mph': float(np.min(interp['min_speed'])),
            'max_possible_speed_mph': float(np.max(interp['max_speed'])),
            'avg_likely_speed_mph':   float(np.mean(interp['likely_speed'])),
            'min_possible_velocity_ms': float(np.min(interp['min_speed']) * self.MPH_TO_MPS),
            'max_possible_velocity_ms': float(np.max(interp['max_speed']) * self.MPH_TO_MPS),
            'avg_likely_velocity_ms':   float(np.mean(interp['likely_speed']) * self.MPH_TO_MPS),
            'max_speed_uncertainty_mph': float(np.max(interp['max_speed'] - interp['min_speed'])),
            'vehicle_id': int(r1['vehicle_id']) if 'vehicle_id' in data.columns and pd.notna(r1['vehicle_id']) else None,
            'trip_id':    str(r1['trip_id'])    if 'trip_id'    in data.columns and pd.notna(r1['trip_id'])    else None,
            'direction':  str(r1['direction'])  if 'direction'  in data.columns and pd.notna(r1['direction'])  else None
        }

    def create_summary_report(self, data, interps, seg_reports, label=""):
        summary = {
            'analysis_timestamp': datetime.now().isoformat(),
            'label': label,
            'points': len(data),
            'segments': len(interps),
            'time_range_s': [float(data['t_rel'].min()), float(data['t_rel'].max())],
            'duration_minutes': float((data['t_rel'].max() - data['t_rel'].min())/60.0),
            'distance_ft': float(data['dist_from_start'].max() - data['dist_from_start'].min()),
            'speed_stats': {
                'avg_mph': float((data['speed'] * self.MPS_TO_MPH).mean()),
                'max_mph': float((data['speed'] * self.MPS_TO_MPH).max()),
                'min_mph': float((data['speed'] * self.MPS_TO_MPH).min()),
                'std_mph': float((data['speed'] * self.MPS_TO_MPH).std())
            },
            'velocity_stats_ms': {
                'avg_ms': float(data['speed'].mean()),
                'max_ms': float(data['speed'].max()),
                'min_ms': float(data['speed'].min()),
                'std_ms': float(data['speed'].std())
            },
            'constraints': {
                'a_mph_s': self.MAX_ACCEL_MS2 * self.MPS_TO_MPH,
                'b_mph_s': self.MAX_DECEL_MS2 * self.MPS_TO_MPH,
                'vmax_mph': self.MAX_SPEED_MPH,
                'reverse_allowed': True
            }
        }
        with open(self.dirs['reports'] / f'analysis_summary_{label}.json', 'w') as f:
            json.dump(summary, f, indent=2)
        with open(self.dirs['reports'] / f'analysis_summary_{label}.txt', 'w') as f:
            f.write(json.dumps(summary, indent=2))

    def export_all_data(self, data, interps, label=""):
        data.to_csv(self.dirs['data'] / f'original_gps_points_{label}.csv', index=False)
        rows = []
        for si, p in enumerate(interps, start=1):
            for k in range(len(p['time'])):
                rows.append({
                    'segment': si,
                    'time_s': float(p['time'][k]),
                    'min_speed_mph': float(p['min_speed'][k]),
                    'max_speed_mph': float(p['max_speed'][k]),
                    'likely_speed_mph': float(p['likely_speed'][k]),
                    'min_velocity_ms': float(p['min_speed'][k] * self.MPH_TO_MPS),
                    'max_velocity_ms': float(p['max_speed'][k] * self.MPH_TO_MPS),
                    'likely_velocity_ms': float(p['likely_speed'][k] * self.MPH_TO_MPS),
                    'min_position_ft': float(p['min_position'][k]),
                    'max_position_ft': float(p['max_position'][k]),
                    'likely_position_ft': float(p['likely_position'][k]),
                })
        pd.DataFrame(rows).to_csv(self.dirs['data'] / f'interpolated_trajectories_{label}.csv', index=False)

    # ---------------------------- Direction utils ---------------------------- #
    def _guess_direction_values(self):
        """Return (inbound_value, outbound_value) or (None, None) if not available."""
        if 'direction' not in self.df.columns:
            print("[WARN] No 'direction' column in CSV.")
            return (None, None)
        vals = pd.Series(self.df['direction'].dropna().unique())
        if vals.empty:
            print("[WARN] 'direction' column empty.")
            return (None, None)
        if vals.dtype == object:
            lo = vals.astype(str).str.lower()
            inbound  = vals[lo.str.contains(r'\b(in|ib|inbound)\b', regex=True, na=False)]
            outbound = vals[lo.str.contains(r'\b(out|ob|outbound)\b', regex=True, na=False)]
            in_val  = inbound.iloc[0]  if len(inbound)  else (vals.iloc[0] if len(vals) else None)
            out_val = outbound.iloc[0] if len(outbound) else (vals.iloc[1] if len(vals) > 1 else None)
            return (in_val, out_val)
        return (vals.min(), vals.max() if len(vals) > 1 else None)

    def _select_full_trip(self, direction_value):
        """Pick the trip in this direction with the largest distance span, rebase time to 0."""
        if ('trip_id' not in self.df.columns) or ('direction' not in self.df.columns):
            print("[WARN] Need 'trip_id' and 'direction' columns to split trips.")
            return (None, None)
        sub = self.df[self.df['direction'] == direction_value].copy()
        if sub.empty: return (None, None)
        spans = sub.groupby('trip_id')['dist_from_start'].agg(lambda s: s.max()-s.min()).sort_values(ascending=False)
        if spans.empty: return (None, None)
        best = spans.index[0]
        trip = sub[sub['trip_id']==best].sort_values('timestamp').reset_index(drop=True)
        t0 = trip['time_seconds'].min()
        trip['t_rel'] = trip['time_seconds'] - t0
        return (best, trip)

    def analyze_trip_by_id(self, trip_id: str, label_prefix: str | None = None, segments_to_plot: int | None = None, generate_plots: bool = True):
        """Analyze a specific trip by exact trip_id, generating all PNGs and CSVs."""
        if 'trip_id' not in self.df.columns:
            print("[ERROR] CSV has no 'trip_id' column.")
            return None
        trip = self.df[self.df['trip_id'] == trip_id].copy()
        if trip.empty or len(trip) < 2:
            print(f"[ERROR] No rows for trip_id={trip_id} or too few points.")
            return None
        trip = trip.sort_values('timestamp').reset_index(drop=True)
        t0 = trip['time_seconds'].min()
        trip['t_rel'] = trip['time_seconds'] - t0
        if label_prefix is None:
            dir_val = str(trip.iloc[0]['direction']) if 'direction' in trip.columns else ''
            lp = 'Inbound' if isinstance(dir_val, str) and dir_val.lower().startswith('in') else 'Outbound'
        else:
            lp = label_prefix
        label = f"{lp}_trip_{trip_id}"
        print(f"\n=== {label} (t starts at 0 s; reverse allowed) ===")
        interps, seg_reports = [], []
        for i in range(len(trip)-1):
            p = self.calculate_linear_velocity_segment(
                trip.iloc[i]['t_rel'], trip.iloc[i+1]['t_rel'],
                trip.iloc[i]['dist_from_start'], trip.iloc[i+1]['dist_from_start'],
                trip.iloc[i]['speed_fps'], trip.iloc[i+1]['speed_fps'],
                num_points=120
            )
            if p:
                interps.append(p)
                seg_reports.append(self.generate_segment_report(i, trip, p, tcol='t_rel'))
                if generate_plots and (segments_to_plot is None or i < segments_to_plot):
                    self.plot_segment_detail(i, trip, p, tcol='t_rel', save=True, label_prefix=f"{lp} ")
        if generate_plots:
            self.plot_full_trip_overview_with_bands(
                trip, interps,
                title=f"{lp} Full Trip Overview — nonlinear interpolation",
                filename=f"{lp.lower()}_full_trip_overview.png",
                tcol='t_rel'
            )
        if generate_plots:
            interps_alt = []
            for j in range(len(trip)-1):
                alt = self.calculate_pure_linear_segment(
                    trip.iloc[j]['t_rel'], trip.iloc[j+1]['t_rel'],
                    trip.iloc[j]['dist_from_start'], trip.iloc[j+1]['dist_from_start'],
                    trip.iloc[j]['speed_fps'], trip.iloc[j+1]['speed_fps'], num_points=120)
                if alt:
                    interps_alt.append(alt)
            self.plot_full_trip_overview_linear(
                trip, interps_alt,
                title=f"{lp} Full Trip Overview — linear interpolation",
                filename=f"{lp.lower()}_full_trip_overview_linear.png",
                tcol='t_rel'
            )
        self.create_summary_report(trip, interps, seg_reports, label=label)
        self.export_all_data(trip, interps, label=label)
        return trip, interps, seg_reports

    def analyze_direction_full_trip(self, direction_value, label_prefix="Inbound", segments_to_plot: int | None = None):
        trip_id, data = self._select_full_trip(direction_value)
        if trip_id is None or len(data) < 2:
            print(f"[WARN] No full trip for direction={direction_value}")
            return None

        label = f"{label_prefix}_trip_{trip_id}"
        print(f"\n=== {label} (t starts at 0 s; reverse allowed) ===")
        interps, seg_reports = [], []

        for i in range(len(data)-1):
            p = self.calculate_linear_velocity_segment(
                data.iloc[i]['t_rel'], data.iloc[i+1]['t_rel'],
                data.iloc[i]['dist_from_start'], data.iloc[i+1]['dist_from_start'],
                data.iloc[i]['speed_fps'], data.iloc[i+1]['speed_fps'],
                num_points=120
            )
            if p:
                interps.append(p)
                seg_reports.append(self.generate_segment_report(i, data, p, tcol='t_rel'))
                if segments_to_plot is None or i < segments_to_plot:
                    self.plot_segment_detail(i, data, p, tcol='t_rel', save=True, label_prefix=f"{label_prefix} ")

        self.plot_full_trip_overview_with_bands(
            data, interps,
            title=f"{label_prefix} Full Trip Overview (Trip {trip_id}) — lens w/ reverse",
            filename=f"{label_prefix.lower()}_full_trip_overview.png",
            tcol='t_rel'
        )
        interps_alt = []
        for i in range(len(data)-1):
            alt = self.calculate_pure_linear_segment(
                data.iloc[i]['t_rel'], data.iloc[i+1]['t_rel'],
                data.iloc[i]['dist_from_start'], data.iloc[i+1]['dist_from_start'],
                data.iloc[i]['speed_fps'], data.iloc[i+1]['speed_fps'], num_points=120)
            if alt:
                interps_alt.append(alt)
        self.plot_full_trip_overview_linear(
            data, interps_alt,
            title=f"{label_prefix} Full Trip Overview (linear velocity)",
            filename=f"{label_prefix.lower()}_full_trip_overview_linear.png",
            tcol='t_rel'
        )
        self.create_summary_report(data, interps, seg_reports, label=label)
        self.export_all_data(data, interps, label=label)
        return data, interps, seg_reports

    def analyze_all_trips(self, segments_to_plot: int | None = 0, generate_plots: bool = False):
        """Interpolate every trip_id present in the CSV and export per-trip CSV + summary.
        By default, skips PNG generation for speed; set generate_plots=True to also save figures.
        """
        if 'trip_id' not in self.df.columns:
            print("[ERROR] CSV has no 'trip_id' column.")
            return
        trip_ids = self.df['trip_id'].dropna().astype(str).unique().tolist()
        print(f"Found {len(trip_ids)} trips. Interpolating all…")
        done = 0
        for tid in trip_ids:
            try:
                self.analyze_trip_by_id(tid, segments_to_plot=segments_to_plot, generate_plots=generate_plots)
                done += 1
            except Exception as e:
                print(f"[WARN] Failed trip_id={tid}: {e}")
        print(f"Completed {done}/{len(trip_ids)} trips. Outputs in {self.output_dir}.")

# =================================== MAIN ================================== #
if __name__ == "__main__":
    print("="*70)
    print("  TRAIN TRAJECTORY INTERPOLATOR — lens envelopes, v can be < 0, t=0 per trip")
    print("="*70)
    set_plot_style()
    plt.ioff()

    parser = argparse.ArgumentParser()
    parser.add_argument('--trip-id', dest='trip_id', default=None, help='Analyze only this exact trip_id')
    parser.add_argument('--all-trips', dest='all_trips', action='store_true', help='Analyze all trips in the CSV (no plots by default)')
    parser.add_argument('--plots', dest='plots', action='store_true', help='When analyzing many trips, also generate PNG plots')
    parser.add_argument('--csv', dest='csv', default='4_k_line_data_with_trip_id.csv')
    args = parser.parse_args()

    ti = TrainTrajectoryInterpolator(
        csv_file=args.csv,
        output_dir='trajectory_analysis_output/Results'
    )

    if args.all_trips:
        ti.analyze_all_trips(segments_to_plot=0, generate_plots=bool(args.plots))
    elif args.trip_id:
        ti.analyze_trip_by_id(args.trip_id, segments_to_plot=None)
    else:
        inbound_val, outbound_val = ti._guess_direction_values()
        print(f"Detected: inbound={inbound_val}, outbound={outbound_val}")
        if inbound_val is not None:
            ti.analyze_direction_full_trip(inbound_val, label_prefix="Inbound", segments_to_plot=None)
        else:
            print("[WARN] No inbound direction detected.")
        if outbound_val is not None:
            ti.analyze_direction_full_trip(outbound_val, label_prefix="Outbound", segments_to_plot=None)
        else:
            print("[WARN] No outbound direction detected.")

    print("\nDone. Check the 'overview' and 'individual_segments' folders.")