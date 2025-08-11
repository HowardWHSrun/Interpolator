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
import argparse
warnings.filterwarnings('ignore')

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
    Time for each full trip is rebased to start at 0 s. Reverse motion is disabled.
    Builds one inbound and one outbound full-trip overview.
    """

    def __init__(self, csv_file='4_k_line_data_with_trip_id.csv', output_dir='trajectory_analysis_output'):
        # Siemens S200 constraints (hard bounds)
        self.MAX_ACCEL_MPH_S = 3.0   # a+ (mph/s)
        self.MAX_DECEL_MPH_S = 5.0   # b   (mph/s)
        self.MAX_SPEED_MPH   = 60.0

        # Likely-profile assumptions (user-provided)
        self.LIKELY_ACCEL_MPH_S = 2.0   # symmetric accel/decel for likely curve
        self.ALLOW_REVERSE      = False # no reverse travel for bounds/likely

        # Stop/dwell assumptions (seconds) and thresholds
        self.STATION_DWELL_MIN_S = 10.0
        self.STATION_DWELL_MAX_S = 40.0
        self.INTERSECTION_DWELL_MAX_S = 90.0
        self.SHORT_MOVE_THRESHOLD_FT = 80.0
        self.STOP_SPEED_THRESHOLD_MPH = 0.5

        # Conversions
        self.MPH_TO_FPS = 1.46667
        self.a  = self.MAX_ACCEL_MPH_S * self.MPH_TO_FPS   # ft/s^2 (accelerating)
        self.b  = self.MAX_DECEL_MPH_S * self.MPH_TO_FPS   # ft/s^2 (braking)
        self.V  = self.MAX_SPEED_MPH   * self.MPH_TO_FPS   # ft/s (|v| <= V)
        self.a_like = self.LIKELY_ACCEL_MPH_S * self.MPH_TO_FPS  # ft/s^2 for likely

        # Dirs + data
        self.output_dir = Path(output_dir)
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
        df['speed_fps']    = df['speed'] * self.MPH_TO_FPS
        self.df = df
        print(f"Loaded {len(df)} rows; {df['timestamp'].min()} → {df['timestamp'].max()}")

    # ------------------------- Forward distances (allow v<0) ----------------- #
    def _fwd_min_dist(self, v1, t):
        """
        Minimal displacement in time t from start, allowing reverse:
        - If v1>0: brake at b to 0, then accelerate at a to -V and cruise.
        - If v1<=0: accelerate at a toward -V and cruise.
        """
        if t <= 0: return 0.0
        if v1 > 0:
            t_stop = v1 / self.b
            s_stop = v1*t_stop - 0.5*self.b*t_stop*t_stop
            if t <= t_stop:
                return v1*t - 0.5*self.b*t*t
            # After stopping: either wait (no reverse) or reverse if allowed
            if not self.ALLOW_REVERSE:
                return s_stop
            t_left = t - t_stop
            t_to_negV = self.V / self.a
            if t_left <= t_to_negV:
                return s_stop - 0.5*self.a*t_left*t_left
            s_to_negV = -0.5*self.a*t_to_negV*t_to_negV
            return s_stop + s_to_negV + (-self.V)*(t_left - t_to_negV)
        else:
            # v1 <= 0 in normalized frame is rare here; if occurs, handle per reverse setting
            if not self.ALLOW_REVERSE:
                # minimal displacement is to stay put; do not go negative
                return 0.0
            t_to_negV = max(0.0, (-self.V - v1) / self.a)
            if t <= t_to_negV:
                return v1*t - 0.5*self.a*t*t
            s_to_negV = v1*t_to_negV - 0.5*self.a*t_to_negV*t_to_negV
            return s_to_negV + (-self.V)*(t - t_to_negV)

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
                if self.ALLOW_REVERSE:
                    v_min = max(-self.V, -self.a*(t - t_stop))
                else:
                    v_min = 0.0
        else:
            if self.ALLOW_REVERSE:
                v_min = max(-self.V, v1 - self.a*t)
            else:
                v_min = 0.0
        # upper bound
        v_max = min(self.V, v1 + self.a*t)
        if not self.ALLOW_REVERSE:
            v_min = max(0.0, v_min)
            v_max = max(0.0, v_max)
        return v_min, v_max

    def _to_end_speed_bounds_simple(self, v2, t_to_end):
        """Conservative current-speed interval that can still hit v2 in t_to_end."""
        if self.ALLOW_REVERSE:
            lo = max(-self.V, v2 - self.a*t_to_end)
            hi = min( self.V, v2 + self.b*t_to_end)
        else:
            lo = max(0.0, v2 - self.a*t_to_end)
            hi = max(0.0, min(self.V, v2 + self.b*t_to_end))
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
        # In normalized frame, enforce non-negative speeds (we do not trust speed sign)
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

        # 5) Instantaneous speed bounds (respect no-reverse if requested)
        if self.ALLOW_REVERSE:
            v_from_start_min = np.maximum(-self.V, v1n - self.b * tau)
            v_from_start_max = np.minimum( self.V, v1n + self.a * tau)
            v_to_end_min     = np.maximum(-self.V, v2n - self.a * (T - tau))
            v_to_end_max     = np.minimum( self.V, v2n + self.b * (T - tau))
        else:
            v_from_start_min = np.maximum(0.0, v1n - self.b * tau)
            v_from_start_max = np.maximum(0.0, np.minimum(self.V, v1n + self.a * tau))
            v_to_end_min     = np.maximum(0.0, v2n - self.a * (T - tau))
            v_to_end_max     = np.maximum(0.0, np.minimum(self.V, v2n + self.b * (T - tau)))
        v_lo_n = np.maximum(v_from_start_min, v_to_end_min)
        v_hi_n = np.minimum(v_from_start_max, v_to_end_max)

        # likely speed: physics-based profile (no reverse, 2 mph/s), with short-dwell heuristics
        v_like_n = self._likely_speed_profile_no_reverse(
            T=T,
            S=abs(d2n - d1n),
            v_start=v1n,
            v_end=v2n,
            tau=tau
        )
        # ensure within feasible speed bounds
        v_like_n = np.clip(v_like_n, v_lo_n, v_hi_n)
        x_like_n = np.empty_like(tau)
        x_like_n[0] = d1n
        for i in range(1, len(tau)):
            dt = tau[i] - tau[i - 1]
            x_like_n[i] = x_like_n[i - 1] + 0.5 * (v_like_n[i - 1] + v_like_n[i]) * dt
            x_like_n[i] = min(max(x_like_n[i], x_lo_n[i]), x_hi_n[i])

        # 6) Transform back to original frame
        x_min = sgn * x_lo_n
        x_max = sgn * x_hi_n
        x_like = sgn * x_like_n
        x_min, x_max = np.minimum(x_min, x_max), np.maximum(x_min, x_max)

        v_lo = (sgn * v_lo_n) / self.MPH_TO_FPS
        v_hi = (sgn * v_hi_n) / self.MPH_TO_FPS
        v_like = (sgn * v_like_n) / self.MPH_TO_FPS
        v_min = np.minimum(v_lo, v_hi)
        v_max = np.maximum(v_lo, v_hi)

        return {
            'time': t1 + tau,
            'min_speed': v_min,
            'max_speed': v_max,
            'likely_speed': v_like,
            'min_position': x_min,
            'max_position': x_max,
            'likely_position': x_like
        }

    # -------------------------- Likely profile (no reverse) ------------------ #
    def _likely_speed_profile_no_reverse(self, T, S, v_start, v_end, tau):
        """
        Build a likely speed profile using a triangular/trapezoidal velocity curve
        with symmetric accel/decel at LIKELY accel, optional dwell when both ends
        are stopped and S is short (car-length moves). All speeds in ft/s.
        """
        if T <= 0:
            return np.zeros_like(tau)

        a = max(1e-6, self.a_like)
        V = self.V
        v0 = float(np.clip(v_start, 0.0, V))
        v1 = float(np.clip(v_end,   0.0, V))
        S = max(0.0, S)

        # Helper: compute minimal-time profile (triangle or trapezoid at V)
        def minimal_time_profile(v0_, v1_, S_):
            # Triangular v_peak needed ignoring V
            v_peak_needed_sq = a * S_ + 0.5 * (v0_ * v0_ + v1_ * v1_)
            v_peak_needed = np.sqrt(max(0.0, v_peak_needed_sq))
            if v_peak_needed <= V + 1e-9:
                # Triangular
                vpk = v_peak_needed
                ta = max(0.0, (vpk - v0_) / a)
                td = max(0.0, (vpk - v1_) / a)
                tc = 0.0
                Tm = ta + td
                return Tm, ta, tc, td, vpk
            # Trapezoid capped at V
            ta = max(0.0, (V - v0_) / a)
            td = max(0.0, (V - v1_) / a)
            S_acc = (v0_ + V) * 0.5 * ta
            S_dec = (V + v1_) * 0.5 * td
            S_cruise = S_ - S_acc - S_dec
            if S_cruise <= 0.0:
                # Actually triangular below V
                vpk = np.sqrt(max(0.0, a * S_ + 0.5 * (v0_ * v0_ + v1_ * v1_)))
                ta = max(0.0, (vpk - v0_) / a)
                td = max(0.0, (vpk - v1_) / a)
                tc = 0.0
                Tm = ta + td
                return Tm, ta, tc, td, vpk
            tc = S_cruise / V
            Tm = ta + tc + td
            return Tm, ta, tc, td, V

        T_min, ta_min, tc_min, td_min, vpk_min = minimal_time_profile(v0, v1, S)

        # Dwell heuristics based on both ends nearly stopped
        mph_eps = 0.5
        v_eps = mph_eps * self.MPH_TO_FPS
        dwell_start = 0.0

        if v0 <= v_eps and v1 <= v_eps:
            slack = max(0.0, T - T_min)
            if S <= 80.0:
                # Intersection-like creep: allow 0–90 s dwell if slack permits
                dwell_start = min(90.0, slack)
            else:
                # Station-like stop: prefer 10–40 s if slack allows
                if slack >= 10.0:
                    dwell_start = min(40.0, slack)
                else:
                    dwell_start = 0.0

        # Final movement time and phases
        T_move = max(0.0, T - dwell_start)
        # Recompute phases to exactly meet S without exceeding V
        # If T_move > T_min, distribute the extra as cruise time at vpk_min (or V)
        extra = max(0.0, T_move - T_min)
        # Try to keep peak at vpk_min and extend cruise if possible
        ta, tc, td, vpk = ta_min, tc_min + extra, td_min, vpk_min

        # Build piecewise speed profile
        speeds = np.zeros_like(tau)
        t0 = dwell_start
        t1 = t0 + ta
        t2 = t1 + tc
        t3 = t2 + td

        for i, t in enumerate(tau):
            if t <= t0:
                speeds[i] = 0.0 if v0 <= v_eps else v0
            elif t <= t1:
                dt = t - t0
                speeds[i] = v0 + a * dt
            elif t <= t2:
                speeds[i] = vpk
            elif t <= t3:
                dt = t - t2
                speeds[i] = max(0.0, vpk - a * dt)
            else:
                speeds[i] = 0.0 if v1 <= v_eps else v1

        return speeds

    # ------------------------------ Plotting -------------------------------- #
    def plot_segment_detail(self, i, data, interp, tcol='t_rel', save=True, label_prefix=""):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
        fig.set_constrained_layout_pads(w_pad=0.08, h_pad=0.10, wspace=0.12, hspace=0.15)
        r1, r2 = data.iloc[i], data.iloc[i+1]
        fig.suptitle(
            f"{label_prefix}Segment {i+1}\n"
            f"Time: {r1['timestamp'].strftime('%H:%M:%S')} → {r2['timestamp'].strftime('%H:%M:%S')}",
            fontweight='bold'
        )

        # Distance vs time (lens)
        ax1.fill_between(interp['time'], interp['min_position'], interp['max_position'],
                         alpha=0.28, color='lightgray', rasterized=True)
        ax1.plot(interp['time'], interp['min_position'],  lw=2, label='Min feasible')
        ax1.plot(interp['time'], interp['max_position'],  lw=2, label='Max feasible')
        ax1.plot(interp['time'], interp['likely_position'],'k--', lw=2, alpha=0.85, label='Likely')
        ax1.scatter([r1[tcol], r2[tcol]], [r1['dist_from_start'], r2['dist_from_start']],
                    s=80, color='crimson', label='GPS', zorder=5, rasterized=True)
        ax1.set_ylabel('Distance (ft)')
        ax1.set_title('Distance vs Time (lens; no reverse)')
        ax1.legend(loc='upper left')
        t0, t1 = float(interp['time'][0]), float(interp['time'][-1])
        dx = max(1e-6, t1 - t0)
        ax1.set_xlim(t0 - 0.03*dx, t1 + 0.03*dx)
        ylo = min(interp['min_position'].min(), r1['dist_from_start'], r2['dist_from_start'])
        yhi = max(interp['max_position'].max(), r1['dist_from_start'], r2['dist_from_start'])
        dy = max(1.0, yhi - ylo)
        ax1.set_ylim(ylo - 0.05*dy, yhi + 0.05*dy)

        # Speed vs time
        ax2.fill_between(interp['time'], interp['min_speed'], interp['max_speed'],
                         alpha=0.28, color='lightgray', rasterized=True)
        ax2.plot(interp['time'], interp['min_speed'],  lw=2, label='Min feasible')
        ax2.plot(interp['time'], interp['max_speed'],  lw=2, label='Max feasible')
        ax2.plot(interp['time'], interp['likely_speed'],'k--', lw=2, alpha=0.85, label='Likely')
        ax2.scatter([r1[tcol], r2[tcol]], [r1['speed'], r2['speed']],
                    s=80, color='crimson', label='GPS', zorder=5, rasterized=True)
        ax2.axhline(self.MAX_SPEED_MPH, ls=':', lw=1, alpha=0.7, label='Max Speed')
        ax2.set_xlabel('Time (s)')  # trip-relative (starts at 0)
        ax2.set_ylabel('Speed (mph)')
        ax2.set_title('Speed vs Time (no reverse)')
        ax2.legend(loc='upper right')
        ax2.set_xlim(t0 - 0.03*dx, t1 + 0.03*dx)
        y2lo = min(interp['min_speed'].min(), r1['speed'], r2['speed'])
        y2hi = max(interp['max_speed'].max(), r1['speed'], r2['speed'], self.MAX_SPEED_MPH)
        dy2 = max(0.5, y2hi - y2lo)
        ax2.set_ylim(y2lo - 0.06*dy2, y2hi + 0.06*dy2)

        if save:
            fn = self.dirs['segments'] / f"{label_prefix.lower()}segment_{i+1:03d}.png"
            safe_savefig(fig, fn)
            plt.close(fig)
            return fn
        else:
            plt.show()

    def plot_full_trip_overview_with_bands(self, trip_df, interps, title, filename, tcol='t_rel', stops=None):
        if not interps: return None
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), constrained_layout=True)
        fig.set_constrained_layout_pads(w_pad=0.08, h_pad=0.10, wspace=0.12, hspace=0.15)
        fig.suptitle(title, fontweight='bold')

        # Distance vs time — overlapping semi-transparent bands (darker where many)
        for p in interps:
            ax1.fill_between(p['time'], p['min_position'], p['max_position'],
                             alpha=0.22, color='gray', rasterized=True)
            ax1.plot(p['time'], p['likely_position'], '-', lw=1.1, alpha=0.80,
                     color='black', rasterized=True)
        ax1.scatter(trip_df[tcol], trip_df['dist_from_start'], s=10, alpha=0.65,
                    label='GPS', rasterized=True)
        # annotate detected stops as vertical bands
        if stops:
            for s in stops:
                t0, t1 = s['t_start_s'], s['t_end_s']
                color = 'orange' if s['type'] == 'station' else 'skyblue'
                ax1.axvspan(t0, t1, color=color, alpha=0.15)
        ax1.set_xlabel('Time (s)')  # starts at 0 for the trip
        ax1.set_ylabel('Distance (ft)')
        ax1.set_title('Distance vs Time (lens bands; no reverse; overlap → darker)')
        ax1.legend(loc='upper left')

        # Speed vs time
        for p in interps:
            ax2.fill_between(p['time'], p['min_speed'], p['max_speed'],
                             alpha=0.22, color='gray', rasterized=True)
            ax2.plot(p['time'], p['likely_speed'], '-', lw=1.1, alpha=0.80,
                     color='black', rasterized=True)
        ax2.scatter(trip_df[tcol], trip_df['speed'], s=10, alpha=0.65,
                    label='GPS', rasterized=True)
        ax2.axhline(self.MAX_SPEED_MPH, ls='--', lw=1, alpha=0.6, label='Max Speed')
        # annotate stops on speed plot
        if stops:
            for s in stops:
                t0, t1 = s['t_start_s'], s['t_end_s']
                color = 'orange' if s['type'] == 'station' else 'skyblue'
                ax2.axvspan(t0, t1, color=color, alpha=0.15)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (mph)')
        ax2.set_title('Speed vs Time (feasible bands; no reverse)')
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
            'start_speed_mph': float(r1['speed']),
            'end_speed_mph':   float(r2['speed']),
            'min_possible_speed_mph': float(np.min(interp['min_speed'])),
            'max_possible_speed_mph': float(np.max(interp['max_speed'])),
            'avg_likely_speed_mph':   float(np.mean(interp['likely_speed'])),
            'max_speed_uncertainty_mph': float(np.max(interp['max_speed'] - interp['min_speed'])),
            'vehicle_id': int(r1['vehicle_id']) if 'vehicle_id' in data.columns and pd.notna(r1['vehicle_id']) else None,
            'trip_id':    str(r1['trip_id'])    if 'trip_id'    in data.columns and pd.notna(r1['trip_id'])    else None,
            'direction':  str(r1['direction'])  if 'direction'  in data.columns and pd.notna(r1['direction'])  else None
        }

    def create_summary_report(self, data, interps, seg_reports, stops, label=""):
        summary = {
            'analysis_timestamp': datetime.now().isoformat(),
            'label': label,
            'points': len(data),
            'segments': len(interps),
            'time_range_s': [float(data['t_rel'].min()), float(data['t_rel'].max())],
            'duration_minutes': float((data['t_rel'].max() - data['t_rel'].min())/60.0),
            'distance_ft': float(data['dist_from_start'].max() - data['dist_from_start'].min()),
            'speed_stats': {
                'avg_mph': float(data['speed'].mean()),
                'max_mph': float(data['speed'].max()),
                'min_mph': float(data['speed'].min()),
                'std_mph': float(data['speed'].std())
            },
            'constraints': {
                'a_mph_s': self.MAX_ACCEL_MPH_S,
                'b_mph_s': self.MAX_DECEL_MPH_S,
                'vmax_mph': self.MAX_SPEED_MPH,
                'reverse_allowed': bool(self.ALLOW_REVERSE),
                'likely_accel_mph_s': self.LIKELY_ACCEL_MPH_S
            },
            'dwell_rules': {
                'short_move_threshold_ft': 80,
                'station_dwell_min_s': 10,
                'station_dwell_max_s': 40,
                'intersection_dwell_max_s': 90
            },
            'stops_summary': {
                'total_stops': len(stops) if stops else 0,
                'stations': int(sum(1 for s in stops if s['type']== 'station')) if stops else 0,
                'intersections': int(sum(1 for s in stops if s['type']== 'intersection')) if stops else 0
            }
        }
        with open(self.dirs['reports'] / f'analysis_summary_{label}.json', 'w') as f:
            json.dump(summary, f, indent=2)
        with open(self.dirs['reports'] / f'analysis_summary_{label}.txt', 'w') as f:
            f.write(json.dumps(summary, indent=2))
        # write segment and stops details
        if seg_reports:
            with open(self.dirs['reports'] / f'segment_reports_{label}.json', 'w') as f:
                json.dump(seg_reports, f, indent=2)
        if stops is not None:
            with open(self.dirs['reports'] / f'detected_stops_{label}.json', 'w') as f:
                json.dump(stops, f, indent=2)

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
                    'min_position_ft': float(p['min_position'][k]),
                    'max_position_ft': float(p['max_position'][k]),
                    'likely_position_ft': float(p['likely_position'][k]),
                })
        pd.DataFrame(rows).to_csv(self.dirs['data'] / f'interpolated_trajectories_{label}.csv', index=False)

        # Also export a decimated overview for interactive web plotting (performance)
        try:
            MAX_POINTS = 5000
            if len(rows) > MAX_POINTS:
                step = max(1, len(rows) // MAX_POINTS)
                decimated = [rows[i] for i in range(0, len(rows), step)]
                # ensure last point is included
                if decimated[-1] is not rows[-1]:
                    decimated.append(rows[-1])
            else:
                decimated = rows
            pd.DataFrame(decimated).to_csv(self.dirs['data'] / f'interpolated_trajectories_overview_{label}.csv', index=False)
        except Exception as e:
            print(f"[WARN] Failed to write decimated overview: {e}")

    def export_stops_csv(self, stops, label=""):
        if not stops:
            return
        df = pd.DataFrame(stops)
        # keep a tidy column order if present
        cols = [c for c in ['type','t_start_s','t_end_s','duration_s','distance_ft','latitude','longitude'] if c in df.columns]
        df = df[cols + [c for c in df.columns if c not in cols]]
        df.to_csv(self.dirs['data'] / f'detected_stops_{label}.csv', index=False)

    # ----------------------------- Stop detection ---------------------------- #
    def detect_stops(self, trip_df, speed_col='speed', tcol='t_rel'):
        if speed_col not in trip_df.columns or tcol not in trip_df.columns:
            return []
        thr = self.STOP_SPEED_THRESHOLD_MPH
        stopped = (trip_df[speed_col].astype(float) <= thr).values
        tvals = trip_df[tcol].astype(float).values
        dvals = trip_df['dist_from_start'].astype(float).values if 'dist_from_start' in trip_df.columns else np.zeros_like(tvals)
        lats = trip_df['latitude'].values if 'latitude' in trip_df.columns else None
        lons = trip_df['longitude'].values if 'longitude' in trip_df.columns else None

        stops = []
        n = len(trip_df)
        i = 0
        while i < n:
            if not stopped[i]:
                i += 1
                continue
            j = i
            while j+1 < n and stopped[j+1]:
                j += 1
            t0 = float(tvals[i])
            t1 = float(tvals[j])
            duration = max(0.0, t1 - t0)
            # classify
            stype = 'other'
            if self.STATION_DWELL_MIN_S <= duration <= self.STATION_DWELL_MAX_S:
                stype = 'station'
            elif duration <= self.INTERSECTION_DWELL_MAX_S:
                stype = 'intersection'
            d_here = float(dvals[i])
            lat = float(lats[int((i+j)//2)]) if lats is not None else None
            lon = float(lons[int((i+j)//2)]) if lons is not None else None
            stops.append({
                'type': stype,
                't_start_s': t0,
                't_end_s': t1,
                'duration_s': duration,
                'distance_ft': d_here,
                'latitude': lat,
                'longitude': lon
            })
            i = j + 1
        return stops

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
        # numeric directions
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
        # rebase time for this trip to 0 s
        t0 = trip['time_seconds'].min()
        trip['t_rel'] = trip['time_seconds'] - t0
        return (best, trip)

    # ----------------------- End-to-end per direction ------------------------ #
    def analyze_direction_full_trip(self, direction_value, label_prefix="Inbound", segments_to_plot=10, only_segment_index=None, skip_overview=False):
        trip_id, data = self._select_full_trip(direction_value)
        if trip_id is None or len(data) < 2:
            print(f"[WARN] No full trip for direction={direction_value}")
            return None

        label = f"{label_prefix}_trip_{trip_id}"
        mode = "no reverse" if not self.ALLOW_REVERSE else "reverse allowed"
        print(f"\n=== {label} (t starts at 0 s; {mode}) ===")
        interps, seg_reports = [], []

        indices = [only_segment_index] if (only_segment_index is not None) else list(range(len(data)-1))
        for i in indices:
            p = self.calculate_feasible_bounds(
                data.iloc[i]['t_rel'], data.iloc[i+1]['t_rel'],
                data.iloc[i]['dist_from_start'], data.iloc[i+1]['dist_from_start'],
                data.iloc[i]['speed_fps'], data.iloc[i+1]['speed_fps'],
                num_points=120
            )
            if p:
                interps.append(p)
                seg_reports.append(self.generate_segment_report(i, data, p, tcol='t_rel'))
                if (only_segment_index is not None) or (i < segments_to_plot):
                    self.plot_segment_detail(i, data, p, tcol='t_rel', save=True, label_prefix=f"{label_prefix} ")

        stops = self.detect_stops(data)
        if not skip_overview and (only_segment_index is None):
            self.plot_full_trip_overview_with_bands(
                data, interps,
                title=f"{label_prefix} Full Trip Overview (Trip {trip_id}) — lens (no reverse)",
                filename=f"{label_prefix.lower()}_full_trip_overview.png",
                tcol='t_rel',
                stops=stops
            )
        self.create_summary_report(data, interps, seg_reports, stops, label=label)
        self.export_all_data(data, interps, label=label)
        self.export_stops_csv(stops, label=label)
        return data, interps, seg_reports

# =================================== MAIN ================================== #
if __name__ == "__main__":
    print("="*70)
    print("  TRAIN TRAJECTORY INTERPOLATOR — lens envelopes, no reverse (likely 2 mph/s)")
    print("="*70)
    set_plot_style()
    plt.ioff()

    parser = argparse.ArgumentParser(description="Train Trajectory Interpolator")
    parser.add_argument("--csv", default='4_k_line_data_with_trip_id.csv', help="Input CSV path")
    parser.add_argument("--out", default='trajectory_analysis_output', help="Output directory")
    parser.add_argument("--only-inbound-segment", type=int, default=None, help="Only process this inbound segment index (0-based)")
    parser.add_argument("--only-outbound-segment", type=int, default=None, help="Only process this outbound segment index (0-based)")
    parser.add_argument("--segments-to-plot", type=int, default=12, help="Number of initial segments to plot per direction if not filtering")
    parser.add_argument("--skip-overview", action='store_true', help="Skip building full overview plots")
    args = parser.parse_args()

    ti = TrainTrajectoryInterpolator(
        csv_file=args.csv,
        output_dir=args.out
    )

    inbound_val, outbound_val = ti._guess_direction_values()
    print(f"Detected: inbound={inbound_val}, outbound={outbound_val}")

    if inbound_val is not None:
        ti.analyze_direction_full_trip(
            inbound_val, label_prefix="Inbound",
            segments_to_plot=args.segments_to_plot,
            only_segment_index=args.only_inbound_segment,
            skip_overview=args.skip_overview
        )
    else:
        print("[WARN] No inbound direction detected.")

    if outbound_val is not None and args.only_inbound_segment is None:
        ti.analyze_direction_full_trip(
            outbound_val, label_prefix="Outbound",
            segments_to_plot=args.segments_to_plot,
            only_segment_index=args.only_outbound_segment,
            skip_overview=args.skip_overview
        )
    elif outbound_val is None:
        print("[WARN] No outbound direction detected.")

    print("\nDone. Check the 'overview' and 'individual_segments' folders.")