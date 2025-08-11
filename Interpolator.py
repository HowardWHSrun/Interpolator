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
        # Siemens S200 constraints
        self.MAX_ACCEL_MPH_S = 3.0   # a+ (mph/s)
        self.MAX_DECEL_MPH_S = 5.0   # b   (mph/s)
        self.MAX_SPEED_MPH   = 60.0

        # Conversions
        self.MPH_TO_FPS = 1.46667
        self.a  = self.MAX_ACCEL_MPH_S * self.MPH_TO_FPS   # ft/s^2 (accelerating)
        self.b  = self.MAX_DECEL_MPH_S * self.MPH_TO_FPS   # ft/s^2 (braking)
        self.V  = self.MAX_SPEED_MPH   * self.MPH_TO_FPS   # ft/s (|v| <= V)

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
            t_left = t - t_stop
            t_to_negV = self.V / self.a
            if t_left <= t_to_negV:
                return s_stop - 0.5*self.a*t_left*t_left
            s_to_negV = -0.5*self.a*t_to_negV*t_to_negV
            return s_stop + s_to_negV + (-self.V)*(t_left - t_to_negV)
        else:
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
        v1n, v2n = sgn * v1_fps, sgn * v2_fps

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

        # 5) Instantaneous speed bounds
        v_from_start_min = np.maximum(-self.V, v1n - self.b * tau)
        v_from_start_max = np.minimum( self.V, v1n + self.a * tau)
        v_to_end_min     = np.maximum(-self.V, v2n - self.a * (T - tau))
        v_to_end_max     = np.minimum( self.V, v2n + self.b * (T - tau))
        v_lo_n = np.maximum(v_from_start_min, v_to_end_min)
        v_hi_n = np.minimum(v_from_start_max, v_to_end_max)

        # likely speed (linear → clipped), likely position (integrate → clamp)
        v_like_n = (1 - tau/T) * v1n + (tau/T) * v2n
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
        ax1.set_title('Distance vs Time (lens; reverse allowed)')
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
        ax2.set_title('Speed vs Time (reverse allowed)')
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

    def plot_full_trip_overview_with_bands(self, trip_df, interps, title, filename, tcol='t_rel'):
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
        ax1.set_xlabel('Time (s)')  # starts at 0 for the trip
        ax1.set_ylabel('Distance (ft)')
        ax1.set_title('Distance vs Time (lens bands; reverse allowed; overlap → darker)')
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
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (mph)')
        ax2.set_title('Speed vs Time (feasible bands)')
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
                'avg_mph': float(data['speed'].mean()),
                'max_mph': float(data['speed'].max()),
                'min_mph': float(data['speed'].min()),
                'std_mph': float(data['speed'].std())
            },
            'constraints': {
                'a_mph_s': self.MAX_ACCEL_MPH_S,
                'b_mph_s': self.MAX_DECEL_MPH_S,
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
    def analyze_direction_full_trip(self, direction_value, label_prefix="Inbound", segments_to_plot=10):
        trip_id, data = self._select_full_trip(direction_value)
        if trip_id is None or len(data) < 2:
            print(f"[WARN] No full trip for direction={direction_value}")
            return None

        label = f"{label_prefix}_trip_{trip_id}"
        print(f"\n=== {label} (t starts at 0 s; reverse allowed) ===")
        interps, seg_reports = [], []

        for i in range(len(data)-1):
            p = self.calculate_feasible_bounds(
                data.iloc[i]['t_rel'], data.iloc[i+1]['t_rel'],
                data.iloc[i]['dist_from_start'], data.iloc[i+1]['dist_from_start'],
                data.iloc[i]['speed_fps'], data.iloc[i+1]['speed_fps'],
                num_points=120
            )
            if p:
                interps.append(p)
                seg_reports.append(self.generate_segment_report(i, data, p, tcol='t_rel'))
                if i < segments_to_plot:
                    self.plot_segment_detail(i, data, p, tcol='t_rel', save=True, label_prefix=f"{label_prefix} ")

        self.plot_full_trip_overview_with_bands(
            data, interps,
            title=f"{label_prefix} Full Trip Overview (Trip {trip_id}) — lens w/ reverse",
            filename=f"{label_prefix.lower()}_full_trip_overview.png",
            tcol='t_rel'
        )
        self.create_summary_report(data, interps, seg_reports, label=label)
        self.export_all_data(data, interps, label=label)
        return data, interps, seg_reports

# =================================== MAIN ================================== #
if __name__ == "__main__":
    print("="*70)
    print("  TRAIN TRAJECTORY INTERPOLATOR — lens envelopes, v can be < 0, t=0 per trip")
    print("="*70)
    set_plot_style()
    plt.ioff()

    ti = TrainTrajectoryInterpolator(
        csv_file='4_k_line_data_with_trip_id.csv',
        output_dir='trajectory_analysis_output'
    )

    inbound_val, outbound_val = ti._guess_direction_values()
    print(f"Detected: inbound={inbound_val}, outbound={outbound_val}")

    if inbound_val is not None:
        ti.analyze_direction_full_trip(inbound_val, label_prefix="Inbound", segments_to_plot=12)
    else:
        print("[WARN] No inbound direction detected.")

    if outbound_val is not None:
        ti.analyze_direction_full_trip(outbound_val, label_prefix="Outbound", segments_to_plot=12)
    else:
        print("[WARN] No outbound direction detected.")

    print("\nDone. Check the 'overview' and 'individual_segments' folders.")