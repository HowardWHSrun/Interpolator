#!/usr/bin/env python3
"""
Simple GTFS Interpolator - Basic interpolation between consecutive data points
with physical constraints for Siemens S200 trains.

Physical constraints:
- Max acceleration: 1.3 m/s²
- Max deceleration: 2.2 m/s²
- Max speed: 60 mph (26.8 m/s)

This creates smooth interpolation between each pair of consecutive GPS points.
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# Siemens S200 physical constraints
MAX_ACCEL = 1.3  # m/s²
MAX_DECEL = 2.2  # m/s²
MAX_SPEED_MPH = 60
MAX_SPEED = MAX_SPEED_MPH * 0.44704  # m/s (26.8 m/s)

DATA_PATH = os.path.join('data', '4_k_line_data_with_trip_id.csv')
PLOTS_DIR = os.path.join('outputs', 'plots')
REPORTS_DIR = os.path.join('outputs', 'reports')


def ensure_dirs():
    """Create output directories if they don't exist."""
    os.makedirs(PLOTS_DIR, exist_ok=True)
    os.makedirs(REPORTS_DIR, exist_ok=True)


def load_data(path: str) -> pd.DataFrame:
    """Load and preprocess the GTFS data."""
    df = pd.read_csv(path)
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    
    # Convert units
    df['dist_m'] = df['dist_from_start'] * 0.3048  # feet to meters
    df['speed_ms'] = df['speed'] * 0.44704  # mph to m/s
    
    return df


def interpolate_segment(t1, d1, v1, t2, d2, v2, num_points=50):
    """
    Simple interpolation between two consecutive GPS points.
    
    Args:
        t1, t2: timestamps (datetime objects)
        d1, d2: distances in meters
        v1, v2: speeds in m/s
        num_points: number of interpolated points
    
    Returns:
        t_interp: interpolated time points (seconds from t1)
        d_interp: interpolated distances
        v_interp: interpolated speeds
        feasible: whether the segment is physically feasible
    """
    dt = (t2 - t1).total_seconds()
    if dt <= 0:
        return None, None, None, False
    
    # Create time array
    t_interp = np.linspace(0, dt, num_points)
    
    # Calculate required average acceleration
    dd = d2 - d1  # distance change
    dv = v2 - v1  # speed change
    
    # Check if constant acceleration can achieve this
    # Using kinematic equation: d = v1*t + 0.5*a*t^2
    # and v2 = v1 + a*t
    # Solving: a = (v2 - v1) / dt
    required_accel = dv / dt
    
    # Check if required acceleration is within limits
    accel_feasible = -MAX_DECEL <= required_accel <= MAX_ACCEL
    
    # Check if the kinematic equation is satisfied
    # d = v1*t + 0.5*a*t^2 should equal dd
    kinematic_distance = v1 * dt + 0.5 * required_accel * dt**2
    distance_feasible = abs(kinematic_distance - dd) < 0.1  # 10cm tolerance
    
    feasible = accel_feasible and distance_feasible
    
    if feasible:
        # Use constant acceleration model
        a = required_accel
        d_interp = d1 + v1 * t_interp + 0.5 * a * t_interp**2
        v_interp = v1 + a * t_interp
        
        # Clamp speeds to physical limits
        v_interp = np.clip(v_interp, 0, MAX_SPEED)
        
    else:
        # Fall back to simple linear interpolation for infeasible segments
        d_interp = np.linspace(d1, d2, num_points)
        v_interp = np.linspace(v1, v2, num_points)
        v_interp = np.clip(v_interp, 0, MAX_SPEED)
    
    return t_interp, d_interp, v_interp, feasible


def plot_trip_interpolation(df, trip_id, max_intervals=8):
    """Plot interpolation for a single trip with distance and speed subplots."""
    trip_data = df[df['trip_id'] == trip_id].sort_values('timestamp').copy()
    
    if len(trip_data) < 2:
        return
    
    # Calculate relative time from trip start
    start_time = trip_data['timestamp'].iloc[0]
    trip_data['time_rel'] = (trip_data['timestamp'] - start_time).dt.total_seconds()
    
    direction = trip_data['direction'].iloc[0]
    
    # Create figure with two subplots
    fig, (ax_dist, ax_speed) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    # Plot original data points
    ax_dist.scatter(trip_data['time_rel'], trip_data['dist_m'], 
                   color='red', s=50, zorder=5, label='GPS points')
    ax_speed.scatter(trip_data['time_rel'], trip_data['speed_ms'] / 0.44704, 
                    color='red', s=50, zorder=5, label='GPS points')
    
    # Interpolate each segment
    feasible_count = 0
    total_count = 0
    
    for i in range(min(len(trip_data) - 1, max_intervals)):
        row1, row2 = trip_data.iloc[i], trip_data.iloc[i + 1]
        
        t1, t2 = row1['timestamp'], row2['timestamp']
        d1, d2 = row1['dist_m'], row2['dist_m']
        v1, v2 = max(row1['speed_ms'], 0), max(row2['speed_ms'], 0)
        
        t_interp, d_interp, v_interp, feasible = interpolate_segment(t1, d1, v1, t2, d2, v2)
        
        if t_interp is not None:
            t_abs = row1['time_rel'] + t_interp
            
            # Choose color based on feasibility
            color = 'green' if feasible else 'orange'
            alpha = 0.8 if feasible else 0.6
            
            # Plot interpolated curves
            ax_dist.plot(t_abs, d_interp, color=color, alpha=alpha, linewidth=2)
            ax_speed.plot(t_abs, v_interp / 0.44704, color=color, alpha=alpha, linewidth=2)
            
            if feasible:
                feasible_count += 1
            total_count += 1
    
    # Formatting
    ax_dist.set_ylabel('Distance (m)')
    ax_dist.set_title(f'{direction.title()} Trip {trip_id[:8]}... - Distance vs Time\n'
                     f'Feasible segments: {feasible_count}/{total_count}')
    ax_dist.grid(True, alpha=0.3)
    ax_dist.legend()
    
    ax_speed.set_xlabel('Time from trip start (s)')
    ax_speed.set_ylabel('Speed (mph)')
    ax_speed.axhline(MAX_SPEED_MPH, color='red', linestyle='--', alpha=0.7, 
                    label=f'Max speed ({MAX_SPEED_MPH} mph)')
    ax_speed.set_title('Speed vs Time')
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()
    
    plt.tight_layout()
    
    # Save plot
    filename = f'simple_interp_{direction}_{trip_id[:8]}.png'
    plt.savefig(os.path.join(PLOTS_DIR, filename), dpi=300, bbox_inches='tight')
    plt.close()
    
    return feasible_count, total_count


def analyze_feasibility(df):
    """Analyze the physical feasibility of observed GPS segments."""
    results = []
    
    for trip_id in df['trip_id'].unique():
        trip_data = df[df['trip_id'] == trip_id].sort_values('timestamp')
        
        for i in range(len(trip_data) - 1):
            row1, row2 = trip_data.iloc[i], trip_data.iloc[i + 1]
            
            t1, t2 = row1['timestamp'], row2['timestamp']
            dt = (t2 - t1).total_seconds()
            
            if dt <= 0:
                continue
                
            d1, d2 = row1['dist_m'], row2['dist_m']
            v1, v2 = max(row1['speed_ms'], 0), max(row2['speed_ms'], 0)
            
            # Calculate required acceleration
            dv = v2 - v1
            required_accel = dv / dt
            
            # Check feasibility
            accel_feasible = -MAX_DECEL <= required_accel <= MAX_ACCEL
            
            # Check kinematic consistency
            dd = d2 - d1
            kinematic_distance = v1 * dt + 0.5 * required_accel * dt**2
            distance_error = abs(kinematic_distance - dd)
            
            results.append({
                'trip_id': trip_id,
                'direction': row1['direction'],
                'dt': dt,
                'distance_change': dd,
                'speed_change': dv,
                'required_accel': required_accel,
                'accel_feasible': accel_feasible,
                'distance_error': distance_error,
                'feasible': accel_feasible and distance_error < 1.0  # 1m tolerance
            })
    
    return pd.DataFrame(results)


def create_summary_plots(feasibility_df):
    """Create summary plots of feasibility analysis."""
    
    # Plot 1: Acceleration feasibility
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Scatter plot of required vs feasible accelerations
    feasible = feasibility_df[feasibility_df['feasible']]
    infeasible = feasibility_df[~feasibility_df['feasible']]
    
    ax1.scatter(feasible['dt'], feasible['required_accel'], 
               alpha=0.6, color='green', s=20, label=f'Feasible ({len(feasible)})')
    ax1.scatter(infeasible['dt'], infeasible['required_accel'], 
               alpha=0.6, color='red', s=20, label=f'Infeasible ({len(infeasible)})')
    
    ax1.axhline(MAX_ACCEL, color='blue', linestyle='--', label=f'Max accel ({MAX_ACCEL} m/s²)')
    ax1.axhline(-MAX_DECEL, color='orange', linestyle='--', label=f'Max decel ({MAX_DECEL} m/s²)')
    ax1.set_xlabel('Time interval (s)')
    ax1.set_ylabel('Required acceleration (m/s²)')
    ax1.set_title('Acceleration Feasibility')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Histogram of feasibility by time interval
    bins = np.linspace(0, feasibility_df['dt'].max(), 20)
    ax2.hist(feasible['dt'], bins=bins, alpha=0.7, color='green', label='Feasible')
    ax2.hist(infeasible['dt'], bins=bins, alpha=0.7, color='red', label='Infeasible')
    ax2.set_xlabel('Time interval (s)')
    ax2.set_ylabel('Count')
    ax2.set_title('Feasibility by Time Interval')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, 'feasibility_analysis.png'), dpi=300, bbox_inches='tight')
    plt.close()


def main():
    """Main execution function."""
    ensure_dirs()
    
    if not os.path.exists(DATA_PATH):
        raise FileNotFoundError(f'Missing data file: {DATA_PATH}')
    
    print("Loading data...")
    df = load_data(DATA_PATH)
    
    print("Analyzing feasibility...")
    feasibility_df = analyze_feasibility(df)
    
    print("Creating summary plots...")
    create_summary_plots(feasibility_df)
    
    print("Plotting sample trip interpolations...")
    # Plot a few example trips
    outbound_trips = df[df['direction'] == 'outbound']['trip_id'].unique()[:3]
    inbound_trips = df[df['direction'] == 'inbound']['trip_id'].unique()[:3]
    
    total_feasible = 0
    total_segments = 0
    
    for trip_id in list(outbound_trips) + list(inbound_trips):
        feasible, total = plot_trip_interpolation(df, trip_id)
        if feasible is not None:
            total_feasible += feasible
            total_segments += total
    
    # Write summary report
    feasible_pct = feasibility_df['feasible'].mean() * 100
    
    summary = f"""Simple GTFS Interpolation Analysis
=====================================

Physical Constraints:
- Max acceleration: {MAX_ACCEL} m/s²
- Max deceleration: {MAX_DECEL} m/s²
- Max speed: {MAX_SPEED_MPH} mph ({MAX_SPEED:.1f} m/s)

Analysis Results:
- Total segments analyzed: {len(feasibility_df)}
- Physically feasible segments: {feasibility_df['feasible'].sum()} ({feasible_pct:.1f}%)
- Mean time interval: {feasibility_df['dt'].mean():.1f} seconds
- Mean required acceleration: {feasibility_df['required_accel'].mean():.3f} m/s²

The interpolation uses simple constant acceleration between consecutive GPS points
where physically feasible, and falls back to linear interpolation otherwise.

Green curves = physically feasible interpolation
Orange curves = fallback linear interpolation
Red points = original GPS measurements
"""
    
    with open(os.path.join(REPORTS_DIR, 'simple_interpolation_summary.txt'), 'w') as f:
        f.write(summary)
    
    print(f"\nAnalysis complete!")
    print(f"Feasible segments: {feasibility_df['feasible'].sum()}/{len(feasibility_df)} ({feasible_pct:.1f}%)")
    print(f"Outputs saved to: {PLOTS_DIR} and {REPORTS_DIR}")


if __name__ == '__main__':
    main()
