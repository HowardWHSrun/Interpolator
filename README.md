# GTFS Physics-Aware Interpolator (Siemens S200)

This tool ingests GTFS-realtime snapshots from `data/4_k_line_data_with_trip_id.csv`, applies Siemens S200 physics (max accel 1.3 m/s², max decel 2.2 m/s², max speed 60 mph), and generates:

- Time–distance plots with interpolation bounds
- Zoomed interval diagnostics (distance & speed stacked)
- Acceleration feasibility analysis
- Bounds validation report

## Quick start

1. Place your CSV at `data/4_k_line_data_with_trip_id.csv`.
2. Install deps: `pip install -r requirements.txt`
3. Run: `python src/interpolator.py`

Outputs are written to `outputs/plots/` and `outputs/reports/`.

## Layout

- `src/interpolator.py` — main script
- `requirements.txt` — Python deps
- `data/` — input data (CSV)
- `outputs/plots/` — generated figures
- `outputs/reports/` — text reports
- `README.md` — this file
- `task_description.md` — project notes and constraints

