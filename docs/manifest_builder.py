import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'webapp' / 'manifest.json'

def find_summary(prefix):
    reports = ROOT / 'trajectory_analysis_output' / 'Results' / 'reports'
    for p in reports.glob(f'analysis_summary_{prefix}*.json'):
        return '/' + str(p.relative_to(ROOT))
    return None

def find_stops(prefix):
    reports = ROOT / 'trajectory_analysis_output' / 'Results' / 'reports'
    for p in reports.glob(f'detected_stops_{prefix}*.json'):
        return '/' + str(p.relative_to(ROOT))
    return None

def find_interp(prefix):
    data = ROOT / 'trajectory_analysis_output' / 'Results' / 'exported_data'
    # prefer decimated overview if available
    for p in data.glob(f'interpolated_trajectories_overview_{prefix}*.csv'):
        return '/' + str(p.relative_to(ROOT))
    for p in data.glob(f'interpolated_trajectories_{prefix}*.csv'):
        return '/' + str(p.relative_to(ROOT))
    return None

def build_manifest():
    inbound = {
        'summary': find_summary('Inbound_trip_'),
        'stops': find_stops('Inbound_trip_'),
        'interpolated': find_interp('Inbound_trip_'),
        'overview_img': '/trajectory_analysis_output/Results/overview/inbound_full_trip_overview.png',
        'segments_dir': '/trajectory_analysis_output/Results/individual_segments/',
        'segment_prefix': 'inbound segment_'
    }
    outbound = {
        'summary': find_summary('Outbound_trip_'),
        'stops': find_stops('Outbound_trip_'),
        'interpolated': find_interp('Outbound_trip_'),
        'overview_img': '/trajectory_analysis_output/Results/overview/outbound_full_trip_overview.png',
        'segments_dir': '/trajectory_analysis_output/Results/individual_segments/',
        'segment_prefix': 'outbound segment_'
    }
    manifest = {
        'trains': [
            {
                'id': 'K-line',
                'label': 'K Line — SFMTA',
                'inbound': inbound,
                'outbound': outbound
            }
        ]
    }
    OUT.write_text(json.dumps(manifest, indent=2))
    print(f'Wrote {OUT}')

if __name__ == '__main__':
    build_manifest()


