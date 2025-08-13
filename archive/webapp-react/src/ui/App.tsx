import React, { useEffect, useMemo, useState } from 'react'
import { AppBar, Box, Button, Chip, Container, FormControl, Grid, InputLabel, MenuItem, Select, Stack, Toolbar, Typography, Paper } from '@mui/material'
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap'
import Plot from 'react-plotly.js'
import Papa from 'papaparse'

type DirSpec = {
  summary: string
  stops: string
  interpolated: string
}

type Manifest = {
  trains: Array<{
    id: string
    label: string
    inbound: DirSpec
    outbound: DirSpec
  }>
}

async function fetchJSON<T>(path: string): Promise<T> {
  const res = await fetch(path)
  if (!res.ok) throw new Error(`Failed ${path}`)
  return res.json()
}

async function fetchCSV(path: string): Promise<any[]> {
  const text = await (await fetch(path)).text()
  return new Promise((resolve, reject) => {
    Papa.parse(text, { header: true, dynamicTyping: true, complete: r => resolve(r.data), error: reject })
  }) as Promise<any[]>
}

const defaultManifest: Manifest = {
  trains: [
    {
      id: 'K-line',
      label: 'K Line — SFMTA',
      inbound: {
        summary: '/trajectory_analysis_output/Results/reports/analysis_summary_Inbound_trip_f1246aa0-95f9-4ad0-a3d5-3c03fec4e44f.json',
        stops: '/trajectory_analysis_output/Results/reports/detected_stops_Inbound_trip_f1246aa0-95f9-4ad0-a3d5-3c03fec4e44f.json',
        interpolated: '/trajectory_analysis_output/Results/exported_data/interpolated_trajectories_Inbound_trip_f1246aa0-95f9-4ad0-a3d5-3c03fec4e44f.csv'
      },
      outbound: {
        summary: '/trajectory_analysis_output/Results/reports/analysis_summary_Outbound_trip_a5014681-674a-4b9e-ab76-a2dd87ab6b22_seg_1.json',
        stops: '/trajectory_analysis_output/Results/reports/detected_stops_Outbound_trip_a5014681-674a-4b9e-ab76-a2dd87ab6b22_seg_1.json',
        interpolated: '/trajectory_analysis_output/Results/exported_data/interpolated_trajectories_Outbound_trip_a5014681-674a-4b9e-ab76-a2dd87ab6b22_seg_1.csv'
      }
    }
  ]
}

export default function App() {
  const [manifest, setManifest] = useState<Manifest>(defaultManifest)
  const [trainIdx, setTrainIdx] = useState(0)
  const [dir, setDir] = useState<'inbound' | 'outbound'>('inbound')
  const [rows, setRows] = useState<any[]>([])
  const [summary, setSummary] = useState<any | null>(null)
  const [stops, setStops] = useState<any[] | null>(null)
  const spec = manifest.trains[trainIdx][dir]

  useEffect(() => {
    (async () => {
      try {
        const m = await fetchJSON<Manifest>('/data/webapp/manifest.json')
        setManifest(m)
      } catch {}
    })()
  }, [])

  useEffect(() => {
    (async () => {
      const r = await fetchCSV(spec.interpolated)
      setRows(r)
    })()
  }, [spec.interpolated])

  useEffect(() => {
    (async () => {
      try {
        const s = await fetchJSON<any>(spec.summary)
        setSummary(s)
      } catch { setSummary(null) }
      try {
        const st = await fetchJSON<any[]>(spec.stops)
        setStops(st)
      } catch { setStops(null) }
    })()
  }, [spec.summary, spec.stops])

  const time = useMemo(() => rows.map(r => r.time_s), [rows])
  const minPos = useMemo(() => rows.map(r => r.min_position_ft), [rows])
  const maxPos = useMemo(() => rows.map(r => r.max_position_ft), [rows])
  const likePos = useMemo(() => rows.map(r => r.likely_position_ft), [rows])
  const minSpd = useMemo(() => rows.map(r => (Number(r.min_speed_mph) || 0) * 0.44704), [rows])
  const maxSpd = useMemo(() => rows.map(r => (Number(r.max_speed_mph) || 0) * 0.44704), [rows])
  const likeSpd = useMemo(() => rows.map(r => (Number(r.likely_speed_mph) || 0) * 0.44704), [rows])

  const distData = useMemo(() => ([
    { x: time, y: minPos, type: 'scattergl', mode: 'lines', line: { width: 0 }, hoverinfo: 'skip', showlegend: false },
    { x: time, y: maxPos, type: 'scattergl', mode: 'lines', line: { width: 0 }, fill: 'tonexty', fillcolor: 'rgba(100,116,139,0.35)' },
    { x: time, y: minPos, type: 'scattergl', mode: 'lines', line: { color: '#64748b', width: 0.6 }, hoverinfo: 'skip', showlegend: false },
    { x: time, y: maxPos, type: 'scattergl', mode: 'lines', line: { color: '#64748b', width: 0.6 }, hoverinfo: 'skip', showlegend: false },
    { x: time, y: likePos, type: 'scattergl', mode: 'lines', name: 'Likely', line: { color: '#111827', dash: 'dot', width: 2 } }
  ]), [time, minPos, maxPos, likePos])

  const spdData = useMemo(() => ([
    { x: time, y: minSpd, type: 'scattergl', mode: 'lines', line: { width: 0 }, hoverinfo: 'skip', showlegend: false },
    { x: time, y: maxSpd, type: 'scattergl', mode: 'lines', line: { width: 0 }, fill: 'tonexty', fillcolor: 'rgba(100,116,139,0.35)' },
    { x: time, y: minSpd, type: 'scattergl', mode: 'lines', line: { color: '#64748b', width: 0.6 }, hoverinfo: 'skip', showlegend: false },
    { x: time, y: maxSpd, type: 'scattergl', mode: 'lines', line: { color: '#64748b', width: 0.6 }, hoverinfo: 'skip', showlegend: false },
    { x: time, y: likeSpd, type: 'scattergl', mode: 'lines', name: 'Likely', line: { color: '#111827', dash: 'dot', width: 2 } }
  ]), [time, minSpd, maxSpd, likeSpd])

  const stopShapes = useMemo(() => {
    if (!stops) return []
    return stops.map(s => ({ type: 'rect', xref: 'x', yref: 'paper', y0: 0, y1: 1, x0: s.t_start_s, x1: s.t_end_s, line: { width: 0 }, fillcolor: s.type === 'station' ? 'rgba(251,146,60,0.15)' : 'rgba(125,211,252,0.15)' }))
  }, [stops])

  return (
    <Box>
      <AppBar position="sticky" color="inherit" elevation={0} sx={{ borderBottom: '1px solid #e5e7eb' }}>
        <Toolbar>
          <Typography variant="h6" sx={{ fontWeight: 700, mr: 2 }}>K Line Analytics</Typography>
          <Chip label="No reverse" size="small" sx={{ mr: 1 }} />
          <Chip label="Station 10–40 s" size="small" sx={{ mr: 1 }} />
          <Chip label="Intersection 0–90 s" size="small" sx={{ mr: 1 }} />
          <Chip label="Likely accel 2 mph/s" size="small" />
          <Box flexGrow={1} />
          <Button startIcon={<ZoomOutMapIcon />} onClick={() => {
            const ev = new Event('reset-plots')
            window.dispatchEvent(ev)
          }}>Reset Zoom</Button>
        </Toolbar>
      </AppBar>
      <Container sx={{ py: 2 }}>
        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
          <Grid container spacing={2} alignItems="center">
            <Grid item xs={12} md={4}>
              <FormControl fullWidth>
                <InputLabel id="train-label">Trip</InputLabel>
                <Select labelId="train-label" label="Trip" value={trainIdx} onChange={e => setTrainIdx(Number(e.target.value))}>
                  {manifest.trains.map((t, i) => (
                    <MenuItem key={t.id} value={i}>{t.label}</MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>
            <Grid item xs={12} md={4}>
              <Stack direction="row" spacing={1}>
                <Button variant={dir==='inbound'?'contained':'outlined'} onClick={() => setDir('inbound')}>Inbound</Button>
                <Button variant={dir==='outbound'?'contained':'outlined'} onClick={() => setDir('outbound')}>Outbound</Button>
              </Stack>
            </Grid>
            <Grid item xs={12} md={4}>
              <Stack direction="row" spacing={2} sx={{ justifyContent: { xs: 'flex-start', md: 'flex-end' } }}>
                <Stack spacing={0} alignItems="center">
                  <Typography variant="caption">Duration</Typography>
                  <Typography variant="subtitle1" sx={{ fontWeight: 700 }}>{summary ? `${summary.duration_minutes.toFixed(1)} min` : '–'}</Typography>
                </Stack>
                <Stack spacing={0} alignItems="center">
                  <Typography variant="caption">Distance</Typography>
                  <Typography variant="subtitle1" sx={{ fontWeight: 700 }}>{summary ? `${(summary.distance_ft/5280).toFixed(2)} mi` : '–'}</Typography>
                </Stack>
                <Stack spacing={0} alignItems="center">
                  <Typography variant="caption">Avg</Typography>
                  <Typography variant="subtitle1" sx={{ fontWeight: 700 }}>{summary ? `${summary.speed_stats.avg_mph.toFixed(1)} mph` : '–'}</Typography>
                </Stack>
                <Stack spacing={0} alignItems="center">
                  <Typography variant="caption">Max</Typography>
                  <Typography variant="subtitle1" sx={{ fontWeight: 700 }}>{summary ? `${summary.speed_stats.max_mph.toFixed(1)} mph` : '–'}</Typography>
                </Stack>
                <Stack spacing={0} alignItems="center">
                  <Typography variant="caption">Stops</Typography>
                  <Typography variant="subtitle1" sx={{ fontWeight: 700 }}>{summary?.stops_summary?.total_stops ?? '–'}</Typography>
                </Stack>
              </Stack>
            </Grid>
          </Grid>
        </Paper>

        <Grid container spacing={2}>
          <Grid item xs={12} md={6}>
            <Paper variant="outlined" sx={{ p: 1 }}>
              <Typography variant="subtitle1" sx={{ px: 1, py: .5, fontWeight: 600 }}>{dir} distance vs time (no reverse)</Typography>
              <Plot data={distData as any}
                layout={{ margin: { l: 50, r: 20, t: 10, b: 40 }, xaxis: { title: 'Time (s)' }, yaxis: { title: 'Distance (ft)' }, shapes: stopShapes as any }}
                config={{ responsive: true }}
                style={{ width: '100%', height: 420 }}
              />
            </Paper>
          </Grid>
          <Grid item xs={12} md={6}>
            <Paper variant="outlined" sx={{ p: 1 }}>
              <Typography variant="subtitle1" sx={{ px: 1, py: .5, fontWeight: 600 }}>{dir} velocity vs time (no reverse)</Typography>
              <Plot data={spdData as any}
                layout={{ margin: { l: 50, r: 20, t: 10, b: 40 }, xaxis: { title: 'Time (s)' }, yaxis: { title: 'Velocity (m/s)' }, shapes: stopShapes as any }}
                config={{ responsive: true }}
                style={{ width: '100%', height: 420 }}
              />
            </Paper>
          </Grid>
        </Grid>
      </Container>
    </Box>
  )
}


