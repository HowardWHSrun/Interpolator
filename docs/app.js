/* global Papa, Plotly */

function resolvePath(p) {
  if (!p) return p;
  if (p.startsWith('/')) {
    const isWebappSubdir = window.location.pathname.includes('/webapp/') || window.location.pathname.includes('/docs/');
    if (isWebappSubdir) return `..${p}`;
    return p.replace(/^\//, '');
  }
  return p;
}

async function fetchJSON(path) {
  const res = await fetch(resolvePath(path));
  if (!res.ok) throw new Error(`Failed to fetch ${path}`);
  return res.json();
}

async function fetchCSV(path) {
  const text = await (await fetch(resolvePath(path))).text();
  return new Promise((resolve, reject) => {
    Papa.parse(text, {
      header: true,
      dynamicTyping: true,
      complete: (r) => resolve(r.data),
      error: reject,
      skipEmptyLines: true,
    });
  });
}

function ensureTrace(name, x, y, opts = {}) {
  return Object.assign({ name, x, y, mode: 'lines', line: { width: 1.5 } }, opts);
}

function layout(title, xLabel, yLabel) {
  return {
    title,
    margin: { l: 50, r: 20, t: 40, b: 45 },
    xaxis: { title: xLabel, zeroline: false },
    yaxis: { title: yLabel, zeroline: false },
    legend: { orientation: 'h' },
  };
}

// ----------------------- Raw trips overlay (from source CSV) ----------------------- //
let RAW_TRIPS_CACHE = null;
async function fetchRawTrips() {
  if (RAW_TRIPS_CACHE) return RAW_TRIPS_CACHE;
  // Try multiple likely CSV locations to be robust across hosting setups
  const candidates = [
    '/data/4_k_line_data_with_trip_id.csv',
    '/4_k_line_data_with_trip_id.csv',
    '4_k_line_data_with_trip_id.csv'
  ];
  let text = '';
  let lastErr = null;
  for (const p of candidates) {
    try {
      const res = await fetch(resolvePath(p));
      if (res.ok) { text = await res.text(); break; }
    } catch (e) { lastErr = e; }
  }
  if (!text) throw lastErr || new Error('Could not locate 4_k_line_data_with_trip_id.csv');
  const rows = await new Promise((resolve, reject) => {
    Papa.parse(text, { header: true, dynamicTyping: true, complete: r => resolve(r.data), error: reject, skipEmptyLines: true });
  });
  // group by direction+trip_id
  const map = new Map();
  for (const r of rows) {
    if (!r.trip_id || !r.direction) continue;
    const key = `${String(r.direction).toLowerCase()}__${r.trip_id}`;
    if (!map.has(key)) map.set(key, []);
    map.get(key).push(r);
  }
  // build per-trip arrays with t_rel and dist/speed
  const trips = [];
  for (const [key, arr] of map.entries()) {
    arr.sort((a,b) => new Date(a.timestamp) - new Date(b.timestamp));
    const dir = key.split('__')[0];
    const tStart = new Date(arr[0].timestamp);
    const tEnd   = new Date(arr[arr.length-1].timestamp);
    const t0 = tStart.getTime()/1000;
    let dmin = Infinity, dmax = -Infinity;
    const t_rel = [], dist = [], spd = [];
    for (const r of arr) {
      const t = new Date(r.timestamp).getTime()/1000 - t0;
      t_rel.push(t);
      const d = Number(r.dist_from_start);
      dist.push(d);
      spd.push(Number(r.speed));
      if (d < dmin) dmin = d; if (d > dmax) dmax = d;
    }
    const spanFt = dmax - dmin;
    const miles = spanFt / 5280;
    const durationMin = Math.max(0, (tEnd - tStart) / 60000);
    const dateKey = tStart.toISOString().slice(0,10); // YYYY-MM-DD
    const vehicleId = arr[0]?.vehicle_id ?? '';
    trips.push({ dir, trip_id: key.split('__')[1], span: spanFt, miles, durationMin, dateKey, start: tStart, end: tEnd, vehicleId, t_rel, dist, spd });
  }
  RAW_TRIPS_CACHE = trips;
  return trips;
}

function decimateXY(x, y, maxPoints=1000) {
  if (x.length <= maxPoints) return { x, y };
  const step = Math.ceil(x.length / maxPoints);
  const xd = [], yd = [];
  for (let i = 0; i < x.length; i += step) { xd.push(x[i]); yd.push(y[i]); }
  if (xd[xd.length-1] !== x[x.length-1]) { xd.push(x[x.length-1]); yd.push(y[y.length-1]); }
  return { x: xd, y: yd };
}

async function overlayOtherTrips(dirKey, selectedTripId) {
  try {
    const trips = await fetchRawTrips();
    const dirTrips = trips.filter(t => t.dir === dirKey.toLowerCase());
    // pick up to 3 largest span trips excluding selected
    const others = dirTrips.filter(t => t.trip_id !== selectedTripId)
      .sort((a,b) => b.span - a.span).slice(0, 3);
    const traces = [];
    let first = true;
    for (const t of others) {
      const d1 = decimateXY(t.t_rel, t.dist, 1200);
      traces.push({
        x: d1.x,
        y: d1.y,
        type: 'scattergl',
        mode: 'lines',
        line: { width: 1, color: 'rgba(2,132,199,0.35)' },
        name: first ? 'Other trips (raw)' : t.trip_id.slice(0,8) + '…',
        showlegend: first,
        hoverinfo: 'skip',
        legendgroup: 'others'
      });
      first = false;
    }
    if (traces.length) {
      Plotly.addTraces(`distanceChart-${dirKey}`, traces);
    }
  } catch {}
}

async function populateTripSelect(dirKey) {
  const sel = document.getElementById('tripIdSelect');
  if (!sel) return;
  sel.innerHTML = '';
  try {
    const trips = await fetchRawTrips();
    const dirTrips = trips.filter(t => t.dir === dirKey.toLowerCase());
    const byDate = new Map();
    for (const t of dirTrips) {
      if (!byDate.has(t.dateKey)) byDate.set(t.dateKey, []);
      byDate.get(t.dateKey).push(t);
    }
    // No placeholder option
    // sort dates descending; trips by start time
    const dates = Array.from(byDate.keys()).sort((a,b)=> (a<b?1:-1));
    for (const d of dates) {
      const group = document.createElement('optgroup');
      group.label = d;
      const items = byDate.get(d).sort((a,b)=> a.start - b.start);
      items.forEach((t) => {
        const opt = document.createElement('option');
        opt.value = t.trip_id;
        const tStart = `${String(t.start.getHours()).padStart(2,'0')}:${String(t.start.getMinutes()).padStart(2,'0')}`;
        const tEnd   = `${String(t.end.getHours()).padStart(2,'0')}:${String(t.end.getMinutes()).padStart(2,'0')}`;
        const miles = t.miles.toFixed(2);
        const dur = t.durationMin.toFixed(1);
        const veh = t.vehicleId ? ` • Veh ${t.vehicleId}` : '';
        const shortId = ` • ${String(t.trip_id).slice(0,8)}…`;
        opt.textContent = `${tStart}–${tEnd} — ${miles} mi — ${dur} min${veh}${shortId}`;
        group.appendChild(opt);
      });
      sel.appendChild(group);
    }
  } catch {}
}

async function buildManifest() {
  // Manifest is a static JSON generated by a helper script; fallback to default paths
  try {
    return await fetchJSON('manifest.json');
  } catch {
    return {
      trains: [
        {
          id: 'K-line',
          label: 'K Line — SFMTA',
          inbound: {
            summary: '/trajectory_analysis_output/Results/reports/analysis_summary_Inbound_trip_f1246aa0-95f9-4ad0-a3d5-3c03fec4e44f.json',
            stops: '/trajectory_analysis_output/Results/reports/detected_stops_Inbound_trip_f1246aa0-95f9-4ad0-a3d5-3c03fec4e44f.json',
            interpolated: '/trajectory_analysis_output/Results/exported_data/interpolated_trajectories_Inbound_trip_f1246aa0-95f9-4ad0-a3d5-3c03fec4e44f.csv',
            overview_img: '/trajectory_analysis_output/Results/overview/inbound_full_trip_overview.png',
            segments_dir: '/trajectory_analysis_output/Results/individual_segments/',
            segment_prefix: 'inbound segment_'
          },
          outbound: {
            summary: '/trajectory_analysis_output/Results/reports/analysis_summary_Outbound_trip_a5014681-674a-4b9e-ab76-a2dd87ab6b22_seg_1.json',
            stops: '/trajectory_analysis_output/Results/reports/detected_stops_Outbound_trip_a5014681-674a-4b9e-ab76-a2dd87ab6b22_seg_1.json',
            interpolated: '/trajectory_analysis_output/Results/exported_data/interpolated_trajectories_Outbound_trip_a5014681-674a-4b9e-ab76-a2dd87ab6b22_seg_1.csv',
            overview_img: '/trajectory_analysis_output/Results/overview/outbound_full_trip_overview.png',
            segments_dir: '/trajectory_analysis_output/Results/individual_segments/',
            segment_prefix: 'outbound segment_'
          }
        }
      ]
    };
  }
}

async function urlExists(url) {
  try {
    const res = await fetch(resolvePath(url), { method: 'GET' });
    return res.ok;
  } catch { return false; }
}

async function buildSpecForTrip(dirKey, tripId) {
  if (!tripId) return null;
  const base = '/trajectory_analysis_output/Results';
  const pref = dirKey === 'inbound' ? 'Inbound_trip_' : 'Outbound_trip_';
  const spec = {
    summary: `${base}/reports/analysis_summary_${pref}${tripId}.json`,
    stops: `${base}/reports/detected_stops_${pref}${tripId}.json`,
    interpolated: `${base}/exported_data/interpolated_trajectories_${pref}${tripId}.csv`
  };
  if (await urlExists(spec.interpolated)) return spec;
  return null;
}

function renderSegments(containerId, dirSpec, startIndex = 1, endIndex = 12) {
  const container = document.getElementById(containerId);
  container.innerHTML = '';
  // Build mini-charts by slicing the time series into equal segments of ~N points
  // If the CSV includes a 'segment' column, prefer grouping by it; otherwise chunk by time gaps
  fetchCSV(dirSpec.interpolated).then(rows => {
    const bySeg = {};
    rows.forEach(r => {
      const key = r.segment || 0;
      if (!bySeg[key]) bySeg[key] = [];
      bySeg[key].push(r);
    });
    const keys = Object.keys(bySeg)
      .map(k => parseInt(k, 10))
      .filter(k => k >= startIndex && k <= endIndex)
      .sort((a,b) => a - b);
    keys.forEach(key => {
      const seg = bySeg[key];
      seg.sort((a,b) => a.time_s - b.time_s);
      const t = seg.map(r => r.time_s);
      const minPos = seg.map(r => r.min_position_ft);
      const maxPos = seg.map(r => r.max_position_ft);
      const likePos = seg.map(r => r.likely_position_ft);
      const card = document.createElement('div');
      card.className = 'segment-card';
      const mini = document.createElement('div');
      mini.className = 'mini-chart';
      card.appendChild(mini);
      container.appendChild(card);
      const miniData = [
        { x: t, y: minPos, mode: 'lines', line: { width: 0 }, hoverinfo: 'skip', showlegend: false },
        { x: t, y: maxPos, mode: 'lines', line: { width: 0 }, fill: 'tonexty', fillcolor: 'rgba(148,163,184,0.25)' },
        ensureTrace('Likely position', t, likePos, { line: { color: '#111827', width: 1 } }),
      ];
      Plotly.newPlot(mini, miniData, { margin: {l:30,r:10,t:10,b:24}, xaxis: {title:'t (s)', tickfont:{size:9}}, yaxis:{title:'ft', tickfont:{size:9}} }, {displayModeBar:false, responsive:true});
      mini.addEventListener('click', () => {
        // clicking mini focuses the detail panel to this segment window
        const dirKey = containerId.includes('inbound') ? 'inbound' : 'outbound';
        const minSpd = seg.map(r => Number(r.min_speed_mph) * 0.44704);
        const maxSpd = seg.map(r => Number(r.max_speed_mph) * 0.44704);
        const likeSpd = seg.map(r => Number(r.likely_speed_mph) * 0.44704);
        const titleEl = document.getElementById(`detail-title-${dirKey}`);
        if (titleEl) titleEl.textContent = `${dirKey} segment ${String(key).padStart(3,'0')} detail`;
        Plotly.newPlot(`detail-distance-${dirKey}`, [
          { x: t, y: minPos, mode: 'lines', line: { width: 0 }, hoverinfo: 'skip', showlegend: false },
          { x: t, y: maxPos, mode: 'lines', line: { width: 0 }, fill: 'tonexty', fillcolor: 'rgba(148,163,184,0.25)' },
          ensureTrace('Likely position', t, likePos, { line: { color: '#111827', dash: 'dot', width: 2 } }),
        ], layout('Distance detail', 'Time (s)', 'Distance (ft)'), {responsive: true});
        Plotly.newPlot(`detail-speed-${dirKey}`, [
          { x: t, y: minSpd, mode: 'lines', line: { width: 0 }, hoverinfo: 'skip', showlegend: false },
          { x: t, y: maxSpd, mode: 'lines', line: { width: 0 }, fill: 'tonexty', fillcolor: 'rgba(148,163,184,0.25)' },
          ensureTrace('Likely speed', t, likeSpd, { line: { color: '#111827', dash: 'dot', width: 2 } }),
        ], layout('Speed detail', 'Time (s)', 'Speed (m/s)'), {responsive: true});
      });
    });
  });
}

async function renderDirection(dirKey, dirSpec) {
  // Load interpolated CSV for distance/time bands and likely curve
  const rows = await fetchCSV(dirSpec.interpolated);
  const time = [];
  const minPos = [], maxPos = [], likePos = [];
  const minSpd = [], maxSpd = [], likeSpd = [];
  for (const r of rows) {
    const t = Number(r.time_s);
    const pmin = Number(r.min_position_ft);
    const pmax = Number(r.max_position_ft);
    const plike = Number(r.likely_position_ft);
    const smin = Number(r.min_speed_mph) * 0.44704; // mph → m/s
    const smax = Number(r.max_speed_mph) * 0.44704; // mph → m/s
    const slike = Number(r.likely_speed_mph) * 0.44704; // mph → m/s
    if (!Number.isFinite(t)) continue;
    // position series must be finite to render distance
    if (!Number.isFinite(pmin) || !Number.isFinite(pmax) || !Number.isFinite(plike)) continue;
    time.push(t);
    minPos.push(pmin);
    maxPos.push(pmax);
    likePos.push(plike);
    // push speeds only if finite; otherwise fall back to 0 to avoid empty arrays
    minSpd.push(Number.isFinite(smin) ? smin : 0);
    maxSpd.push(Number.isFinite(smax) ? smax : 0);
    likeSpd.push(Number.isFinite(slike) ? slike : 0);
  }

  // Overlay stop windows if available
  let stopSpans = [];
  try {
    if (dirSpec.stops && await urlExists(dirSpec.stops)) {
      const stops = await fetchJSON(dirSpec.stops);
      stopSpans = (stops || []).map(s => ({
        x0: s.t_start_s, x1: s.t_end_s,
        fillcolor: s.type === 'station' ? 'rgba(251, 146, 60, 0.15)' : 'rgba(125, 211, 252, 0.15)'
      }));
    }
  } catch {}

  // Distance chart: only likely trajectory
  const likePosM = likePos.map(v => v * 0.3048);
  const distData = [
    ensureTrace('Likely', time, likePosM, { type: 'scatter', line: { color: getComputedStyle(document.documentElement).getPropertyValue('--text') || '#111827', width: 2 }, showlegend: true })
  ];
  const distLayout = layout(`${dirKey} distance vs time (no reverse)`, 'Time (s)', 'Distance (m)');
  distLayout.paper_bgcolor = '#ffffff';
  distLayout.plot_bgcolor = '#ffffff';
  distLayout.shapes = stopSpans.map(s => ({ type: 'rect', xref: 'x', yref: 'paper', y0: 0, y1: 1, x0: s.x0, x1: s.x1, fillcolor: s.fillcolor, line: { width: 0 } }));
  distLayout.legend = { orientation: 'h', x: 0, y: -0.25, yanchor: 'top', font: { size: 10 } };
  distLayout.xaxis = Object.assign({}, distLayout.xaxis, { rangeslider: { visible: true, thickness: 0.08 }, automargin: true });
  distLayout.yaxis = Object.assign({}, distLayout.yaxis, { automargin: true });
  distLayout.margin = Object.assign({}, distLayout.margin, { b: 80 });
  distLayout.hovermode = 'x unified';
  distLayout.dragmode = 'select';
  Plotly.newPlot(`distanceChart-${dirKey}`, distData, distLayout, { responsive: true, displaylogo: false, scrollZoom: true, modeBarButtonsToRemove: ['lasso2d','toImage','resetScale2d'] });
  // Overlay original points on main charts
  try {
    const tripSel = document.getElementById('tripIdSelect');
    const selectedId = (tripSel && tripSel.value) ? tripSel.value : '';
    if (selectedId) {
      const trips = await fetchRawTrips();
      const t = trips.find(r => r.dir === dirKey.toLowerCase() && String(r.trip_id) === String(selectedId));
      if (t) {
        Plotly.addTraces(`distanceChart-${dirKey}`, [{ x: t.t_rel, y: t.dist.map(v => v*0.3048), type: 'scattergl', mode: 'markers', marker: { size: 4, color: '#111827', opacity: 0.75 }, name: 'Original points', showlegend: true, hoverinfo: 'skip' }]);
        Plotly.addTraces(`speedChart-${dirKey}`, [{ x: t.t_rel, y: t.spd, type: 'scattergl', mode: 'markers', marker: { size: 4, color: '#111827', opacity: 0.75 }, name: 'Original points', showlegend: false, hoverinfo: 'skip' }]);
      }
    }
  } catch {}

  // Speed chart: only likely
  const spdData = [
    ensureTrace('Likely', time, likeSpd, { type: 'scatter', line: { color: getComputedStyle(document.documentElement).getPropertyValue('--text') || '#111827', width: 2 }, showlegend: true })
  ];
  const spdLayout = layout(`${dirKey} velocity vs time (no reverse)`, 'Time (s)', 'Velocity (m/s)');
  spdLayout.paper_bgcolor = '#ffffff';
  spdLayout.plot_bgcolor = '#ffffff';
  spdLayout.shapes = stopSpans.map(s => ({ type: 'rect', xref: 'x', yref: 'paper', y0: 0, y1: 1, x0: s.x0, x1: s.x1, fillcolor: s.fillcolor, line: { width: 0 } }));
  spdLayout.legend = { orientation: 'h', x: 0, y: -0.25, yanchor: 'top', font: { size: 10 } };
  spdLayout.xaxis = Object.assign({}, spdLayout.xaxis, { rangeslider: { visible: true, thickness: 0.08 }, automargin: true });
  spdLayout.yaxis = Object.assign({}, spdLayout.yaxis, { automargin: true });
  spdLayout.margin = Object.assign({}, spdLayout.margin, { b: 80 });
  spdLayout.hovermode = 'x unified';
  spdLayout.dragmode = 'select';
  try {
    Plotly.newPlot(`speedChart-${dirKey}`, spdData, spdLayout, { responsive: true, displaylogo: false, scrollZoom: true, modeBarButtonsToRemove: ['lasso2d','toImage','resetScale2d'] });
  } catch (e) {
    // If speed arrays are empty, render an empty placeholder chart
    Plotly.newPlot(`speedChart-${dirKey}`, [], spdLayout, { responsive: true, displaylogo: false });
  }

  // No segment grid rendering (removed for cleaner UI)

  // Rectangle select on overview → render detail charts (likely-only)
  async function renderDetail(dir, tSel, minPosSel, maxPosSel, likePosSel, minSpdSel, maxSpdSel, likeSpdSel) {
    const titleEl = document.getElementById(`detail-title-${dir}`);
    if (titleEl && tSel.length) {
      const t0 = tSel[0].toFixed(1), t1 = tSel[tSel.length - 1].toFixed(1);
      titleEl.textContent = `${dir} detail ${t0}–${t1} s`;
    }
    const distSeries = [
      ensureTrace('Likely', tSel, likePosSel, { type: 'scattergl', line: { color: '#111827', width: 2 }, showlegend: false })
    ];
    try {
      const tripSel = document.getElementById('tripIdSelect');
      const selectedId = (tripSel && tripSel.value) ? tripSel.value : '';
      if (selectedId) {
        const trips = await fetchRawTrips();
        const tr = trips.find(r => r.dir === dir && String(r.trip_id) === String(selectedId));
        if (tr) {
          const idx = tr.t_rel.map((t, i) => [t, i]).filter(([t]) => t >= tSel[0] && t <= tSel[tSel.length-1]).map(([,i]) => i);
          distSeries.push({ x: idx.map(i => tr.t_rel[i]), y: idx.map(i => tr.dist[i]*0.3048), type: 'scattergl', mode: 'markers', marker: { size: 4, color: '#111827', opacity: 0.75 }, name: 'Original points', showlegend: false, hoverinfo: 'skip' });
        }
      }
    } catch {}
    Plotly.newPlot(`detail-distance-${dir}`, distSeries, layout('Distance detail', 'Time (s)', 'Distance (ft)'), { responsive: true, displaylogo: false });

    const spdSeries = [
      ensureTrace('Likely', tSel, likeSpdSel, { type: 'scattergl', line: { color: '#111827', width: 2 }, showlegend: false })
    ];
    try {
      const tripSel = document.getElementById('tripIdSelect');
      const selectedId = (tripSel && tripSel.value) ? tripSel.value : '';
      if (selectedId) {
        const trips = await fetchRawTrips();
        const tr = trips.find(r => r.dir === dir && String(r.trip_id) === String(selectedId));
        if (tr) {
          const idx = tr.t_rel.map((t, i) => [t, i]).filter(([t]) => t >= tSel[0] && t <= tSel[tSel.length-1]).map(([,i]) => i);
          spdSeries.push({ x: idx.map(i => tr.t_rel[i]), y: idx.map(i => tr.spd[i]), type: 'scattergl', mode: 'markers', marker: { size: 4, color: '#111827', opacity: 0.75 }, name: 'Original points', showlegend: false, hoverinfo: 'skip' });
        }
      }
    } catch {}
    Plotly.newPlot(`detail-speed-${dir}`, spdSeries, layout('Velocity detail', 'Time (s)', 'Velocity (m/s)'), { responsive: true, displaylogo: false });
  }

  function attachSelectHandler(chartId) {
    const gd = document.getElementById(chartId);
    if (!gd || !gd.on) return;
    // Support both plotly_selected and relayout (range) to be robust
    function handleRange(lo, hi) {
      const idx = time.map((t, i) => [t, i]).filter(([t]) => t >= lo && t <= hi).map(([,i]) => i);
      if (idx.length < 2) return;
      const tSel = idx.map(i => time[i]);
      renderDetail(
        dirKey,
        tSel,
        idx.map(i => minPos[i]),
        idx.map(i => maxPos[i]),
        idx.map(i => likePos[i]),
        idx.map(i => minSpd[i]),
        idx.map(i => maxSpd[i]),
        idx.map(i => likeSpd[i])
      );
    }
    gd.on('plotly_selected', (ev) => {
      if (!ev || !ev.range) return;
      const x0 = ev.range.x[0];
      const x1 = ev.range.x[1];
      const lo = Math.min(x0, x1);
      const hi = Math.max(x0, x1);
      handleRange(lo, hi);
    });
    gd.on('plotly_relayout', (ev) => {
      // If user uses range slider or double click reset, ignore
      if (!ev) return;
      const xa0 = ev['xaxis.range[0]'];
      const xa1 = ev['xaxis.range[1]'];
      if (typeof xa0 === 'number' && typeof xa1 === 'number') {
        const lo = Math.min(xa0, xa1);
        const hi = Math.max(xa0, xa1);
        handleRange(lo, hi);
      }
    });
  }

  attachSelectHandler(`distanceChart-${dirKey}`);
  attachSelectHandler(`speedChart-${dirKey}`);

  // Wire up Clear button to reset detail panel
  const clearBtn = document.getElementById(`clearDetail-${dirKey}`);
  if (clearBtn) {
    clearBtn.onclick = () => {
      const tEl = document.getElementById(`detail-title-${dirKey}`);
      if (tEl) tEl.textContent = 'Select a segment by clicking the overview';
      try { Plotly.purge(`detail-distance-${dirKey}`); } catch {}
      try { Plotly.purge(`detail-speed-${dirKey}`); } catch {}
    };
  }
}

async function bootstrap() {
  const manifest = await buildManifest();
  const trainSelect = document.getElementById('trainSelect');
  manifest.trains.forEach((t, idx) => {
    const opt = document.createElement('option');
    opt.value = idx;
    opt.textContent = t.label || t.id;
    trainSelect.appendChild(opt);
  });

  async function updateKpisAndLinks(spec) {
    try {
      const summary = await fetchJSON(spec.summary);
      document.getElementById('kpi-duration').textContent = `${summary.duration_minutes.toFixed(1)} min`;
      document.getElementById('kpi-distance').textContent = `${(summary.distance_ft/5280).toFixed(2)} mi`;
      const avg_ms = Number(summary.speed_stats.avg_mph) * 0.44704;
      const max_ms = Number(summary.speed_stats.max_mph) * 0.44704;
      document.getElementById('kpi-avg').textContent = `${avg_ms.toFixed(2)} m/s`;
      document.getElementById('kpi-max').textContent = `${max_ms.toFixed(2)} m/s`;
      document.getElementById('kpi-stops').textContent = (summary.stops_summary?.total_stops ?? 0);
    } catch {
      // leave placeholders
    }
    document.getElementById('dl-csv').href = spec.interpolated;
    document.getElementById('dl-stops').href = spec.stops;
    document.getElementById('dl-summary').href = spec.summary;
  }

  function setActiveSection(dir) {
    document.querySelectorAll('.direction-section').forEach(s => s.classList.remove('active'));
    document.getElementById(`tab-${dir}`).classList.add('active');
  }

  async function loadTrain(idx) {
    const t = manifest.trains[idx];
    // render currently selected direction only for performance
    const selected = document.querySelector('input[name="dir"]:checked')?.value || 'inbound';
    setActiveSection(selected);
    await populateTripSelect(selected);
    await renderDirection(selected, t[selected]);
    await updateKpisAndLinks(t[selected]);
  }

  trainSelect.addEventListener('change', (e) => loadTrain(e.target.value));
  // Removed segment range buttons
  const resetBtn = document.getElementById('resetZoomBtn');
  if (resetBtn) {
    resetBtn.addEventListener('click', () => {
      ['distanceChart-inbound','speedChart-inbound','distanceChart-outbound','speedChart-outbound']
        .forEach(id => Plotly.relayout(id, { 'xaxis.autorange': true, 'yaxis.autorange': true }));
    });
  }

  // Direction radios
  document.querySelectorAll('input[name="dir"]').forEach(r => {
    r.addEventListener('change', async () => {
      const dir = document.querySelector('input[name=\"dir\"]:checked').value;
      const t = manifest.trains[trainSelect.value];
      setActiveSection(dir);
      await populateTripSelect(dir);
      await renderDirection(dir, t[dir]);
      await updateKpisAndLinks(t[dir]);
    });
  });

  // Trip selection change: switch dataset to selected trip if available
  const selTrip = document.getElementById('tripIdSelect');
  if (selTrip) {
    selTrip.addEventListener('change', async () => {
      const dir = document.querySelector('input[name="dir"]:checked')?.value || 'inbound';
      const tripId = selTrip.value;
      // If we have precomputed interpolation for this trip, switch data sources entirely
      const manifest = await buildManifest();
      const fallbackSpec = manifest.trains[0][dir];
      const customSpec = tripId ? await buildSpecForTrip(dir, tripId) : null;
      const spec = customSpec || fallbackSpec;
      await renderDirection(dir, spec);
      await updateKpisAndLinks(spec);
    });
  }
  await loadTrain(0);
}

bootstrap().catch(err => {
  console.error(err);
  const banner = document.createElement('div');
  banner.style.cssText = 'position:fixed;left:0;right:0;bottom:0;background:#fee2e2;color:#7f1d1d;padding:8px 12px;border-top:1px solid #fecaca;font-size:12px;z-index:9999;';
  banner.textContent = 'Failed to load data. Check console for details (try hard refresh).';
  document.body.appendChild(banner);
});

// Tabs behavior
document.querySelectorAll('.tab').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    document.querySelectorAll('.direction-section').forEach(s => s.classList.remove('active'));
    const tab = btn.dataset.tab;
    document.getElementById(`tab-${tab}`).classList.add('active');
  });
});


