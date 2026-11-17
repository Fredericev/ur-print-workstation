#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def load_points(csv_path: Path):
    points = []
    with csv_path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append(
                {
                    "layer": int(row["layer"]),
                    "index": int(row["index"]),
                    "segment": row["segment"],
                    "x": round(float(row["x_m"]) * 1000.0, 3),
                    "y": round(float(row["y_m"]) * 1000.0, 3),
                    "z": round(float(row["z_m"]) * 1000.0, 3),
                }
            )
    return points


def build_html(points: list[dict], title: str, subtitle: str) -> str:
    template = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>__TITLE_TEXT__</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b1020;
      --panel: #111831;
      --panel2: #16203f;
      --text: #eef2ff;
      --muted: #94a3b8;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, sans-serif;
      background: radial-gradient(circle at top, #16203f 0%, var(--bg) 55%);
      color: var(--text);
    }
    .wrap { max-width: 1240px; margin: 0 auto; padding: 20px; }
    .hero {
      background: rgba(17, 24, 49, 0.92);
      border: 1px solid rgba(148, 163, 184, 0.18);
      border-radius: 20px;
      padding: 18px 20px;
      box-shadow: 0 20px 50px rgba(0,0,0,.28);
      backdrop-filter: blur(10px);
    }
    h1 { margin: 0 0 6px; font-size: 28px; }
    .sub { color: var(--muted); margin-bottom: 16px; line-height: 1.5; }
    .stats { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 14px; }
    .chip {
      background: rgba(96,165,250,.12);
      border: 1px solid rgba(96,165,250,.28);
      padding: 8px 12px;
      border-radius: 999px;
      font-size: 13px;
    }
    .controls { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; margin-bottom: 16px; }
    button {
      background: linear-gradient(180deg, #2b4fa1, #17306b);
      color: white;
      border: none;
      border-radius: 12px;
      padding: 10px 14px;
      cursor: pointer;
      font-weight: 600;
    }
    button.secondary { background: linear-gradient(180deg, #1f2937, #111827); }
    .speed { display: flex; align-items: center; gap: 8px; color: var(--muted); }
    input[type=range] { width: 180px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(300px, 1fr)); gap: 16px; }
    .card {
      background: rgba(22, 32, 63, 0.92);
      border: 1px solid rgba(148, 163, 184, 0.16);
      border-radius: 18px;
      padding: 12px;
    }
    .card h2 { font-size: 15px; margin: 0 0 10px; color: #dbeafe; }
    canvas { width: 100%; height: auto; display: block; border-radius: 12px; background: #060b18; }
    .foot { margin-top: 12px; color: var(--muted); font-size: 13px; line-height: 1.45; }
    .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 10px; color: var(--muted); font-size: 12px; }
    .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; margin-right: 6px; vertical-align: middle; }
    @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1 id="title"></h1>
      <div class="sub" id="subtitle"></div>
      <div class="stats" id="stats"></div>
      <div class="controls">
        <button id="toggle">Pause</button>
        <button class="secondary" id="restart">Replay</button>
        <div class="speed">Speed <input id="speed" type="range" min="0.4" max="3.0" step="0.1" value="1.4" /><span id="speedLabel">1.4×</span></div>
        <div class="speed">Current layer <strong id="layerLabel">1</strong> / <span id="layerCount"></span></div>
      </div>
      <div class="grid">
        <div class="card">
          <h2>Top view (nozzle path)</h2>
          <canvas id="topCanvas" width="560" height="560"></canvas>
        </div>
        <div class="card">
          <h2>Isometric view (layer buildup)</h2>
          <canvas id="isoCanvas" width="560" height="560"></canvas>
        </div>
      </div>
      <div class="legend">
        <span><span class="dot" style="background:#f43f5e"></span>Current nozzle position</span>
        <span><span class="dot" style="background:#60a5fa"></span>Printed path</span>
        <span><span class="dot" style="background:#94a3b8"></span>Travel between layers</span>
      </div>
      <div class="foot">
        This animation previews the <strong>nozzle path</strong> by treating UR3e <code>tool0</code> as the nozzle position.<br/>
        It shows the toolpath only; it does not animate the complete robot arm or validate robot motion.
      </div>
    </div>
  </div>

  <script>
    const title = __TITLE_JSON__;
    const subtitle = __SUBTITLE_JSON__;
    const points = __POINTS_JSON__;
    const layerColors = ['#60a5fa', '#38bdf8', '#22c55e', '#f59e0b', '#f97316', '#e879f9', '#a78bfa'];

    document.getElementById('title').textContent = title;
    document.getElementById('subtitle').textContent = subtitle;

    const layers = [...new Set(points.map(p => p.layer))];
    const xVals = points.map(p => p.x);
    const yVals = points.map(p => p.y);
    const zVals = points.map(p => p.z);
    document.getElementById('layerCount').textContent = String(layers.length);
    document.getElementById('stats').innerHTML = [
      `Dimensions: 50 × 50 × 10 mm`,
      `Layers: ${layers.length}`,
      `Path points: ${points.length}`,
      `X range: ${Math.min(...xVals).toFixed(0)} ~ ${Math.max(...xVals).toFixed(0)} mm`,
      `Y range: ${Math.min(...yVals).toFixed(0)} ~ ${Math.max(...yVals).toFixed(0)} mm`,
      `Z range: ${Math.min(...zVals).toFixed(0)} ~ ${Math.max(...zVals).toFixed(0)} mm`
    ].map(v => `<span class="chip">${v}</span>`).join('');

    const segments = [];
    let totalDuration = 0;
    for (let i = 1; i < points.length; i++) {
      const from = points[i - 1];
      const to = points[i];
      const travel = from.layer !== to.layer;
      const dx = to.x - from.x;
      const dy = to.y - from.y;
      const dz = to.z - from.z;
      const dist = Math.hypot(dx, dy, dz);
      const duration = travel ? 0.45 : Math.max(0.12, dist / 220.0);
      segments.push({ from, to, travel, duration, start: totalDuration, end: totalDuration + duration });
      totalDuration += duration;
    }
    totalDuration += 0.9;

    const topCanvas = document.getElementById('topCanvas');
    const isoCanvas = document.getElementById('isoCanvas');
    const topCtx = topCanvas.getContext('2d');
    const isoCtx = isoCanvas.getContext('2d');
    const speedSlider = document.getElementById('speed');
    const speedLabel = document.getElementById('speedLabel');
    const layerLabel = document.getElementById('layerLabel');
    const toggleBtn = document.getElementById('toggle');
    const restartBtn = document.getElementById('restart');

    let speed = Number(speedSlider.value);
    let playing = true;
    let elapsed = 0;
    let lastTs = null;

    speedSlider.addEventListener('input', () => {
      speed = Number(speedSlider.value);
      speedLabel.textContent = `${speed.toFixed(1)}×`;
    });

    toggleBtn.addEventListener('click', () => {
      playing = !playing;
      toggleBtn.textContent = playing ? 'Pause' : 'Play';
    });

    restartBtn.addEventListener('click', () => {
      elapsed = 0;
      lastTs = null;
    });

    function projectTop(p) {
      return { x: p.x, y: p.y };
    }

    function projectIso(p) {
      const px = (p.x - p.y) * 0.866;
      const py = (p.x + p.y) * 0.5 - p.z * 5.6;
      return { x: px, y: py };
    }

    function computeBounds(project) {
      const arr = points.map(project);
      return {
        minX: Math.min(...arr.map(p => p.x)),
        maxX: Math.max(...arr.map(p => p.x)),
        minY: Math.min(...arr.map(p => p.y)),
        maxY: Math.max(...arr.map(p => p.y)),
      };
    }

    const topBounds = computeBounds(projectTop);
    const isoBounds = computeBounds(projectIso);

    function makeMapper(bounds, canvas) {
      const pad = 42;
      const w = canvas.width;
      const h = canvas.height;
      const scaleX = (w - pad * 2) / Math.max(1, bounds.maxX - bounds.minX);
      const scaleY = (h - pad * 2) / Math.max(1, bounds.maxY - bounds.minY);
      const scale = Math.min(scaleX, scaleY);
      return (p) => {
        const x = (p.x - bounds.minX) * scale + pad;
        const y = h - ((p.y - bounds.minY) * scale + pad);
        return { x, y };
      };
    }

    const mapTop = makeMapper(topBounds, topCanvas);
    const mapIso = makeMapper(isoBounds, isoCanvas);

    function clearCanvas(ctx, canvas) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const g = ctx.createLinearGradient(0, 0, 0, canvas.height);
      g.addColorStop(0, '#08101e');
      g.addColorStop(1, '#050913');
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    }

    function drawGrid(ctx, canvas) {
      ctx.save();
      ctx.strokeStyle = 'rgba(148,163,184,0.09)';
      ctx.lineWidth = 1;
      for (let i = 40; i < canvas.width; i += 40) {
        ctx.beginPath();
        ctx.moveTo(i, 0);
        ctx.lineTo(i, canvas.height);
        ctx.stroke();
      }
      for (let i = 40; i < canvas.height; i += 40) {
        ctx.beginPath();
        ctx.moveTo(0, i);
        ctx.lineTo(canvas.width, i);
        ctx.stroke();
      }
      ctx.restore();
    }

    function drawSegment(ctx, mapper, from, to, color, width, dashed) {
      const a = mapper(from);
      const b = mapper(to);
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.setLineDash(dashed ? [7, 7] : []);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      ctx.restore();
    }

    function mixPoint(a, b, t) {
      return {
        x: a.x + (b.x - a.x) * t,
        y: a.y + (b.y - a.y) * t,
        z: a.z + (b.z - a.z) * t,
        layer: t < 1 ? a.layer : b.layer,
      };
    }

    function getPlaybackState(time) {
      const activeTime = Math.min(time, totalDuration - 0.9);
      let currentSeg = null;
      let currentIndex = 0;
      let t = 0;
      for (let i = 0; i < segments.length; i++) {
        const seg = segments[i];
        if (activeTime <= seg.end) {
          currentSeg = seg;
          currentIndex = i;
          t = (activeTime - seg.start) / seg.duration;
          t = Math.max(0, Math.min(1, t));
          break;
        }
      }
      if (!currentSeg) {
        currentSeg = segments[segments.length - 1];
        currentIndex = segments.length - 1;
        t = 1;
      }
      return { currentSeg, currentIndex, t };
    }

    function drawScene(ctx, canvas, mapper, projectionName, playback) {
      clearCanvas(ctx, canvas);
      drawGrid(ctx, canvas);

      ctx.save();
      ctx.fillStyle = 'rgba(148,163,184,0.05)';
      ctx.strokeStyle = 'rgba(148,163,184,0.18)';
      ctx.lineWidth = 1.5;
      const bounds = projectionName === 'top' ? topBounds : isoBounds;
      const a = mapper({ x: bounds.minX, y: bounds.minY });
      const b = mapper({ x: bounds.maxX, y: bounds.maxY });
      const left = Math.min(a.x, b.x);
      const top = Math.min(a.y, b.y);
      const width = Math.abs(a.x - b.x);
      const height = Math.abs(a.y - b.y);
      ctx.fillRect(left, top, width, height);
      ctx.strokeRect(left, top, width, height);
      ctx.restore();

      for (let i = 0; i < playback.currentIndex; i++) {
        const seg = segments[i];
        const color = seg.travel ? 'rgba(148,163,184,0.55)' : layerColors[seg.to.layer % layerColors.length];
        drawSegment(ctx, mapper, projectionName === 'top' ? projectTop(seg.from) : projectIso(seg.from), projectionName === 'top' ? projectTop(seg.to) : projectIso(seg.to), color, seg.travel ? 2 : 3.4, seg.travel);
      }

      const active = playback.currentSeg;
      const partialPoint = mixPoint(active.from, active.to, playback.t);
      const fromProjected = projectionName === 'top' ? projectTop(active.from) : projectIso(active.from);
      const toProjected = projectionName === 'top' ? projectTop(partialPoint) : projectIso(partialPoint);
      const activeColor = active.travel ? 'rgba(148,163,184,0.65)' : layerColors[active.to.layer % layerColors.length];
      drawSegment(ctx, mapper, fromProjected, toProjected, activeColor, active.travel ? 2.5 : 4.2, active.travel);

      const dotPos = mapper(projectionName === 'top' ? projectTop(partialPoint) : projectIso(partialPoint));
      ctx.save();
      ctx.fillStyle = '#f43f5e';
      ctx.beginPath();
      ctx.arc(dotPos.x, dotPos.y, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,0.85)';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();

      ctx.save();
      ctx.fillStyle = '#cbd5e1';
      ctx.font = '13px Inter, sans-serif';
      const p = partialPoint;
      ctx.fillText(`Nozzle: x=${p.x.toFixed(1)}mm  y=${p.y.toFixed(1)}mm  z=${p.z.toFixed(1)}mm`, 18, canvas.height - 18);
      ctx.restore();
    }

    function frame(ts) {
      if (lastTs == null) lastTs = ts;
      const dt = (ts - lastTs) / 1000;
      lastTs = ts;
      if (playing) {
        elapsed += dt * speed;
        if (elapsed > totalDuration) elapsed = 0;
      }
      const playback = getPlaybackState(elapsed);
      layerLabel.textContent = String(playback.currentSeg.to.layer + 1);
      drawScene(topCtx, topCanvas, mapTop, 'top', playback);
      drawScene(isoCtx, isoCanvas, mapIso, 'iso', playback);
      requestAnimationFrame(frame);
    }

    requestAnimationFrame(frame);
  </script>
</body>
</html>
"""
    return (
        template.replace("__TITLE_TEXT__", title)
        .replace("__TITLE_JSON__", json.dumps(title, ensure_ascii=False))
        .replace("__SUBTITLE_JSON__", json.dumps(subtitle, ensure_ascii=False))
        .replace("__POINTS_JSON__", json.dumps(points, ensure_ascii=False))
    )


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    output_dir = project_root / "workspaces" / "ur_ros2_ws" / "output"
    parser = argparse.ArgumentParser(description="Generate an animated HTML preview for the UR3e print toolpath.")
    parser.add_argument(
        "--csv",
        type=Path,
        default=output_dir / "print_box_50x50x10_toolpath.csv",
    )
    parser.add_argument(
        "--output-html",
        type=Path,
        default=output_dir / "print_box_50x50x10_animation.html",
    )
    parser.add_argument(
        "--canvas-html",
        type=Path,
        default=None,
    )
    args = parser.parse_args()

    csv_path = args.csv
    output_html = args.output_html
    canvas_html = args.canvas_html
    output_html.parent.mkdir(parents=True, exist_ok=True)
    if canvas_html is not None:
        canvas_html.parent.mkdir(parents=True, exist_ok=True)

    points = load_points(csv_path)
    html = build_html(
        points,
        title="UR3e 3D Print Path Animation Preview",
        subtitle="Preview of a 50 × 50 × 10 mm print path, treating UR3e tool0 as the nozzle position.",
    )
    output_html.write_text(html, encoding="utf-8")
    if canvas_html is not None:
        canvas_html.write_text(html, encoding="utf-8")

    print(output_html)
    print(canvas_html)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
