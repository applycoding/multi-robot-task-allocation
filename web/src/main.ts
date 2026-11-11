// SPDX-License-Identifier: Apache-2.0
import "./styles.css";

export const HONESTY = [
  "Simulation only",
  "Synthetic fleet scenarios; not a live allocator",
  "Recorded CI output; this page does not run the simulator",
  "Independent travel is not multi-agent path finding; robots may occupy the same edge.",
] as const;

interface ReportRow {
  allocator: string;
  family: string;
  seed: number;
  makespan_s: number;
  pass: boolean;
  expected: string;
  observed: string;
}

interface Report {
  sample: boolean;
  git_sha: string;
  rows: ReportRow[];
}

async function loadReport(): Promise<Report | null> {
  const base = import.meta.env.BASE_URL;
  const url = `${base}report.json`;
  try {
    const res = await fetch(url);
    if (!res.ok) {
      return null;
    }
    return (await res.json()) as Report;
  } catch {
    return null;
  }
}

function drawCanvas(rows: ReportRow[]): void {
  const canvas = document.getElementById("timeline") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return;
  }
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#0f172a";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#38bdf8";
  const barW = Math.max(4, canvas.width / Math.max(rows.length, 1) - 2);
  rows.slice(0, 80).forEach((r, i) => {
    const h = Math.min(canvas.height - 20, (r.makespan_s / 320) * (canvas.height - 20));
    const x = 10 + i * (barW + 2);
    ctx.fillStyle = r.pass ? "#22c55e" : "#ef4444";
    ctx.fillRect(x, canvas.height - h - 10, barW, h);
  });
}

function render(report: Report | null): void {
  const root = document.getElementById("app");
  if (!root) {
    return;
  }
  root.innerHTML = "";

  const title = document.createElement("h1");
  title.textContent = "Multi-robot task allocation";
  root.appendChild(title);

  for (const line of HONESTY) {
    const p = document.createElement("p");
    p.className = "honesty";
    p.textContent = line;
    root.appendChild(p);
  }

  if (report === null) {
    const err = document.createElement("p");
    err.id = "error-state";
    err.textContent = "Recorded report missing or malformed.";
    root.appendChild(err);
    return;
  }

  if (report.sample) {
    const banner = document.createElement("p");
    banner.id = "sample-banner";
    banner.textContent = "Sample report for layout only";
    root.appendChild(banner);
  }

  const meta = document.createElement("p");
  meta.textContent = `git_sha: ${report.git_sha} | rows: ${report.rows.length}`;
  root.appendChild(meta);

  const canvas = document.createElement("canvas");
  canvas.id = "timeline";
  canvas.width = 900;
  canvas.height = 280;
  root.appendChild(canvas);

  const table = document.createElement("table");
  table.innerHTML =
    "<thead><tr><th>allocator</th><th>family</th><th>seed</th><th>makespan_s</th><th>expected</th><th>observed</th></tr></thead>";
  const tbody = document.createElement("tbody");
  for (const row of report.rows.slice(0, 55)) {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${row.allocator}</td><td>${row.family}</td><td>${row.seed}</td><td>${row.makespan_s.toFixed(
      1,
    )}</td><td>${row.expected}</td><td>${row.observed}</td>`;
    tbody.appendChild(tr);
  }
  table.appendChild(tbody);
  root.appendChild(table);

  drawCanvas(report.rows.filter((r) => r.expected === "pass"));
}

export function bootDashboard(): void {
  void loadReport().then(render);
}

if (import.meta.env.MODE !== "test") {
  bootDashboard();
}
