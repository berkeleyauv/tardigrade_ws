export const PANEL_STYLE = `
  :host, .tg-panel { color: var(--tg-text); font: 13px Inter, system-ui, sans-serif; }
  .tg-panel { --tg-bg:#10151d; --tg-card:#19212c; --tg-border:#304052; --tg-text:#eef4fa;
    --tg-muted:#91a2b5; --tg-blue:#58a6ff; --tg-green:#3fd16f; --tg-amber:#f5b642;
    --tg-red:#ef6b5b; height:100%; box-sizing:border-box; background:var(--tg-bg); overflow:auto; }
  .tg-panel[data-theme="light"] { --tg-bg:#f4f7fa; --tg-card:#fff; --tg-border:#cad4df;
    --tg-text:#17202a; --tg-muted:#64748b; }
  .tg-pad { padding:10px; box-sizing:border-box; min-height:100%; }
  .tg-title { font-size:15px; font-weight:700; letter-spacing:.02em; margin:0 0 8px; }
  .tg-subtitle { color:var(--tg-muted); font-size:11px; margin-top:-4px; margin-bottom:8px; }
  .tg-card { background:var(--tg-card); border:1px solid var(--tg-border); border-radius:6px; padding:9px; }
  .tg-grid { display:grid; gap:8px; }
  .tg-grid-2 { grid-template-columns:repeat(2,minmax(0,1fr)); }
  .tg-grid-3 { grid-template-columns:repeat(3,minmax(0,1fr)); }
  .tg-grid-4 { grid-template-columns:repeat(4,minmax(0,1fr)); }
  .tg-section { color:var(--tg-muted); font-size:10px; font-weight:700; letter-spacing:.12em;
    text-transform:uppercase; margin:12px 0 6px; }
  .tg-kpi { min-width:0; }
  .tg-kpi-label { color:var(--tg-muted); font-size:10px; text-transform:uppercase; }
  .tg-kpi-value { font-size:17px; font-variant-numeric:tabular-nums; font-weight:700; margin-top:2px; }
  .tg-dot { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:6px; background:var(--tg-muted); }
  .ok .tg-dot, .tg-dot.ok { background:var(--tg-green); box-shadow:0 0 7px color-mix(in srgb,var(--tg-green) 65%,transparent); }
  .warn .tg-dot, .tg-dot.warn { background:var(--tg-amber); }
  .bad .tg-dot, .tg-dot.bad { background:var(--tg-red); }
  .muted .tg-dot, .tg-dot.muted { background:var(--tg-muted); }
  .tg-form { display:grid; grid-template-columns:repeat(5,minmax(66px,1fr)); gap:6px; }
  .tg-field label { display:block; color:var(--tg-muted); font-size:10px; margin:0 0 3px; }
  .tg-field input, .tg-select { width:100%; box-sizing:border-box; color:var(--tg-text); background:var(--tg-bg);
    border:1px solid var(--tg-border); border-radius:4px; padding:7px; font:inherit; font-variant-numeric:tabular-nums; }
  .tg-actions { display:flex; gap:6px; flex-wrap:wrap; margin-top:8px; }
  .tg-button { color:var(--tg-text); background:#263445; border:1px solid var(--tg-border); border-radius:4px;
    padding:7px 10px; font-weight:650; cursor:pointer; }
  .tg-button:hover { filter:brightness(1.15); }
  .tg-button:disabled { cursor:not-allowed; opacity:.45; }
  .tg-button.primary { background:#1769aa; }
  .tg-button.danger { background:#8e2d28; }
  .tg-button.safe { background:#176b3a; }
  .tg-note { min-height:17px; color:var(--tg-muted); font-size:11px; margin-top:6px; }
  .tg-note.error { color:var(--tg-red); }
  .tg-canvas { width:100%; height:180px; display:block; background:var(--tg-bg); border-radius:4px; }
  .tg-legend { display:flex; gap:12px; color:var(--tg-muted); font-size:11px; margin-top:5px; }
  .tg-swatch { display:inline-block; width:14px; height:3px; vertical-align:middle; margin-right:4px; }
  .tg-meter { height:10px; background:var(--tg-bg); border:1px solid var(--tg-border); position:relative; border-radius:3px; overflow:hidden; }
  .tg-meter:after { content:""; position:absolute; left:50%; top:0; bottom:0; width:1px; background:var(--tg-muted); }
  .tg-meter-fill { height:100%; position:absolute; background:var(--tg-blue); }
  .tg-terms { display:grid; grid-template-columns:repeat(3,1fr); gap:6px; }
  .tg-term { text-align:center; font-variant-numeric:tabular-nums; }
  .tg-term b { display:block; font-size:15px; }
  .tg-status { padding:8px; min-height:42px; box-sizing:border-box; }
  .tg-status .tg-kpi-value { font-size:12px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .tg-sensors { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:5px; }
  .tg-sensor { padding:6px; border:1px solid var(--tg-border); border-radius:4px; font-size:11px; }
  .tg-sensor-age { display:block; color:var(--tg-muted); font-variant-numeric:tabular-nums; margin-top:2px; }
  .tg-thrusters { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:5px 10px; }
  .tg-thruster-row { display:grid; grid-template-columns:42px 1fr 42px; align-items:center; gap:5px; font-size:10px; }
  .tg-thruster-track { height:9px; background:var(--tg-bg); border:1px solid var(--tg-border); position:relative; }
  .tg-thruster-track:after { content:""; position:absolute; left:50%; top:0; bottom:0; width:1px; background:var(--tg-muted); }
  .tg-thruster-fill { position:absolute; top:1px; bottom:1px; background:var(--tg-blue); }
  .tg-number { text-align:right; font-variant-numeric:tabular-nums; }
  .tg-instrument { display:grid; grid-template-columns:minmax(220px,1.5fr) minmax(130px,1fr); gap:8px; height:100%; }
  .tg-attitude-canvas { width:100%; height:100%; min-height:210px; display:block; background:#07121f; border-radius:6px; }
  .tg-readouts { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:6px; align-content:start; }
  @media (max-width:700px) { .tg-form{grid-template-columns:repeat(2,1fr)} .tg-grid-4{grid-template-columns:repeat(2,1fr)}
    .tg-instrument{grid-template-columns:1fr}.tg-readouts{grid-template-columns:repeat(4,1fr)} }
`;

export function createPanelRoot(className: string): HTMLDivElement {
  const root = document.createElement("div");
  root.className = `tg-panel ${className}`;
  const style = document.createElement("style");
  style.textContent = PANEL_STYLE;
  root.appendChild(style);
  return root;
}

export function applyTheme(root: HTMLElement, colorScheme: string | undefined): void {
  root.dataset.theme = colorScheme === "light" ? "light" : "dark";
}
