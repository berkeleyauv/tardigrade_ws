import type { Immutable, MessageEvent, PanelExtensionContext, RenderState } from "@foxglove/extension";

import type { BoolMsg, PidDebug } from "./messages";
import { finite, timeToSec } from "./messages";
import { applyTheme, createPanelRoot } from "./style";

const AXES = ["surge", "sway", "heave", "roll", "pitch", "yaw"] as const;
type Axis = (typeof AXES)[number];
type Sample = { time: number; setpoint: number; measurement: number };

const SERVICE = "/tardigrade/control/set_velocity_pid_gains";
const RESET_SERVICE = "/tardigrade/control/reset_pid";
const TOPIC = (axis: Axis) => `/tardigrade/control/${axis}/debug`;

function isAxis(value: unknown): value is Axis {
  return typeof value === "string" && (AXES as readonly string[]).includes(value);
}

function format(value: number, digits = 3): string {
  return Number.isFinite(value) ? value.toFixed(digits) : "—";
}

function drawResponse(canvas: HTMLCanvasElement, samples: Sample[]): void {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(200, canvas.clientWidth);
  const height = Math.max(140, canvas.clientHeight);
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const context = canvas.getContext("2d");
  if (context == undefined) {
    return;
  }
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  const css = getComputedStyle(canvas.closest(".tg-panel") ?? canvas);
  const grid = css.getPropertyValue("--tg-border").trim() || "#304052";
  const muted = css.getPropertyValue("--tg-muted").trim() || "#91a2b5";
  const pad = { left: 42, right: 9, top: 9, bottom: 23 };
  const chartWidth = width - pad.left - pad.right;
  const chartHeight = height - pad.top - pad.bottom;
  context.strokeStyle = grid;
  context.lineWidth = 1;
  context.fillStyle = muted;
  context.font = "10px system-ui";
  context.textAlign = "right";
  const peak = Math.max(0.05, ...samples.flatMap((sample) => [Math.abs(sample.setpoint), Math.abs(sample.measurement)]));
  const limit = peak * 1.15;
  for (let index = 0; index <= 4; index += 1) {
    const y = pad.top + chartHeight * index / 4;
    context.beginPath();
    context.moveTo(pad.left, y);
    context.lineTo(width - pad.right, y);
    context.stroke();
    context.fillText(format(limit * (1 - index / 2), 2), pad.left - 5, y + 3);
  }
  if (samples.length < 2) {
    context.textAlign = "center";
    context.fillText("Waiting for PID debug samples", pad.left + chartWidth / 2, pad.top + chartHeight / 2);
    return;
  }
  const newest = samples[samples.length - 1]?.time ?? 0;
  const oldest = Math.max(samples[0]?.time ?? newest - 8, newest - 8);
  const duration = Math.max(0.1, newest - oldest);
  const drawLine = (pick: (sample: Sample) => number, color: string): void => {
    context.beginPath();
    context.strokeStyle = color;
    context.lineWidth = 2;
    let started = false;
    for (const sample of samples) {
      if (sample.time < oldest) {
        continue;
      }
      const x = pad.left + (sample.time - oldest) / duration * chartWidth;
      const y = pad.top + (limit - pick(sample)) / (2 * limit) * chartHeight;
      if (!started) {
        context.moveTo(x, y);
        started = true;
      } else {
        context.lineTo(x, y);
      }
    }
    context.stroke();
  };
  drawLine((sample) => sample.setpoint, "#58a6ff");
  drawLine((sample) => sample.measurement, "#f5b642");
  context.fillStyle = muted;
  context.textAlign = "left";
  context.fillText("8 s", pad.left, height - 6);
  context.textAlign = "right";
  context.fillText("now", width - pad.right, height - 6);
}

export function initPidTunerPanel(context: PanelExtensionContext): () => void {
  const initial = context.initialState as { axis?: unknown } | undefined;
  let selected: Axis = isAxis(initial?.axis) ? initial.axis : "surge";
  const latest = new Map<Axis, PidDebug>();
  const histories = new Map<Axis, Sample[]>(AXES.map((axis) => [axis, []]));
  let controllerEnabled = false;

  const root = createPanelRoot("tg-pid");
  const body = document.createElement("div");
  body.className = "tg-pad";
  body.innerHTML = `
    <div class="tg-grid tg-grid-2" style="grid-template-columns:minmax(150px,.35fr) minmax(300px,1fr)">
      <div>
        <div class="tg-title">Velocity PID tuner</div>
        <div class="tg-subtitle">One axis at a time · gains update atomically · controller state resets on apply</div>
        <select class="tg-select" data-id="axis">${AXES.map((axis) => `<option value="${axis}">${axis.toUpperCase()}</option>`).join("")}</select>
        <div class="tg-section">Live response</div>
        <div class="tg-grid tg-grid-2">
          <div class="tg-card tg-kpi"><div class="tg-kpi-label">Error</div><div class="tg-kpi-value" data-id="error">—</div></div>
          <div class="tg-card tg-kpi"><div class="tg-kpi-label">Output</div><div class="tg-kpi-value" data-id="output">—</div></div>
        </div>
        <div class="tg-card" style="margin-top:8px">
          <div style="display:flex;justify-content:space-between;margin-bottom:5px"><span>Effort</span><span data-id="effort-label">—</span></div>
          <div class="tg-meter"><div class="tg-meter-fill" data-id="effort-fill"></div></div>
        </div>
        <div class="tg-section">PID contributions</div>
        <div class="tg-terms">
          <div class="tg-card tg-term">P<b data-id="p-term">—</b></div>
          <div class="tg-card tg-term">I<b data-id="i-term">—</b></div>
          <div class="tg-card tg-term">D<b data-id="d-term">—</b></div>
        </div>
      </div>
      <div>
        <div class="tg-card">
          <canvas class="tg-canvas" data-id="response"></canvas>
          <div class="tg-legend"><span><i class="tg-swatch" style="background:#58a6ff"></i>setpoint</span><span><i class="tg-swatch" style="background:#f5b642"></i>measurement</span><span data-id="units"></span></div>
        </div>
        <div class="tg-section">Gains and limits</div>
        <div class="tg-form">
          ${["kp", "ki", "kd", "integral_limit", "output_limit"].map((field) => `<div class="tg-field"><label>${field.replace("_", " ").toUpperCase()}</label><input data-field="${field}" type="number" step="0.1"></div>`).join("")}
        </div>
        <div class="tg-actions">
          <button class="tg-button primary" data-id="apply">Apply gains</button>
          <button class="tg-button" data-id="reload">Reload live values</button>
          <button class="tg-button" data-id="reset">Reset PID state</button>
          <span style="margin:auto 0 auto auto"><i class="tg-dot" data-id="active-dot"></i><span data-id="active-label">controller idle</span></span>
        </div>
        <div class="tg-note" data-id="notice"></div>
      </div>
    </div>`;
  root.appendChild(body);
  context.panelElement.appendChild(root);
  context.setDefaultPanelTitle("Tardigrade PID Tuner");

  const query = <T extends Element>(selector: string): T => {
    const element = root.querySelector<T>(selector);
    if (element == undefined) {
      throw new Error(`Missing PID panel element ${selector}`);
    }
    return element;
  };
  const axisSelect = query<HTMLSelectElement>("[data-id=axis]");
  const canvas = query<HTMLCanvasElement>("[data-id=response]");
  const notice = query<HTMLElement>("[data-id=notice]");
  axisSelect.value = selected;

  const gainInput = (name: string): HTMLInputElement => query<HTMLInputElement>(`[data-field=${name}]`);
  const setNotice = (message: string, error = false): void => {
    notice.textContent = message;
    notice.classList.toggle("error", error);
  };
  const syncForm = (): void => {
    const debug = latest.get(selected);
    if (debug == undefined) {
      return;
    }
    const values: Record<string, number> = {
      kp: debug.kp,
      ki: debug.ki,
      kd: debug.kd,
      integral_limit: debug.integral_limit,
      output_limit: debug.output_limit,
    };
    for (const [name, value] of Object.entries(values)) {
      const input = gainInput(name);
      if (document.activeElement !== input) {
        input.value = String(finite(value));
      }
    }
  };
  const render = (): void => {
    const debug = latest.get(selected);
    drawResponse(canvas, histories.get(selected) ?? []);
    query<HTMLElement>("[data-id=units]").textContent = AXES.indexOf(selected) < 3 ? "m/s" : "rad/s";
    const activeDot = query<HTMLElement>("[data-id=active-dot]");
    activeDot.className = `tg-dot ${controllerEnabled ? "ok" : "muted"}`;
    query<HTMLElement>("[data-id=active-label]").textContent = controllerEnabled ? "controller active" : "controller idle";
    if (debug == undefined) {
      return;
    }
    query<HTMLElement>("[data-id=error]").textContent = format(debug.error);
    query<HTMLElement>("[data-id=output]").textContent = `${format(debug.output, 1)} ${AXES.indexOf(selected) < 3 ? "N" : "N·m"}`;
    query<HTMLElement>("[data-id=p-term]").textContent = format(debug.p_term, 2);
    query<HTMLElement>("[data-id=i-term]").textContent = format(debug.i_term, 2);
    query<HTMLElement>("[data-id=d-term]").textContent = format(debug.d_term, 2);
    const fraction = Math.max(-1, Math.min(1, debug.output / Math.max(1e-6, debug.output_limit)));
    const fill = query<HTMLElement>("[data-id=effort-fill]");
    fill.style.left = `${fraction < 0 ? 50 + fraction * 50 : 50}%`;
    fill.style.width = `${Math.abs(fraction) * 50}%`;
    fill.style.background = debug.saturated ? "var(--tg-red)" : "var(--tg-blue)";
    query<HTMLElement>("[data-id=effort-label]").textContent = `${Math.round(fraction * 100)}%${debug.saturated ? " SATURATED" : ""}`;
    syncForm();
  };

  axisSelect.addEventListener("change", () => {
    if (isAxis(axisSelect.value)) {
      selected = axisSelect.value;
      context.saveState({ axis: selected });
      syncForm();
      render();
    }
  });
  query<HTMLButtonElement>("[data-id=reload]").addEventListener("click", () => {
    syncForm();
    setNotice("Reloaded values reported by the running controller.");
  });
  query<HTMLButtonElement>("[data-id=reset]").addEventListener("click", () => {
    if (context.callService == undefined) {
      setNotice("This connection cannot call ROS services.", true);
      return;
    }
    void context.callService(RESET_SERVICE, {}).then(
      () => setNotice("PID integral, derivative, and depth-hold state reset."),
      (error: unknown) => setNotice(String(error), true),
    );
  });
  query<HTMLButtonElement>("[data-id=apply]").addEventListener("click", () => {
    if (context.callService == undefined) {
      setNotice("This connection cannot call ROS services.", true);
      return;
    }
    const values = {
      kp: Number(gainInput("kp").value),
      ki: Number(gainInput("ki").value),
      kd: Number(gainInput("kd").value),
      integral_limit: Number(gainInput("integral_limit").value),
      output_limit: Number(gainInput("output_limit").value),
    };
    if (Object.values(values).some((value) => !Number.isFinite(value)) || values.output_limit <= 0 || values.integral_limit < 0 || values.kp < 0 || values.ki < 0 || values.kd < 0) {
      setNotice("Enter finite non-negative gains/limits and an output limit greater than zero.", true);
      return;
    }
    setNotice(`Applying ${selected.toUpperCase()} gains…`);
    void context.callService(SERVICE, { axis: selected, ...values }).then(
      (response: unknown) => {
        const result = response as { success?: boolean; message?: string };
        setNotice(result.message ?? (result.success ? "Gains applied." : "Gain update rejected."), result.success === false);
      },
      (error: unknown) => setNotice(String(error), true),
    );
  });

  context.subscribe([
    ...AXES.map((axis) => ({ topic: TOPIC(axis) })),
    { topic: "/tardigrade/control/enabled" },
  ]);
  context.watch("currentFrame");
  context.watch("colorScheme");
  context.onRender = (state: Immutable<RenderState>, done) => {
    applyTheme(root, state.colorScheme);
    for (const event of state.currentFrame ?? []) {
      if (event.topic === "/tardigrade/control/enabled") {
        controllerEnabled = Boolean((event as MessageEvent<BoolMsg>).message.data);
        continue;
      }
      const axis = AXES.find((candidate) => TOPIC(candidate) === event.topic);
      if (axis == undefined) {
        continue;
      }
      const debug = (event as MessageEvent<PidDebug>).message;
      latest.set(axis, debug);
      const history = histories.get(axis) ?? [];
      const time = timeToSec(debug.stamp);
      if (history.length > 0 && time < (history[history.length - 1]?.time ?? time)) {
        history.length = 0;
      }
      if (history.length === 0 || history[history.length - 1]?.time !== time) {
        history.push({ time, setpoint: finite(debug.setpoint), measurement: finite(debug.measurement) });
      }
      const cutoff = time - 8;
      while (history.length > 2 && (history[0]?.time ?? time) < cutoff) {
        history.shift();
      }
      histories.set(axis, history);
    }
    render();
    done();
  };
  const resize = new ResizeObserver(render);
  resize.observe(canvas);
  render();

  return () => {
    resize.disconnect();
    context.unsubscribeAll();
    root.remove();
  };
}
