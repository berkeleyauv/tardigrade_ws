import type { Immutable, MessageEvent, PanelExtensionContext, RenderState } from "@foxglove/extension";

import type { Odometry } from "./messages";
import { finite, quaternionToRpy } from "./messages";
import { applyTheme, createPanelRoot } from "./style";

const ODOM_TOPIC = "/tardigrade/state/odometry/filtered";
const RAD_TO_DEG = 180 / Math.PI;

function format(value: number, suffix: string, digits = 1): string {
  return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "—";
}

function drawAttitude(canvas: HTMLCanvasElement, roll: number, pitch: number, yaw: number): void {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(220, canvas.clientWidth);
  const height = Math.max(210, canvas.clientHeight);
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext("2d");
  if (ctx == undefined) return;
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const centerX = width / 2;
  const centerY = height / 2 + 10;
  const radius = Math.min(width * 0.43, height * 0.41);
  ctx.save();
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
  ctx.clip();
  ctx.translate(centerX, centerY);
  ctx.rotate(-roll);
  const pitchOffset = Math.max(-radius, Math.min(radius, pitch * RAD_TO_DEG / 35 * radius));
  ctx.translate(0, pitchOffset);
  ctx.fillStyle = "#17466a";
  ctx.fillRect(-radius * 2, -radius * 2, radius * 4, radius * 2);
  ctx.fillStyle = "#513c31";
  ctx.fillRect(-radius * 2, 0, radius * 4, radius * 2);
  ctx.strokeStyle = "#f4f7fa";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-radius * 2, 0);
  ctx.lineTo(radius * 2, 0);
  ctx.stroke();
  ctx.font = "10px system-ui";
  ctx.fillStyle = "#dbe8f5";
  ctx.textAlign = "center";
  for (const degrees of [-30, -20, -10, 10, 20, 30]) {
    const y = -degrees / 35 * radius;
    const length = degrees % 20 === 0 ? radius * 0.42 : radius * 0.26;
    ctx.beginPath();
    ctx.moveTo(-length, y);
    ctx.lineTo(length, y);
    ctx.stroke();
    ctx.fillText(String(Math.abs(degrees)), -length - 14, y + 3);
    ctx.fillText(String(Math.abs(degrees)), length + 14, y + 3);
  }
  ctx.restore();

  ctx.strokeStyle = "#dbe8f5";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
  ctx.stroke();
  ctx.strokeStyle = "#f5b642";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(centerX - radius * 0.42, centerY);
  ctx.lineTo(centerX - radius * 0.12, centerY);
  ctx.lineTo(centerX, centerY + 6);
  ctx.lineTo(centerX + radius * 0.12, centerY);
  ctx.lineTo(centerX + radius * 0.42, centerY);
  ctx.stroke();

  const heading = ((yaw * RAD_TO_DEG % 360) + 360) % 360;
  ctx.fillStyle = "rgba(3,8,14,.86)";
  ctx.fillRect(0, 0, width, 34);
  ctx.strokeStyle = "#91a2b5";
  ctx.fillStyle = "#dbe8f5";
  ctx.font = "10px system-ui";
  ctx.textAlign = "center";
  for (let delta = -60; delta <= 60; delta += 15) {
    const x = centerX + delta / 60 * width * 0.45;
    const angle = ((Math.round(heading / 15) * 15 + delta) % 360 + 360) % 360;
    ctx.beginPath();
    ctx.moveTo(x, 22);
    ctx.lineTo(x, 30);
    ctx.stroke();
    const cardinal: Record<number, string> = { 0: "E", 90: "N", 180: "W", 270: "S" };
    ctx.fillText(cardinal[angle] ?? String(angle), x, 15);
  }
  ctx.fillStyle = "#f5b642";
  ctx.beginPath();
  ctx.moveTo(centerX - 6, 34);
  ctx.lineTo(centerX + 6, 34);
  ctx.lineTo(centerX, 27);
  ctx.fill();
}

export function initAttitudePanel(context: PanelExtensionContext): () => void {
  let odometry: Odometry | undefined;
  let rpy: [number, number, number] = [0, 0, 0];
  const root = createPanelRoot("tg-attitude");
  const body = document.createElement("div");
  body.className = "tg-pad";
  body.innerHTML = `
    <div class="tg-title">Attitude and motion</div>
    <div class="tg-subtitle">Filtered estimate · ROS ENU/FLU · yaw 0° is +X/East</div>
    <div class="tg-instrument">
      <canvas class="tg-attitude-canvas" data-id="canvas"></canvas>
      <div class="tg-readouts">
        ${["roll", "pitch", "yaw", "depth", "surge", "sway", "heave", "yaw-rate"].map((name) => `<div class="tg-card tg-kpi"><div class="tg-kpi-label">${name}</div><div class="tg-kpi-value" data-value="${name}">—</div></div>`).join("")}
      </div>
    </div>`;
  root.appendChild(body);
  context.panelElement.appendChild(root);
  context.setDefaultPanelTitle("Tardigrade Attitude");
  const canvas = root.querySelector<HTMLCanvasElement>("[data-id=canvas]");
  if (canvas == undefined) throw new Error("Attitude canvas missing");
  const value = (name: string): HTMLElement => {
    const element = root.querySelector<HTMLElement>(`[data-value=${name}]`);
    if (element == undefined) throw new Error(`Attitude readout ${name} missing`);
    return element;
  };
  const render = (): void => {
    drawAttitude(canvas, ...rpy);
    if (odometry == undefined) return;
    value("roll").textContent = format(rpy[0] * RAD_TO_DEG, "°");
    value("pitch").textContent = format(rpy[1] * RAD_TO_DEG, "°");
    value("yaw").textContent = format(rpy[2] * RAD_TO_DEG, "°");
    value("depth").textContent = format(-finite(odometry.pose.pose.position.z), " m", 2);
    value("surge").textContent = format(finite(odometry.twist.twist.linear.x), " m/s", 2);
    value("sway").textContent = format(finite(odometry.twist.twist.linear.y), " m/s", 2);
    value("heave").textContent = format(finite(odometry.twist.twist.linear.z), " m/s", 2);
    value("yaw-rate").textContent = format(finite(odometry.twist.twist.angular.z) * RAD_TO_DEG, "°/s", 1);
  };
  context.subscribe([{ topic: ODOM_TOPIC }]);
  context.watch("currentFrame");
  context.watch("colorScheme");
  context.onRender = (state: Immutable<RenderState>, done) => {
    applyTheme(root, state.colorScheme);
    for (const rawEvent of state.currentFrame ?? []) {
      if (rawEvent.topic !== ODOM_TOPIC) continue;
      const message = (rawEvent as MessageEvent<Odometry>).message;
      odometry = message;
      rpy = quaternionToRpy(message.pose.pose.orientation);
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
