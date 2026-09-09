import type { Immutable, MessageEvent, PanelExtensionContext, RenderState } from "@foxglove/extension";

import type { AllocationStatus, BoolMsg, EspState, RobotStatus, ThrusterCommands } from "./messages";
import { finite, timeToSec } from "./messages";
import { applyTheme, createPanelRoot } from "./style";

const TOPICS = {
  status: "/tardigrade/status",
  esp: "/tardigrade/esp/state",
  controller: "/tardigrade/control/enabled",
  command: "/tardigrade/control/command_fresh",
  odomFresh: "/tardigrade/control/odometry_fresh",
  allocation: "/tardigrade/control/allocation_status",
  thrusters: "/tardigrade/actuators/thruster_commands",
  filtered: "/tardigrade/state/odometry/filtered",
  simImu: "/tardigrade/sensors/imu/data",
  realImu: "/tardigrade/sensors/imu",
  pressure: "/tardigrade/sensors/pressure",
  simVio: "/tardigrade/sensors/visual_odometry",
  realVio: "/zed/zed_node/odom",
  simCamera: "/tardigrade/sensors/camera/front/left/image_raw",
  realCamera: "/zed/zed_node/left/image_rect_color",
} as const;

type StatusValue = "ok" | "warn" | "bad" | "muted";

const SHORT_NAMES: Record<string, string> = {
  front_left_horizontal: "FL-H",
  front_right_horizontal: "FR-H",
  rear_left_horizontal: "RL-H",
  rear_right_horizontal: "RR-H",
  front_left_vertical: "FL-V",
  front_right_vertical: "FR-V",
  rear_left_vertical: "RL-V",
  rear_right_vertical: "RR-V",
};

function boolMessage(event: MessageEvent): boolean {
  return Boolean((event.message as BoolMsg).data);
}

export function initVehicleStatusPanel(context: PanelExtensionContext): () => void {
  let now = 0;
  let previousNow = 0;
  const lastSeen = new Map<string, number>();
  let status: RobotStatus | undefined;
  let esp: EspState | undefined;
  let controller = false;
  let commandFresh = false;
  let odomFresh = false;
  let allocation: AllocationStatus | undefined;
  let thrusters: ThrusterCommands = { names: [], setpoints: [] };
  let armConfirmUntil = 0;

  const root = createPanelRoot("tg-vehicle-status");
  const body = document.createElement("div");
  body.className = "tg-pad";
  body.innerHTML = `
    <div style="display:flex;align-items:baseline;justify-content:space-between;gap:8px">
      <div class="tg-title">Vehicle status</div><div class="tg-subtitle" data-id="mode">WAITING</div>
    </div>
    <div class="tg-grid tg-grid-4" data-id="status-grid">
      ${["link", "armed", "external", "controller", "command", "odom", "allocation"].map((name) => `<div class="tg-card tg-status muted" data-status="${name}"><div class="tg-kpi-label">${name}</div><div class="tg-kpi-value"><i class="tg-dot"></i><span>waiting</span></div></div>`).join("")}
    </div>
    <div class="tg-actions">
      <button class="tg-button danger" data-id="disarm">Disarm now</button>
      <button class="tg-button" data-id="arm">Arm…</button>
      <button class="tg-button safe" data-id="external-on">External on</button>
      <button class="tg-button" data-id="external-off">External off</button>
    </div>
    <div class="tg-note" data-id="notice"></div>
    <div class="tg-section">Sensor heartbeat</div>
    <div class="tg-sensors" data-id="sensors"></div>
    <div class="tg-section">Normalized thruster commands</div>
    <div class="tg-card"><div class="tg-thrusters" data-id="thrusters"></div></div>
    <div class="tg-note">Thruster bars are telemetry only. Direct motor checkout remains restricted to the bounded ROS service.</div>`;
  root.appendChild(body);
  context.panelElement.appendChild(root);
  context.setDefaultPanelTitle("Tardigrade Vehicle Status");

  const query = <T extends Element>(selector: string): T => {
    const element = root.querySelector<T>(selector);
    if (element == undefined) {
      throw new Error(`Missing vehicle status element ${selector}`);
    }
    return element;
  };
  const notice = query<HTMLElement>("[data-id=notice]");
  const setNotice = (message: string, error = false): void => {
    notice.textContent = message;
    notice.classList.toggle("error", error);
  };
  const setStatus = (name: string, value: StatusValue, label: string): void => {
    const card = query<HTMLElement>(`[data-status=${name}]`);
    card.classList.remove("ok", "warn", "bad", "muted");
    card.classList.add(value);
    const text = card.querySelector("span");
    if (text != undefined) {
      text.textContent = label;
    }
  };
  const ageOf = (...topics: string[]): number | undefined => {
    const times = topics.map((topic) => lastSeen.get(topic)).filter((value): value is number => value != undefined);
    if (times.length === 0) {
      return undefined;
    }
    return Math.max(0, now - Math.max(...times));
  };
  const fresh = (topic: string, timeout: number): boolean => {
    const age = ageOf(topic);
    return age != undefined && age <= timeout;
  };
  const renderSensor = (label: string, timeout: number, ...topics: string[]): string => {
    const age = ageOf(...topics);
    const state = age != undefined && age <= timeout ? "ok" : age == undefined ? "muted" : "bad";
    const ageText = age == undefined ? "never seen" : age < 0.1 ? "live" : `${age.toFixed(1)} s ago`;
    return `<div class="tg-sensor ${state}"><i class="tg-dot"></i>${label}<span class="tg-sensor-age">${ageText}</span></div>`;
  };
  const renderThrusters = (): void => {
    const holder = query<HTMLElement>("[data-id=thrusters]");
    holder.replaceChildren();
    const names = thrusters.names.length === 8 ? thrusters.names : Array.from({ length: 8 }, (_, index) => `thruster_${index + 1}`);
    names.forEach((name, index) => {
      const value = Math.max(-1, Math.min(1, finite(thrusters.setpoints[index])));
      const row = document.createElement("div");
      row.className = "tg-thruster-row";
      const short = SHORT_NAMES[name] ?? `T${index + 1}`;
      const left = value < 0 ? 50 + value * 50 : 50;
      const label = document.createElement("span");
      label.title = name;
      label.textContent = short;
      const track = document.createElement("span");
      track.className = "tg-thruster-track";
      const fill = document.createElement("i");
      fill.className = "tg-thruster-fill";
      fill.style.left = `${left}%`;
      fill.style.width = `${Math.abs(value) * 50}%`;
      fill.style.background = Math.abs(value) > 0.9 ? "var(--tg-red)" : "var(--tg-blue)";
      track.appendChild(fill);
      const number = document.createElement("span");
      number.className = "tg-number";
      number.textContent = value.toFixed(2);
      row.append(label, track, number);
      holder.appendChild(row);
    });
  };
  const render = (): void => {
    const simFresh = fresh(TOPICS.status, 0.5);
    const espFresh = fresh(TOPICS.esp, 0.5);
    query<HTMLElement>("[data-id=mode]").textContent = simFresh ? "SIMULATION" : espFresh ? "POOL / HARDWARE" : "NO BACKEND";
    if (simFresh && status != undefined) {
      setStatus("link", status.control_connected ? "ok" : "bad", status.control_connected ? "Unity online" : "Unity offline");
      setStatus("armed", status.armed ? "bad" : "ok", status.armed ? "ARMED" : "disarmed");
      setStatus("external", status.external_control_enabled ? "ok" : "warn", status.external_control_enabled ? "enabled" : "disabled");
    } else if (espFresh && esp != undefined) {
      setStatus("link", esp.link_ok ? "ok" : "bad", esp.link_ok ? "ESP link OK" : "ESP link bad");
      setStatus("armed", esp.armed ? "bad" : "ok", esp.armed ? "ARMED" : "disarmed");
      setStatus("external", "muted", "hardware N/A");
    } else {
      setStatus("link", "bad", "no backend");
      setStatus("armed", "muted", "unknown");
      setStatus("external", "muted", "unknown");
    }
    setStatus("controller", controller ? "ok" : "muted", controller ? "active" : "idle");
    setStatus("command", commandFresh ? "ok" : "muted", commandFresh ? "fresh" : "idle / stale");
    setStatus("odom", odomFresh ? "ok" : "bad", odomFresh ? "fresh" : "stale");
    setStatus("allocation", allocation?.feasible === true ? "ok" : allocation == undefined ? "muted" : "warn", allocation == undefined ? "waiting" : allocation.feasible ? "feasible" : "limited");
    query<HTMLElement>("[data-id=sensors]").innerHTML = [
      renderSensor("IMU", 0.25, TOPICS.simImu, TOPICS.realImu),
      renderSensor("VIO", 0.5, TOPICS.simVio, TOPICS.realVio),
      renderSensor("Pressure", 0.5, TOPICS.pressure, TOPICS.esp),
      renderSensor("Filtered odom", 0.5, TOPICS.filtered),
      renderSensor("Front camera", 1.0, TOPICS.simCamera, TOPICS.realCamera),
      renderSensor("Actuator cmd", 0.5, TOPICS.thrusters),
    ].join("");
    const externalVisible = simFresh;
    query<HTMLButtonElement>("[data-id=external-on]").style.display = externalVisible ? "" : "none";
    query<HTMLButtonElement>("[data-id=external-off]").style.display = externalVisible ? "" : "none";
    renderThrusters();
  };
  const call = (service: string, request: unknown, success: string): void => {
    if (context.callService == undefined) {
      setNotice("This connection cannot call ROS services.", true);
      return;
    }
    setNotice(`${success}…`);
    void context.callService(service, request).then(
      (response: unknown) => {
        const result = response as { success?: boolean; message?: string };
        setNotice(result.message ?? success, result.success === false);
      },
      (error: unknown) => setNotice(String(error), true),
    );
  };
  query<HTMLButtonElement>("[data-id=disarm]").addEventListener("click", () => {
    armConfirmUntil = 0;
    query<HTMLButtonElement>("[data-id=arm]").textContent = "Arm…";
    call("/tardigrade/set_armed", { armed: false }, "Disarming");
  });
  query<HTMLButtonElement>("[data-id=arm]").addEventListener("click", () => {
    const button = query<HTMLButtonElement>("[data-id=arm]");
    const wallNow = Date.now();
    if (wallNow > armConfirmUntil) {
      armConfirmUntil = wallNow + 5000;
      button.textContent = "Confirm arm";
      setNotice("Press Confirm arm within five seconds. Secure the vehicle first.");
      window.setTimeout(() => {
        if (Date.now() >= armConfirmUntil) {
          button.textContent = "Arm…";
        }
      }, 5100);
      return;
    }
    armConfirmUntil = 0;
    button.textContent = "Arm…";
    call("/tardigrade/set_armed", { armed: true }, "Arming");
  });
  query<HTMLButtonElement>("[data-id=external-on]").addEventListener("click", () => call("/tardigrade/set_external_control", { enabled: true }, "Enabling external control"));
  query<HTMLButtonElement>("[data-id=external-off]").addEventListener("click", () => call("/tardigrade/set_external_control", { enabled: false }, "Disabling external control"));

  context.subscribe(Object.values(TOPICS).map((topic) => ({ topic })));
  context.watch("currentFrame");
  context.watch("currentTime");
  context.watch("colorScheme");
  context.onRender = (state: Immutable<RenderState>, done) => {
    applyTheme(root, state.colorScheme);
    now = state.currentTime == undefined ? performance.now() / 1000 : timeToSec(state.currentTime);
    if (now < previousNow) {
      lastSeen.clear();
    }
    previousNow = now;
    for (const rawEvent of state.currentFrame ?? []) {
      const event = rawEvent as MessageEvent;
      lastSeen.set(event.topic, now);
      if (event.topic === TOPICS.status) status = event.message as RobotStatus;
      else if (event.topic === TOPICS.esp) esp = event.message as EspState;
      else if (event.topic === TOPICS.controller) controller = boolMessage(event);
      else if (event.topic === TOPICS.command) commandFresh = boolMessage(event);
      else if (event.topic === TOPICS.odomFresh) odomFresh = boolMessage(event);
      else if (event.topic === TOPICS.allocation) allocation = event.message as AllocationStatus;
      else if (event.topic === TOPICS.thrusters) thrusters = event.message as ThrusterCommands;
    }
    render();
    done();
  };
  render();
  return () => {
    context.unsubscribeAll();
    root.remove();
  };
}
