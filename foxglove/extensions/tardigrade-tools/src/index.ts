import type { ExtensionContext } from "@foxglove/extension";

import { initAttitudePanel } from "./AttitudePanel";
import { initPidTunerPanel } from "./PidTunerPanel";
import { initVehicleStatusPanel } from "./VehicleStatusPanel";

export function activate(context: ExtensionContext): void {
  context.registerPanel({ name: "PID Tuner", initPanel: initPidTunerPanel });
  context.registerPanel({ name: "Vehicle Status", initPanel: initVehicleStatusPanel });
  context.registerPanel({ name: "Attitude", initPanel: initAttitudePanel });
}
