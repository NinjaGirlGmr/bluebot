import { ros2humble as ros2 } from "@foxglove/rosmsg-msgs-common";
import { ExtensionContext } from "@foxglove/studio";

type UnknownRecord = Record<string, unknown>;

type PanelState = {
  frameId: string;
  goalX: number;
  goalY: number;
  goalYawDeg: number;
  initialX: number;
  initialY: number;
  initialYawDeg: number;
  waypointText: string;
};

const DEFAULT_STATE: PanelState = {
  frameId: "map",
  goalX: 0.0,
  goalY: 0.0,
  goalYawDeg: 0.0,
  initialX: 0.0,
  initialY: 0.0,
  initialYawDeg: 0.0,
  waypointText: "1.0, 0.0, 0\n2.0, 0.0, 0",
};

const GOAL_TOPIC = "/goal_pose";
const INITIALPOSE_TOPIC = "/initialpose";
const WAYPOINT_TOPIC = "/foxglove/waypoints";
const WAYPOINT_STATUS_TOPIC = "/foxglove/waypoints/status";
const NAV_TO_STATUS_TOPIC = "/navigate_to_pose/_action/status";
const NAV_THROUGH_STATUS_TOPIC = "/navigate_through_poses/_action/status";
const NAV_TO_CANCEL_SERVICE = "/navigate_to_pose/_action/cancel_goal";
const NAV_THROUGH_CANCEL_SERVICE = "/navigate_through_poses/_action/cancel_goal";

const STATUS_LABEL: Record<number, string> = {
  0: "UNKNOWN",
  1: "ACCEPTED",
  2: "EXECUTING",
  3: "CANCELING",
  4: "SUCCEEDED",
  5: "CANCELED",
  6: "ABORTED",
};

function isRecord(value: unknown): value is UnknownRecord {
  return typeof value === "object" && value != undefined;
}

function nowStamp(): { sec: number; nanosec: number } {
  const ms = Date.now();
  return {
    sec: Math.floor(ms / 1000),
    nanosec: Math.floor((ms % 1000) * 1e6),
  };
}

function degToQuaternion(yawDeg: number): { x: number; y: number; z: number; w: number } {
  const yawRad = (yawDeg * Math.PI) / 180.0;
  return {
    x: 0.0,
    y: 0.0,
    z: Math.sin(yawRad / 2.0),
    w: Math.cos(yawRad / 2.0),
  };
}

function summarizeGoalStatusArray(message: unknown): string {
  if (!isRecord(message)) {
    return "no status";
  }
  const statusListRaw = message["status_list"];
  if (!Array.isArray(statusListRaw) || statusListRaw.length === 0) {
    return "idle";
  }

  const last = statusListRaw[statusListRaw.length - 1];
  if (!isRecord(last)) {
    return `entries=${statusListRaw.length}`;
  }

  const codeRaw = last["status"];
  const code = typeof codeRaw === "number" ? codeRaw : Number.NaN;
  const label = Number.isFinite(code) ? STATUS_LABEL[code] ?? String(code) : "unknown";
  return `${label} (entries=${statusListRaw.length})`;
}

function extractStringData(message: unknown): string {
  if (!isRecord(message)) {
    return "";
  }
  const value = message["data"];
  return typeof value === "string" ? value : "";
}

function buildDatatypeMap(typeNames: string[]): Map<string, unknown> {
  const datatypes = new Map<string, unknown>();
  for (const typeName of typeNames) {
    const definition = (ros2 as UnknownRecord)[typeName];
    if (definition != undefined) {
      datatypes.set(typeName, definition);
    }
  }
  return datatypes;
}

function advertiseRos2Topic(
  context: any,
  topic: string,
  schemaName: string,
  datatypeNames: string[],
): void {
  if (!context.advertise) {
    return;
  }

  const datatypes = buildDatatypeMap(datatypeNames);
  if (datatypes.size > 0) {
    context.advertise(topic, schemaName, { datatypes });
  } else {
    context.advertise(topic, schemaName);
  }
}

function parseWaypointText(
  text: string,
): { poses: Array<{ position: { x: number; y: number; z: number }; orientation: UnknownRecord }>; error?: string } {
  const poses: Array<{ position: { x: number; y: number; z: number }; orientation: UnknownRecord }> = [];
  const lines = text
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter((line) => line.length > 0 && !line.startsWith("#"));

  for (let idx = 0; idx < lines.length; idx += 1) {
    const line = lines[idx] as string;
    const normalized = line.replace(/,/g, " ");
    const fields = normalized
      .split(/\s+/)
      .map((field) => field.trim())
      .filter((field) => field.length > 0);

    if (fields.length < 2 || fields.length > 3) {
      return {
        poses: [],
        error: `Invalid waypoint line ${idx + 1}. Use: x y [yaw_deg]`,
      };
    }

    const x = Number(fields[0]);
    const y = Number(fields[1]);
    const yawDeg = fields[2] != undefined ? Number(fields[2]) : 0.0;
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(yawDeg)) {
      return {
        poses: [],
        error: `Non-numeric waypoint line ${idx + 1}: "${line}"`,
      };
    }

    poses.push({
      position: { x, y, z: 0.0 },
      orientation: degToQuaternion(yawDeg),
    });
  }

  return { poses };
}

function createNumberInput(initialValue: number): HTMLInputElement {
  const input = document.createElement("input");
  input.type = "number";
  input.step = "0.1";
  input.value = String(initialValue);
  input.style.width = "6.5rem";
  input.style.padding = "4px 6px";
  return input;
}

function createButton(label: string): HTMLButtonElement {
  const button = document.createElement("button");
  button.textContent = label;
  button.style.padding = "6px 10px";
  button.style.cursor = "pointer";
  return button;
}

function createSection(title: string): HTMLDivElement {
  const section = document.createElement("div");
  section.style.border = "1px solid #2d2d2d";
  section.style.borderRadius = "8px";
  section.style.padding = "10px";
  section.style.display = "flex";
  section.style.flexDirection = "column";
  section.style.gap = "8px";

  const titleEl = document.createElement("div");
  titleEl.textContent = title;
  titleEl.style.fontWeight = "600";
  titleEl.style.fontSize = "13px";
  section.appendChild(titleEl);
  return section;
}

function createRow(): HTMLDivElement {
  const row = document.createElement("div");
  row.style.display = "flex";
  row.style.flexWrap = "wrap";
  row.style.alignItems = "center";
  row.style.gap = "8px";
  return row;
}

function createLabel(text: string): HTMLLabelElement {
  const label = document.createElement("label");
  label.textContent = text;
  label.style.fontSize = "12px";
  label.style.opacity = "0.9";
  return label;
}

function initializePanel(context: any): (() => void) | void {
  const root = context.panelElement as HTMLDivElement;
  root.innerHTML = "";
  root.style.display = "flex";
  root.style.flexDirection = "column";
  root.style.gap = "10px";
  root.style.padding = "10px";
  root.style.fontFamily = "ui-sans-serif, system-ui, -apple-system, Segoe UI, sans-serif";
  root.style.fontSize = "12px";
  root.style.color = "#e8e8e8";
  root.style.background = "#121212";

  const panelState: PanelState = {
    ...DEFAULT_STATE,
    ...(isRecord(context.initialState) ? (context.initialState as Partial<PanelState>) : {}),
  };

  const infoEl = document.createElement("div");
  infoEl.style.padding = "8px";
  infoEl.style.border = "1px solid #2d2d2d";
  infoEl.style.borderRadius = "8px";
  infoEl.style.background = "#1a1a1a";
  infoEl.textContent = "BlueBot Nav2 Controls";
  root.appendChild(infoEl);

  const frameSection = createSection("Frame");
  const frameRow = createRow();
  const frameLabel = createLabel("frame_id");
  const frameInput = document.createElement("input");
  frameInput.type = "text";
  frameInput.value = panelState.frameId;
  frameInput.style.width = "10rem";
  frameInput.style.padding = "4px 6px";
  frameRow.appendChild(frameLabel);
  frameRow.appendChild(frameInput);
  frameSection.appendChild(frameRow);
  root.appendChild(frameSection);

  const goalSection = createSection("Single Goal");
  const goalRow = createRow();
  const goalXInput = createNumberInput(panelState.goalX);
  const goalYInput = createNumberInput(panelState.goalY);
  const goalYawInput = createNumberInput(panelState.goalYawDeg);
  const sendGoalButton = createButton(`Publish ${GOAL_TOPIC}`);
  goalRow.appendChild(createLabel("x"));
  goalRow.appendChild(goalXInput);
  goalRow.appendChild(createLabel("y"));
  goalRow.appendChild(goalYInput);
  goalRow.appendChild(createLabel("yaw_deg"));
  goalRow.appendChild(goalYawInput);
  goalRow.appendChild(sendGoalButton);
  goalSection.appendChild(goalRow);
  root.appendChild(goalSection);

  const initialSection = createSection("Initial Pose");
  const initialRow = createRow();
  const initialXInput = createNumberInput(panelState.initialX);
  const initialYInput = createNumberInput(panelState.initialY);
  const initialYawInput = createNumberInput(panelState.initialYawDeg);
  const sendInitialButton = createButton(`Publish ${INITIALPOSE_TOPIC}`);
  initialRow.appendChild(createLabel("x"));
  initialRow.appendChild(initialXInput);
  initialRow.appendChild(createLabel("y"));
  initialRow.appendChild(initialYInput);
  initialRow.appendChild(createLabel("yaw_deg"));
  initialRow.appendChild(initialYawInput);
  initialRow.appendChild(sendInitialButton);
  initialSection.appendChild(initialRow);
  root.appendChild(initialSection);

  const routeSection = createSection("Waypoint Route");
  const routeText = document.createElement("textarea");
  routeText.value = panelState.waypointText;
  routeText.rows = 4;
  routeText.style.width = "100%";
  routeText.style.minHeight = "90px";
  routeText.style.padding = "6px";
  routeText.style.resize = "vertical";
  routeText.placeholder = "One waypoint per line: x y [yaw_deg]";
  const routeRow = createRow();
  const sendRouteButton = createButton(`Publish ${WAYPOINT_TOPIC}`);
  routeRow.appendChild(sendRouteButton);
  routeSection.appendChild(routeText);
  routeSection.appendChild(routeRow);
  root.appendChild(routeSection);

  const actionSection = createSection("Actions");
  const actionRow = createRow();
  const cancelNavButton = createButton("Cancel NavigateToPose");
  const cancelRouteButton = createButton("Cancel NavigateThroughPoses");
  actionRow.appendChild(cancelNavButton);
  actionRow.appendChild(cancelRouteButton);
  actionSection.appendChild(actionRow);
  root.appendChild(actionSection);

  const statusSection = createSection("Status");
  const navToStatusEl = document.createElement("div");
  const navThroughStatusEl = document.createElement("div");
  const waypointStatusEl = document.createElement("div");
  const resultEl = document.createElement("div");
  resultEl.style.paddingTop = "6px";
  resultEl.style.opacity = "0.95";
  navToStatusEl.textContent = `${NAV_TO_STATUS_TOPIC}: waiting`;
  navThroughStatusEl.textContent = `${NAV_THROUGH_STATUS_TOPIC}: waiting`;
  waypointStatusEl.textContent = `${WAYPOINT_STATUS_TOPIC}: waiting`;
  resultEl.textContent = "Ready";
  statusSection.appendChild(navToStatusEl);
  statusSection.appendChild(navThroughStatusEl);
  statusSection.appendChild(waypointStatusEl);
  statusSection.appendChild(resultEl);
  root.appendChild(statusSection);

  let latestNavToStatus = "waiting";
  let latestNavThroughStatus = "waiting";
  let latestWaypointStatus = "waiting";

  const writeResult = (text: string): void => {
    resultEl.textContent = text;
  };

  const persistState = (): void => {
    context.saveState?.(panelState);
  };

  frameInput.addEventListener("input", () => {
    panelState.frameId = frameInput.value || "map";
    persistState();
  });
  goalXInput.addEventListener("input", () => {
    panelState.goalX = Number(goalXInput.value);
    persistState();
  });
  goalYInput.addEventListener("input", () => {
    panelState.goalY = Number(goalYInput.value);
    persistState();
  });
  goalYawInput.addEventListener("input", () => {
    panelState.goalYawDeg = Number(goalYawInput.value);
    persistState();
  });
  initialXInput.addEventListener("input", () => {
    panelState.initialX = Number(initialXInput.value);
    persistState();
  });
  initialYInput.addEventListener("input", () => {
    panelState.initialY = Number(initialYInput.value);
    persistState();
  });
  initialYawInput.addEventListener("input", () => {
    panelState.initialYawDeg = Number(initialYawInput.value);
    persistState();
  });
  routeText.addEventListener("input", () => {
    panelState.waypointText = routeText.value;
    persistState();
  });

  advertiseRos2Topic(context, GOAL_TOPIC, "geometry_msgs/PoseStamped", [
    "builtin_interfaces/Time",
    "std_msgs/Header",
    "geometry_msgs/Point",
    "geometry_msgs/Quaternion",
    "geometry_msgs/Pose",
    "geometry_msgs/PoseStamped",
  ]);
  advertiseRos2Topic(context, INITIALPOSE_TOPIC, "geometry_msgs/PoseWithCovarianceStamped", [
    "builtin_interfaces/Time",
    "std_msgs/Header",
    "geometry_msgs/Point",
    "geometry_msgs/Quaternion",
    "geometry_msgs/Pose",
    "geometry_msgs/PoseWithCovariance",
    "geometry_msgs/PoseWithCovarianceStamped",
  ]);
  advertiseRos2Topic(context, WAYPOINT_TOPIC, "geometry_msgs/PoseArray", [
    "builtin_interfaces/Time",
    "std_msgs/Header",
    "geometry_msgs/Point",
    "geometry_msgs/Quaternion",
    "geometry_msgs/Pose",
    "geometry_msgs/PoseArray",
  ]);

  sendGoalButton.addEventListener("click", () => {
    if (!context.publish) {
      writeResult("Publish unsupported by current data source.");
      return;
    }
    const stamp = nowStamp();
    const message = {
      header: {
        stamp,
        frame_id: panelState.frameId || "map",
      },
      pose: {
        position: {
          x: panelState.goalX,
          y: panelState.goalY,
          z: 0.0,
        },
        orientation: degToQuaternion(panelState.goalYawDeg),
      },
    };
    try {
      context.publish(GOAL_TOPIC, message);
      writeResult(
        `Published goal: x=${panelState.goalX.toFixed(2)} y=${panelState.goalY.toFixed(2)} yaw=${panelState.goalYawDeg.toFixed(1)}deg`,
      );
    } catch (error) {
      writeResult(`Goal publish failed: ${String(error)}`);
    }
  });

  sendInitialButton.addEventListener("click", () => {
    if (!context.publish) {
      writeResult("Publish unsupported by current data source.");
      return;
    }
    const covariance = new Array<number>(36).fill(0.0);
    covariance[0] = 0.25;
    covariance[7] = 0.25;
    covariance[35] = 0.0685;
    const stamp = nowStamp();
    const message = {
      header: {
        stamp,
        frame_id: panelState.frameId || "map",
      },
      pose: {
        pose: {
          position: {
            x: panelState.initialX,
            y: panelState.initialY,
            z: 0.0,
          },
          orientation: degToQuaternion(panelState.initialYawDeg),
        },
        covariance,
      },
    };
    try {
      context.publish(INITIALPOSE_TOPIC, message);
      writeResult(
        `Published initial pose: x=${panelState.initialX.toFixed(2)} y=${panelState.initialY.toFixed(2)} yaw=${panelState.initialYawDeg.toFixed(1)}deg`,
      );
    } catch (error) {
      writeResult(`Initial pose publish failed: ${String(error)}`);
    }
  });

  sendRouteButton.addEventListener("click", () => {
    if (!context.publish) {
      writeResult("Publish unsupported by current data source.");
      return;
    }
    const parsed = parseWaypointText(panelState.waypointText);
    if (parsed.error) {
      writeResult(parsed.error);
      return;
    }
    if (parsed.poses.length === 0) {
      writeResult("No waypoint rows to publish.");
      return;
    }
    const message = {
      header: {
        stamp: nowStamp(),
        frame_id: panelState.frameId || "map",
      },
      poses: parsed.poses,
    };
    try {
      context.publish(WAYPOINT_TOPIC, message);
      writeResult(`Published ${parsed.poses.length} waypoint(s) to ${WAYPOINT_TOPIC}.`);
    } catch (error) {
      writeResult(`Waypoint publish failed: ${String(error)}`);
    }
  });

  const cancelRequest = {
    goal_info: {
      goal_id: { uuid: new Array<number>(16).fill(0) },
      stamp: { sec: 0, nanosec: 0 },
    },
  };

  const callCancelService = async (serviceName: string): Promise<void> => {
    if (!context.callService) {
      writeResult("Service calls unsupported by current data source.");
      return;
    }
    try {
      const result = await context.callService(serviceName, cancelRequest);
      writeResult(`Called ${serviceName}: ${JSON.stringify(result)}`);
    } catch (error) {
      writeResult(`Service call failed (${serviceName}): ${String(error)}`);
    }
  };

  cancelNavButton.addEventListener("click", () => {
    void callCancelService(NAV_TO_CANCEL_SERVICE);
  });

  cancelRouteButton.addEventListener("click", () => {
    void callCancelService(NAV_THROUGH_CANCEL_SERVICE);
  });

  context.subscribe?.([
    { topic: NAV_TO_STATUS_TOPIC },
    { topic: NAV_THROUGH_STATUS_TOPIC },
    { topic: WAYPOINT_STATUS_TOPIC },
  ]);
  context.watch?.("currentFrame");

  context.onRender = (renderState: any, done: () => void): void => {
    const frame = renderState.currentFrame;
    if (Array.isArray(frame)) {
      for (const event of frame) {
        if (!isRecord(event)) {
          continue;
        }
        const topic = event["topic"];
        const message = event["message"];
        if (topic === NAV_TO_STATUS_TOPIC) {
          latestNavToStatus = summarizeGoalStatusArray(message);
        } else if (topic === NAV_THROUGH_STATUS_TOPIC) {
          latestNavThroughStatus = summarizeGoalStatusArray(message);
        } else if (topic === WAYPOINT_STATUS_TOPIC) {
          const status = extractStringData(message);
          if (status.length > 0) {
            latestWaypointStatus = status;
          }
        }
      }
    }

    navToStatusEl.textContent = `${NAV_TO_STATUS_TOPIC}: ${latestNavToStatus}`;
    navThroughStatusEl.textContent = `${NAV_THROUGH_STATUS_TOPIC}: ${latestNavThroughStatus}`;
    waypointStatusEl.textContent = `${WAYPOINT_STATUS_TOPIC}: ${latestWaypointStatus}`;
    done();
  };

  return () => {
    context.onRender = undefined;
    context.unsubscribeAll?.();
    context.unadvertise?.(GOAL_TOPIC);
    context.unadvertise?.(INITIALPOSE_TOPIC);
    context.unadvertise?.(WAYPOINT_TOPIC);
    root.innerHTML = "";
  };
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "BlueBot Nav2 Controls",
    initPanel: initializePanel,
  });
}
