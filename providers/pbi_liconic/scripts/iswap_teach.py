"""iSWAP teach pendant — web UI for calibrating off-deck handoff positions.

Serves a single-page control panel on http://localhost:8765/ that lets
you jog the iSWAP with +/- buttons, slide+submit to absolute target mm,
control orientation and the gripper, and save calibrated positions to
``iswap_calibrations.json``.

Run with:
    pixi run iswap-teach
    pixi run iswap-teach --port 9000                  # different port
    pixi run iswap-teach --host 127.0.0.1             # lock to loopback

Binds to 0.0.0.0 by default so the control panel is reachable on any
interface — including a Tailscale address. The startup banner prints
the Tailscale IPv4 if ``tailscale`` is on the PATH.

Nothing moves on startup. Click **Initialize iSWAP** when you're sure
the arm has clear swing-out space toward your target.

Safety:
- Jogs are relative, one step size per click; nothing fires on slider
  drag — only on Go.
- Close-gripper refuses plate_width < 76.5 mm (firmware minimum).
- Server holds exactly one STAR backend for its whole lifetime — per the
  "keep pylabrobot connection alive" rule — so repeated reloads of the
  web page don't re-trigger end-stop calibration.
- No auth. Binding to 0.0.0.0 exposes motion control to anything that
  can reach the host — fine inside a Tailnet, not fine on the open
  internet. Lock to ``--host 127.0.0.1`` if you want loopback only.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any, Literal, Optional

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel, Field

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources.hamilton import STARDeck


CALIBRATIONS_FILE = Path(__file__).resolve().parent / "iswap_calibrations.json"


class _State:
    backend: Optional[STARBackend] = None
    lh: Optional[LiquidHandler] = None
    # Serialize commands so two tabs / stacked clicks can't race motion.
    lock: asyncio.Lock = asyncio.Lock()


state = _State()


def _tailscale_ipv4s() -> list[str]:
    """Return this host's Tailscale IPv4 addresses, or [] if tailscale is not up."""
    try:
        out = subprocess.check_output(
            ["tailscale", "ip", "-4"], text=True, timeout=1.0,
            stderr=subprocess.DEVNULL,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return []
    return [line.strip() for line in out.splitlines() if line.strip()]


def _print_access_urls(bind_host: str, port: int) -> None:
    print("STAR ready. Open the control panel at:")
    seen: set[str] = set()
    candidates: list[str] = []
    if bind_host in ("0.0.0.0", ""):
        candidates.extend(["127.0.0.1", *_tailscale_ipv4s()])
    else:
        candidates.append(bind_host)
    for host in candidates:
        if host in seen:
            continue
        seen.add(host)
        print(f"  http://{host}:{port}/")
    if not candidates:
        print(f"  http://{bind_host}:{port}/")


@asynccontextmanager
async def lifespan(app: FastAPI):
    backend = STARBackend()
    lh = LiquidHandler(backend=backend, deck=STARDeck())
    print("connecting to STAR (end-stop calibration on cold start)…")
    await lh.setup()
    state.backend = backend
    state.lh = lh
    _print_access_urls(
        app.extra.get("bind_host", "0.0.0.0"),
        app.extra.get("bind_port", 8765),
    )
    try:
        yield
    finally:
        try:
            await lh.stop()
        except Exception as exc:  # noqa: BLE001
            print(f"lh.stop() failed: {exc}")


app = FastAPI(lifespan=lifespan, title="iSWAP Teach Pendant")


@app.exception_handler(Exception)
async def _all_exceptions_as_json(_request, exc: Exception) -> JSONResponse:
    """Any uncaught exception — including STARFirmwareError — comes back as
    JSON so the browser can display the firmware message instead of failing
    to parse a plain-text 500 body."""
    return JSONResponse(
        status_code=500,
        content={"detail": f"{type(exc).__name__}: {exc}"},
    )


# ------- request models -------

class JogReq(BaseModel):
    axis: Literal["x", "y", "z"]
    delta: float


class GotoReq(BaseModel):
    axis: Literal["x", "y", "z"]
    target: float


class GripperReq(BaseModel):
    plate_width: float = Field(..., gt=0)


class OpenReq(BaseModel):
    # Optional: if plate_width is set, opens to plate_width + tolerance + 2,
    # which is the firmware-required pre-close position for a subsequent
    # close_gripper(plate_width=...). If omitted, uses pylabrobot's default
    # (132 mm for iSWAP 4.0, 91 mm for iSWAP 3).
    plate_width: Optional[float] = None


class RotateReq(BaseModel):
    which: Literal["rotation", "wrist"]
    orientation: str


class SaveReq(BaseModel):
    name: str
    plate_width: float
    rotation: str
    wrist: str


def _backend() -> STARBackend:
    if state.backend is None:
        raise HTTPException(status_code=503, detail="backend not yet ready")
    return state.backend


# ------- HTML page -------

INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>iSWAP Teach Pendant</title>
<style>
  body { font-family: system-ui, sans-serif; max-width: 720px; margin: 1em auto;
         padding: 0 1em; color: #222; }
  h1 { margin: 0 0 0.2em 0; }
  .warn { color: #8a0000; background: #fff3f3; border-left: 4px solid #8a0000;
          padding: 0.6em 0.8em; margin: 0.6em 0 1em 0; }
  fieldset { border: 1px solid #ccc; border-radius: 6px; margin: 0.6em 0;
             padding: 0.6em 0.8em; }
  legend { font-weight: 600; padding: 0 0.4em; }
  .row { display: flex; gap: 0.6em; align-items: center; flex-wrap: wrap;
         margin: 0.3em 0; }
  .axis-label { display: inline-block; width: 1.4em; font-weight: 700; }
  .pos { font-family: ui-monospace, monospace; font-size: 1.1em;
         background: #f5f5f5; padding: 0.1em 0.5em; border-radius: 4px;
         min-width: 5em; text-align: right; }
  button { padding: 0.4em 0.8em; font-size: 0.95em; cursor: pointer;
           border: 1px solid #888; border-radius: 4px; background: #f8f8f8; }
  button:hover:not(:disabled) { background: #eee; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button.primary { background: #1d4ed8; color: white; border-color: #1d4ed8; }
  button.primary:hover:not(:disabled) { background: #1e40af; }
  button.danger { background: #fee2e2; border-color: #b91c1c; color: #7f1d1d; }
  input[type=number], input[type=text], select {
      padding: 0.3em 0.4em; font-size: 0.95em; border: 1px solid #aaa;
      border-radius: 4px; }
  input[type=range] { flex: 1; min-width: 180px; }
  #status { font-family: ui-monospace, monospace; white-space: pre-wrap;
            background: #f0f7f0; border-left: 4px solid #2e7d32;
            padding: 0.4em 0.6em; margin-top: 0.8em; min-height: 1.2em; }
  #status.err { background: #fff3f3; border-color: #8a0000; color: #8a0000; }
</style>
</head>
<body>

<h1>iSWAP Teach Pendant</h1>
<div class="warn">
  Off-deck teaching — confirm the arm has clear swing space before
  initializing. Prefer small jog steps near collision bodies.
</div>

<fieldset>
  <legend>Arm</legend>
  <div class="row">
    <button onclick="cmd('POST','/api/initialize')">Initialize iSWAP</button>
    <button onclick="cmd('POST','/api/park')">Park</button>
    <button onclick="refreshPosition()">Refresh position</button>
  </div>
</fieldset>

<fieldset>
  <legend>Current position (mm)</legend>
  <div class="row">
    <span class="axis-label">X</span><span class="pos" id="pos-x">—</span>
    <span class="axis-label">Y</span><span class="pos" id="pos-y">—</span>
    <span class="axis-label">Z</span><span class="pos" id="pos-z">—</span>
  </div>
</fieldset>

<fieldset>
  <legend>Jog step size (mm)</legend>
  <div class="row">
    <label><input type="radio" name="step" value="0.1"> 0.1</label>
    <label><input type="radio" name="step" value="1" checked> 1</label>
    <label><input type="radio" name="step" value="10"> 10</label>
    <label><input type="radio" name="step" value="50"> 50 (coarse approach)</label>
  </div>
</fieldset>

<fieldset>
  <legend>Jog (relative, runs immediately)</legend>
  <div class="row">
    <span class="axis-label">X</span>
    <button onclick="jog('x',-1)">− X</button>
    <button onclick="jog('x',+1)">+ X</button>
  </div>
  <div class="row">
    <span class="axis-label">Y</span>
    <button onclick="jog('y',-1)">− Y</button>
    <button onclick="jog('y',+1)">+ Y</button>
  </div>
  <div class="row">
    <span class="axis-label">Z</span>
    <button onclick="jog('z',-1)">− Z</button>
    <button onclick="jog('z',+1)">+ Z</button>
  </div>
</fieldset>

<fieldset>
  <legend>Slide + submit (absolute mm, fires on Go)</legend>
  <!-- Sliders span ±100 mm around the last refreshed position. Drag or
       type a target; the arm only moves when Go is clicked. -->
  <div class="row">
    <span class="axis-label">X</span>
    <input type="range" id="slide-x" min="-100" max="100" step="0.1"
           oninput="syncSlider('x')">
    <input type="number" id="target-x" step="0.1" style="width: 6em"
           oninput="syncEntry('x')">
    <button onclick="gotoAxis('x')">Go X</button>
  </div>
  <div class="row">
    <span class="axis-label">Y</span>
    <input type="range" id="slide-y" min="-100" max="100" step="0.1"
           oninput="syncSlider('y')">
    <input type="number" id="target-y" step="0.1" style="width: 6em"
           oninput="syncEntry('y')">
    <button onclick="gotoAxis('y')">Go Y</button>
  </div>
  <div class="row">
    <span class="axis-label">Z</span>
    <input type="range" id="slide-z" min="-100" max="100" step="0.1"
           oninput="syncSlider('z')">
    <input type="number" id="target-z" step="0.1" style="width: 6em"
           oninput="syncEntry('z')">
    <button onclick="gotoAxis('z')">Go Z</button>
  </div>
  <div class="row">
    <button onclick="recentreSliders()">Reset slider ranges to current pos (±100 mm)</button>
  </div>
</fieldset>

<fieldset>
  <legend>Orientation</legend>
  <div class="row">
    <label>rotation
      <select id="rotation">
        <option>LEFT</option><option selected>FRONT</option><option>RIGHT</option>
      </select>
    </label>
    <button onclick="applyOrient('rotation')">Apply</button>
  </div>
  <div class="row">
    <label>wrist
      <select id="wrist">
        <option>RIGHT</option><option selected>STRAIGHT</option>
        <option>LEFT</option><option>REVERSE</option>
      </select>
    </label>
    <button onclick="applyOrient('wrist')">Apply</button>
  </div>
</fieldset>

<fieldset>
  <legend>Gripper</legend>
  <!-- Open uses plate_width + 4 mm (the exact pre-close position firmware
       requires), so Close can immediately succeed. If you want the wide
       default open (132 mm), click "Open wide" — but note Close will then
       fail with "Parameter out of range" until you Open again with the
       right plate width. -->
  <div class="row">
    <label>plate width (mm)
      <input type="number" id="plate-width" value="86.0" step="0.1" style="width: 6em">
    </label>
    <button onclick="openGripper()">Open (for plate width)</button>
    <button onclick="openGripperWide()">Open wide (default)</button>
    <button onclick="closeGripper()">Close</button>
  </div>
</fieldset>

<fieldset>
  <legend>Save calibration</legend>
  <div class="row">
    <label>name
      <input type="text" id="cal-name" value="incubator_handoff" style="width: 16em">
    </label>
    <button class="primary" onclick="saveCalibration()">Save</button>
  </div>
</fieldset>

<div id="status">ready</div>

<script>
const $ = (id) => document.getElementById(id);
const currentPos = { x: null, y: null, z: null };

function setStatus(msg, isErr = false) {
  const el = $("status");
  el.textContent = msg;
  el.classList.toggle("err", isErr);
}

function setButtonsEnabled(enabled) {
  document.querySelectorAll("button").forEach((b) => b.disabled = !enabled);
}

async function cmd(method, path, body) {
  setButtonsEnabled(false);
  setStatus("working…");
  try {
    const res = await fetch(path, {
      method,
      headers: body ? { "Content-Type": "application/json" } : undefined,
      body: body ? JSON.stringify(body) : undefined,
    });
    // Read the body as text first — on 500s or proxy errors it may not
    // be JSON, so calling res.json() directly would throw a confusing
    // "SyntaxError: did not match expected pattern" that hides the
    // actual server message.
    const raw = await res.text();
    let data;
    try { data = raw ? JSON.parse(raw) : {}; }
    catch { data = { detail: raw || `HTTP ${res.status}` }; }
    if (!res.ok) {
      setStatus(data.detail || `HTTP ${res.status}`, true);
      return null;
    }
    if (data.message) setStatus(data.message);
    else setStatus("ok");
    await refreshPosition();
    return data;
  } catch (e) {
    setStatus(`network error: ${e}`, true);
    return null;
  } finally {
    setButtonsEnabled(true);
  }
}

async function refreshPosition() {
  try {
    const res = await fetch("/api/position");
    if (!res.ok) return;
    const d = await res.json();
    currentPos.x = d.x; currentPos.y = d.y; currentPos.z = d.z;
    $("pos-x").textContent = d.x.toFixed(2);
    $("pos-y").textContent = d.y.toFixed(2);
    $("pos-z").textContent = d.z.toFixed(2);
    // Re-centre sliders whenever they're still at defaults (null currentPos).
    for (const a of ["x", "y", "z"]) {
      const slider = $(`slide-${a}`);
      const entry = $(`target-${a}`);
      if (slider.dataset.centred !== "true") {
        slider.min = (d[a] - 100).toFixed(1);
        slider.max = (d[a] + 100).toFixed(1);
        slider.value = d[a].toFixed(2);
        entry.value = d[a].toFixed(2);
        slider.dataset.centred = "true";
      }
    }
  } catch {}
}

function currentStep() {
  return parseFloat(document.querySelector('input[name="step"]:checked').value);
}

function jog(axis, direction) {
  return cmd("POST", "/api/jog", { axis, delta: direction * currentStep() });
}

function syncSlider(axis) { $(`target-${axis}`).value = $(`slide-${axis}`).value; }
function syncEntry(axis)  { $(`slide-${axis}`).value = $(`target-${axis}`).value; }

async function gotoAxis(axis) {
  const target = parseFloat($(`target-${axis}`).value);
  if (isNaN(target)) { setStatus(`${axis.toUpperCase()} target not a number`, true); return; }
  if (currentPos[axis] === null) {
    setStatus("current position unknown — click Refresh first", true); return;
  }
  const delta = target - currentPos[axis];
  if (Math.abs(delta) > 30 &&
      !confirm(`${axis.toUpperCase()} delta = ${delta.toFixed(2)} mm. Proceed?`)) {
    return;
  }
  return cmd("POST", "/api/goto", { axis, target });
}

function recentreSliders() {
  for (const a of ["x", "y", "z"]) {
    $(`slide-${a}`).dataset.centred = "false";
  }
  return refreshPosition();
}

function applyOrient(which) {
  const orientation = $(which).value;
  return cmd("POST", "/api/rotate", { which, orientation });
}

function _plateWidth() {
  const w = parseFloat($("plate-width").value);
  if (isNaN(w) || w < 76.5) {
    setStatus("plate_width must be ≥ 76.5 mm (firmware minimum)", true);
    return null;
  }
  return w;
}

function openGripper() {
  const w = _plateWidth();
  if (w === null) return;
  return cmd("POST", "/api/open_gripper", { plate_width: w });
}

function openGripperWide() {
  return cmd("POST", "/api/open_gripper", {});
}

function closeGripper() {
  const w = _plateWidth();
  if (w === null) return;
  return cmd("POST", "/api/close_gripper", { plate_width: w });
}

async function saveCalibration() {
  const name = $("cal-name").value.trim();
  if (!name) { setStatus("calibration name is required", true); return; }
  return cmd("POST", "/api/save_calibration", {
    name,
    plate_width: parseFloat($("plate-width").value),
    rotation: $("rotation").value,
    wrist: $("wrist").value,
  });
}

// Poll position every 2s so position stays fresh if something outside this
// UI moves the arm. Idle polling rate is gentle — actual motion commands
// trigger an immediate refresh via cmd().
refreshPosition();
setInterval(refreshPosition, 2000);
</script>
</body>
</html>
"""


# ------- endpoints -------

@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return INDEX_HTML


@app.get("/api/position")
async def get_position() -> dict:
    b = _backend()
    try:
        pos = await b.request_iswap_position()
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=500, detail=str(exc))
    return {"x": pos.x, "y": pos.y, "z": pos.z}


@app.post("/api/initialize")
async def initialize() -> dict:
    async with state.lock:
        await _backend().initialize_iswap()
    return {"message": "iSWAP initialized"}


@app.post("/api/park")
async def park() -> dict:
    async with state.lock:
        await _backend().park_iswap()
    return {"message": "iSWAP parked"}


@app.post("/api/jog")
async def jog(req: JogReq) -> dict:
    fn = getattr(_backend(), f"move_iswap_{req.axis}_relative")
    async with state.lock:
        await fn(step_size=req.delta, allow_splitting=False)
    return {"message": f"jogged {req.axis.upper()} by {req.delta:+.2f} mm"}


@app.post("/api/goto")
async def goto(req: GotoReq) -> dict:
    fn = getattr(_backend(), f"move_iswap_{req.axis}")
    async with state.lock:
        await fn(req.target)
    return {"message": f"{req.axis.upper()} → {req.target:.2f} mm"}


@app.post("/api/rotate")
async def rotate(req: RotateReq) -> dict:
    b = _backend()
    if req.which == "rotation":
        orientation = b.RotationDriveOrientation[req.orientation]
        async with state.lock:
            await b.rotate_iswap_rotation_drive(orientation)
    else:
        orientation = b.WristDriveOrientation[req.orientation]
        async with state.lock:
            await b.rotate_iswap_wrist(orientation)
    return {"message": f"{req.which} → {req.orientation}"}


@app.post("/api/open_gripper")
async def open_gripper(req: OpenReq) -> dict:
    b = _backend()
    if req.plate_width is not None:
        # Pre-position to the exact width close_gripper expects:
        # plate_width + tolerance(2) + 2 mm.
        target = req.plate_width + 4.0
        async with state.lock:
            await b.iswap_open_gripper(open_position=target)
        return {"message": f"gripper opened to {target:.1f} mm (ready for {req.plate_width:.1f} mm close)"}
    async with state.lock:
        await b.iswap_open_gripper()
    return {"message": "gripper opened (default position)"}


@app.post("/api/close_gripper")
async def close_gripper(req: GripperReq) -> dict:
    if req.plate_width < 76.5:
        raise HTTPException(
            status_code=400,
            detail="plate_width must be ≥ 76.5 mm (firmware minimum)",
        )
    # Match the open-to formula: tolerance=2.0 so the pre-close position is
    # plate_width + 4 mm. The Open button uses the same plate_width, so this
    # is consistent as long as the user Opens with a plate_width set.
    async with state.lock:
        await _backend().iswap_close_gripper(
            plate_width=req.plate_width, plate_width_tolerance=2.0,
        )
    return {"message": f"gripper closed on {req.plate_width:.1f} mm plate"}


@app.post("/api/save_calibration")
async def save_calibration(req: SaveReq) -> JSONResponse:
    try:
        pos = await _backend().request_iswap_position()
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=500, detail=str(exc))

    data: dict[str, Any] = {}
    if CALIBRATIONS_FILE.exists():
        try:
            data = json.loads(CALIBRATIONS_FILE.read_text())
        except json.JSONDecodeError:
            data = {}
    data[req.name] = {
        "x": pos.x, "y": pos.y, "z": pos.z,
        "rotation": req.rotation,
        "wrist": req.wrist,
        "plate_width": req.plate_width,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
    }
    CALIBRATIONS_FILE.write_text(json.dumps(data, indent=2, sort_keys=True))
    return JSONResponse({"message": f"saved '{req.name}' → {CALIBRATIONS_FILE}"})


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--host", default="0.0.0.0",
        help="bind address; default 0.0.0.0 (reachable on loopback + Tailscale)",
    )
    p.add_argument("--port", type=int, default=8765)
    args = p.parse_args()
    app.extra["bind_host"] = args.host
    app.extra["bind_port"] = args.port
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
