"use strict";
// Streams both controllers' grip poses, triggers, grips and buttons to the ROS bridge every XR
// frame, and shows the camera mosaic it sends back: as an <img> on the 2D page and as a panel
// floating in front of the operator inside the XR session.

const token = new URLSearchParams(location.search).get("token");
const statusNode = document.getElementById("status");
const preview = document.getElementById("preview");
const byId = id => document.getElementById(id);
let ws, session, referenceSpace, gl, program, texture, quad, uMVP, lastUrl = null;
let rttMs = null, transparent = false;

function setStatus(text) { statusNode.textContent = text; }
function send(obj) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj)); }
for (const name of ["start", "save", "discard", "ready", "reset_scene"])
  byId(name).onclick = () => send({type: "event", name});

function describe(s) {
  const rec = s.recorder || {}, tel = s.teleop || {};
  const eng = tel.engaged || {}, ik = tel.ik_error || {};
  return [
    `${rec.recording ? `RECORDING ${rec.frames} steps (${rec.dropped} dropped)` : "not recording"}` +
      ` · next episode ${rec.next_episode ?? "?"}${rec.saving ? " · saving…" : ""}`,
    `task: ${rec.instruction ?? "?"}`,
    `arms: left ${eng.left ? "ENGAGED" : "free"} (ik ${ik.left ?? "-"}) · right ${eng.right ? "ENGAGED" : "free"} (ik ${ik.right ?? "-"})` +
      `${tel.ready_motion ? " · moving to ready" : ""}${tel.motion_enabled === false ? " · MOTION DISABLED" : ""}`,
    `round trip ${s.rtt_ms != null ? s.rtt_ms.toFixed(0) + " ms" : "?"}`,
    rec.last_saved ? `last saved: ${rec.last_saved}` : "",
    rec.error ? `error: ${rec.error}` : "",
  ].filter(Boolean).join("\n");
}

function connect() {
  if (!token) return setStatus("URL has no ?token=… — open the full URL printed by the bridge.");
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${scheme}//${location.host}/ws?token=${encodeURIComponent(token)}`);
  ws.binaryType = "blob";
  ws.onopen = () => setStatus("connected; waiting for status…");
  // A page that lost control to a newer client stays off; auto-reconnecting would take control
  // back and the two pages would kick each other every 2 s.
  let replaced = false;
  ws.onclose = () => {
    if (replaced) return setStatus("another client took control; reload this page to take it back");
    setStatus("disconnected (arms released); retrying in 2 s");
    setTimeout(connect, 2000);
  };
  ws.onmessage = async event => {
    if (typeof event.data === "string") {
      const data = JSON.parse(event.data);
      if (data.type === "pong") rttMs = performance.now() - data.t;
      else if (data.type === "status") setStatus(describe(data));
      else if (data.type === "error") {
        replaced = data.message === "another client took control";
        setStatus(`error: ${data.message}`);
      }
      return;
    }
    const url = URL.createObjectURL(event.data);
    preview.onload = () => { if (lastUrl) URL.revokeObjectURL(lastUrl); lastUrl = url; };
    preview.src = url;
    if (gl) {
      try {
        const bitmap = await createImageBitmap(event.data);
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, bitmap);
        bitmap.close();
      } catch (error) { console.error("texture update failed", error); }
    }
  };
}
connect();
setInterval(() => send({type: "ping", t: performance.now()}), 1000);

// ---------------------------------------------------------------- WebGL panel
function multiply(a, b) {
  const out = new Float32Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++)
    for (let k = 0; k < 4; k++) out[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
  return out;
}
function shader(type, source) {
  const obj = gl.createShader(type);
  gl.shaderSource(obj, source); gl.compileShader(obj);
  if (!gl.getShaderParameter(obj, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(obj));
  return obj;
}
function setupGL() {
  program = gl.createProgram();
  gl.attachShader(program, shader(gl.VERTEX_SHADER, `
    attribute vec2 aPos; attribute vec2 aUV; uniform mat4 uMVP; varying vec2 vUV;
    // Image rows start at the top; the quad's UV origin is bottom-left.
    void main() { gl_Position = uMVP * vec4(aPos, 0.0, 1.0); vUV = vec2(aUV.x, 1.0 - aUV.y); }`));
  gl.attachShader(program, shader(gl.FRAGMENT_SHADER, `
    precision mediump float; varying vec2 vUV; uniform sampler2D uImage;
    void main() { gl_FragColor = texture2D(uImage, vUV); }`));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  uMVP = gl.getUniformLocation(program, "uMVP");
  quad = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, quad);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
    -1, -1, 0, 0,  1, -1, 1, 0,  -1, 1, 0, 1,   -1, 1, 0, 1,  1, -1, 1, 0,  1, 1, 1, 1]), gl.STATIC_DRAW);
  const pos = gl.getAttribLocation(program, "aPos"), uv = gl.getAttribLocation(program, "aUV");
  gl.enableVertexAttribArray(pos); gl.vertexAttribPointer(pos, 2, gl.FLOAT, false, 16, 0);
  gl.enableVertexAttribArray(uv); gl.vertexAttribPointer(uv, 2, gl.FLOAT, false, 16, 8);
  texture = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, texture);
  for (const [k, v] of [[gl.TEXTURE_MIN_FILTER, gl.LINEAR], [gl.TEXTURE_MAG_FILTER, gl.LINEAR],
                        [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE], [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE]])
    gl.texParameteri(gl.TEXTURE_2D, k, v);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 1, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, new Uint8Array([30, 40, 50, 255]));
}

// ---------------------------------------------------------------- XR input + rendering
function sendInput(frame) {
  const hands = {left: null, right: null};
  for (const source of session.inputSources) {
    if (!source.gripSpace || !(source.handedness in hands)) continue;
    const pose = frame.getPose(source.gripSpace, referenceSpace);
    if (!pose) continue;
    const {position: p, orientation: o} = pose.transform;
    const buttons = source.gamepad ? source.gamepad.buttons : [];
    hands[source.handedness] = {
      pose: {position: [p.x, p.y, p.z], orientation: [o.x, o.y, o.z, o.w]},
      trigger: buttons[0] ? buttons[0].value : 0,
      squeeze: buttons[1] ? buttons[1].value : 0,
      buttons: Array.from(buttons, b => b.pressed),
    };
  }
  send({type: "pose", hands, rtt_ms: rttMs});
}

function xrFrame(_time, frame) {
  session.requestAnimationFrame(xrFrame);
  const viewer = frame.getViewerPose(referenceSpace);
  if (!viewer) return;
  sendInput(frame);
  gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
  // Panel 1.6 m x 1.2 m (4:3 mosaic), 1.3 m in front of the start pose, centre 1.25 m high.
  const model = new Float32Array([0.8, 0, 0, 0,  0, 0.6, 0, 0,  0, 0, 1, 0,  0, 1.25, -1.3, 1]);
  gl.useProgram(program); gl.bindBuffer(gl.ARRAY_BUFFER, quad);
  for (const view of viewer.views) {
    const vp = session.renderState.baseLayer.getViewport(view);
    gl.viewport(vp.x, vp.y, vp.width, vp.height);
    if (transparent) gl.clearColor(0, 0, 0, 0); else gl.clearColor(0.02, 0.035, 0.06, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.uniformMatrix4fv(uMVP, false, multiply(view.projectionMatrix, multiply(view.transform.inverse.matrix, model)));
    gl.drawArrays(gl.TRIANGLES, 0, 6);
  }
}

async function enter(mode) {
  try {
    session = await navigator.xr.requestSession(mode, {requiredFeatures: ["local-floor"]});
    transparent = mode === "immersive-ar";
    const canvas = document.createElement("canvas");
    gl = canvas.getContext("webgl", {xrCompatible: true, alpha: true});
    await gl.makeXRCompatible();
    setupGL();
    session.updateRenderState({baseLayer: new XRWebGLLayer(session, gl)});
    referenceSpace = await session.requestReferenceSpace("local-floor");
    session.addEventListener("end", () => { session = null; gl = null; send({type: "pose", hands: {}}); });
    session.requestAnimationFrame(xrFrame);
  } catch (error) { setStatus(`cannot start ${mode}: ${error.message}`); }
}
byId("enter-ar").onclick = () => enter("immersive-ar");
byId("enter-vr").onclick = () => enter("immersive-vr");

if (navigator.xr && window.isSecureContext) {
  for (const [id, mode] of [["enter-ar", "immersive-ar"], ["enter-vr", "immersive-vr"]])
    navigator.xr.isSessionSupported(mode).then(ok => { byId(id).disabled = !ok; });
} else {
  setStatus("WebXR needs the Quest Browser on http://localhost (via adb reverse) or HTTPS; " +
            "this browser can only monitor.");
}
