"use strict";
// Servomotor Production Test System — frontend SPA (no build step).
// All test execution is in the backend; this page only views/controls it and
// streams live state over a WebSocket.

const BUSES = ["A", "B", "C"];
let phaseDefs = [];
let settings = null;
let lastSnapshot = null;
let portsData = { available: [], assigned: {}, ready: false };

// ---- helpers ----------------------------------------------------------------
async function api(method, path, body) {
  const opts = { method, headers: { "Content-Type": "application/json" } };
  if (body !== undefined) opts.body = JSON.stringify(body);
  const r = await fetch(path, opts);
  if (!r.ok) throw new Error(await r.text());
  return r.json();
}
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k === "html") e.innerHTML = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v !== null && v !== undefined) e.setAttribute(k, v);
  }
  for (const kid of kids) if (kid != null) e.append(kid.nodeType ? kid : document.createTextNode(kid));
  return e;
};
const $ = (id) => document.getElementById(id);
const fmtDate = (ts) => ts ? new Date(ts * 1000).toLocaleString() : "—";

// ---- tabs -------------------------------------------------------------------
function buildTabs() {
  const nav = $("tabs");
  const tabs = [["collect", "Detect & Collect"], ["database", "Database"]];
  phaseDefs.forEach((p) => tabs.push(["phase-" + p.number, `P${p.number} ${p.name}`]));
  nav.innerHTML = "";
  tabs.forEach(([id, label]) => {
    nav.append(el("button", { id: "navbtn-" + id, onclick: () => showTab(id) }, label));
  });
  // build phase tab containers
  const host = $("phase-tabs");
  host.innerHTML = "";
  phaseDefs.forEach((p) => host.append(buildPhaseTab(p)));
  const initial = (location.hash || "").replace(/^#/, "");
  showTab(document.getElementById("navbtn-" + initial) ? initial : "collect");
}
function showTab(id) {
  if (!$("navbtn-" + id)) return;
  location.hash = id;
  document.querySelectorAll("nav button").forEach((b) => b.classList.remove("active"));
  $("navbtn-" + id).classList.add("active");
  document.querySelectorAll("main .tab").forEach((t) => (t.hidden = true));
  const sec = id === "collect" ? $("tab-collect") : id === "database" ? $("tab-database") : $("tab-" + id);
  sec.hidden = false;
  if (id === "database") loadDevices();
  if (id.startsWith("phase-")) refreshPhaseHistograms(parseInt(id.slice(6)));
}

// ---- Tab 1: ports -----------------------------------------------------------
async function loadPorts() {
  portsData = await api("GET", "/api/ports");
  renderPorts();
}
function renderPorts() {
  const host = $("ports");
  host.innerHTML = "";
  BUSES.forEach((bus) => {
    const sel = el("select", { onchange: async (e) => {
      try { portsData = await api("POST", "/api/ports/" + bus, { port: e.target.value || null });
            renderPorts(); } catch (err) { alert(err.message); }
    } });
    sel.append(el("option", { value: "" }, "— none —"));
    portsData.available.forEach((port) => {
      const usedElsewhere = Object.entries(portsData.assigned)
        .some(([b, p]) => b !== bus && p === port);
      const opt = el("option", { value: port }, port);
      if (portsData.assigned[bus] === port) opt.selected = true;
      if (usedElsewhere) opt.disabled = true;  // uniqueness: grey out
      sel.append(opt);
    });
    host.append(el("div", { class: "bus" }, el("strong", {}, "Bus " + bus), sel));
  });
  $("ports-status").textContent = portsData.ready
    ? "✓ three distinct ports assigned" : "assign three distinct ports to enable Start";
  $("start-all").disabled = !portsData.ready;
}

// ---- Tab 1: detect columns + rack ------------------------------------------
function renderDetect(snap) {
  const host = $("detect-cols");
  host.innerHTML = "";
  BUSES.forEach((bus) => {
    const b = snap.buses[bus];
    const col = el("div", { class: "detect-col" });
    col.append(el("h4", {}, "Bus " + bus));
    col.append(el("div", {},
      el("button", { onclick: () => api("POST", "/api/detect/" + bus) }, "Detect"),
      " ",
      el("span", { class: "pending" }, String(b.pending_detections)),
      b.detecting ? " detecting…" : ""));
    col.append(el("div", { class: "count" }, `Bus ${bus}: ${b.motor_count} motors`));
    col.append(el("button", { onclick: () => api("POST", "/api/clear/" + bus) }, "Clear"));
    const list = el("div", { class: "motor-list" });
    b.motors.forEach((m) => {
      list.append(el("div", { class: "m " + m.color },
        m.unique_id + (m.color === "orange" ? "  ⚠ in DB, not in set" : "")));
    });
    col.append(list);
    host.append(col);
  });
}
function renderRack(snap) {
  const host = $("rack");
  host.innerHTML = "";
  BUSES.forEach((bus) => {
    const b = snap.buses[bus];
    const wrap = el("div", { class: "rack-bus" });
    let title = `Bus ${bus}`;
    if (b.running) title += ` — running${b.current_phase ? " phase " + b.current_phase : ""}`;
    if (b.paused) title += " (paused)";
    wrap.append(el("h4", {}, title));
    const grid = el("div", { class: "grid" });
    b.motors.forEach((m) => {
      grid.append(el("div", { class: "cell " + m.grid_status, title: m.unique_id }));
    });
    wrap.append(grid);
    host.append(wrap);
  });
}

// ---- LED modal --------------------------------------------------------------
function renderLed(snap) {
  const modal = $("led-modal");
  const prompt = snap.led_prompt;
  if (prompt && prompt.pending) {
    modal.hidden = false;
    $("led-question").textContent = prompt.question || "Did all motors pass the LED test?";
    $("led-body").innerHTML = "";
  } else if (prompt && prompt.checked_removed) {
    modal.hidden = false;
    $("led-question").textContent = "Removed (failed) motors:";
    const body = $("led-body");
    body.innerHTML = "";
    const list = prompt.missing && prompt.missing.length
      ? prompt.missing.join(", ") : "(none — all motors responded)";
    body.append(el("p", { class: "cat-summary" }, list),
      el("p", { class: "note" }, "Recorded present=pass, missing=fail."));
  } else {
    modal.hidden = true;
  }
}

// ---- live snapshot ----------------------------------------------------------
function applySnapshot(snap) {
  lastSnapshot = snap;
  renderDetect(snap);
  renderRack(snap);
  renderLed(snap);
}

// ---- Tab 2: database --------------------------------------------------------
async function loadDevices() {
  const scope = $("db-scope").value;
  const result = $("db-result").value;
  let q = "/api/db/devices?scope=" + scope;
  if (result) q += "&result=" + result;
  const data = await api("GET", q);
  const table = $("device-table");
  table.querySelector("thead").innerHTML =
    "<tr><th>Unique ID</th><th>Bus</th><th>Product</th><th>HW</th><th>SCC</th>" +
    "<th>Calib</th><th>Result</th><th>Failing phases</th><th>First seen</th>" +
    "<th>Last seen</th><th></th></tr>";
  const tb = table.querySelector("tbody");
  tb.innerHTML = "";
  data.devices.forEach((d) => {
    const fails = d.failing.map((f) => `P${f.phase}${f.metric ? "·" + f.metric : ""}`).join(", ");
    const tr = el("tr", {},
      el("td", { html: `<code>${d.unique_id}</code>` }),
      el("td", {}, d.bus || "—"),
      el("td", {}, d.product_type || "—"),
      el("td", {}, d.hw_version || "—"),
      el("td", {}, d.scc != null ? String(d.scc) : "—"),
      el("td", {}, d.calibration_done ? "✓" : "—"),
      el("td", { class: "result-" + d.result }, d.result),
      el("td", {}, fails || "—"),
      el("td", {}, fmtDate(d.first_detected)),
      el("td", {}, fmtDate(d.last_seen)),
      el("td", {}, el("button", { onclick: () => api("POST", "/api/db/identify/" + d.unique_id) }, "Identify")));
    tb.append(tr);
  });
  $("db-summary").textContent =
    `total ${data.total} · pass ${data.pass} · fail ${data.fail} · yield ${data.yield.toFixed(1)}% · criteria v${data.criteria_version}`;
}

async function pollPngProgress() {
  const elp = $("png-progress");
  let p;
  try { p = await api("GET", "/api/png_progress"); } catch (e) { return; }
  if (p.running) {
    const pct = p.total ? Math.round(100 * p.done / p.total) : 0;
    elp.innerHTML =
      `<span class="progress-wrap">generating ${p.done}/${p.total}` +
      `<span class="progress-bar"><span class="progress-fill" style="width:${pct}%"></span></span></span>`;
    setTimeout(pollPngProgress, 400);
  } else {
    elp.innerHTML = p.total
      ? `<span class="note">generated ${p.pngs} PNG(s)</span>` : "";
    if (p.total) loadPngs();   // auto-show the freshly generated plots
  }
}

function togglePngs() {
  const panel = $("png-panel");
  if (!panel.hidden) { panel.hidden = true; return; }
  panel.hidden = false;
  loadPngs();
}

async function loadPngs() {
  $("png-panel").hidden = false;
  const scope = $("db-scope").value;
  const data = await api("GET", "/api/pngs?scope=" + scope);
  const host = $("png-groups");
  host.innerHTML = "";
  Object.entries(data.groups).forEach(([type, imgs]) => {
    if (!imgs.length) return;
    const group = el("div", { class: "png-group" });
    const body = el("div", { class: "png-imgs" });
    const head = el("h4", {}, `${type} (${imgs.length}) `);
    const toggle = el("button", { onclick: () => (body.hidden = !body.hidden) }, "show/hide");
    head.append(toggle);
    group.append(head, body);
    imgs.forEach((im) => {
      body.append(el("figure", {}, el("img", { src: "/plots/" + im.filename, loading: "lazy" }),
        el("figcaption", {}, im.unique_id)));
    });
    host.append(group);
  });
}

// ---- per-phase tabs ---------------------------------------------------------
function buildPhaseTab(p) {
  const sec = el("section", { id: "tab-phase-" + p.number, class: "tab", hidden: "" });
  sec.append(el("h2", {}, `Phase ${p.number} — ${p.name}`,
    el("span", { class: "stage" }, p.short)));
  sec.append(el("p", {}, p.description));
  if (p.firmware_dependency)
    sec.append(el("p", { class: "note" }, "⚠ " + p.firmware_dependency));

  const ps = settings.phases[String(p.number)];

  // enable toggle
  const enableWrap = el("label", {},
    el("input", { type: "checkbox", onchange: (e) =>
      api("POST", "/api/phases/" + p.number, { enabled: e.target.checked }) }),
    " Phase enabled");
  enableWrap.querySelector("input").checked = ps.enabled;
  sec.append(el("div", { class: "panel" }, el("h3", {}, "Configuration"), enableWrap,
    buildParamInputs(p, "params", p.params, ps.params)));

  // measured items: criterion + histogram
  if (p.measured.length) {
    const mpanel = el("div", { class: "panel" }, el("h3", {}, "Measured items & criteria"));
    p.measured.forEach((item) => mpanel.append(buildMeasured(p, item, ps)));
    sec.append(mpanel);
  }
  return sec;
}

function buildParamInputs(p, kind, defs, values) {
  const wrap = el("div", { class: "controls", style: "margin-top:10px" });
  defs.forEach((d) => {
    if (d.type === "bool") {
      const inp = el("input", { type: "checkbox", onchange: (e) =>
        sendPhaseUpdate(p.number, kind, d.key, e.target.checked) });
      inp.checked = values[d.key];
      wrap.append(el("label", {}, inp, " " + d.label));
    } else {
      const inp = el("input", {
        type: d.type === "str" ? "text" : "number", value: values[d.key],
        step: d.type === "float" ? "any" : "1", style: "width:120px",
        onchange: (e) => sendPhaseUpdate(p.number, kind, d.key, coerce(d.type, e.target.value)),
      });
      wrap.append(el("label", {}, d.label + (d.unit ? ` (${d.unit})` : "") + " ", inp));
    }
  });
  return wrap;
}

function buildMeasured(p, item, ps) {
  const row = el("div", { class: "measured" });
  const crit = el("div", { class: "crit" });
  crit.append(el("strong", {}, item.label + (item.unit ? ` (${item.unit})` : "")));
  // criterion inputs that drive the threshold lines
  item.threshold_keys.forEach((tk) => {
    const cdef = p.criteria.find((c) => c.key === tk);
    if (!cdef) return;
    const inp = el("input", { type: "number", step: "any", value: ps.criteria[tk],
      style: "width:120px",
      onchange: (e) => sendPhaseUpdate(p.number, "criteria", tk, coerce(cdef.type, e.target.value), true) });
    crit.append(el("label", {}, cdef.label + " ", inp));
  });
  if (!item.threshold_keys.length && item.kind !== "categorical")
    crit.append(el("div", { class: "note" }, "categorical — counts shown in the Database tab"));
  row.append(crit);

  if (item.kind === "histogram") {
    const canvas = el("canvas", { class: "hist", width: 600, height: 180,
      "data-phase": p.number, "data-metric": item.key });
    row.append(canvas);
  } else {
    row.append(el("div", { class: "cat-summary", "data-phase": p.number,
      "data-metric": item.key }, "loading…"));
  }
  return row;
}

function coerce(type, v) {
  if (type === "int") return parseInt(v, 10);
  if (type === "float") return parseFloat(v);
  return v;
}
async function sendPhaseUpdate(number, kind, key, value, refreshHist) {
  const body = {}; body[kind] = {}; body[kind][key] = value;
  const r = await api("POST", "/api/phases/" + number, body);
  settings.phases[String(number)][kind][key] = value;
  if (refreshHist) refreshPhaseHistograms(number);
}

async function refreshPhaseHistograms(number) {
  const scope = $("db-scope") ? $("db-scope").value : "test_set";
  const canvases = document.querySelectorAll(`#tab-phase-${number} canvas.hist`);
  for (const cv of canvases) {
    const metric = cv.getAttribute("data-metric");
    try {
      const data = await api("GET", `/api/histogram/${number}/${metric}?scope=${scope}`);
      drawHistogram(cv, data.values, Object.values(data.thresholds));
    } catch (e) { /* ignore */ }
  }
  // categorical / boolean items: render a count summary instead of a histogram
  const cats = document.querySelectorAll(`#tab-phase-${number} .cat-summary[data-metric]`);
  for (const div of cats) {
    const metric = div.getAttribute("data-metric");
    try {
      const data = await api("GET", `/api/histogram/${number}/${metric}?scope=${scope}`);
      renderCategorical(div, data.counts || {});
    } catch (e) { /* ignore */ }
  }
}

function renderCategorical(div, counts) {
  div.innerHTML = "";
  const entries = Object.entries(counts).sort((a, b) => b[1] - a[1]);
  const total = entries.reduce((s, [, c]) => s + c, 0);
  if (!total) {
    div.append(el("span", { class: "note" }, "no evaluated data (run evaluation first)"));
    return;
  }
  const max = Math.max(...entries.map(([, c]) => c));
  div.append(el("div", { class: "note" }, `${total} device(s) in filtered set`));
  entries.forEach(([value, count]) => {
    const bar = el("div", { class: "catbar" },
      el("span", { class: "catlabel" }, value),
      el("span", { class: "catfill", style: `width:${Math.round(100 * count / max)}%` }),
      el("span", { class: "catcount" }, String(count)));
    div.append(bar);
  });
}

function drawHistogram(canvas, values, thresholds) {
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height, pad = 24;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#8a93a6"; ctx.font = "11px sans-serif";
  if (!values.length) { ctx.fillText("no evaluated data (run evaluation first)", pad, H / 2); return; }
  let lo = Math.min(...values, ...thresholds);
  let hi = Math.max(...values, ...thresholds);
  if (lo === hi) { lo -= 1; hi += 1; }
  const span = hi - lo;
  const nbins = 24;
  const bins = new Array(nbins).fill(0);
  values.forEach((v) => {
    let b = Math.floor(((v - lo) / span) * nbins);
    if (b >= nbins) b = nbins - 1; if (b < 0) b = 0;
    bins[b]++;
  });
  const maxc = Math.max(...bins);
  const x = (v) => pad + ((v - lo) / span) * (W - 2 * pad);
  // bars
  ctx.fillStyle = "#7aa5e8";
  const bw = (W - 2 * pad) / nbins;
  bins.forEach((c, i) => {
    const h = maxc ? (c / maxc) * (H - 2 * pad) : 0;
    ctx.fillRect(pad + i * bw, H - pad - h, bw - 1, h);
  });
  // axes baseline
  ctx.strokeStyle = "#c8d0de"; ctx.beginPath();
  ctx.moveTo(pad, H - pad); ctx.lineTo(W - pad, H - pad); ctx.stroke();
  ctx.fillStyle = "#51607d";
  ctx.fillText(lo.toPrecision(4), pad, H - 8);
  ctx.fillText(hi.toPrecision(4), W - pad - 40, H - 8);
  // threshold lines
  ctx.strokeStyle = "#c0392b"; ctx.lineWidth = 2;
  thresholds.forEach((t) => {
    const tx = x(t);
    ctx.beginPath(); ctx.moveTo(tx, pad - 6); ctx.lineTo(tx, H - pad); ctx.stroke();
    ctx.fillStyle = "#c0392b"; ctx.fillText(String(t), tx + 2, pad);
  });
  ctx.lineWidth = 1;
}

// ---- wire static controls ---------------------------------------------------
function wireControls() {
  $("refresh-ports").onclick = loadPorts;
  $("clear-all").onclick = () => api("POST", "/api/clear_all");
  $("run-scope").onchange = (e) => api("POST", "/api/settings/run_scope", { scope: e.target.value });
  $("start-all").onclick = () => api("POST", "/api/run/start_all");
  $("pause").onclick = () => api("POST", "/api/run/pause");
  $("resume").onclick = () => api("POST", "/api/run/resume");
  $("cancel").onclick = () => api("POST", "/api/run/cancel");
  $("stop").onclick = () => api("POST", "/api/run/stop");
  $("system-reset").onclick = () => api("POST", "/api/system_reset");
  $("run-eval").onclick = async () => {
    const r = await api("POST", "/api/evaluate");
    alert(`Evaluated ${r.motors} motors: ${r.pass} pass / ${r.fail} fail (criteria v${r.criteria_version})`);
    loadDevices();
  };
  $("gen-pngs").onclick = async () => {
    await api("POST", "/api/generate_pngs", { scope: $("db-scope").value });
    pollPngProgress();
  };
  $("show-pngs").onclick = togglePngs;
  $("db-scope").onchange = loadDevices;
  $("db-result").onchange = loadDevices;
  $("led-yes").onclick = async () => { await api("POST", "/api/led/confirm"); };
  $("led-no").onclick = async () => {
    alert("Remove the failing motor(s), power-cycle the rack, then click OK to ping.");
    await api("POST", "/api/led/check_removed");
  };
}

// ---- websocket --------------------------------------------------------------
function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => { $("conn").textContent = "live"; $("conn").classList.add("ok"); };
  ws.onclose = () => {
    $("conn").textContent = "disconnected"; $("conn").classList.remove("ok");
    setTimeout(connectWS, 1500);
  };
  ws.onmessage = (ev) => applySnapshot(JSON.parse(ev.data));
}

// ---- logs polling -----------------------------------------------------------
async function pollLogs() {
  try {
    const r = await api("GET", "/api/logs");
    $("log").textContent = r.logs.join("\n");
    $("log").scrollTop = $("log").scrollHeight;
  } catch (e) { /* ignore */ }
  setTimeout(pollLogs, 2000);
}

// ---- init -------------------------------------------------------------------
async function init() {
  phaseDefs = (await api("GET", "/api/phases")).phases;
  settings = await api("GET", "/api/settings");
  buildTabs();
  wireControls();
  $("run-scope").value = settings.run_scope;
  await loadPorts();
  connectWS();
  pollLogs();
}
init();
