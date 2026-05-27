const DEFAULTS = {
  tim1ClkHz: 128_000_000,
  tim1Arr: 1024,
  tim1Psc: 0,
  tim1Ccr3: 512,
  tim16ClkHz: 64_000_000,
  tim16Arr: 2048,
  tim16Psc: 0,
  adcClockSourceHz: 64_000_000,
  adcPrescaler: 1,
  adcSmp1: 0,
  adcSmp2: 6,
  adcSmp2Channel: 6,
  adcSequence: [6, 0, 9, 4, 5, 2, 1, 3],
  windowUs: 160,
  singlePeriod: false,
  dmaBufferLength: 32,
  tim1DutyPercent: 50,
  tim16IsrUs: 22,
};

const ADC_SMP_CYCLES = {
  0: 1.5,
  1: 3.5,
  2: 7.5,
  3: 12.5,
  4: 19.5,
  5: 39.5,
  6: 79.5,
  7: 160.5,
};

const ADC_PRESC_DIV = {
  0: 1,
  1: 2,
  2: 4,
  3: 6,
  4: 8,
  5: 10,
  6: 12,
  7: 16,
};

const CHANNEL_LABELS = {
  6: "Dummy (VDDA)",
  0: "Temp",
  9: "Supply",
  4: "Phase A",
  5: "Phase B",
  2: "Hall1",
  1: "Hall2",
  3: "Hall3",
};

const COLORS = {
  pwm: "#2c60f5",
  adcTrigger: "#e23b3b",
  adcScan: "#ff9a1f",
  adcDetail: "#32b5c5",
  adcPhase: "#ff5fa2",
  dma: "#1d9c3c",
  tim16: "#7a3ef0",
  laneBg: "#f6f6ff",
  axis: "#333",
};

const LANE_HEIGHT = 32;
const LANE_GAP = 10;

const LANE_CONFIG = [
  { key: "pwm", label: "TIM1 CH1 PWM Output", height: 90 },
  { key: "adcTrigger", label: "ADC Triggers (CCR3 both edges)", height: LANE_HEIGHT },
  { key: "adcScan", label: "ADC Scan Window", height: LANE_HEIGHT },
  { key: "adcDetail", label: "ADC Channel Detail", height: 110 },
  { key: "dma", label: "DMA Update", height: LANE_HEIGHT },
  { key: "tim16Counter", label: "TIM16 Counter", height: LANE_HEIGHT },
  { key: "tim16", label: "TIM16 ISR", height: LANE_HEIGHT },
];

const ui = {
  adcClockSource: document.getElementById("adcClockSource"),
  adcPrescaler: document.getElementById("adcPrescaler"),
  adcSmp1: document.getElementById("adcSmp1"),
  adcSmp2: document.getElementById("adcSmp2"),
  adcSmp2Channel: document.getElementById("adcSmp2Channel"),
  adcSequence: document.getElementById("adcSequence"),
  tim1Duty: document.getElementById("tim1Duty"),
  dmaBufferLength: document.getElementById("dmaBufferLength"),
  tim16IsrUs: document.getElementById("tim16IsrUs"),
  windowUs: document.getElementById("windowUs"),
  singlePeriod: document.getElementById("singlePeriod"),
  tim1Clk: document.getElementById("tim1Clk"),
  tim1Arr: document.getElementById("tim1Arr"),
  tim1Psc: document.getElementById("tim1Psc"),
  tim1Ccr3: document.getElementById("tim1Ccr3"),
  tim16Clk: document.getElementById("tim16Clk"),
  tim16Arr: document.getElementById("tim16Arr"),
  tim16Psc: document.getElementById("tim16Psc"),
  timeline: document.getElementById("timeline"),
  legend: document.getElementById("legend"),
  guidance: document.getElementById("guidance"),
  exportCsv: document.getElementById("exportCsv"),
  exportSvg: document.getElementById("exportSvg"),
  exportPng: document.getElementById("exportPng"),
};

const viewState = {
  panUs: 0,
  zoom: 1,
  dragStartX: null,
  dragStartPan: 0,
};

const math = {
  pwmPeriodUs(cfg) {
    return (2 * cfg.tim1Arr * 1e6) / cfg.tim1ClkHz;
  },
  tim16PeriodUs(cfg) {
    return ((cfg.tim16Psc + 1) * (cfg.tim16Arr + 1) * 1e6) / cfg.tim16ClkHz;
  },
  adcClkHz(cfg) {
    return cfg.adcClockSourceHz / ADC_PRESC_DIV[cfg.adcPrescaler];
  },
  adcChannelCycles(cfg, channel) {
    const smpCode = channel === cfg.adcSmp2Channel ? cfg.adcSmp2 : cfg.adcSmp1;
    return ADC_SMP_CYCLES[smpCode];
  },
};

function buildDefaultOptions() {
  [16_000_000, 32_000_000, 64_000_000, 128_000_000].forEach((freq) => {
    const option = document.createElement("option");
    option.value = String(freq);
    option.textContent = `${(freq / 1_000_000).toFixed(0)} MHz`;
    ui.adcClockSource.appendChild(option);
  });

  Object.entries(ADC_PRESC_DIV).forEach(([code, div]) => {
    const option = document.createElement("option");
    option.value = code;
    option.textContent = `Code ${code} (÷${div})`;
    ui.adcPrescaler.appendChild(option);
  });

  Object.entries(ADC_SMP_CYCLES).forEach(([code, cycles]) => {
    const label = `Code ${code} (${cycles} cycles)`;
    const option1 = document.createElement("option");
    option1.value = code;
    option1.textContent = label;
    ui.adcSmp1.appendChild(option1);

    const option2 = document.createElement("option");
    option2.value = code;
    option2.textContent = label;
    ui.adcSmp2.appendChild(option2);
  });
}

function syncDefaults() {
  ui.tim1Clk.value = DEFAULTS.tim1ClkHz;
  ui.tim1Arr.value = DEFAULTS.tim1Arr;
  ui.tim1Psc.value = DEFAULTS.tim1Psc;
  ui.tim1Ccr3.value = DEFAULTS.tim1Ccr3;
  ui.tim16Clk.value = DEFAULTS.tim16ClkHz;
  ui.tim16Arr.value = DEFAULTS.tim16Arr;
  ui.tim16Psc.value = DEFAULTS.tim16Psc;
  ui.adcClockSource.value = DEFAULTS.adcClockSourceHz;

  ui.adcPrescaler.value = DEFAULTS.adcPrescaler;
  ui.adcSmp1.value = DEFAULTS.adcSmp1;
  ui.adcSmp2.value = DEFAULTS.adcSmp2;
  ui.adcSmp2Channel.value = DEFAULTS.adcSmp2Channel;
  ui.adcSequence.value = DEFAULTS.adcSequence.join(",");
  ui.dmaBufferLength.value = DEFAULTS.dmaBufferLength;
  ui.tim1Duty.value = DEFAULTS.tim1DutyPercent;
  ui.tim16IsrUs.value = DEFAULTS.tim16IsrUs;
  ui.windowUs.value = DEFAULTS.windowUs;
  ui.singlePeriod.checked = DEFAULTS.singlePeriod;
}

function getConfig() {
  const sequence = ui.adcSequence.value
    .split(/[,\s]+/)
    .filter((v) => v.length)
    .map((v) => Number(v.trim()))
    .filter((v) => !Number.isNaN(v));

  return {
    tim1ClkHz: Number(ui.tim1Clk.value),
    tim1Arr: Number(ui.tim1Arr.value),
    tim1Psc: Number(ui.tim1Psc.value),
    tim1Ccr3: Number(ui.tim1Ccr3.value),
    tim16ClkHz: Number(ui.tim16Clk.value),
    tim16Arr: Number(ui.tim16Arr.value),
    tim16Psc: Number(ui.tim16Psc.value),
    adcClockSourceHz: Number(ui.adcClockSource.value),
    adcPrescaler: Number(ui.adcPrescaler.value),
    adcSmp1: Number(ui.adcSmp1.value),
    adcSmp2: Number(ui.adcSmp2.value),
    adcSmp2Channel: Number(ui.adcSmp2Channel.value),
    adcSequence: sequence.length ? sequence : DEFAULTS.adcSequence,
    dmaBufferLength: Math.max(1, Number(ui.dmaBufferLength.value) || DEFAULTS.dmaBufferLength),
    tim1DutyPercent: Number(ui.tim1Duty.value),
    tim16IsrUs: Math.max(1, Number(ui.tim16IsrUs.value) || DEFAULTS.tim16IsrUs),
    windowUs: Number(ui.windowUs.value),
    singlePeriod: ui.singlePeriod.checked,
  };
}

function buildAdcSegments(cfg) {
  const segments = [];
  let cursorUs = 0;
  const adcClk = math.adcClkHz(cfg);
  cfg.adcSequence.forEach((ch) => {
    const sampleCycles = math.adcChannelCycles(cfg, ch);
    const sampleUs = (sampleCycles / adcClk) * 1e6;
    const convUs = (12.5 / adcClk) * 1e6;
    segments.push({ label: `CH${ch} sample`, startUs: cursorUs, endUs: cursorUs + sampleUs, channel: ch });
    cursorUs += sampleUs;
    segments.push({ label: `CH${ch} convert`, startUs: cursorUs, endUs: cursorUs + convUs, channel: ch });
    cursorUs += convUs;
  });
  return { segments, totalUs: cursorUs };
}

function buildEvents(cfg) {
  const windowUs = cfg.singlePeriod ? math.pwmPeriodUs(cfg) : cfg.windowUs;
  const periodUs = math.pwmPeriodUs(cfg);
  const nPeriods = Math.max(1, Math.ceil(windowUs / periodUs));
  const triggerOffsetsUs = [
    (cfg.tim1Ccr3 / cfg.tim1ClkHz) * 1e6,
    ((2 * cfg.tim1Arr - cfg.tim1Ccr3) / cfg.tim1ClkHz) * 1e6,
  ];

  const { segments, totalUs } = buildAdcSegments(cfg);
  const events = [];
  const detailSegments = [];

  const dmaCycleCount = Math.max(1, Math.floor(cfg.dmaBufferLength / cfg.adcSequence.length));
  let dmaIndex = 0;
  for (let i = 0; i < nPeriods; i += 1) {
    const base = i * periodUs;
    triggerOffsetsUs.forEach((offset, idx) => {
      const trig = base + offset;
      events.push({ type: "adcTrigger", startUs: trig, endUs: trig, label: `ADC trigger ${idx + 1}` });
      events.push({ type: "adcScan", startUs: trig, endUs: trig + totalUs, label: "ADC scan" });
      const dmaLabel = `DMA update (idx ${dmaIndex})`;
      events.push({ type: "dma", startUs: trig + totalUs, endUs: trig + totalUs, label: dmaLabel });
      dmaIndex = (dmaIndex + 1) % dmaCycleCount;

      segments.forEach((seg) => {
        detailSegments.push({
          type: "adcDetail",
          startUs: trig + seg.startUs,
          endUs: trig + seg.endUs,
          label: seg.label,
          channel: seg.channel,
        });
      });
    });
  }

  const tim16PeriodUs = math.tim16PeriodUs(cfg);
  const nTim16 = Math.max(1, Math.ceil(windowUs / tim16PeriodUs));
  for (let i = 0; i < nTim16; i += 1) {
    const start = i * tim16PeriodUs;
    const end = start + Math.min(cfg.tim16IsrUs, tim16PeriodUs);
    events.push({ type: "tim16", startUs: start, endUs: end, label: "TIM16 ISR" });
  }

  return { windowUs, periodUs, tim16PeriodUs, events, detailSegments };
}

function computePwmSamples(cfg, windowUs) {
  const periodUs = math.pwmPeriodUs(cfg);
  const samples = 500;
  const duty = Math.min(100, Math.max(0, cfg.tim1DutyPercent)) / 100;
  const points = [];
  for (let i = 0; i <= samples; i += 1) {
    const tUs = (i / samples) * windowUs;
    const phase = (tUs % periodUs) / periodUs;
    const carrier = phase <= 0.5 ? phase * 2 : (1 - phase) * 2;
    const output = carrier <= duty ? 1 : 0;
    points.push({ tUs, value: output, carrier });
  }
  return points;
}

function tim1CountAt(cfg, tUs) {
  const periodUs = math.pwmPeriodUs(cfg);
  const phaseUs = tUs % periodUs;
  const halfUs = periodUs / 2;
  return phaseUs <= halfUs
    ? (phaseUs / halfUs) * cfg.tim1Arr
    : ((periodUs - phaseUs) / halfUs) * cfg.tim1Arr;
}

function tim16CountAt(cfg, tUs) {
  const periodUs = math.tim16PeriodUs(cfg);
  const phaseUs = tUs % periodUs;
  return (phaseUs / periodUs) * (cfg.tim16Arr + 1);
}

function computeTim1Counts(cfg, windowUs) {
  const points = [];
  const samples = 500;
  for (let i = 0; i <= samples; i += 1) {
    const tUs = (i / samples) * windowUs;
    points.push({ tUs, value: tim1CountAt(cfg, tUs) });
  }
  return points;
}

function computeTim16Counts(cfg, windowUs) {
  const points = [];
  const samples = 500;
  for (let i = 0; i <= samples; i += 1) {
    const tUs = (i / samples) * windowUs;
    points.push({ tUs, value: tim16CountAt(cfg, tUs) });
  }
  return points;
}

function setLegend() {
  const items = [
    { label: "TIM1 CH1 PWM output", color: COLORS.pwm },
    { label: "TIM1 carrier (triangle)", color: "#9bb2ff" },
    { label: "ADC trigger", color: COLORS.adcTrigger },
    { label: "ADC scan", color: COLORS.adcScan },
    { label: "ADC channel segments", color: COLORS.adcDetail },
    { label: "Phase A/B sample/convert", color: COLORS.adcPhase },
    { label: "DMA update", color: COLORS.dma },
    { label: "TIM16 ISR", color: COLORS.tim16 },
  ];
  ui.legend.innerHTML = "";
  items.forEach((item) => {
    const div = document.createElement("div");
    div.className = "item";
    const swatch = document.createElement("span");
    swatch.className = "swatch";
    swatch.style.background = item.color;
    const text = document.createElement("span");
    text.textContent = item.label;
    div.appendChild(swatch);
    div.appendChild(text);
    ui.legend.appendChild(div);
  });
}

function resetSvg(svg) {
  while (svg.firstChild) {
    svg.removeChild(svg.firstChild);
  }
}

function drawAxes(svg, width, height, view, cfg) {
  const axisGroup = document.createElementNS("http://www.w3.org/2000/svg", "g");
  axisGroup.setAttribute("font-size", "11");
  axisGroup.setAttribute("fill", COLORS.axis);

  const tickCount = 8;
  for (let i = 0; i <= tickCount; i += 1) {
    const x = (i / tickCount) * width;
    const tUs = view.startUs + (i / tickCount) * view.spanUs;
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttribute("x1", x);
    line.setAttribute("x2", x);
    line.setAttribute("y1", 0);
    line.setAttribute("y2", height);
    line.setAttribute("stroke", "#ececf5");
    axisGroup.appendChild(line);

    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("x", x + 2);
    label.setAttribute("y", 12);
    label.textContent = `${tUs.toFixed(1)} us`;
    axisGroup.appendChild(label);
  }

  const tim1AxisY = height - 26;
  const tim16AxisY = height - 10;
  for (let i = 0; i <= tickCount; i += 1) {
    const x = (i / tickCount) * width;
    const tUs = view.startUs + (i / tickCount) * view.spanUs;
    const tim1Counts = tim1CountAt(cfg, tUs);
    const tim16Counts = tim16CountAt(cfg, tUs);

    const tim1Label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    tim1Label.setAttribute("x", x + 2);
    tim1Label.setAttribute("y", tim1AxisY);
    tim1Label.textContent = `${tim1Counts.toFixed(0)} cts`;
    axisGroup.appendChild(tim1Label);

    const tim16Label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    tim16Label.setAttribute("x", x + 2);
    tim16Label.setAttribute("y", tim16AxisY);
    tim16Label.textContent = `${tim16Counts.toFixed(0)} cts`;
    axisGroup.appendChild(tim16Label);
  }

  const tim1Title = document.createElementNS("http://www.w3.org/2000/svg", "text");
  tim1Title.setAttribute("x", width - 180);
  tim1Title.setAttribute("y", tim1AxisY);
  tim1Title.textContent = "TIM1 counter";
  axisGroup.appendChild(tim1Title);

  const tim16Title = document.createElementNS("http://www.w3.org/2000/svg", "text");
  tim16Title.setAttribute("x", width - 180);
  tim16Title.setAttribute("y", tim16AxisY);
  tim16Title.textContent = "TIM16 counter";
  axisGroup.appendChild(tim16Title);

  svg.appendChild(axisGroup);
}

function drawLaneLabels(svg, lanes, width) {
  lanes.forEach((lane) => {
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    text.setAttribute("x", 8);
    text.setAttribute("y", lane.y + 14);
    text.setAttribute("fill", COLORS.axis);
    text.setAttribute("font-size", "12");
    text.textContent = lane.label;
    svg.appendChild(text);
  });
}

function drawLanes(svg, lanes, width) {
  lanes.forEach((lane) => {
    const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    rect.setAttribute("x", 0);
    rect.setAttribute("y", lane.y);
    rect.setAttribute("width", width);
    rect.setAttribute("height", lane.height);
    rect.setAttribute("fill", COLORS.laneBg);
    rect.setAttribute("opacity", "0.55");
    svg.appendChild(rect);
  });
}

function toX(view, width, timeUs) {
  return ((timeUs - view.startUs) / view.spanUs) * width;
}

function drawPwm(svg, lane, view, width, cfg) {
  const points = computePwmSamples(cfg, view.spanUs);
  const carrierPath = document.createElementNS("http://www.w3.org/2000/svg", "path");
  const carrierD = points
    .map((pt, idx) => {
      const x = toX(view, width, pt.tUs + view.startUs);
      const y = lane.y + lane.height - pt.carrier * (lane.height - 30) - 10;
      return `${idx === 0 ? "M" : "L"} ${x.toFixed(2)} ${y.toFixed(2)}`;
    })
    .join(" ");
  carrierPath.setAttribute("d", carrierD);
  carrierPath.setAttribute("stroke", "#9bb2ff");
  carrierPath.setAttribute("fill", "none");
  carrierPath.setAttribute("stroke-width", "1.5");
  carrierPath.setAttribute("stroke-dasharray", "3 3");
  svg.appendChild(carrierPath);

  const pwmPath = document.createElementNS("http://www.w3.org/2000/svg", "path");
  const pwmD = points
    .map((pt, idx) => {
      const x = toX(view, width, pt.tUs + view.startUs);
      const y = lane.y + lane.height - pt.value * (lane.height - 30) - 10;
      return `${idx === 0 ? "M" : "L"} ${x.toFixed(2)} ${y.toFixed(2)}`;
    })
    .join(" ");
  pwmPath.setAttribute("d", pwmD);
  pwmPath.setAttribute("stroke", COLORS.pwm);
  pwmPath.setAttribute("fill", "none");
  pwmPath.setAttribute("stroke-width", "2.4");
  svg.appendChild(pwmPath);
}

function drawCounter(svg, lane, view, width, points, color, labelText) {
  const maxValue = Math.max(...points.map((pt) => pt.value)) || 1;
  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  const d = points
    .map((pt, idx) => {
      const x = toX(view, width, pt.tUs + view.startUs);
      const y = lane.y + lane.height - (pt.value / maxValue) * (lane.height - 16) - 8;
      return `${idx === 0 ? "M" : "L"} ${x.toFixed(2)} ${y.toFixed(2)}`;
    })
    .join(" ");
  path.setAttribute("d", d);
  path.setAttribute("stroke", color);
  path.setAttribute("fill", "none");
  path.setAttribute("stroke-width", "2");
  svg.appendChild(path);

  const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
  label.setAttribute("x", width - 160);
  label.setAttribute("y", lane.y + 14);
  label.setAttribute("font-size", "11");
  label.setAttribute("fill", color);
  label.textContent = labelText;
  svg.appendChild(label);
}

function drawEvents(svg, lane, view, width, events) {
  events.forEach((ev) => {
    if (ev.endUs < view.startUs || ev.startUs > view.startUs + view.spanUs) {
      return;
    }
    if (ev.startUs === ev.endUs) {
      const x = toX(view, width, ev.startUs);
      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("x1", x);
      line.setAttribute("x2", x);
      line.setAttribute("y1", lane.y + 4);
      line.setAttribute("y2", lane.y + lane.height - 4);
      line.setAttribute("stroke", COLORS[ev.type]);
      line.setAttribute("stroke-width", "2");
      svg.appendChild(line);
    } else {
      const x = toX(view, width, ev.startUs);
      const w = toX(view, width, ev.endUs) - x;
      const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      rect.setAttribute("x", x);
      rect.setAttribute("y", lane.y + 6);
      rect.setAttribute("width", w);
      rect.setAttribute("height", lane.height - 12);
      rect.setAttribute("fill", COLORS[ev.type]);
      rect.setAttribute("opacity", "0.5");
      svg.appendChild(rect);
    }
  });
}

function drawAdcDetail(svg, lane, view, width, segments) {
  const bandHeight = (lane.height - 16) / 2;
  segments.forEach((seg, idx) => {
    const x = toX(view, width, seg.startUs);
    const w = toX(view, width, seg.endUs) - x;
    const band = idx % 2;
    const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    rect.setAttribute("x", x);
    rect.setAttribute("y", lane.y + 6 + band * bandHeight);
    rect.setAttribute("width", w);
    rect.setAttribute("height", bandHeight - 2);
    const isPhase = seg.channel === 4 || seg.channel === 5;
    rect.setAttribute("fill", isPhase ? COLORS.adcPhase : COLORS.adcDetail);
    rect.setAttribute("opacity", isPhase ? "0.75" : "0.6");
    svg.appendChild(rect);

    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("x", x + 2);
    label.setAttribute("y", lane.y + 6 + band * bandHeight + 12);
    label.setAttribute("font-size", "10");
    const channelLabel = CHANNEL_LABELS[seg.channel] ? ` (${CHANNEL_LABELS[seg.channel]})` : "";
    label.textContent = `${seg.label.replace("CH", "CH ")}${channelLabel}`;
    svg.appendChild(label);
  });
}

function render(cfg) {
  const { windowUs, periodUs, tim16PeriodUs, events, detailSegments } = buildEvents(cfg);
  const adcClk = math.adcClkHz(cfg);
  const viewSpanUs = windowUs / viewState.zoom;
  const maxPan = Math.max(0, windowUs - viewSpanUs);
  viewState.panUs = Math.min(Math.max(0, viewState.panUs), maxPan);
  const viewStartUs = viewState.panUs;
  const view = { startUs: viewStartUs, spanUs: viewSpanUs };

  const svg = ui.timeline;
  const width = svg.viewBox.baseVal.width || svg.clientWidth;
  const height = svg.viewBox.baseVal.height || svg.clientHeight;
  svg.setAttribute("viewBox", `0 0 ${width} ${height}`);
  resetSvg(svg);

  const lanes = [];
  let cursorY = 20;
  LANE_CONFIG.forEach((lane) => {
    lanes.push({ ...lane, y: cursorY });
    cursorY += lane.height + LANE_GAP;
  });

  drawLanes(svg, lanes, width);
  drawAxes(svg, width, height, view, cfg);
  drawLaneLabels(svg, lanes, width);

  const laneMap = Object.fromEntries(lanes.map((lane) => [lane.key, lane]));
  drawPwm(svg, laneMap.pwm, view, width, cfg);
  drawEvents(
    svg,
    laneMap.adcTrigger,
    view,
    width,
    events.filter((e) => e.type === "adcTrigger")
  );
  drawEvents(
    svg,
    laneMap.adcScan,
    view,
    width,
    events.filter((e) => e.type === "adcScan")
  );
  drawEvents(
    svg,
    laneMap.dma,
    view,
    width,
    events.filter((e) => e.type === "dma")
  );
  annotateDma(svg, laneMap.dma, view, width, events.filter((e) => e.type === "dma"));
  drawEvents(
    svg,
    laneMap.tim16,
    view,
    width,
    events.filter((e) => e.type === "tim16")
  );
  drawCounter(
    svg,
    laneMap.tim16Counter,
    view,
    width,
    computeTim16Counts(cfg, view.spanUs),
    COLORS.tim16,
    `TIM16 0..${cfg.tim16Arr} counts`
  );
  drawAdcDetail(svg, laneMap.adcDetail, view, width, detailSegments);

  updateGuidance(cfg, { windowUs, periodUs, tim16PeriodUs, adcClk });
}

function annotateDma(svg, lane, view, width, events) {
  events.forEach((ev) => {
    if (ev.startUs < view.startUs || ev.startUs > view.startUs + view.spanUs) {
      return;
    }
    const x = toX(view, width, ev.startUs);
    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("x", x + 2);
    label.setAttribute("y", lane.y + lane.height - 6);
    label.setAttribute("font-size", "10");
    label.setAttribute("fill", COLORS.dma);
    label.textContent = ev.label;
    svg.appendChild(label);
  });
}

function updateGuidance(cfg, derived) {
  const smp1 = ADC_SMP_CYCLES[cfg.adcSmp1];
  const smp2 = ADC_SMP_CYCLES[cfg.adcSmp2];
  const prescDiv = ADC_PRESC_DIV[cfg.adcPrescaler];
  const adcClk = derived.adcClk;
  const tim1Freq = cfg.tim1ClkHz / (2 * cfg.tim1Arr);
  const tim16Freq = cfg.tim16ClkHz / ((cfg.tim16Psc + 1) * (cfg.tim16Arr + 1));
  const guidance = [
    `ADC clock source (Hz): ${cfg.adcClockSourceHz}`,
    `ADC prescaler (ADC_CCR_PRESC): code ${cfg.adcPrescaler} (÷${prescDiv})`,
    `ADC clock (Hz): ${adcClk.toFixed(0)}`,
    `TIM1 frequency (center-aligned): ${tim1Freq.toFixed(1)} Hz`,
    `TIM16 frequency: ${tim16Freq.toFixed(1)} Hz`,
    `TIM16 ISR duration: ${cfg.tim16IsrUs.toFixed(1)} us`,
    `ADC SMP1 code: ${cfg.adcSmp1} (${smp1} cycles)`,
    `ADC SMP2 code: ${cfg.adcSmp2} (${smp2} cycles)`,
    `SMP2 channel: ${cfg.adcSmp2Channel}`,
    "",
    "C register guidance (read-only):",
    `  ADC_CCR_PRESC = ${cfg.adcPrescaler} (RM0444 19.4.1)`,
    `  ADC_SMPR_SMP1 = ${cfg.adcSmp1} (RM0444 19.4.12)`,
    `  ADC_SMPR_SMP2 = ${cfg.adcSmp2} (RM0444 19.4.12)`,
    `  DMA buffer length = ${cfg.dmaBufferLength} (circular)`,
    `  DMA cycle count = ${Math.max(1, Math.floor(cfg.dmaBufferLength / cfg.adcSequence.length))} (index 0..${Math.max(1, Math.floor(cfg.dmaBufferLength / cfg.adcSequence.length)) - 1})`,
    "",
    "Trigger source: TIM1 TRGO2 = OC3REF (MMS2), EXTEN=both edges",
    `PWM period (center-aligned): T_PWM = 2 * ARR / f_TIM1 = ${derived.periodUs.toFixed(2)} us`,
    `TIM16 period: T_TIM16 = (PSC+1)*(ARR+1)/f_TIM16 = ${derived.tim16PeriodUs.toFixed(2)} us`,
    "",
    "ADC conversion time formula:",
    "  t_conv = (sampling + 12.5) / f_ADC (RM0444 19.3.4, 19.4.20)",
  ];

  ui.guidance.textContent = guidance.join("\n");
}

function exportCsv(cfg) {
  const { events, detailSegments } = buildEvents(cfg);
  const rows = ["event_type,start_us,end_us,label"]; 
  events.concat(detailSegments).forEach((ev) => {
    rows.push(`${ev.type},${ev.startUs.toFixed(3)},${ev.endUs.toFixed(3)},${ev.label}`);
  });
  const blob = new Blob([rows.join("\n")], { type: "text/csv" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "pwm_adc_timing_events.csv";
  link.click();
  URL.revokeObjectURL(url);
}

function exportSvg() {
  const serializer = new XMLSerializer();
  const svgString = serializer.serializeToString(ui.timeline);
  const blob = new Blob([svgString], { type: "image/svg+xml" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "pwm_adc_timing.svg";
  link.click();
  URL.revokeObjectURL(url);
}

function exportPng() {
  const serializer = new XMLSerializer();
  const svgString = serializer.serializeToString(ui.timeline);
  const img = new Image();
  const blob = new Blob([svgString], { type: "image/svg+xml" });
  const url = URL.createObjectURL(blob);
  img.onload = () => {
    const canvas = document.createElement("canvas");
    canvas.width = ui.timeline.clientWidth;
    canvas.height = ui.timeline.clientHeight;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(img, 0, 0);
    canvas.toBlob((pngBlob) => {
      const pngUrl = URL.createObjectURL(pngBlob);
      const link = document.createElement("a");
      link.href = pngUrl;
      link.download = "pwm_adc_timing.png";
      link.click();
      URL.revokeObjectURL(pngUrl);
    });
    URL.revokeObjectURL(url);
  };
  img.src = url;
}

function handleWheel(event) {
  event.preventDefault();
  const delta = Math.sign(event.deltaY);
  viewState.zoom = Math.min(12, Math.max(0.5, viewState.zoom + delta * 0.2));
  render(getConfig());
}

function handleMouseDown(event) {
  viewState.dragStartX = event.clientX;
  viewState.dragStartPan = viewState.panUs;
}

function handleMouseMove(event) {
  if (viewState.dragStartX === null) {
    return;
  }
  const cfg = getConfig();
  const { windowUs } = buildEvents(cfg);
  const span = windowUs / viewState.zoom;
  const dx = event.clientX - viewState.dragStartX;
  const usPerPx = span / ui.timeline.clientWidth;
  const newPan = Math.max(0, viewState.dragStartPan - dx * usPerPx);
  viewState.panUs = Math.min(newPan, Math.max(0, windowUs - span));
  render(cfg);
}

function handleMouseUp() {
  viewState.dragStartX = null;
}

function attachListeners() {
  [
    ui.adcPrescaler,
    ui.adcSmp1,
    ui.adcSmp2,
    ui.adcSmp2Channel,
    ui.adcSequence,
    ui.tim1Duty,
    ui.tim16IsrUs,
    ui.windowUs,
    ui.singlePeriod,
  ].forEach((el) => {
    el.addEventListener("input", () => render(getConfig()));
  });

  ui.exportCsv.addEventListener("click", () => exportCsv(getConfig()));
  ui.exportSvg.addEventListener("click", () => exportSvg());
  ui.exportPng.addEventListener("click", () => exportPng());

  ui.timeline.addEventListener("wheel", handleWheel, { passive: false });
  ui.timeline.addEventListener("mousedown", handleMouseDown);
  ui.timeline.addEventListener("mousemove", handleMouseMove);
  ui.timeline.addEventListener("mouseup", handleMouseUp);
  ui.timeline.addEventListener("mouseleave", handleMouseUp);
}

function init() {
  buildDefaultOptions();
  syncDefaults();
  setLegend();
  attachListeners();
  render(getConfig());
}

init();
