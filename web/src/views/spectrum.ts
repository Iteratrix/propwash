import { get_spectrogram } from "../../pkg/propwash_web.js";
import { heatColor } from "../format.js";
import type { SessionRef, VibrationAnalysis, Spectrum } from "../types.js";
import { $, chartWidth, filterConfig, echartsInstances } from "../state.js";
import { AXIS_COLORS } from "../chart-config.js";

export function renderSpectraEcharts(vibration: VibrationAnalysis | null): void {
  const container = $("#spectrum-plots");
  container.innerHTML = "";

  if (!vibration || !vibration.spectra || vibration.spectra.length === 0) {
    container.innerHTML = '<p class="hint">No spectrum data available.</p>';
    return;
  }

  for (const spectrum of vibration.spectra) {
    const row = document.createElement("div");
    row.className = "spectrum-row";

    const title = document.createElement("h3");
    title.textContent = spectrum.axis;
    row.appendChild(title);

    const chartDiv = document.createElement("div");
    chartDiv.style.width = "100%";
    chartDiv.style.height = "250px";
    row.appendChild(chartDiv);
    container.appendChild(row);

    const maxFreq = Math.min(spectrum.sample_rate_hz / 2, 1000);
    const endIdx = spectrum.frequencies_hz.findIndex((f: number) => f > maxFreq);
    const n = endIdx > 0 ? endIdx : spectrum.frequencies_hz.length;
    const freqs = spectrum.frequencies_hz.slice(0, n);
    const mags = spectrum.magnitudes_db.slice(0, n);
    const color = AXIS_COLORS[spectrum.axis] || "#5b8def";

    const chart = echarts.init(chartDiv, null, { renderer: "canvas" });
    echartsInstances.push(chart);

    const markLines: any[] = [];

    if (filterConfig && !filterConfig.error) {
      const filters = [
        { val: filterConfig.gyro_lpf_hz, label: "LPF1", color: "#e8944a" },
        { val: filterConfig.gyro_lpf2_hz, label: "LPF2", color: "#e8944a" },
        { val: filterConfig.dterm_lpf_hz, label: "D-LPF", color: "#9b59b6" },
        { val: filterConfig.gyro_notch1_hz, label: "Notch1", color: "#e85454" },
        { val: filterConfig.gyro_notch2_hz, label: "Notch2", color: "#e85454" },
      ];
      for (const f of filters) {
        if (f.val && f.val > 0 && f.val <= maxFreq) {
          markLines.push({ xAxis: f.val, label: { formatter: f.label, position: "end", fontSize: 9, color: f.color }, lineStyle: { color: f.color, type: "dashed", width: 1 } });
        }
      }
    }

    // RPM harmonic lines (1x, 2x, 3x of average motor frequency)
    if (vibration.avg_motor_hz) {
      const rpm = vibration.avg_motor_hz;
      for (let h = 1; h <= 3; h++) {
        const freq = rpm * h;
        if (freq > 0 && freq <= maxFreq) {
          markLines.push({ xAxis: freq, label: { formatter: `${h}x RPM`, position: "start", fontSize: 9, color: "#4ec88c" }, lineStyle: { color: "#4ec88c", type: "dotted", width: 1 } });
        }
      }
    }

    const peakPoints = (spectrum.peaks || []).slice(0, 3).map((p: any) => ({
      coord: [p.frequency_hz, p.magnitude_db],
      label: { formatter: `${p.frequency_hz.toFixed(0)} Hz`, fontSize: 10, color: "#e0e0e6", position: "top" },
      symbol: "circle",
      symbolSize: 6,
      itemStyle: { color: p.classification === "MotorNoise" ? "#e8944a" : p.classification === "FrameResonance" ? "#e85454" : color },
    }));

    chart.setOption({
      grid: { left: 50, right: 20, top: 40, bottom: 50 },
      toolbox: {
        show: true,
        right: 10,
        top: 0,
        feature: {
          dataZoom: { title: { zoom: "Zoom", back: "Reset" }, iconStyle: { borderColor: "#8888a0" }, emphasis: { iconStyle: { borderColor: "#5b8def" } } },
          restore: { title: "Reset", iconStyle: { borderColor: "#8888a0" }, emphasis: { iconStyle: { borderColor: "#5b8def" } } },
          saveAsImage: { title: "Save", iconStyle: { borderColor: "#8888a0" }, emphasis: { iconStyle: { borderColor: "#5b8def" } } },
        },
        iconStyle: { borderColor: "#8888a0" },
      },
      xAxis: { type: "value", name: "Hz", nameLocation: "center", nameGap: 25, min: 0, max: maxFreq, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
      yAxis: { type: "value", name: "dB", nameLocation: "center", nameGap: 35, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
      tooltip: { trigger: "axis", backgroundColor: "#1a1d27", borderColor: "#2a2d3a", textStyle: { color: "#e0e0e6", fontFamily: "JetBrains Mono, monospace", fontSize: 11 } },
      dataZoom: [
        { type: "inside", xAxisIndex: 0 },
        { type: "slider", xAxisIndex: 0, height: 18, bottom: 2, borderColor: "#2a2d3a", fillerColor: "rgba(91,141,239,0.15)", handleStyle: { color: "#5b8def" }, textStyle: { color: "#8888a0", fontSize: 9 } },
      ],
      series: [{
        type: "line",
        data: freqs.map((f: number, i: number) => [f, mags[i]]),
        smooth: false,
        symbol: "none",
        lineStyle: { color, width: 1.5 },
        areaStyle: { color: color + "18" },
        markLine: markLines.length > 0 ? { data: markLines, symbol: "none", silent: true } : undefined,
        markPoint: peakPoints.length > 0 ? { data: peakPoints } : undefined,
      }],
    });

    if (spectrum.peaks && spectrum.peaks.length > 0) {
      const peaks = document.createElement("div");
      peaks.className = "peaks-list";
      for (const p of spectrum.peaks) {
        const badge = document.createElement("span");
        badge.className = "peak-badge";
        let inner = `<span class="freq">${p.frequency_hz.toFixed(0)} Hz</span> (${p.magnitude_db.toFixed(1)} dB)`;
        if (p.classification) {
          const cls = p.classification === "MotorNoise" ? "Motor" : p.classification === "FrameResonance" ? "Frame" : "";
          if (cls) inner += `<span class="class">${cls}</span>`;
        }
        badge.innerHTML = inner;
        peaks.appendChild(badge);
      }
      row.appendChild(peaks);
    }
  }
}

export function renderThrottleBandsEcharts(vibration: VibrationAnalysis | null): void {
  const container = $("#throttle-plots");
  container.innerHTML = "";

  if (!vibration || !vibration.throttle_bands || vibration.throttle_bands.length === 0) {
    container.innerHTML = '<p class="hint">No throttle band data.</p>';
    return;
  }

  for (const band of vibration.throttle_bands) {
    const section = document.createElement("div");
    section.className = "throttle-band";

    const title = document.createElement("h3");
    title.textContent = `Throttle ${band.label}`;
    section.appendChild(title);

    const meta = document.createElement("div");
    meta.className = "band-meta";
    const rpmInfo = band.avg_motor_hz ? ` — avg ${(band.avg_motor_hz * 60).toFixed(0)} RPM (${band.avg_motor_hz.toFixed(0)} Hz)` : "";
    meta.textContent = `${band.frame_count.toLocaleString()} frames${rpmInfo}`;
    section.appendChild(meta);

    if (band.spectra && band.spectra.length > 0) {
      const chartDiv = document.createElement("div");
      chartDiv.style.width = "100%";
      chartDiv.style.height = "200px";
      section.appendChild(chartDiv);
      container.appendChild(section);

      const maxFreq = Math.min(band.spectra[0].sample_rate_hz / 2, 1000);
      const endIdx = band.spectra[0].frequencies_hz.findIndex((f: number) => f > maxFreq);
      const n = endIdx > 0 ? endIdx : band.spectra[0].frequencies_hz.length;

      const chart = echarts.init(chartDiv, null, { renderer: "canvas" });
      echartsInstances.push(chart);

      // RPM harmonic markLines for this throttle band
      const rpmMarkLines: any[] = [];
      if (band.avg_motor_hz) {
        for (let h = 1; h <= 3; h++) {
          const freq = band.avg_motor_hz * h;
          if (freq > 0 && freq <= maxFreq) {
            rpmMarkLines.push({ xAxis: freq, label: { formatter: `${h}x`, position: "start", fontSize: 8, color: "#4ec88c" }, lineStyle: { color: "#4ec88c", type: "dotted", width: 1 } });
          }
        }
      }

      const series = [];
      for (let si = 0; si < band.spectra.length; si++) {
        const s = band.spectra[si];
        const color = AXIS_COLORS[s.axis] || "#5b8def";
        const entry: any = {
          type: "line",
          name: s.axis,
          data: s.frequencies_hz.slice(0, n).map((f: number, i: number) => [f, s.magnitudes_db[i]]),
          smooth: false,
          symbol: "none",
          lineStyle: { color, width: 1.5 },
        };
        // Attach RPM lines to the first series
        if (si === 0 && rpmMarkLines.length > 0) {
          entry.markLine = { data: rpmMarkLines, symbol: "none", silent: true };
        }
        series.push(entry);
      }

      chart.setOption({
        grid: { left: 50, right: 20, top: 30, bottom: 40 },
        legend: { show: true, textStyle: { color: "#8888a0", fontSize: 10 }, top: 0 },
        toolbox: {
          show: true,
          right: 10,
          top: 0,
          feature: {
            dataZoom: { title: { zoom: "Zoom", back: "Reset" }, iconStyle: { borderColor: "#8888a0" } },
            restore: { title: "Reset", iconStyle: { borderColor: "#8888a0" } },
          },
          iconStyle: { borderColor: "#8888a0" },
        },
        xAxis: { type: "value", name: "Hz", nameLocation: "center", nameGap: 25, min: 0, max: maxFreq, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
        yAxis: { type: "value", name: "dB", nameLocation: "center", nameGap: 35, axisLabel: { color: "#8888a0", fontSize: 10 }, splitLine: { lineStyle: { color: "rgba(42,45,58,0.6)" } } },
        tooltip: { trigger: "axis", backgroundColor: "#1a1d27", borderColor: "#2a2d3a", textStyle: { color: "#e0e0e6", fontFamily: "JetBrains Mono, monospace", fontSize: 11 } },
        dataZoom: [{ type: "inside", xAxisIndex: 0 }],
        series,
      });
    } else {
      container.appendChild(section);
    }
  }
}

export function renderSpectrogram(ref: SessionRef): void {
  const container = $("#spectrogram-plots");
  container.innerHTML = "";

  const json = get_spectrogram(ref.fileId, ref.sessionIdx, "roll,pitch,yaw");
  const data = JSON.parse(json);
  if (data.error || !data.axes || data.axes.length === 0) {
    container.innerHTML = '<p class="hint">No spectrogram data available.</p>';
    return;
  }

  for (const axis of data.axes) {
    const row = document.createElement("div");
    row.className = "spectrogram-row";

    const title = document.createElement("h3");
    title.textContent = axis.axis;
    row.appendChild(title);

    const canvas = document.createElement("canvas");
    const width = chartWidth();
    const height = 160;
    canvas.width = width;
    canvas.height = height;
    canvas.style.width = width + "px";
    canvas.style.height = height + "px";
    row.appendChild(canvas);

    drawSpectrogram(canvas, axis);

    const legend = document.createElement("div");
    legend.className = "spectrogram-legend";
    legend.innerHTML = `<span>0s</span><span>${axis.time_s[axis.time_s.length - 1]?.toFixed(1) || 0}s</span>`;
    row.appendChild(legend);

    container.appendChild(row);
  }
}

export function drawSpectrogram(canvas: HTMLCanvasElement, axis: any): void {
  const ctx = canvas.getContext("2d")!;
  const { width, height } = canvas;
  const nTime = axis.time_s.length;
  const nFreq = axis.frequencies_hz.length;

  if (nTime === 0 || nFreq === 0) return;

  let dbMin = Infinity;
  let dbMax = -Infinity;
  for (const row of axis.magnitudes_db) {
    for (const v of row) {
      if (v > -120) {
        if (v < dbMin) dbMin = v;
        if (v > dbMax) dbMax = v;
      }
    }
  }

  const dbRange = dbMax - dbMin || 1;
  const img = ctx.createImageData(nTime, nFreq);

  for (let t = 0; t < nTime; t++) {
    const row = axis.magnitudes_db[t];
    for (let f = 0; f < nFreq; f++) {
      const val = (row[f] - dbMin) / dbRange;
      const clamped = Math.max(0, Math.min(1, val));
      const [r, g, b] = heatColor(clamped);
      const yFlipped = nFreq - 1 - f;
      const idx = (yFlipped * nTime + t) * 4;
      img.data[idx] = r;
      img.data[idx + 1] = g;
      img.data[idx + 2] = b;
      img.data[idx + 3] = 255;
    }
  }

  const tmpCanvas = document.createElement("canvas");
  tmpCanvas.width = nTime;
  tmpCanvas.height = nFreq;
  tmpCanvas.getContext("2d")!.putImageData(img, 0, 0);

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tmpCanvas, 0, 0, width, height);

  ctx.fillStyle = "#e0e0e6";
  ctx.font = "10px JetBrains Mono, monospace";
  ctx.textAlign = "right";
  ctx.fillText(`${axis.frequencies_hz[nFreq - 1]?.toFixed(0) || ""} Hz`, width - 4, 12);
  ctx.fillText("0 Hz", width - 4, height - 4);
}

export function createMultiAxisPlot(container: HTMLElement, spectra: Spectrum[]): void {
  if (spectra.length === 0) return;

  const maxFreq = Math.min(spectra[0].sample_rate_hz / 2, 1000);
  const endIdx = spectra[0].frequencies_hz.findIndex((f: number) => f > maxFreq);
  const n = endIdx > 0 ? endIdx : spectra[0].frequencies_hz.length;

  const xData = spectra[0].frequencies_hz.slice(0, n);
  const datasets = [xData];
  const series = [{}];

  for (const s of spectra) {
    datasets.push(s.magnitudes_db.slice(0, n));
    series.push({
      label: s.axis,
      stroke: AXIS_COLORS[s.axis] || "#5b8def",
      width: 1.5,
    });
  }

  const width = chartWidth();

  const opts = {
    width,
    height: 180,
    cursor: { show: true },
    scales: { x: { time: false } },
    axes: [
      { label: "Hz", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
      { label: "dB", stroke: "#8888a0", grid: { stroke: "rgba(42,45,58,0.6)" }, ticks: { stroke: "rgba(42,45,58,0.6)" }, font: "11px JetBrains Mono, monospace", labelFont: "11px JetBrains Mono, monospace" },
    ],
    series,
  };

  new uPlot(opts, datasets, container);
}

export function renderAccel(vibration: VibrationAnalysis | null): void {
  const panel = $("#accel-panel");
  const infoDiv = $("#accel-info");
  const plotsDiv = $("#accel-plots");

  if (!vibration || !vibration.accel) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");
  const accel = vibration.accel;

  const axes = ["X", "Y", "Z"];
  infoDiv.innerHTML = `<div class="accel-rms">` +
    accel.rms.map((v: number, i: number) =>
      `<div class="rms-card">
        <div class="axis">${axes[i]}</div>
        <div class="rms-value">${v.toFixed(1)}</div>
      </div>`
    ).join("") +
    `</div>`;

  plotsDiv.innerHTML = "";
  if (accel.spectra && accel.spectra.length > 0) {
    const plotDiv = document.createElement("div");
    plotsDiv.appendChild(plotDiv);
    createMultiAxisPlot(plotDiv, accel.spectra);
  }
}

export function renderPropwash(vibration: VibrationAnalysis | null): void {
  const panel = $("#propwash-panel");
  const infoDiv = $("#propwash-info");
  const plotsDiv = $("#propwash-plots");

  if (!vibration || !vibration.propwash) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");
  const pw = vibration.propwash;

  let infoHtml = `<p class="propwash-summary">${pw.chop_count} throttle chop${pw.chop_count === 1 ? "" : "s"} analyzed`;
  if (pw.dominant_frequency_hz != null && pw.dominant_magnitude_db != null) {
    infoHtml += ` — dominant propwash frequency: <strong>${pw.dominant_frequency_hz.toFixed(0)} Hz</strong> (${pw.dominant_magnitude_db.toFixed(1)} dB)`;
  } else {
    infoHtml += ` — no dominant frequency in 20-100 Hz range`;
  }
  infoHtml += `</p>`;
  infoDiv.innerHTML = infoHtml;

  plotsDiv.innerHTML = "";
  if (pw.spectra && pw.spectra.length > 0) {
    const plotDiv = document.createElement("div");
    plotsDiv.appendChild(plotDiv);
    createMultiAxisPlot(plotDiv, pw.spectra);
  }
}
