#include "HttpConsole.h"
#include "Logging.h"

HttpConsole::HttpConsole(StatusStore& store, RingLogger& logger)
  : _server(80), _store(store), _logger(logger) {}

void HttpConsole::begin() {
  _server.on("/", HTTP_GET, [this]() { handleRoot(); });
  _server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  _server.on("/api/logs.txt", HTTP_GET, [this]() { handleLogs(); });
  _server.onNotFound([this]() { handleNotFound(); });
  _server.begin();
}

void HttpConsole::update() {
  _server.handleClient();
}

void HttpConsole::handleRoot() {
  static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Pool Cover Console</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root { color-scheme: light dark; font-family: "Inter", system-ui, -apple-system, sans-serif; }
    body { margin: 0; padding: 1rem; background: #0b0d10; color: #e6e9ef; }
    header { font-size: 1.5rem; font-weight: 600; margin-bottom: 1rem; }
    .grid { display: grid; gap: 1rem; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); }
    .card { background: rgba(20, 24, 32, 0.85); border: 1px solid rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 1rem; box-shadow: 0 10px 25px rgba(0,0,0,0.45); }
    h2 { margin: 0 0 0.75rem; font-size: 1rem; text-transform: uppercase; letter-spacing: 0.06em; color: #8fb7ff; }
    .telemetry { display: grid; gap: 0.5rem; }
    .telemetry .kv { display: flex; justify-content: space-between; font-size: 0.95rem; }
    .telemetry .kv span { color: #7d8da5; }
    table { width: 100%; border-collapse: collapse; }
    td { padding: 0.35rem 0; border-bottom: 1px solid rgba(255,255,255,0.05); font-size: 0.9rem; }
    td:first-child { color: #7d8da5; width: 40%; }
    .log-card { margin-top: 1rem; }
    #logView { background: #050607; min-height: 320px; max-height: 60vh; overflow-y: auto; font-family: "JetBrains Mono", monospace; font-size: 0.85rem; line-height: 1.4; padding: 1rem; border-radius: 12px; border: 1px solid rgba(143,183,255,0.2); white-space: pre-wrap; }
    .log-header { display: flex; align-items: baseline; justify-content: space-between; gap: 1rem; }
    .log-meta { font-size: 0.85rem; color: #7d8da5; }
    a { color: #8fb7ff; }
  </style>
</head>
<body>
  <header>Pool Cover Status Console</header>
  <section class="grid">
    <article class="card">
      <h2>Live Telemetry</h2>
      <div id="telemetry" class="telemetry"></div>
    </article>
    <article class="card">
      <h2>Status Rows</h2>
      <table><tbody id="statusRows"></tbody></table>
    </article>
  </section>
  <section class="card log-card">
    <div class="log-header">
      <h2>Live Log Console</h2>
      <span class="log-meta" id="logMeta">Loading…</span>
    </div>
    <pre id="logView">Booting…</pre>
  </section>
  <script>
    const telemetryEl = document.getElementById('telemetry');
    const statusRowsEl = document.getElementById('statusRows');
    const logViewEl = document.getElementById('logView');
    const logMetaEl = document.getElementById('logMeta');

    const esc = function(str) {
      const value = (str === undefined || str === null) ? '' : String(str);
      return value.replace(/&/g, '&amp;')
                  .replace(/</g, '&lt;')
                  .replace(/>/g, '&gt;');
    };

    const fallback = function(value, def) {
      return (value === undefined || value === null) ? def : value;
    };

    function renderTelemetry(data) {
      const wifi = data.wifi || {};
      const mqtt = data.mqtt || {};
      const clicks = data.clicks || {};
      const safety = data.safety || {};
      const rows = [
        ['Mode', data.mode || ''],
        ['Command Source', data.command_source || ''],
        ['Relay State', data.relay_state || ''],
        ['Commanded', data.commanded_motion || ''],
        ['Set Mode', data.set_mode_active ? 'Active' : 'Normal'],
        ['Drive', data.drive_state || (data.drive_active ? 'Active' : 'Idle')],
        ['Panic', data.panic ? (data.panic_reboot ? 'REBOOT' : 'Latched') : 'Clear'],
        ['Clicks', 'pos ' + fallback(clicks.pos, '?') + ' / end ' + fallback(clicks.end, '?')],
        ['Click Guard', clicks.guard || 'idle'],
        ['Wi-Fi', wifi.connected ? (fallback(wifi.ip, '-') + ' (' + fallback(wifi.rssi, '?') + ' dBm)') : 'connecting…'],
        ['MQTT', mqtt.connected ? 'online' : 'offline'],
        ['Runtime Limit', fallback(safety.elapsed, 0) + 's / ' + fallback(safety.max, 0) + 's'],
      ];
      telemetryEl.innerHTML = rows.map(([label, value]) =>
        `<div class="kv"><span>${esc(label)}</span><strong>${esc(value)}</strong></div>`).join('');
    }

    function renderStatusRows(rows) {
      statusRowsEl.innerHTML = rows.map(row =>
        `<tr><td>${esc(row.label)}</td><td>${esc(row.value)}</td></tr>`).join('');
    }

    async function refreshStatus() {
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        if (!res.ok) throw new Error('status fetch failed');
        const data = await res.json();
        const telem = data.telemetry || {};
        renderStatusRows(data.status_rows || []);
        renderTelemetry(telem);
        const lines = (typeof data.log_lines === 'number') ? data.log_lines : 0;
        const bytes = (typeof data.log_bytes === 'number') ? data.log_bytes : 0;
        const updated = (typeof telem.timestamp_ms === 'number') ? telem.timestamp_ms :
                        ((typeof data.uptime_ms === 'number') ? data.uptime_ms : 0);
        let meta = lines + ' lines • ' + bytes + ' bytes';
        if (updated > 0) {
          meta += ' • updated ' + Math.round(updated / 1000) + 's';
        }
        logMetaEl.textContent = meta;
      } catch (err) {
        console.error(err);
        logMetaEl.textContent = 'status unavailable';
      }
    }

    async function refreshLog() {
      try {
        const res = await fetch('/api/logs.txt', { cache: 'no-store' });
        if (!res.ok) throw new Error('log fetch failed');
        const text = await res.text();
        const nearBottom = (logViewEl.scrollHeight - logViewEl.scrollTop - logViewEl.clientHeight) < 40;
        logViewEl.textContent = text || 'No log entries yet.';
        if (nearBottom) {
          logViewEl.scrollTop = logViewEl.scrollHeight;
        }
      } catch (err) {
        console.error(err);
      }
    }

    refreshStatus();
    refreshLog();
    setInterval(refreshStatus, 2500);
    setInterval(refreshLog, 1500);
  </script>
</body>
</html>
)HTML";
  _server.send_P(200, "text/html", PAGE);
}

void HttpConsole::handleStatus() {
  StaticJsonDocument<3072> doc;
  doc["uptime_ms"] = millis();
  doc["log_bytes"] = static_cast<uint32_t>(_logger.sizeBytes());
  doc["log_lines"] = static_cast<uint32_t>(_logger.lineCount());

  JsonArray rows = doc.createNestedArray("status_rows");
  const uint8_t count = _store.count();
  for (uint8_t i = 0; i < count; ++i) {
    JsonObject obj = rows.createNestedObject();
    obj["label"] = _store.entry(i).label;
    obj["value"] = _store.entry(i).value;
  }

  JsonObject telem = doc.createNestedObject("telemetry");
  telem["timestamp_ms"] = millis();

  if (_statusCallback) {
    _statusCallback(doc);
  }

  String payload;
  serializeJson(doc, payload);
  _server.send(200, "application/json", payload);
}

void HttpConsole::handleLogs() {
  String text = _logger.blob(LogLevel::DEBUG);
  _server.send(200, "text/plain", text);
}

void HttpConsole::handleNotFound() {
  _server.send(404, "text/plain", "Not found");
}
