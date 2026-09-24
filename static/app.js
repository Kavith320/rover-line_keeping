/**
 * Autonomous Agricultural Rover - Client Telemetry & Remote Control Logic
 * Hardware PWM & Serial Motor Driver Controller
 * Pure Vanilla JavaScript (Zero External Dependencies)
 */

document.addEventListener('DOMContentLoaded', () => {
  // --- DOM Elements: Top Navbar & Telemetry Badges ---
  const txtVideoName = document.getElementById('txt-video-name');
  const txtSerialStatus = document.getElementById('txt-serial-status');
  const serialDot = document.getElementById('serial-dot');
  const statusPillSerial = document.getElementById('status-pill-serial');
  const statusBeacon = document.getElementById('status-beacon');
  const txtStatus = document.getElementById('txt-status');
  const txtFps = document.getElementById('txt-fps');
  const txtPing = document.getElementById('txt-ping');
  const txtConnIp = document.getElementById('txt-conn-ip');
  const txtFooterSbc = document.getElementById('txt-footer-sbc');
  const btnNavEstop = document.getElementById('btn-nav-estop');

  // --- Video Stream Viewport ---
  const roverStream = document.getElementById('rover-stream');
  const txtStreamDesc = document.getElementById('txt-stream-desc');
  const txtOverlayErr = document.getElementById('txt-overlay-err');
  const txtOverlayPwm = document.getElementById('txt-overlay-pwm');
  const txtOverlayAct = document.getElementById('txt-overlay-act');
  const viewModeButtons = document.querySelectorAll('#view-mode-selector .tab-btn');

  // --- Quick Actions Bar ---
  const btnPause = document.getElementById('btn-pause');
  const txtBtnPause = document.getElementById('txt-btn-pause');
  const btnToggleLight = document.getElementById('btn-toggle-light');
  const selectDevice = document.getElementById('select-device');
  const selectSbcProfile = document.getElementById('select-sbc-profile');
  const btnSaveCfg = document.getElementById('btn-save-cfg');

  // --- Telemetry Cluster ---
  const actionBanner = document.getElementById('action-banner');
  const txtDecision = document.getElementById('txt-decision');
  const txtErrorBanner = document.getElementById('txt-error-banner');
  const txtRpmL = document.getElementById('txt-rpm-l');
  const txtRpmR = document.getElementById('txt-rpm-r');
  const txtPwmL = document.getElementById('txt-pwm-l');
  const txtPwmR = document.getElementById('txt-pwm-r');
  const barRpmL = document.getElementById('bar-rpm-l');
  const barRpmR = document.getElementById('bar-rpm-r');
  const txtSerialTx = document.getElementById('txt-serial-tx');

  // --- PID Balance & Terms ---
  const txtPidNet = document.getElementById('txt-pid-net');
  const balanceFill = document.getElementById('balance-fill');
  const balancePointer = document.getElementById('balance-pointer');
  const txtTermP = document.getElementById('txt-term-p');
  const txtTermI = document.getElementById('txt-term-i');
  const txtTermD = document.getElementById('txt-term-d');

  // --- Environment & Tiller ---
  const barDensity = document.getElementById('bar-density');
  const txtDensity = document.getElementById('txt-density');
  const txtAmbient = document.getElementById('txt-ambient');
  const txtLightMode = document.getElementById('txt-light-mode');
  const txtDynThresh = document.getElementById('txt-dyn-thresh');
  const txtTillerStatus = document.getElementById('txt-tiller-status');

  // --- Calibration Deck Tabs ---
  const ctrlTabs = document.querySelectorAll('.ctrl-tab');
  const tabPanels = document.querySelectorAll('.tab-panel');
  const btnResetDefaults = document.getElementById('btn-reset-defaults');

  // --- Hardware Tab: Serial & Motor Driver Elements ---
  const selectSerialPort = document.getElementById('select-serial-port');
  const btnRefreshPorts = document.getElementById('btn-refresh-ports');
  const inputCustomPort = document.getElementById('input-custom-port');
  const selectBaudrate = document.getElementById('select-baudrate');
  const btnSerialConnect = document.getElementById('btn-serial-connect');
  const btnSerialDisconnect = document.getElementById('btn-serial-disconnect');
  const terminalPacketLog = document.getElementById('terminal-packet-log');

  const checkMotorsEnable = document.getElementById('check-motors-enable');
  const rangePwmMin = document.getElementById('range-pwm-min');
  const valPwmMin = document.getElementById('val-pwm-min');
  const rangePwmMax = document.getElementById('range-pwm-max');
  const valPwmMax = document.getElementById('val-pwm-max');
  const checkInvertLeft = document.getElementById('check-invert-left');
  const checkInvertRight = document.getElementById('check-invert-right');
  const selectProtocol = document.getElementById('select-protocol');
  const btnSaveHardwareCfg = document.getElementById('btn-save-hardware-cfg');

  const inputCustomCamera = document.getElementById('input-custom-camera');
  const btnApplyCamera = document.getElementById('btn-apply-camera');
  const btnEstopCard = document.getElementById('btn-estop-card');

  // Benchtop D-Pad Test Buttons
  const btnTestFwd = document.getElementById('btn-test-fwd');
  const btnTestRev = document.getElementById('btn-test-rev');
  const btnTestSpinL = document.getElementById('btn-test-spin-l');
  const btnTestSpinR = document.getElementById('btn-test-spin-r');
  const btnTestStop = document.getElementById('btn-test-stop');

  // --- PID & ROI & Light Sliders ---
  const rangeKp = document.getElementById('range-kp');
  const valKp = document.getElementById('val-kp');
  const rangeKi = document.getElementById('range-ki');
  const valKi = document.getElementById('val-ki');
  const rangeKd = document.getElementById('range-kd');
  const valKd = document.getElementById('val-kd');
  const rangeBaseRpm = document.getElementById('range-base-rpm');
  const valBaseRpm = document.getElementById('val-base-rpm');

  const rangeRoiTop = document.getElementById('range-roi-top');
  const valRoiTop = document.getElementById('val-roi-top');
  const rangeRoiBottom = document.getElementById('range-roi-bottom');
  const valRoiBottom = document.getElementById('val-roi-bottom');
  const rangeRoiLeft = document.getElementById('range-roi-left');
  const valRoiLeft = document.getElementById('val-roi-left');
  const rangeRoiRight = document.getElementById('range-roi-right');
  const valRoiRight = document.getElementById('val-roi-right');

  const rangeClahe = document.getElementById('range-clahe');
  const valClahe = document.getElementById('val-clahe');
  const checkAdaptive = document.getElementById('check-adaptive');

  // --- State Tracking ---
  let isUserInteracting = false;
  let userInteractTimeout = null;
  let currentView = 'combined';
  let availableDevicesLoaded = false;
  let currentPaused = false;
  let isEmergencyStopped = false;

  txtConnIp.textContent = window.location.host;

  // 1. Tab Switching
  ctrlTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      ctrlTabs.forEach(t => t.classList.remove('active'));
      tabPanels.forEach(p => p.classList.remove('active'));
      tab.classList.add('active');
      const targetPanel = document.getElementById(`panel-${tab.dataset.tab}`);
      if (targetPanel) targetPanel.classList.add('active');
    });
  });

  // 2. View Mode Tabs (Combined, Camera, Mask)
  viewModeButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      viewModeButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      currentView = btn.dataset.view;
      roverStream.src = `/video_feed?view=${currentView}&t=${Date.now()}`;

      if (currentView === 'combined') {
        txtStreamDesc.textContent = 'Combined Digital Twin View';
      } else if (currentView === 'camera') {
        txtStreamDesc.textContent = 'Camera View (Crop Line Tracking)';
      } else {
        txtStreamDesc.textContent = 'Isolated Crop Plant Mask';
      }
    });
  });

  // 3. Remote Control POST Helper
  async function sendControl(payload) {
    try {
      const res = await fetch('/api/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      return await res.json();
    } catch (err) {
      console.error('Error sending control update:', err);
    }
  }

  function setInteracting() {
    isUserInteracting = true;
    clearTimeout(userInteractTimeout);
    userInteractTimeout = setTimeout(() => {
      isUserInteracting = false;
    }, 1200);
  }

  // --- SERIAL LINK & HARDWARE API CALLS ---

  async function loadSerialPorts() {
    try {
      const res = await fetch('/api/serial/ports');
      const data = await res.json();
      if (data.status === 'ok' && Array.isArray(data.ports)) {
        selectSerialPort.innerHTML = '';
        data.ports.forEach(p => {
          const opt = document.createElement('option');
          opt.value = p.port;
          opt.textContent = p.description;
          selectSerialPort.appendChild(opt);
        });
      }
    } catch (e) {
      console.error('Failed to load serial ports:', e);
    }
  }

  btnRefreshPorts.addEventListener('click', async (e) => {
    e.preventDefault();
    btnRefreshPorts.textContent = '⏳';
    await loadSerialPorts();
    btnRefreshPorts.textContent = '🔄';
  });

  btnSerialConnect.addEventListener('click', async () => {
    const customPort = inputCustomPort.value.trim();
    const port = customPort ? customPort : selectSerialPort.value;
    const baudrate = parseInt(selectBaudrate.value, 10);

    btnSerialConnect.disabled = true;
    btnSerialConnect.textContent = 'Connecting...';
    try {
      const res = await fetch('/api/serial/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port, baudrate })
      });
      const data = await res.json();
      if (data.status === 'ok') {
        alert(`Connected to ${port} @ ${baudrate} baud!`);
      } else {
        alert(`Serial note: ${data.message}`);
      }
    } catch (err) {
      alert(`Serial Connection Error: ${err}`);
    } finally {
      btnSerialConnect.disabled = false;
      btnSerialConnect.textContent = 'Connect Serial';
    }
  });

  btnSerialDisconnect.addEventListener('click', async () => {
    try {
      await fetch('/api/serial/disconnect', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  });

  // Save Motor Hardware Configuration
  async function saveHardwareConfig() {
    const payload = {
      pwm_min: parseInt(rangePwmMin.value, 10),
      pwm_max: parseInt(rangePwmMax.value, 10),
      invert_left: checkInvertLeft.checked,
      invert_right: checkInvertRight.checked,
      protocol: selectProtocol.value,
      motors_enabled: checkMotorsEnable.checked
    };
    try {
      const res = await fetch('/api/serial/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      if (data.status === 'ok') {
        alert('Hardware motor settings saved successfully!');
      }
    } catch (err) {
      alert('Error saving motor hardware config: ' + err);
    }
  }

  btnSaveHardwareCfg.addEventListener('click', saveHardwareConfig);
  checkMotorsEnable.addEventListener('change', () => {
    fetch('/api/serial/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ motors_enabled: checkMotorsEnable.checked })
    });
  });

  // Emergency Stop Handler
  async function handleEstop(action = 'trigger') {
    try {
      const res = await fetch('/api/serial/estop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action })
      });
      const data = await res.json();
      isEmergencyStopped = data.serial.emergency_stopped;
      updateEstopButtons(isEmergencyStopped);
    } catch (err) {
      console.error('E-Stop error:', err);
    }
  }

  function updateEstopButtons(stopped) {
    if (stopped) {
      btnNavEstop.innerHTML = '<span class="estop-icon">⚠️</span><span>RESET E-STOP</span>';
      btnNavEstop.classList.add('estop-active');
      btnEstopCard.textContent = '⚠️ E-STOP ACTIVE (CLICK TO RESET)';
      btnEstopCard.classList.add('estop-active');
    } else {
      btnNavEstop.innerHTML = '<span class="estop-icon">🛑</span><span>E-STOP</span>';
      btnNavEstop.classList.remove('estop-active');
      btnEstopCard.textContent = '🚨 EMERGENCY CUTOFF (E-STOP)';
      btnEstopCard.classList.remove('estop-active');
    }
  }

  btnNavEstop.addEventListener('click', () => {
    handleEstop(isEmergencyStopped ? 'reset' : 'trigger');
  });

  btnEstopCard.addEventListener('click', () => {
    handleEstop(isEmergencyStopped ? 'reset' : 'trigger');
  });

  // Benchtop D-Pad Test Drive Commands
  async function sendTestDrive(cmd) {
    try {
      await fetch('/api/serial/test_drive', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: cmd })
      });
    } catch (e) {
      console.error('Test drive error:', e);
    }
  }

  btnTestFwd.addEventListener('mousedown', () => sendTestDrive('forward'));
  btnTestRev.addEventListener('mousedown', () => sendTestDrive('reverse'));
  btnTestSpinL.addEventListener('mousedown', () => sendTestDrive('spin_left'));
  btnTestSpinR.addEventListener('mousedown', () => sendTestDrive('spin_right'));
  btnTestStop.addEventListener('click', () => sendTestDrive('stop'));

  // Mobile touch support for test buttons
  btnTestFwd.addEventListener('touchstart', (e) => { e.preventDefault(); sendTestDrive('forward'); });
  btnTestRev.addEventListener('touchstart', (e) => { e.preventDefault(); sendTestDrive('reverse'); });
  btnTestSpinL.addEventListener('touchstart', (e) => { e.preventDefault(); sendTestDrive('spin_left'); });
  btnTestSpinR.addEventListener('touchstart', (e) => { e.preventDefault(); sendTestDrive('spin_right'); });

  // On release, send stop
  ['mouseup', 'touchend', 'mouseleave'].forEach(evt => {
    btnTestFwd.addEventListener(evt, () => sendTestDrive('stop'));
    btnTestRev.addEventListener(evt, () => sendTestDrive('stop'));
    btnTestSpinL.addEventListener(evt, () => sendTestDrive('stop'));
    btnTestSpinR.addEventListener(evt, () => sendTestDrive('stop'));
  });

  // Video Device Selection
  selectDevice.addEventListener('change', async () => {
    const devId = selectDevice.value;
    if (devId) {
      await fetch('/api/device/select', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ device_id: devId })
      });
      roverStream.src = `/video_feed?view=${currentView}&t=${Date.now()}`;
    }
  });

  btnApplyCamera.addEventListener('click', async () => {
    const customCam = inputCustomCamera.value.trim();
    if (customCam) {
      await fetch('/api/device/select', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ device_id: customCam })
      });
      roverStream.src = `/video_feed?view=${currentView}&t=${Date.now()}`;
    }
  });

  // SBC Optimization Profile Switching
  selectSbcProfile.addEventListener('change', async () => {
    const profile = selectSbcProfile.value;
    await fetch('/api/sbc/profile', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ profile })
    });
  });

  // Quick Action Buttons
  btnPause.addEventListener('click', () => {
    sendControl({ paused: !currentPaused });
  });

  btnToggleLight.addEventListener('click', () => {
    sendControl({ use_adaptive: !checkAdaptive.checked });
  });

  btnSaveCfg.addEventListener('click', async () => {
    await sendControl({ save: true });
    alert('Vision calibration successfully saved to roi_config.json!');
  });

  btnResetDefaults.addEventListener('click', () => {
    if (confirm('Reset all ROI and PID parameters back to defaults?')) {
      sendControl({ reset: true });
    }
  });

  // Slider Input Listeners
  rangePwmMin.addEventListener('input', () => { valPwmMin.textContent = rangePwmMin.value; });
  rangePwmMax.addEventListener('input', () => { valPwmMax.textContent = rangePwmMax.value; });

  rangeKp.addEventListener('input', () => {
    setInteracting();
    valKp.textContent = (rangeKp.value / 100.0).toFixed(2);
    sendControl({ kp: parseInt(rangeKp.value) });
  });

  rangeKi.addEventListener('input', () => {
    setInteracting();
    valKi.textContent = (rangeKi.value / 100.0).toFixed(3);
    sendControl({ ki: parseInt(rangeKi.value) });
  });

  rangeKd.addEventListener('input', () => {
    setInteracting();
    valKd.textContent = (rangeKd.value / 100.0).toFixed(2);
    sendControl({ kd: parseInt(rangeKd.value) });
  });

  rangeBaseRpm.addEventListener('input', () => {
    setInteracting();
    valBaseRpm.textContent = `${rangeBaseRpm.value} RPM`;
    sendControl({ base_rpm: parseInt(rangeBaseRpm.value) });
  });

  rangeRoiTop.addEventListener('input', () => {
    setInteracting();
    valRoiTop.textContent = `${rangeRoiTop.value}%`;
    sendControl({ top_pct: parseInt(rangeRoiTop.value) });
  });

  rangeRoiBottom.addEventListener('input', () => {
    setInteracting();
    valRoiBottom.textContent = `${rangeRoiBottom.value}%`;
    sendControl({ bottom_pct: parseInt(rangeRoiBottom.value) });
  });

  rangeRoiLeft.addEventListener('input', () => {
    setInteracting();
    valRoiLeft.textContent = `${rangeRoiLeft.value}%`;
    sendControl({ left_pct: parseInt(rangeRoiLeft.value) });
  });

  rangeRoiRight.addEventListener('input', () => {
    setInteracting();
    valRoiRight.textContent = `${rangeRoiRight.value}%`;
    sendControl({ right_pct: parseInt(rangeRoiRight.value) });
  });

  rangeClahe.addEventListener('input', () => {
    setInteracting();
    valClahe.textContent = (rangeClahe.value / 10.0).toFixed(1);
    sendControl({ clahe_clip: parseFloat(rangeClahe.value / 10.0) });
  });

  checkAdaptive.addEventListener('change', () => {
    sendControl({ use_adaptive: checkAdaptive.checked });
  });

  // --- TELEMETRY POLLING LOOP ---

  async function pollTelemetry() {
    const t0 = performance.now();
    try {
      const res = await fetch('/api/telemetry');
      if (res.ok) {
        const pingMs = Math.round(performance.now() - t0);
        txtPing.textContent = `${pingMs} ms`;
        const data = await res.json();
        updateUI(data);
      }
    } catch (err) {
      console.warn('Telemetry fetch error:', err);
      txtPing.textContent = '-- ms';
    } finally {
      setTimeout(pollTelemetry, 100);
    }
  }

  function updateUI(d) {
    if (!d) return;

    // Header Badges
    txtVideoName.textContent = d.video || d.active_device || 'Live Feed';
    txtFps.textContent = d.fps.toFixed(1);

    // Status Beacon
    txtStatus.textContent = d.status.replace(/_/g, ' ');
    statusBeacon.className = 'badge';
    if (d.status === 'ROW_FOLLOWING') {
      statusBeacon.classList.add('badge-success');
    } else if (d.status === 'APPROACHING_EDGE') {
      statusBeacon.classList.add('badge-warn');
    } else {
      statusBeacon.classList.add('badge-danger');
    }

    // Serial Status Badge
    if (d.serial) {
      const ser = d.serial;
      txtSerialStatus.textContent = `SERIAL: ${ser.status_text} (${ser.port} @ ${ser.baudrate})`;
      serialDot.className = 'dot';
      statusPillSerial.textContent = ser.status_text;
      statusPillSerial.className = 'status-pill';

      if (ser.connected && !ser.simulated) {
        serialDot.classList.add('dot-connected');
        statusPillSerial.classList.add('pill-success');
      } else if (ser.simulated) {
        serialDot.classList.add('dot-simulated');
        statusPillSerial.classList.add('pill-warn');
      } else {
        serialDot.classList.add('dot-disconnected');
        statusPillSerial.classList.add('pill-danger');
      }

      txtPwmL.textContent = `PWM: ${ser.pwm_l}`;
      txtPwmR.textContent = `PWM: ${ser.pwm_r}`;
      txtSerialTx.textContent = ser.last_tx || '<0,0>';
      terminalPacketLog.textContent = ser.last_tx || '<0,0>';
      txtOverlayPwm.textContent = `L${ser.pwm_l} R${ser.pwm_r}`;

      isEmergencyStopped = ser.emergency_stopped;
      updateEstopButtons(isEmergencyStopped);
    }

    // Populate Available Devices Dropdown
    if (!availableDevicesLoaded && d.available_devices && d.available_devices.length > 0) {
      selectDevice.innerHTML = '';
      d.available_devices.forEach(dev => {
        const opt = document.createElement('option');
        opt.value = dev.id;
        opt.textContent = dev.name;
        if (dev.id === d.active_device) opt.selected = true;
        selectDevice.appendChild(opt);
      });
      availableDevicesLoaded = true;
    }

    // SBC Profile sync
    if (d.sbc) {
      txtFooterSbc.textContent = d.sbc.name;
      if (selectSbcProfile.value !== d.sbc.profile) {
        selectSbcProfile.value = d.sbc.profile;
      }
    }

    // Overlay
    txtOverlayErr.textContent = `${d.error_px > 0 ? '+' : ''}${d.error_px}px`;
    txtOverlayAct.textContent = d.decision;

    // Action Banner
    txtDecision.textContent = d.decision;
    txtErrorBanner.textContent = `Offset Error: ${d.error_px > 0 ? '+' : ''}${d.error_px} px`;
    actionBanner.className = 'action-banner';
    if (d.decision.includes('LEFT')) {
      actionBanner.classList.add('action-left');
    } else if (d.decision.includes('RIGHT')) {
      actionBanner.classList.add('action-right');
    } else if (d.decision.includes('TURN')) {
      actionBanner.classList.add('action-turn');
    } else {
      actionBanner.classList.add('action-straight');
    }

    // Track RPM Bars
    txtRpmL.textContent = `${d.speed_l} RPM`;
    txtRpmR.textContent = `${d.speed_r} RPM`;
    const pctL = Math.max(0, Math.min(100, (d.speed_l / 160) * 100));
    const pctR = Math.max(0, Math.min(100, (d.speed_r / 160) * 100));
    barRpmL.style.width = `${pctL}%`;
    barRpmR.style.width = `${pctR}%`;

    // PID Balance Meter
    txtPidNet.textContent = `Net: ${d.pid.output > 0 ? '+' : ''}${d.pid.output.toFixed(1)} RPM`;
    txtTermP.textContent = `${d.pid.p_term > 0 ? '+' : ''}${d.pid.p_term.toFixed(1)}`;
    txtTermI.textContent = `${d.pid.i_term > 0 ? '+' : ''}${d.pid.i_term.toFixed(1)}`;
    txtTermD.textContent = `${d.pid.d_term > 0 ? '+' : ''}${d.pid.d_term.toFixed(1)}`;

    const maxDeflect = 60.0;
    const ratio = Math.max(-1.0, Math.min(1.0, d.pid.output / maxDeflect));
    const pointerPos = 50 + (ratio * 50);
    balancePointer.style.left = `${pointerPos}%`;

    if (ratio >= 0) {
      balanceFill.style.left = '50%';
      balanceFill.style.width = `${(ratio * 50)}%`;
      balanceFill.style.backgroundColor = '#ffd600';
    } else {
      balanceFill.style.left = `${pointerPos}%`;
      balanceFill.style.width = `${Math.abs(ratio * 50)}%`;
      balanceFill.style.backgroundColor = '#00e5ff';
    }

    // Environment & Crop Density
    const dens = d.edge.density;
    txtDensity.textContent = `${dens.toFixed(1)}%`;
    barDensity.style.width = `${Math.min(100, (dens / 15.0) * 100)}%`;
    barDensity.style.backgroundColor = dens < 2.0 ? '#ff5252' : (dens < 6.0 ? '#ffb300' : '#00e676');

    txtAmbient.textContent = `${d.lighting.ambient.toFixed(0)} LUX`;
    txtLightMode.textContent = `${d.lighting.mode} (Clip: ${d.lighting.clahe_clip.toFixed(1)})`;
    txtLightMode.className = `env-val ${d.lighting.mode === 'ADAPTIVE' ? 'text-success' : 'text-warn'}`;
    txtDynThresh.textContent = `S_min: ${d.lighting.s_min} | V_min: ${d.lighting.v_min}`;

    if (txtTillerStatus && d.tiller) {
      txtTillerStatus.textContent = d.tiller.active ? `ACTIVE (${d.tiller.rpm} RPM)` : 'IDLE';
      txtTillerStatus.className = `env-val ${d.tiller.active ? 'text-success' : 'text-warn'}`;
    }

    // Sync sliders if user isn't actively sliding them
    if (!isUserInteracting) {
      valKp.textContent = d.pid.kp.toFixed(2);
      rangeKp.value = Math.round(d.pid.kp * 100);

      valKi.textContent = d.pid.ki.toFixed(3);
      rangeKi.value = Math.round(d.pid.ki * 100);

      valKd.textContent = d.pid.kd.toFixed(2);
      rangeKd.value = Math.round(d.pid.kd * 100);

      valBaseRpm.textContent = `${d.base_rpm} RPM`;
      rangeBaseRpm.value = d.base_rpm;

      valRoiTop.textContent = `${d.roi.top}%`;
      rangeRoiTop.value = d.roi.top;

      valRoiBottom.textContent = `${d.roi.bottom}%`;
      rangeRoiBottom.value = d.roi.bottom;

      valRoiLeft.textContent = `${d.roi.left}%`;
      rangeRoiLeft.value = d.roi.left;

      valRoiRight.textContent = `${d.roi.right}%`;
      rangeRoiRight.value = d.roi.right;

      valClahe.textContent = d.lighting.clahe_clip.toFixed(1);
      rangeClahe.value = Math.round(d.lighting.clahe_clip * 10);

      checkAdaptive.checked = (d.lighting.mode === 'ADAPTIVE');
      currentPaused = d.paused;
      txtBtnPause.textContent = currentPaused ? 'Resume' : 'Pause';
      btnPause.classList.toggle('btn-primary', currentPaused);
    }
  }

  // Initial loads
  loadSerialPorts();
  pollTelemetry();
});
