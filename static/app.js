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
  
  // --- Master RUN / STOP Tracking Controls ---
  const btnNavRun = document.getElementById('btn-nav-run');
  const iconNavRun = document.getElementById('icon-nav-run');
  const txtNavRun = document.getElementById('txt-nav-run');
  const btnQuickRun = document.getElementById('btn-quick-run');
  const iconQuickRun = document.getElementById('icon-quick-run');
  const txtQuickRun = document.getElementById('txt-quick-run');

  // --- Video Stream Viewport ---
  const roverStream = document.getElementById('rover-stream');
  const txtStreamDesc = document.getElementById('txt-stream-desc');
  const txtOverlayState = document.getElementById('txt-overlay-state');
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
  const checkAutoConnect = document.getElementById('check-auto-connect');
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

  // --- Manual Teleoperation Cockpit Elements ---
  const btnQuickManual = document.getElementById('btn-quick-manual');
  const tabBtnManual = document.getElementById('tab-btn-manual');
  const badgeManualStatus = document.getElementById('badge-manual-status');
  const badgeThrottlePct = document.getElementById('badge-throttle-pct');
  const rangeManualThrottle = document.getElementById('range-manual-throttle');
  const valManualThrottle = document.getElementById('val-manual-throttle');
  const btnCockpitDirs = document.querySelectorAll('.btn-cockpit-dir');
  const btnSpeedPresets = document.querySelectorAll('.btn-speed-preset');
  const badgeManualActiveCmd = document.getElementById('badge-manual-active-cmd');
  const vectorArrowCircle = document.getElementById('vector-arrow-circle');
  const vectorIcon = document.getElementById('vector-icon');
  const txtVectorState = document.getElementById('txt-vector-state');
  const txtVectorSub = document.getElementById('txt-vector-sub');
  const meterManualL = document.getElementById('meter-manual-l');
  const meterManualR = document.getElementById('meter-manual-r');
  const valManualPwmL = document.getElementById('val-manual-pwm-l');
  const valManualPwmR = document.getElementById('val-manual-pwm-r');
  const txtManualPacketLog = document.getElementById('txt-manual-packet-log');
  const btnManualEstop = document.getElementById('btn-manual-estop');

  // Benchtop D-Pad Test Buttons (Fallback)
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
  let isTrackingActive = false;

  // --- Tracking State Toggle Functions ---
  async function handleToggleTracking() {
    try {
      const res = await fetch('/api/tracking/toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      });
      const data = await res.json();
      if (data.status === 'ok') {
        isTrackingActive = Boolean(data.tracking_enabled);
        updateTrackingButtons(isTrackingActive);
      }
    } catch (err) {
      console.error('Failed to toggle tracking state:', err);
    }
  }

  function updateTrackingButtons(active) {
    isTrackingActive = Boolean(active);
    if (isTrackingActive) {
      if (btnNavRun) {
        btnNavRun.className = 'btn-run-master btn-run-active';
        if (iconNavRun) iconNavRun.textContent = '⏹';
        if (txtNavRun) txtNavRun.textContent = 'STOP TRACKING';
      }
      if (btnQuickRun) {
        btnQuickRun.className = 'btn btn-run-action btn-run-active';
        if (iconQuickRun) iconQuickRun.textContent = '⏹';
        if (txtQuickRun) txtQuickRun.textContent = 'STOP TRACKING';
      }
      if (txtOverlayState) {
        txtOverlayState.textContent = 'TRACKING ACTIVE';
        txtOverlayState.className = 'mono text-success';
      }
    } else {
      if (btnNavRun) {
        btnNavRun.className = 'btn-run-master btn-run-standby';
        if (iconNavRun) iconNavRun.textContent = '▶';
        if (txtNavRun) txtNavRun.textContent = 'RUN TRACKING';
      }
      if (btnQuickRun) {
        btnQuickRun.className = 'btn btn-run-action btn-run-standby';
        if (iconQuickRun) iconQuickRun.textContent = '▶';
        if (txtQuickRun) txtQuickRun.textContent = 'RUN TRACKING';
      }
      if (txtOverlayState) {
        txtOverlayState.textContent = 'STANDBY (MOTORS OFF)';
        txtOverlayState.className = 'mono text-warn';
      }
    }
  }

  if (btnNavRun) btnNavRun.addEventListener('click', handleToggleTracking);
  if (btnQuickRun) btnQuickRun.addEventListener('click', handleToggleTracking);

  // Keyboard Shortcuts: Space or 'R' toggles RUN/STOP, 'E' triggers E-STOP
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;
    if (e.code === 'Space' || e.key === 'r' || e.key === 'R') {
      e.preventDefault();
      handleToggleTracking();
    } else if (e.key === 'e' || e.key === 'E') {
      e.preventDefault();
      handleEstop(isEmergencyStopped ? 'reset' : 'trigger');
    }
  });

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
      motors_enabled: checkMotorsEnable.checked,
      auto_connect: checkAutoConnect ? checkAutoConnect.checked : true
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

  if (checkAutoConnect) {
    checkAutoConnect.addEventListener('change', () => {
      fetch('/api/serial/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ auto_connect: checkAutoConnect.checked })
      });
    });
  }

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

  // ==============================================================================
  // FULL MANUAL TELEOPERATION COCKPIT CONTROLLER
  // ==============================================================================
  let currentManualThrottle = 60;
  let activeManualCmd = 'stop';
  let activeDriveKey = null;

  function updateThrottleUI(val) {
    if (valManualThrottle) {
      const approxPwm = Math.round(35 + (val / 100.0) * (255 - 35));
      valManualThrottle.textContent = `${val}% (~${approxPwm} PWM)`;
    }
    if (badgeThrottlePct) badgeThrottlePct.textContent = `${val}% POWER`;
    btnSpeedPresets.forEach(btn => {
      btn.classList.toggle('active', parseInt(btn.dataset.speed, 10) === val);
    });
  }

  if (rangeManualThrottle) {
    rangeManualThrottle.addEventListener('input', () => {
      currentManualThrottle = parseInt(rangeManualThrottle.value, 10);
      updateThrottleUI(currentManualThrottle);
    });
  }

  btnSpeedPresets.forEach(btn => {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      const spd = parseInt(btn.dataset.speed, 10);
      currentManualThrottle = spd;
      if (rangeManualThrottle) rangeManualThrottle.value = spd;
      updateThrottleUI(spd);
    });
  });

  if (btnQuickManual && tabBtnManual) {
    btnQuickManual.addEventListener('click', (e) => {
      e.preventDefault();
      tabBtnManual.click();
      tabBtnManual.scrollIntoView({ behavior: 'smooth' });
    });
  }

  const dirVisuals = {
    'forward':    { icon: '⬆️', desc: 'DRIVING FORWARD', isMoving: true },
    'reverse':    { icon: '⬇️', desc: 'DRIVING REVERSE', isMoving: true },
    'spin_left':  { icon: '↩️', desc: 'PIVOT SPIN LEFT', isMoving: true },
    'spin_right': { icon: '↪️', desc: 'PIVOT SPIN RIGHT', isMoving: true },
    'turn_left':  { icon: '↖️', desc: 'ARC TURN LEFT', isMoving: true },
    'turn_right': { icon: '↗️', desc: 'ARC TURN RIGHT', isMoving: true },
    'rev_left':   { icon: '↙️', desc: 'REVERSE ARC LEFT', isMoving: true },
    'rev_right':  { icon: '↘️', desc: 'REVERSE ARC RIGHT', isMoving: true },
    'stop':       { icon: '⏹️', desc: 'STANDBY (HOLD TO DRIVE)', isMoving: false }
  };

  async function sendManualDrive(cmd) {
    activeManualCmd = cmd;
    const vis = dirVisuals[cmd] || dirVisuals['stop'];

    if (vectorIcon) vectorIcon.textContent = vis.icon;
    if (txtVectorState) txtVectorState.textContent = vis.desc;
    if (vectorArrowCircle) {
      if (vis.isMoving) vectorArrowCircle.classList.add('active-vector');
      else vectorArrowCircle.classList.remove('active-vector');
    }
    if (badgeManualActiveCmd) {
      badgeManualActiveCmd.textContent = cmd.toUpperCase().replace('_', ' ');
      badgeManualActiveCmd.className = vis.isMoving ? 'status-pill pill-success' : 'status-pill pill-warn';
    }

    try {
      const res = await fetch('/api/serial/manual_drive', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          command: cmd,
          throttle: currentManualThrottle
        })
      });
      const data = await res.json();
      if (data.status === 'ok') {
        if (txtManualPacketLog) txtManualPacketLog.textContent = data.packet || '<0,0>';
        if (valManualPwmL) valManualPwmL.textContent = data.pwm_l;
        if (valManualPwmR) valManualPwmR.textContent = data.pwm_r;
        if (meterManualL) meterManualL.style.width = `${Math.min(100, Math.abs(data.pwm_l) / 2.55)}%`;
        if (meterManualR) meterManualR.style.width = `${Math.min(100, Math.abs(data.pwm_r) / 2.55)}%`;
      }
    } catch (e) {
      console.error('Manual drive transmission error:', e);
    }
  }

  // D-Pad Direction Button Listeners (Mouse + Touch + Mobile Dead-Man Safety)
  btnCockpitDirs.forEach(btn => {
    const cmd = btn.dataset.cmd;
    const onStart = (e) => {
      if (e.cancelable) e.preventDefault();
      btn.classList.add('active');
      sendManualDrive(cmd);
    };
    const onEnd = (e) => {
      btn.classList.remove('active');
      if (activeManualCmd === cmd) {
        sendManualDrive('stop');
      }
    };

    btn.addEventListener('mousedown', onStart);
    btn.addEventListener('touchstart', onStart, { passive: false });

    btn.addEventListener('mouseup', onEnd);
    btn.addEventListener('mouseleave', onEnd);
    btn.addEventListener('touchend', onEnd);
    btn.addEventListener('touchcancel', onEnd);
  });

  // Global Keyboard Teleoperation Driving
  const driveKeyMap = {
    'KeyW':        { cmd: 'forward',    btnId: 'btn-drive-fwd',     kbdId: 'kbd-w' },
    'ArrowUp':     { cmd: 'forward',    btnId: 'btn-drive-fwd',     kbdId: 'kbd-up' },
    'KeyS':        { cmd: 'reverse',    btnId: 'btn-drive-rev',     kbdId: 'kbd-s' },
    'ArrowDown':   { cmd: 'reverse',    btnId: 'btn-drive-rev',     kbdId: 'kbd-down' },
    'KeyA':        { cmd: 'spin_left',  btnId: 'btn-drive-spin-l',   kbdId: 'kbd-a' },
    'ArrowLeft':   { cmd: 'spin_left',  btnId: 'btn-drive-spin-l',   kbdId: 'kbd-left' },
    'KeyD':        { cmd: 'spin_right', btnId: 'btn-drive-spin-r',   kbdId: 'kbd-d' },
    'ArrowRight':  { cmd: 'spin_right', btnId: 'btn-drive-spin-r',   kbdId: 'kbd-right' },
    'KeyQ':        { cmd: 'turn_left',  btnId: 'btn-drive-turn-l',   kbdId: 'kbd-q' },
    'KeyE':        { cmd: 'turn_right', btnId: 'btn-drive-turn-r',   kbdId: 'kbd-e' },
    'Space':       { cmd: 'stop',       btnId: 'btn-drive-stop',     kbdId: 'kbd-space' }
  };

  window.addEventListener('keydown', (e) => {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;

    const entry = driveKeyMap[e.code];
    if (entry && !e.repeat) {
      if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) {
        e.preventDefault();
      }
      activeDriveKey = e.code;
      const btn = document.getElementById(entry.btnId);
      if (btn) btn.classList.add('active');
      const kbd = document.getElementById(entry.kbdId);
      if (kbd) kbd.classList.add('active-key');

      sendManualDrive(entry.cmd);
    }
  });

  window.addEventListener('keyup', (e) => {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;

    const entry = driveKeyMap[e.code];
    if (entry) {
      const btn = document.getElementById(entry.btnId);
      if (btn) btn.classList.remove('active');
      const kbd = document.getElementById(entry.kbdId);
      if (kbd) kbd.classList.remove('active-key');

      if (activeDriveKey === e.code) {
        activeDriveKey = null;
        sendManualDrive('stop');
      }
    }
  });

  if (btnManualEstop) {
    btnManualEstop.addEventListener('click', () => {
      handleEstop(isEmergencyStopped ? 'reset' : 'trigger');
    });
  }

  // Fallback benchtop button events (if present in Hardware tab)
  if (btnTestFwd) btnTestFwd.addEventListener('mousedown', () => sendManualDrive('forward'));
  if (btnTestRev) btnTestRev.addEventListener('mousedown', () => sendManualDrive('reverse'));
  if (btnTestSpinL) btnTestSpinL.addEventListener('mousedown', () => sendManualDrive('spin_left'));
  if (btnTestSpinR) btnTestSpinR.addEventListener('mousedown', () => sendManualDrive('spin_right'));
  if (btnTestStop) btnTestStop.addEventListener('click', () => sendManualDrive('stop'));
  ['mouseup', 'touchend', 'mouseleave'].forEach(evt => {
    if (btnTestFwd) btnTestFwd.addEventListener(evt, () => sendManualDrive('stop'));
    if (btnTestRev) btnTestRev.addEventListener(evt, () => sendManualDrive('stop'));
    if (btnTestSpinL) btnTestSpinL.addEventListener(evt, () => sendManualDrive('stop'));
    if (btnTestSpinR) btnTestSpinR.addEventListener(evt, () => sendManualDrive('stop'));
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

    // Status Beacon & Tracking State
    if (d.tracking_enabled === false) {
      txtStatus.textContent = 'STANDBY (MOTORS OFF)';
      statusBeacon.className = 'badge badge-warn';
    } else {
      txtStatus.textContent = d.status.replace(/_/g, ' ');
      statusBeacon.className = 'badge';
      if (d.status === 'ROW_FOLLOWING') {
        statusBeacon.classList.add('badge-success');
      } else if (d.status === 'APPROACHING_EDGE') {
        statusBeacon.classList.add('badge-warn');
      } else {
        statusBeacon.classList.add('badge-danger');
      }
    }

    // Keep Master RUN buttons synchronized with rover telemetry state
    updateTrackingButtons(Boolean(d.tracking_enabled));

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
      if (ser.last_rx) {
        terminalPacketLog.textContent = `TX: ${ser.last_tx || '<0,0>'}  |  RX: ${ser.last_rx}`;
      } else {
        terminalPacketLog.textContent = ser.last_tx || '<0,0>';
      }
      txtOverlayPwm.textContent = `L${ser.pwm_l} R${ser.pwm_r}`;

      // Update Manual Cockpit Power Gauges & Packet Echo
      if (valManualPwmL) valManualPwmL.textContent = ser.pwm_l;
      if (valManualPwmR) valManualPwmR.textContent = ser.pwm_r;
      if (meterManualL) meterManualL.style.width = `${Math.min(100, Math.abs(ser.pwm_l) / 2.55)}%`;
      if (meterManualR) meterManualR.style.width = `${Math.min(100, Math.abs(ser.pwm_r) / 2.55)}%`;
      if (txtManualPacketLog) txtManualPacketLog.textContent = ser.last_tx || '<0,0>';

      if (d.manual && d.manual.active) {
        if (badgeManualActiveCmd) {
          badgeManualActiveCmd.textContent = d.manual.command.toUpperCase().replace('_', ' ');
          badgeManualActiveCmd.className = 'status-pill pill-success';
        }
      } else if (activeManualCmd === 'stop') {
        if (badgeManualActiveCmd) {
          badgeManualActiveCmd.textContent = 'STANDBY';
          badgeManualActiveCmd.className = 'status-pill pill-warn';
        }
        if (vectorIcon) vectorIcon.textContent = '⏹️';
        if (txtVectorState) txtVectorState.textContent = 'STANDBY (HOLD KEY/BUTTON TO DRIVE)';
        if (vectorArrowCircle) vectorArrowCircle.classList.remove('active-vector');
      }

      isEmergencyStopped = ser.emergency_stopped;
      updateEstopButtons(isEmergencyStopped);

      if (checkAutoConnect && ser.auto_connect !== undefined && !checkAutoConnect.matches(':active')) {
        checkAutoConnect.checked = !!ser.auto_connect;
      }
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
