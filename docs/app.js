/**
 * Motor Control Dashboard - JavaScript Handler
 * 
 * This script handles communication with the STM32 motor controller
 * via USB CDC (serial) connection. It receives velocity telemetry
 * data in JSON format and sends control commands.
 * 
 * Data format from MCU:
 *   - Telemetry: {"vel":1.23,"target":0.40}
 *   - Commands sent: START, STOP, SET_VELOCITY:value, SET_PID:Kp,Ki,Kd
 */

// Global state
/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

const state = {
    port: null,
    reader: null,
    isConnected: false,
    inputBuffer: '',
    telemetryPollEnabled: true,
    telemetryPollIntervalMs: 200,
    telemetryPollTimer: null,
    telemetryPollInFlight: false,
    telemetryPollTimeout: null,
    telemetryPollQuietUntil: 0,
    telemetryDiagPollDivider: 40,
    telemetryPollCount: 0,
    lastDiagCallbackCount: null,
    lastDiagCallbackTimestamp: 0,
    velocityHistory: [],
    velocityHistoryLimit: 120,
    positionHistory: [],
    positionHistoryLimit: 120,
    targetVelocityHistory: [],
    targetVelocityHistoryLimit: 120,
    targetPositionHistory: [],
    targetPositionHistoryLimit: 120,
    rawAngleHistory: [],
    rawAngleHistoryLimit: 120,
    rawMechanicalHistory: [],
    rawMechanicalHistoryLimit: 120,
    previousMechanicalHistory: [],
    previousMechanicalHistoryLimit: 120,
    mechanicalDeltaHistory: [],
    mechanicalDeltaHistoryLimit: 120,
    previousMultiTurnHistory: [],
    previousMultiTurnHistoryLimit: 120,
    traceMode: 'velocity',
    autotuneResult: null,
    theme: 'light',
    velocitySliderTimer: null,
    positionSliderTimer: null,
    motionProfileWritePendingUntil: 0,
    debugConsoleLines: [],
    seenFirmwareTag: null
};

const THEME_STORAGE_KEY = 'bluepill_foc_theme';

function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function markDirty(input) {
    if (input) {
        input.dataset.dirty = '1';
    }
}

function clearDirty(input) {
    if (input) {
        delete input.dataset.dirty;
    }
}

function motionProfileInputLocked(input) {
    return !!input && (document.activeElement === input || input.dataset.dirty === '1' || Date.now() < state.motionProfileWritePendingUntil);
}

function appendDebugConsole(message, type = 'sys') {
    const frame = document.getElementById('debugConsole');
    const now = new Date();
    const stamp = now.toLocaleTimeString();
    const line = `[${stamp}] ${message}`;
    state.debugConsoleLines.push({ line, type });
    if (state.debugConsoleLines.length > 250) {
        state.debugConsoleLines.shift();
    }
    if (!frame) {
        return;
    }
    const row = document.createElement('div');
    row.className = `debug-console-line debug-console-${type}`;
    row.textContent = line;
    frame.appendChild(row);
    while (frame.childElementCount > 250) {
        frame.removeChild(frame.firstElementChild);
    }
    frame.scrollTop = frame.scrollHeight;
}

function clearDebugConsole() {
    state.debugConsoleLines = [];
    const frame = document.getElementById('debugConsole');
    if (frame) {
        frame.innerHTML = '';
    }
}

function applyTheme(theme) {
    const nextTheme = theme === 'dark' ? 'dark' : 'light';
    state.theme = nextTheme;
    document.documentElement.setAttribute('data-theme', nextTheme);
    try {
        localStorage.setItem(THEME_STORAGE_KEY, nextTheme);
    } catch (error) {
        console.warn('Failed to persist theme:', error);
    }
    const themeBtn = document.getElementById('themeToggleBtn');
    if (themeBtn) {
        themeBtn.textContent = nextTheme === 'dark' ? 'Light Theme' : 'Dark Theme';
    }
}

function loadStoredTheme() {
    try {
        const stored = localStorage.getItem(THEME_STORAGE_KEY);
        if (stored === 'dark' || stored === 'light') {
            return stored;
        }
    } catch (error) {
        console.warn('Failed to load stored theme:', error);
    }
    return 'light';
}

function toggleTheme() {
    applyTheme(state.theme === 'dark' ? 'light' : 'dark');
}

function webSerialSupported() {
    return typeof navigator !== 'undefined' &&
        !!navigator.serial &&
        typeof navigator.serial.requestPort === 'function';
}

function setDiagnosticValue(id, text) {
    const node = document.getElementById(id);
    if (node) {
        node.textContent = text;
    }
}

function updateBrowserDiagnostics() {
    const secure = window.isSecureContext ? 'Yes' : 'No';
    const serial = webSerialSupported()
        ? 'Available'
        : 'Unavailable. Use desktop Chrome or Edge over HTTPS.';
    const origin = `${window.location.protocol}//${window.location.host}${window.location.pathname}`;
    const ua = typeof navigator !== 'undefined' && navigator.userAgent
        ? navigator.userAgent
        : 'Unavailable';

    setDiagnosticValue('diagnosticSecure', secure);
    setDiagnosticValue('diagnosticSerial', serial);
    setDiagnosticValue('diagnosticOrigin', origin);
    setDiagnosticValue('diagnosticBrowser', ua);
}

/**
 * Initialize the Web Serial API connection
 */
async function connectSerialPort() {
    if (!webSerialSupported()) {
        const message = 'Web Serial is not available in this browser. Use Chrome or Edge on desktop over HTTPS.';
        console.error(message);
        appendDebugConsole(`ERR ${message}`, 'err');
        alert(message);
        updateConnectionStatus(false);
        return;
    }

    try {
        // Request a port from the user
        state.port = await navigator.serial.requestPort();

        // Open the port at 115200 baud (or adjust to your MCU's baud rate)
        await state.port.open({ baudRate: 115200 });

        state.isConnected = true;
        state.telemetryPollCount = 0;
        state.lastDiagCallbackCount = null;
        state.lastDiagCallbackTimestamp = 0;
        state.velocityHistory = [];
        state.positionHistory = [];
        state.targetVelocityHistory = [];
        state.targetPositionHistory = [];
        state.rawAngleHistory = [];
        state.rawMechanicalHistory = [];
        state.previousMechanicalHistory = [];
        state.mechanicalDeltaHistory = [];
        state.previousMultiTurnHistory = [];
        updateConnectionStatus(true);
        console.log('Connected to serial port');
        appendDebugConsole('SYS Connected to serial port', 'sys');

        // Start reading data
        readSerialData();
        await sendCommand('GET_MODULATION');
        await delay(40);
        await getPID();
        await delay(40);
        await getPositionPID();
        await delay(40);
        await getFeedforward();
        await delay(40);
        await getLowSpeedBias();
        await delay(40);
        await getMotionProfile();
        await delay(40);
        await getPositionTorqueAssist();
        startTelemetryPolling();
        refreshTelemetryPollingUI();
    } catch (error) {
        console.error('Failed to connect:', error);
        appendDebugConsole(`ERR Connect failed: ${error}`, 'err');
        updateConnectionStatus(false);
    }
}

function startTelemetryPolling() {
    stopTelemetryPolling();
    if (!state.telemetryPollEnabled || !state.isConnected) {
        refreshTelemetryPollingUI();
        return;
    }
    state.telemetryPollInFlight = false;
    pollTelemetry();
    state.telemetryPollTimer = setInterval(pollTelemetry, state.telemetryPollIntervalMs);
    refreshTelemetryPollingUI();
}

function stopTelemetryPolling() {
    if (state.telemetryPollTimer) {
        clearInterval(state.telemetryPollTimer);
        state.telemetryPollTimer = null;
    }
    if (state.telemetryPollTimeout) {
        clearTimeout(state.telemetryPollTimeout);
        state.telemetryPollTimeout = null;
    }
    state.telemetryPollInFlight = false;
    refreshTelemetryPollingUI();
}

async function pollTelemetry() {
    if (!state.telemetryPollEnabled || !state.isConnected || state.telemetryPollInFlight || Date.now() < state.telemetryPollQuietUntil) {
        return;
    }

    state.telemetryPollCount += 1;
    const command = (state.telemetryPollCount % state.telemetryDiagPollDivider === 0) ? 'GET_DIAG' : 'GET_TELEMETRY';
    state.telemetryPollInFlight = true;
    state.telemetryPollTimeout = setTimeout(() => {
        state.telemetryPollInFlight = false;
        state.telemetryPollTimeout = null;
    }, 1000);
    try {
        await sendCommand(command, { silent: true });
    } catch (error) {
        console.error('Telemetry poll failed:', error);
        state.telemetryPollInFlight = false;
        if (state.telemetryPollTimeout) {
            clearTimeout(state.telemetryPollTimeout);
            state.telemetryPollTimeout = null;
        }
    }
}

function refreshTelemetryPollingUI() {
    const toggleBtn = document.getElementById('pollingToggleBtn');
    const stateLabel = document.getElementById('pollingStateLabel');
    const slider = document.getElementById('pollingSlider');
    const intervalValue = document.getElementById('pollingIntervalValue');

    if (slider) {
        slider.value = String(state.telemetryPollIntervalMs);
        slider.disabled = !state.telemetryPollEnabled;
    }

    if (intervalValue) {
        intervalValue.textContent = `${state.telemetryPollIntervalMs} ms`;
    }

    if (!toggleBtn || !stateLabel) {
        return;
    }

    toggleBtn.classList.remove('polling-live', 'polling-paused');

    if (state.telemetryPollEnabled) {
        toggleBtn.textContent = '⏸';
        toggleBtn.classList.add('polling-live');
        stateLabel.textContent = state.isConnected
            ? `Live at ${state.telemetryPollIntervalMs} ms`
            : `Ready at ${state.telemetryPollIntervalMs} ms`;
    } else {
        toggleBtn.textContent = '▶';
        toggleBtn.classList.add('polling-paused');
        stateLabel.textContent = state.isConnected ? 'Paused' : 'Disabled';
    }
}

function updateTelemetryPollingInterval(value) {
    const intervalMs = Math.max(100, Math.min(2000, parseInt(value, 10) || 200));
    state.telemetryPollIntervalMs = intervalMs;
    refreshTelemetryPollingUI();

    if (state.isConnected && state.telemetryPollEnabled) {
        startTelemetryPolling();
    }
}

function toggleTelemetryPolling() {
    state.telemetryPollEnabled = !state.telemetryPollEnabled;

    if (state.telemetryPollEnabled) {
        startTelemetryPolling();
    } else {
        stopTelemetryPolling();
    }
}

/**
 * Read data continuously from the serial port
 */
async function readSerialData() {
    try {
        const textDecoder = new TextDecoderStream();
        state.reader = state.port.readable.pipeThrough(textDecoder).getReader();

        while (state.isConnected) {
            const { value, done } = await state.reader.read();

            if (done) {
                break;
            }

            // Accumulate data in buffer
            state.inputBuffer += value;

            // Process complete lines (delimited by \n)
            processInputBuffer();
        }
    } catch (error) {
        console.error('Read error:', error);
        disconnectSerialPort();
    }
}

/**
 * Process accumulated buffer data for complete JSON messages
 */
function processInputBuffer() {
    const lines = state.inputBuffer.split('\n');

    // Keep the last incomplete line in the buffer
    state.inputBuffer = lines[lines.length - 1];

    // Process all complete lines
    for (let i = 0; i < lines.length - 1; i++) {
        const line = lines[i].trim();

        if (line.length === 0) continue;

        try {
            // Try to parse as JSON
            if (line.startsWith('{')) {
                const data = JSON.parse(line);

                // Check for velocity telemetry
                if (data.vel !== undefined) {
                    updateVelocityDisplay(data);
                }
                // Check for status messages
                else if (data.status !== undefined) {
                    console.log('MCU Status:', data.status);
                    appendDebugConsole(`RX ${line}`, 'rx');
                }
                // Check for alignment offset calibration
                else if (data.align_offset !== undefined) {
                    updateAlignmentDisplay(data);
                }
                // Check for PID parameters
                else if (data.pid !== undefined) {
                    updatePIDDisplay(data);
                }
                else if (data.position_pid !== undefined) {
                    updatePositionPIDDisplay(data);
                }
                // Check for target velocity confirmation
                else if (data.target_vel !== undefined) {
                    updateTargetVelocityDisplay(data);
                }
                else if (data.target_pos !== undefined || data.pos !== undefined) {
                    updatePositionDisplay(data);
                }
                // Check for FOC debug information
                else if (data.foc !== undefined) {
                    const focTelemetry = normalizeFocTelemetry(data.foc);
                    if (focTelemetry.fw !== undefined) {
                        if (state.seenFirmwareTag !== focTelemetry.fw) {
                            state.seenFirmwareTag = focTelemetry.fw;
                            appendDebugConsole(`SYS Firmware ${focTelemetry.fw}`, 'sys');
                        }
                    }
                    updateVelocityDisplay(focTelemetry);
                    updateFOCDebugDisplay(focTelemetry);
                    updateMotionProfileFromTelemetry(focTelemetry);
                    if (state.telemetryPollTimeout) {
                        clearTimeout(state.telemetryPollTimeout);
                        state.telemetryPollTimeout = null;
                    }
                    state.telemetryPollInFlight = false;
                }
                // Check for diagnostic (loop count, message status)
                else if (data.diag !== undefined) {
                    appendDebugConsole(`RX ${line}`, 'rx');
                    updateDiagnosticDisplay(data.diag);
                    if (state.telemetryPollTimeout) {
                        clearTimeout(state.telemetryPollTimeout);
                        state.telemetryPollTimeout = null;
                    }
                    state.telemetryPollInFlight = false;
                }
                else if (data.open_loop !== undefined) {
                    updateOpenLoopDisplay(data.open_loop);
                }
                else if (data.encoder_fault !== undefined) {
                    console.warn('Encoder fault:', data);
                    document.getElementById('focStatus').textContent = 'Encoder Fault';
                }
                else if (data.phase_map !== undefined) {
                    updatePhaseMapDisplay(data.phase_map);
                }
                else if (data.vector_test !== undefined) {
                    updateVectorTestDisplay(data.vector_test);
                }
                else if (data.modulation !== undefined) {
                    updateModulationDisplay(data.modulation);
                }
                else if (data.feedforward !== undefined) {
                    updateFeedforwardDisplay(data.feedforward);
                }
                else if (data.low_speed_bias !== undefined) {
                    updateLowSpeedBiasDisplay(data.low_speed_bias);
                }
                else if (data.velocity_ramp !== undefined) {
                    updateVelocityRampDisplay(data.velocity_ramp);
                }
                else if (data.position_velocity_limit !== undefined) {
                    updatePositionVelocityLimitDisplay(data.position_velocity_limit);
                }
                else if (data.position_accel_limit !== undefined) {
                    updatePositionAccelLimitDisplay(data.position_accel_limit);
                }
                else if (data.position_decel_limit !== undefined) {
                    updatePositionDecelLimitDisplay(data.position_decel_limit);
                }
                else if (data.position_torque_assist !== undefined) {
                    updatePositionTorqueAssistDisplay(data.position_torque_assist);
                }
                else if (data.autotune !== undefined) {
                    updateAutotuneDisplay(data.autotune);
                }
                else if (data.position_autotune !== undefined) {
                    updatePositionAutotuneDisplay(data.position_autotune);
                } else {
                    console.log('Unknown JSON:', data);
                    appendDebugConsole(`RX ${line}`, 'rx');
                }
            }
            // Handle text responses
            else {
                console.log('MCU Response:', line);
                appendDebugConsole(`RX ${line}`, 'rx');
            }
        } catch (error) {
            // Not JSON, treat as debug message
            if (line.length > 0) {
                console.log('MCU Message:', line);
                appendDebugConsole(`ERR RX parse failed: ${line}`, 'err');
            }
        }
    }
}

function normalizeFocTelemetry(focData) {
    if (!focData || typeof focData !== 'object') {
        return focData;
    }

    if (focData.vi === undefined && focData.mechi === undefined) {
        return focData;
    }

    const normalized = { ...focData };
    const assignScaled = (srcKey, dstKey, scale) => {
        if (focData[srcKey] !== undefined) {
            normalized[dstKey] = Number(focData[srcKey]) / scale;
        }
    };

    assignScaled('vi', 'v', 100);
    assignScaled('mechi', 'mech', 1000);
    assignScaled('mrawi', 'mraw', 1000);
    assignScaled('pmrawi', 'pmraw', 1000);
    assignScaled('mdelti', 'mdelta', 1000);
    assignScaled('pmechi', 'pmech', 1000);
    assignScaled('ti', 't', 100);
    assignScaled('tri', 'tr', 100);
    assignScaled('tpi', 'tp', 1000);
    assignScaled('erri', 'err', 100);
    assignScaled('uqi', 'uq', 100);
    assignScaled('vlimi', 'vlim', 100);
    assignScaled('vrampi', 'vramp', 100);
    assignScaled('pvlimi', 'pvlim', 100);
    assignScaled('pacci', 'pacc', 100);
    assignScaled('pdeci', 'pdec', 100);
    assignScaled('pboosti', 'pboost', 100);
    assignScaled('adiri', 'adir', 1000);
    assignScaled('aligni', 'align', 1000);

    return normalized;
}

/**
 * Update the velocity display with received data
 * @param {Object} data - Parsed JSON object with vel and target properties
 */
function updateVelocityDisplay(data) {
    // Handle both full and abbreviated field names
    if (data.vel !== undefined) {
        const velocity = parseFloat(data.vel).toFixed(2);
        document.getElementById('velocity').textContent = velocity;
        pushVelocitySample(parseFloat(data.vel));
    } else if (data.v !== undefined) {
        const velocity = parseFloat(data.v).toFixed(2);
        document.getElementById('velocity').textContent = velocity;
        pushVelocitySample(parseFloat(data.v));
    }

    if (data.target !== undefined) {
        const target = parseFloat(data.target).toFixed(2);
        document.getElementById('targetVelocity').textContent = target;
        syncVelocityControls(parseFloat(data.target));
    } else if (data.t !== undefined) {
        const target = parseFloat(data.t).toFixed(2);
        document.getElementById('targetVelocity').textContent = target;
        syncVelocityControls(parseFloat(data.t));
    }

    // Update last update time
    const now = new Date();
    const timeString = now.toLocaleTimeString();
    document.getElementById('lastUpdate').textContent = timeString;
}

/**
 * Update alignment offset display
 * @param {Object} data - Parsed JSON object with align_offset property
 */
function updateAlignmentDisplay(data) {
    if (data.align_offset !== undefined) {
        const offset = parseFloat(data.align_offset).toFixed(4);
        document.getElementById('alignmentOffset').textContent = offset + ' rad';
        console.log('Encoder alignment offset:', offset, 'rad');
    }
}

/**
 * Update PID display with current values
 * @param {Object} data - Parsed JSON object with pid property
 */
function updatePIDDisplay(data) {
    if (data.pid) {
        document.getElementById('kpInput').value = parseFloat(data.pid.kp).toFixed(3);
        document.getElementById('kiInput').value = parseFloat(data.pid.ki).toFixed(3);
        document.getElementById('kdInput').value = parseFloat(data.pid.kd).toFixed(3);
        console.log('PID updated:', data.pid);
    }
}

function updatePositionPIDDisplay(data) {
    if (data.position_pid) {
        document.getElementById('posKpInput').value = parseFloat(data.position_pid.kp).toFixed(3);
        const posKdInput = document.getElementById('posKdInput');
        if (posKdInput && data.position_pid.kd !== undefined) {
            posKdInput.value = parseFloat(data.position_pid.kd).toFixed(3);
        }
        console.log('Position gain updated:', data.position_pid);
    }
}

/**
 * Update target velocity display
 * @param {Object} data - Parsed JSON object with target_vel property
 */
function updateTargetVelocityDisplay(data) {
    if (data.target_vel !== undefined) {
        const target = parseFloat(data.target_vel).toFixed(2);
        document.getElementById('targetVelocity').textContent = target;
        syncVelocityControls(parseFloat(data.target_vel));
        console.log('Target velocity set to:', target, 'rad/s');
    }
}

function syncVelocityControls(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    const input = document.getElementById('velocityInput');
    const slider = document.getElementById('velocitySlider');

    if (input && document.activeElement !== input) {
        input.value = value.toFixed(1);
    }

    if (slider && document.activeElement !== slider) {
        slider.value = value.toFixed(1);
    }
}

function syncVelocityFromSlider() {
    const slider = document.getElementById('velocitySlider');
    const input = document.getElementById('velocityInput');
    if (!slider || !input) {
        return;
    }

    const value = parseFloat(slider.value);
    input.value = value.toFixed(1);

    if (state.velocitySliderTimer) {
        clearTimeout(state.velocitySliderTimer);
    }

    state.velocitySliderTimer = setTimeout(() => {
        setVelocity();
    }, 120);
}

function syncVelocityFromInput() {
    const input = document.getElementById('velocityInput');
    const slider = document.getElementById('velocitySlider');
    if (!input || !slider) {
        return;
    }

    const value = parseFloat(input.value);
    if (!Number.isFinite(value)) {
        return;
    }

    const clamped = Math.max(-100, Math.min(100, value));
    slider.value = clamped.toFixed(1);
}

function updatePositionDisplay(data) {
    if (data.pos !== undefined) {
        syncPositionControls(parseFloat(data.pos));
    } else if (data.target_pos !== undefined) {
        syncPositionControls(parseFloat(data.target_pos));
    } else if (data.tp !== undefined) {
        syncPositionControls(parseFloat(data.tp));
    }
}

const POSITION_SLIDER_MIN = -100 * Math.PI;
const POSITION_SLIDER_MAX = 100 * Math.PI;

function syncPositionControls(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    const input = document.getElementById('positionInput');
    const slider = document.getElementById('positionSlider');

    if (input && document.activeElement !== input) {
        input.value = value.toFixed(3);
    }

    if (slider && document.activeElement !== slider) {
        const clamped = Math.max(POSITION_SLIDER_MIN, Math.min(POSITION_SLIDER_MAX, value));
        slider.value = clamped.toFixed(3);
    }
}

function syncPositionFromSlider() {
    const slider = document.getElementById('positionSlider');
    const input = document.getElementById('positionInput');
    if (!slider || !input) {
        return;
    }

    const value = parseFloat(slider.value);
    input.value = value.toFixed(3);

    if (state.positionSliderTimer) {
        clearTimeout(state.positionSliderTimer);
    }

    state.positionSliderTimer = setTimeout(() => {
        setPosition();
    }, 120);
}

function syncPositionFromInput() {
    const input = document.getElementById('positionInput');
    const slider = document.getElementById('positionSlider');
    if (!input || !slider) {
        return;
    }

    const value = parseFloat(input.value);
    if (!Number.isFinite(value)) {
        return;
    }

    const clamped = Math.max(POSITION_SLIDER_MIN, Math.min(POSITION_SLIDER_MAX, value));
    slider.value = clamped.toFixed(3);
}

/**
 * Update FOC debug display with current control values
 * @param {Object} focData - FOC debug data object
 */
function updateFOCDebugDisplay(focData) {
    // Handle abbreviated field names from firmware (v, t, lc, r, err, uq, pwm)
    // and full field names (vel, target, velocity, error, etc.)

    // Target velocity
    if (focData.target !== undefined) {
        document.getElementById('debugTarget').textContent = parseFloat(focData.target).toFixed(3);
    } else if (focData.t !== undefined) {
        document.getElementById('debugTarget').textContent = parseFloat(focData.t).toFixed(3);
    }
    if (focData.tr !== undefined) {
        pushTargetVelocitySample(parseFloat(focData.tr));
    }

    // Actual velocity
    if (focData.vel !== undefined) {
        document.getElementById('debugActual').textContent = parseFloat(focData.vel).toFixed(3);
    } else if (focData.v !== undefined) {
        document.getElementById('debugActual').textContent = parseFloat(focData.v).toFixed(3);
    }

    // Velocity error
    if (focData.err !== undefined) {
        document.getElementById('debugError').textContent = parseFloat(focData.err).toFixed(3);
    }

    // PID output (Q-axis voltage)
    if (focData.uq !== undefined) {
        document.getElementById('debugUq').textContent = parseFloat(focData.uq).toFixed(3);
    }

    // PWM duty cycles
    if (focData.pwm && Array.isArray(focData.pwm) && focData.pwm.length >= 3) {
        document.getElementById('debugPwmA').textContent = focData.pwm[0];
        document.getElementById('debugPwmB').textContent = focData.pwm[1];
        document.getElementById('debugPwmC').textContent = focData.pwm[2];
    }

    // PWM period
    if (focData.per !== undefined) {
        document.getElementById('debugPwmPeriod').textContent = focData.per;
    }

    if (focData.mech !== undefined) {
        pushPositionSample(parseFloat(focData.mech));
    }
    if (focData.tp !== undefined) {
        pushTargetPositionSample(parseFloat(focData.tp));
    }
    if (focData.mode !== undefined && parseInt(focData.mode, 10) === 0 && focData.mech !== undefined) {
        syncPositionControls(parseFloat(focData.mech));
    } else if (focData.tp !== undefined) {
        syncPositionControls(parseFloat(focData.tp));
    }
    if (focData.r !== undefined) {
        pushRawAngleSample(parseFloat(focData.r));
    }
    if (focData.mraw !== undefined) {
        pushRawMechanicalSample(parseFloat(focData.mraw));
    }
    if (focData.pmraw !== undefined) {
        pushPreviousMechanicalSample(parseFloat(focData.pmraw));
    }
    if (focData.mdelta !== undefined) {
        pushMechanicalDeltaSample(parseFloat(focData.mdelta));
    }
    if (focData.pmech !== undefined) {
        pushPreviousMultiTurnSample(parseFloat(focData.pmech));
    }

    if (focData.lc !== undefined) {
        document.getElementById('debugLoopCount').textContent = focData.lc;
    }
    if (focData.r !== undefined) {
        const encValue = parseInt(focData.r, 10);
        if (Number.isFinite(encValue)) {
            const encRad = (encValue / 4096.0 * 2 * Math.PI).toFixed(3);
            document.getElementById('debugEncoderAngle').textContent = `${encValue} (${encRad} rad)`;
        }
    }
    if (focData.cb !== undefined) {
        const callbackCount = parseInt(focData.cb, 10);
        document.getElementById('debugEncoderCallbacks').textContent = callbackCount;
        const now = Date.now();
        if (Number.isFinite(callbackCount) && state.lastDiagCallbackCount !== null && state.lastDiagCallbackTimestamp > 0) {
            const deltaCount = callbackCount - state.lastDiagCallbackCount;
            const deltaMs = now - state.lastDiagCallbackTimestamp;
            if (deltaMs > 0) {
                const callbackRate = Math.max(0, Math.round((deltaCount * 1000) / deltaMs));
                document.getElementById('debugCallbackRate').textContent = `${callbackRate}/sec`;
            }
        }
        state.lastDiagCallbackCount = callbackCount;
        state.lastDiagCallbackTimestamp = now;
    }
    if (focData.derr !== undefined) {
        document.getElementById('debugI2CErrors').textContent = focData.derr;
    }
    if (focData.start !== undefined) {
        document.getElementById('debugDMAStarts').textContent = focData.start;
    }
    if (focData.run !== undefined) {
        document.getElementById('debugMotorRunning').textContent = focData.run ? 'Yes' : 'No';
    }

    // Update FOC status indicator
    document.getElementById('focStatus').textContent = 'Active';

    if (focData.mode !== undefined) {
        const modeNames = ['Velocity', 'Position', 'Open Loop', 'Vector Test', 'Torque'];
        const modeText = modeNames[focData.mode] || String(focData.mode);
        document.getElementById('controlMode').textContent = modeText;
        const heroMode = document.getElementById('heroMode');
        if (heroMode) {
            heroMode.textContent = modeText;
        }
    }
    if (focData.mod !== undefined) {
        const modulation = (parseInt(focData.mod, 10) === 1) ? 'Sine' : 'SVPWM';
        document.getElementById('modulationStatus').textContent = modulation;
        const heroModulation = document.getElementById('heroModulation');
        if (heroModulation) {
            heroModulation.textContent = modulation;
        }
    }
    if (focData.vlim !== undefined) {
        const voltageText = parseFloat(focData.vlim).toFixed(2) + ' V';
        document.getElementById('voltageLimitStatus').textContent = voltageText;
        const heroVoltageLimit = document.getElementById('heroVoltageLimit');
        if (heroVoltageLimit) {
            heroVoltageLimit.textContent = voltageText;
        }
    }
    if (focData.usat !== undefined) {
        document.getElementById('uqSaturatedStatus').textContent = focData.usat ? 'Yes' : 'No';
    }
    if (focData.align !== undefined) {
        document.getElementById('alignmentOffset').textContent = parseFloat(focData.align).toFixed(4) + ' rad';
    }
    if (focData.vramp !== undefined) {
        updateVelocityRampDisplay(focData.vramp);
    }
    if (focData.pvlim !== undefined) {
        updatePositionVelocityLimitDisplay(focData.pvlim);
    }
}

/**
 * Update diagnostic display with loop count and transmission status
 * @param {Object} diagData - Diagnostic data object with lc, sent, failed, enc, cb, err, start, cbrate, run
 */
function updateDiagnosticDisplay(diagData) {
    // Update loop count (shows FOC_Loop execution frequency)
    if (diagData.lc !== undefined) {
        document.getElementById('debugLoopCount').textContent = diagData.lc;
    }
    // Update message sent count
    if (diagData.sent !== undefined) {
        document.getElementById('debugMessagesSent').textContent = diagData.sent;
    }
    // Update message failed count
    if (diagData.failed !== undefined) {
        document.getElementById('debugMessagesFailed').textContent = diagData.failed;
    }
    // Update encoder raw angle (12-bit value 0-4095)
    if (diagData.enc !== undefined) {
        const encValue = parseInt(diagData.enc);
        const encRad = (encValue / 4096.0 * 2 * Math.PI).toFixed(3);
        document.getElementById('debugEncoderAngle').textContent = encValue + ' (' + encRad + ' rad)';
    }
    // Update I2C DMA callback count
    if (diagData.cb !== undefined) {
        document.getElementById('debugEncoderCallbacks').textContent = diagData.cb;
    }
    // Update callback rate (callbacks per second)
    if (diagData.cbrate !== undefined) {
        document.getElementById('debugCallbackRate').textContent = diagData.cbrate + '/sec';
    }
    // Update I2C error count
    if (diagData.err !== undefined) {
        document.getElementById('debugI2CErrors').textContent = diagData.err;
    }
    // Update DMA start count (how many times we tried to start DMA reads)
    if (diagData.start !== undefined) {
        document.getElementById('debugDMAStarts').textContent = diagData.start;
    }
    // Update motor running status
    if (diagData.run !== undefined) {
        document.getElementById('debugMotorRunning').textContent = diagData.run ? 'Yes' : 'No';
    }
}

function updateOpenLoopDisplay(openLoopData) {
    document.getElementById('focStatus').textContent = 'Open Loop';
    const heroMode = document.getElementById('heroMode');
    if (heroMode) {
        heroMode.textContent = 'Open Loop';
    }
    console.log('Open-loop mode:', openLoopData);
}

/**
 * Update connection status UI
 * @param {Boolean} connected - Current connection state
 */
function updateConnectionStatus(connected) {
    const statusElement = document.getElementById('connectionStatus');
    const statusText = document.getElementById('connectionText');
    const heroConnection = document.getElementById('heroConnection');
    const dfuButton = document.getElementById('dfuModeBtn');

    if (connected) {
        statusElement.classList.add('connected');
        statusElement.classList.remove('disconnected');
        statusText.textContent = 'Connected';
        if (heroConnection) {
            heroConnection.textContent = 'Connected';
        }
    } else {
        statusElement.classList.add('disconnected');
        statusElement.classList.remove('connected');
        statusText.textContent = 'Disconnected';
        if (heroConnection) {
            heroConnection.textContent = 'Disconnected';
        }
    }

    if (dfuButton) {
        dfuButton.style.display = connected ? 'inline-flex' : 'none';
    }

    refreshTelemetryPollingUI();
}

/**
 * Send a command to the MCU
 * @param {String} command - Command to send (e.g., 'START', 'STOP')
 */
async function sendCommand(command, options = {}) {
    if (!state.isConnected || !state.port) {
        alert('Not connected to serial port. Click the connect button first.');
        return;
    }

    try {
        const writer = state.port.writable.getWriter();
        const data = new TextEncoder().encode(command + '\n');
        await writer.write(data);
        writer.releaseLock();
        if (!options.silent) {
            console.log('Sent command:', command);
            appendDebugConsole(`TX ${command}`, 'tx');
        }
        if (!options.silent) {
            state.telemetryPollQuietUntil = Date.now() + 300;
        }

          // Reset FOC status when stopping motor
          if (command === 'STOP') {
              document.getElementById('focStatus').textContent = 'Stopped';
              // Clear debug values
              document.getElementById('debugTarget').textContent = '—';
              document.getElementById('debugActual').textContent = '—';
              document.getElementById('debugError').textContent = '—';
              document.getElementById('debugUq').textContent = '—';
              document.getElementById('debugPwmA').textContent = '—';
              document.getElementById('debugPwmB').textContent = '—';
              document.getElementById('debugPwmC').textContent = '—';
          }
      } catch (error) {
        console.error('Failed to send command:', error);
        appendDebugConsole(`ERR TX failed for ${command}: ${error}`, 'err');
      }
}

/**
 * Set motor velocity via USB
 */
async function setVelocity() {
    const input = document.getElementById('velocityInput');
    const slider = document.getElementById('velocitySlider');
    let velocity = parseFloat(input.value);

    if (slider && document.activeElement === slider) {
        velocity = parseFloat(slider.value);
    }

    if (isNaN(velocity) || velocity < -100 || velocity > 100) {
        alert('Please enter a valid velocity between -100 and 100 rad/s');
        return;
    }

    syncVelocityControls(velocity);
    const command = `SET_VELOCITY:${velocity.toFixed(2)}`;
    await sendCommand(command);
}

async function setPosition() {
    const input = document.getElementById('positionInput');
    const slider = document.getElementById('positionSlider');
    let position = parseFloat(input.value);

    if (slider && document.activeElement === slider) {
        position = parseFloat(slider.value);
    }

    if (isNaN(position) || position < POSITION_SLIDER_MIN || position > POSITION_SLIDER_MAX) {
        alert('Please enter a valid position between -100π and +100π rad');
        return;
    }

    syncPositionControls(position);
    await sendCommand(`SET_POSITION:${position.toFixed(3)}`);
}

async function getVelocity() {
    await sendCommand('GET_VELOCITY');
}

async function getPosition() {
    await sendCommand('GET_POSITION');
}

async function startOpenLoop() {
    await sendCommand('START_OPENLOOP');
}

async function setPhaseMap() {
    const select = document.getElementById('phaseMapSelect');
    const phaseMap = parseInt(select.value, 10);
    await sendCommand(`SET_PHASE_MAP:${phaseMap}`);
}

async function startVectorTest() {
    const select = document.getElementById('vectorTestSelect');
    const vectorIndex = parseInt(select.value, 10);
    await sendCommand(`TEST_VECTOR:${vectorIndex}`);
}

async function setModulation() {
    const select = document.getElementById('modulationSelect');
    await sendCommand(`SET_MODULATION:${select.value}`);
}

async function getModulation() {
    await sendCommand('GET_MODULATION');
}

function updatePhaseMapDisplay(phaseMap) {
    const select = document.getElementById('phaseMapSelect');
    if (select) {
        select.value = String(phaseMap);
    }
    console.log('Phase map set to:', phaseMap);
}

function updateVectorTestDisplay(vectorTest) {
    const select = document.getElementById('vectorTestSelect');
    if (select && vectorTest.index !== undefined) {
        select.value = String(vectorTest.index);
    }
    document.getElementById('focStatus').textContent = 'Vector Test';
    const heroMode = document.getElementById('heroMode');
    if (heroMode) {
        heroMode.textContent = 'Vector Test';
    }
    console.log('Vector test:', vectorTest);
}

function updateModulationDisplay(modulation) {
    const select = document.getElementById('modulationSelect');
    if (select) {
        select.value = modulation;
    }
    document.getElementById('modulationStatus').textContent = modulation === 'SINE' ? 'Sine' : modulation;
    const heroModulation = document.getElementById('heroModulation');
    if (heroModulation) {
        heroModulation.textContent = modulation === 'SINE' ? 'Sine' : modulation;
    }
    console.log('Modulation set to:', modulation);
}

function updateFeedforwardDisplay(feedforward) {
    if (feedforward.voltage !== undefined) {
        document.getElementById('feedforwardVoltageInput').value = parseFloat(feedforward.voltage).toFixed(2);
    }
    if (feedforward.fade_speed !== undefined) {
        document.getElementById('feedforwardFadeInput').value = parseFloat(feedforward.fade_speed).toFixed(2);
    }
    console.log('Feedforward updated:', feedforward);
}

function updateLowSpeedBiasDisplay(lowSpeedBias) {
    if (lowSpeedBias.voltage !== undefined) {
        const voltageInput = document.getElementById('lowSpeedBiasVoltageInput');
        if (voltageInput) {
            voltageInput.value = parseFloat(lowSpeedBias.voltage).toFixed(2);
        }
    }
    if (lowSpeedBias.fade_speed !== undefined) {
        const fadeInput = document.getElementById('lowSpeedBiasFadeInput');
        if (fadeInput) {
            fadeInput.value = parseFloat(lowSpeedBias.fade_speed).toFixed(2);
        }
    }
    console.log('Low-speed bias updated:', lowSpeedBias);
}

function updateVelocityRampDisplay(velocityRamp) {
    const input = document.getElementById('velocityRampInput');
    if (input && velocityRamp !== undefined) {
        if (!motionProfileInputLocked(input)) {
            input.value = parseFloat(velocityRamp).toFixed(2);
        }
    }
}

async function requestDfuMode() {
    if (!state.isConnected || !state.port) {
        alert('Connect to the controller first.');
        return;
    }

    const confirmed = window.confirm('Enter DFU mode now? The controller will reset and the serial connection will disconnect.');
    if (!confirmed) {
        return;
    }

    stopTelemetryPolling();
    appendDebugConsole('SYS Entering DFU mode', 'sys');

    try {
        await sendCommand('SET_MODE_DFU');
    } catch (error) {
        console.error('Failed to enter DFU mode:', error);
        appendDebugConsole(`ERR DFU request failed: ${error}`, 'err');
    }
}

function updatePositionVelocityLimitDisplay(positionVelocityLimit) {
    const input = document.getElementById('positionVelocityLimitInput');
    if (input && positionVelocityLimit !== undefined) {
        if (!motionProfileInputLocked(input)) {
            input.value = parseFloat(positionVelocityLimit).toFixed(2);
        }
        input.dataset.appliedValue = parseFloat(positionVelocityLimit).toFixed(2);
    }
}

function updatePositionAccelLimitDisplay(positionAccelLimit) {
    const input = document.getElementById('positionAccelLimitInput');
    if (input && positionAccelLimit !== undefined) {
        if (!motionProfileInputLocked(input)) {
            input.value = parseFloat(positionAccelLimit).toFixed(2);
        }
    }
}

function updatePositionDecelLimitDisplay(positionDecelLimit) {
    const input = document.getElementById('positionDecelLimitInput');
    if (input && positionDecelLimit !== undefined) {
        if (!motionProfileInputLocked(input)) {
            input.value = parseFloat(positionDecelLimit).toFixed(2);
        }
    }
}

function updatePositionTorqueAssistDisplay(positionTorqueAssist) {
    const input = document.getElementById('positionTorqueAssistInput');
    if (input && positionTorqueAssist !== undefined) {
        if (document.activeElement !== input) {
            input.value = parseFloat(positionTorqueAssist).toFixed(2);
        }
    }
}

function updateMotionProfileFromTelemetry(focData) {
    if (focData.vramp !== undefined) {
        updateVelocityRampDisplay(focData.vramp);
    }
    if (focData.pvlim !== undefined) {
        updatePositionVelocityLimitDisplay(focData.pvlim);
    }
    if (focData.pacc !== undefined) {
        updatePositionAccelLimitDisplay(focData.pacc);
    }
    if (focData.pdec !== undefined) {
        updatePositionDecelLimitDisplay(focData.pdec);
    }
    if (focData.pboost !== undefined) {
        updatePositionTorqueAssistDisplay(focData.pboost);
    }
}

function updateAutotuneDisplay(autotune) {
    if (autotune.status !== undefined) {
        document.getElementById('autotuneStatus').textContent = autotune.status;
    }

    if (autotune.kp !== undefined && autotune.ki !== undefined && autotune.kd !== undefined) {
        state.autotuneResult = autotune;
        document.getElementById('kpInput').value = parseFloat(autotune.kp).toFixed(3);
        document.getElementById('kiInput').value = parseFloat(autotune.ki).toFixed(3);
        document.getElementById('kdInput').value = parseFloat(autotune.kd).toFixed(3);
    }

    console.log('Autotune update:', autotune);
}

function updatePositionAutotuneDisplay(autotune) {
    if (autotune.status !== undefined) {
        document.getElementById('positionAutotuneStatus').textContent = autotune.status;
    }

    if (autotune.kp !== undefined) {
        document.getElementById('posKpInput').value = parseFloat(autotune.kp).toFixed(3);
    }
    if (autotune.kd !== undefined) {
        const posKdInput = document.getElementById('posKdInput');
        if (posKdInput) {
            posKdInput.value = parseFloat(autotune.kd).toFixed(3);
        }
    }

    console.log('Position tuning update:', autotune);
}

function pushVelocitySample(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    state.velocityHistory.push({
        t: Date.now(),
        v: value
    });
    if (state.velocityHistory.length > state.velocityHistoryLimit) {
        state.velocityHistory.shift();
    }
    drawTracePlot();
}

function pushPositionSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    state.positionHistory.push({
        t: Date.now(),
        v: value
    });
    if (state.positionHistory.length > state.positionHistoryLimit) {
        state.positionHistory.shift();
    }
    drawTracePlot();
}

function pushTargetVelocitySample(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    state.targetVelocityHistory.push({
        t: Date.now(),
        v: value
    });
    if (state.targetVelocityHistory.length > state.targetVelocityHistoryLimit) {
        state.targetVelocityHistory.shift();
    }
    drawTracePlot();
}

function pushTargetPositionSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    state.targetPositionHistory.push({
        t: Date.now(),
        v: value
    });
    if (state.targetPositionHistory.length > state.targetPositionHistoryLimit) {
        state.targetPositionHistory.shift();
    }
    drawTracePlot();
}

function pushRawAngleSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }
    state.rawAngleHistory.push({ t: Date.now(), v: value });
    if (state.rawAngleHistory.length > state.rawAngleHistoryLimit) {
        state.rawAngleHistory.shift();
    }
}

function pushRawMechanicalSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }
    state.rawMechanicalHistory.push({ t: Date.now(), v: value });
    if (state.rawMechanicalHistory.length > state.rawMechanicalHistoryLimit) {
        state.rawMechanicalHistory.shift();
    }
}

function pushPreviousMechanicalSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }
    state.previousMechanicalHistory.push({ t: Date.now(), v: value });
    if (state.previousMechanicalHistory.length > state.previousMechanicalHistoryLimit) {
        state.previousMechanicalHistory.shift();
    }
}

function pushMechanicalDeltaSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }
    state.mechanicalDeltaHistory.push({ t: Date.now(), v: value });
    if (state.mechanicalDeltaHistory.length > state.mechanicalDeltaHistoryLimit) {
        state.mechanicalDeltaHistory.shift();
    }
}

function pushPreviousMultiTurnSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }
    state.previousMultiTurnHistory.push({ t: Date.now(), v: value });
    if (state.previousMultiTurnHistory.length > state.previousMultiTurnHistoryLimit) {
        state.previousMultiTurnHistory.shift();
    }
}

function setTraceMode() {
    const select = document.getElementById('traceModeSelect');
    state.traceMode = select ? select.value : 'velocity';
    drawTracePlot();
}

function drawTracePlot() {
    const canvas = document.getElementById('tracePlot');
    const meta = document.getElementById('tracePlotMeta');
    const note = document.getElementById('tracePlotNote');
    if (!canvas || !meta || !note) {
        return;
    }

    const themeVars = getComputedStyle(document.documentElement);
    const plotAxisColor = themeVars.getPropertyValue('--plot-axis').trim() || 'rgba(148, 163, 184, 0.55)';
    const plotZeroColor = themeVars.getPropertyValue('--plot-zero').trim() || 'rgba(37, 99, 235, 0.75)';
    const plotLabelColor = themeVars.getPropertyValue('--plot-label').trim() || 'rgba(82, 98, 119, 0.92)';

    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    const paddingX = 14;
    const paddingY = 16;
    const innerWidth = width - (paddingX * 2);
    const innerHeight = height - (paddingY * 2);
    const measuredHistory = state.traceMode === 'position' ? state.positionHistory : state.velocityHistory;
    const targetHistory = state.traceMode === 'position' ? state.targetPositionHistory : state.targetVelocityHistory;
    const historyLimit = state.traceMode === 'position' ? state.positionHistoryLimit : state.velocityHistoryLimit;
    const values = [...measuredHistory, ...targetHistory].map(sample => (typeof sample === 'number' ? sample : sample.v));
    const isPositionTrace = state.traceMode === 'position';
    const minValue = values.length > 0 ? Math.min(...values) : (isPositionTrace ? 0.0 : -0.5);
    const maxValue = values.length > 0 ? Math.max(...values) : (isPositionTrace ? 1.0 : 0.5);
    const centeredRange = values.length > 0 ? Math.max(0.5, ...values.map(sample => Math.abs(sample))) : 0.5;
    const rangeMin = isPositionTrace ? Math.min(minValue, maxValue - 0.5) : -centeredRange;
    const rangeMax = isPositionTrace ? Math.max(maxValue, minValue + 0.5) : centeredRange;
    const rangeSpan = Math.max(0.5, rangeMax - rangeMin);
    const mapValueToY = (sample) => {
        const normalized = (sample - rangeMin) / rangeSpan;
        return paddingY + innerHeight - (normalized * innerHeight);
    };

    ctx.clearRect(0, 0, width, height);

    ctx.save();
    ctx.strokeStyle = plotAxisColor;
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
        const y = paddingY + (innerHeight * i / 4);
        ctx.beginPath();
        ctx.moveTo(paddingX, y);
        ctx.lineTo(width - paddingX, y);
        ctx.stroke();
    }

    if (!isPositionTrace) {
        const zeroY = mapValueToY(0);
        ctx.strokeStyle = plotZeroColor;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(paddingX, zeroY);
        ctx.lineTo(width - paddingX, zeroY);
        ctx.stroke();
    }

    const drawSeries = (history, startColor, endColor, dotColor) => {
        if (history.length <= 1) {
            return;
        }

        const gradient = ctx.createLinearGradient(0, 0, width, 0);
        gradient.addColorStop(0, startColor);
        gradient.addColorStop(1, endColor);

        ctx.strokeStyle = gradient;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.beginPath();

        history.forEach((sample, index) => {
            const sampleValue = (typeof sample === 'number' ? sample : sample.v);
            const x = paddingX + (innerWidth * index / (historyLimit - 1));
            const y = mapValueToY(sampleValue);
            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });
        ctx.stroke();

        const lastSample = history[history.length - 1];
        const lastValue = (typeof lastSample === 'number' ? lastSample : lastSample.v);
        const lastX = paddingX + (innerWidth * (history.length - 1) / (historyLimit - 1));
        const lastY = mapValueToY(lastValue);
        ctx.fillStyle = dotColor;
        ctx.beginPath();
        ctx.arc(lastX, lastY, 4, 0, Math.PI * 2);
        ctx.fill();
    };

    if (targetHistory.length > 1) {
        drawSeries(targetHistory, 'rgba(239, 68, 68, 0.85)', 'rgba(249, 115, 22, 0.85)', '#b91c1c');
    }
    if (measuredHistory.length > 1) {
        drawSeries(measuredHistory, '#14b8a6', '#2563eb', '#0f766e');
    }

    ctx.fillStyle = plotLabelColor;
    ctx.font = '12px "Segoe UI Variable", "Segoe UI", sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++) {
        const value = rangeMax - ((rangeSpan * i) / 4);
        const y = paddingY + (innerHeight * i / 4);
        const suffix = isPositionTrace ? ' rad' : '';
        ctx.fillText(`${value.toFixed(isPositionTrace ? 2 : 1)}${suffix}`, paddingX + 4, y);
    }

    ctx.restore();
    if (isPositionTrace) {
        meta.textContent = `Window: ${measuredHistory.length}/${targetHistory.length} samples | Range: ${rangeMin.toFixed(2)} to ${rangeMax.toFixed(2)} rad`;
        note.textContent = 'Measured position is teal/blue, target position is red/orange.';
    } else {
        meta.textContent = `Window: ${measuredHistory.length}/${targetHistory.length} samples | Scale: ±${centeredRange.toFixed(2)} rad/s`;
        note.textContent = 'Measured velocity is teal/blue, commanded velocity is red/orange.';
    }
}

function exportTraceData() {
    const readNumber = (id) => {
        const element = document.getElementById(id);
        return element ? parseFloat(element.value) : null;
    };

    const payload = {
        exported_at: new Date().toISOString(),
        connection: state.isConnected ? 'connected' : 'disconnected',
        poll_interval_ms: state.telemetryPollIntervalMs,
        trace_mode: state.traceMode,
        tuning: {
            position_gain: readNumber('posKpInput'),
            velocity_damping: readNumber('posKdInput'),
            velocity_ramp_limit: readNumber('velocityRampInput'),
            position_velocity_limit: readNumber('positionVelocityLimitInput'),
            position_accel_limit: readNumber('positionAccelLimitInput'),
            position_decel_limit: readNumber('positionDecelLimitInput')
        },
        debug_console: {
            line_count: state.debugConsoleLines.length,
            lines: state.debugConsoleLines.map((entry, index) => ({
                index,
                type: entry.type,
                line: entry.line
            }))
        },
        traces: {
            velocity: state.velocityHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            target_velocity: state.targetVelocityHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            position: state.positionHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            target_position: state.targetPositionHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            raw_angle: state.rawAngleHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            raw_mechanical_angle: state.rawMechanicalHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            previous_mechanical_angle: state.previousMechanicalHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            mechanical_delta: state.mechanicalDeltaHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            })),
            previous_multi_turn_angle: state.previousMultiTurnHistory.map((sample, index) => ({
                index,
                timestamp_ms: typeof sample === 'number' ? null : sample.t,
                value: typeof sample === 'number' ? sample : sample.v
            }))
        }
    };

    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    link.href = url;
    link.download = `foc-trace-${stamp}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
}

/**
 * Set PID parameters via USB
 */
async function setPID() {
    const kp = parseFloat(document.getElementById('kpInput').value);
    const ki = parseFloat(document.getElementById('kiInput').value);
    const kd = parseFloat(document.getElementById('kdInput').value);

    if (isNaN(kp) || isNaN(ki) || isNaN(kd) || kp < 0 || kp > 5 || ki < 0 || ki > 10 || kd < 0 || kd > 1) {
        alert('Please enter valid PID values within safe ranges');
        return;
    }

    const command = `SET_PID:${kp.toFixed(3)},${ki.toFixed(3)},${kd.toFixed(3)}`;
    await sendCommand(command);
}

/**
 * Get current PID parameters from MCU
 */
async function getPID() {
    await sendCommand('GET_PID');
}

async function setPositionPID() {
    const kp = parseFloat(document.getElementById('posKpInput').value);
    const kd = parseFloat(document.getElementById('posKdInput').value);

    if (isNaN(kp) || kp < 0 || kp > 20 || isNaN(kd) || kd < 0 || kd > 5) {
        alert('Please enter valid position gain and damping values within the safe range');
        return;
    }

    await sendCommand(`SET_POSITION_PID:${kp.toFixed(3)},0.000,${kd.toFixed(3)}`);
}

async function getPositionPID() {
    await sendCommand('GET_POSITION_PID');
}

async function startPositionAutotune() {
    document.getElementById('positionAutotuneStatus').textContent = 'starting';
    await sendCommand('START_POSITION_AUTOTUNE');
}

async function startAutotune() {
    document.getElementById('autotuneStatus').textContent = 'starting';
    state.autotuneResult = null;
    await sendCommand('START_AUTOTUNE');
}

async function applyAutotuneResults() {
    if (!state.autotuneResult) {
        alert('No autotune result available yet');
        return;
    }

    await setPID();
}

async function setFeedforward() {
    const voltage = parseFloat(document.getElementById('feedforwardVoltageInput').value);
    const fadeSpeed = parseFloat(document.getElementById('feedforwardFadeInput').value);

    if (isNaN(voltage) || isNaN(fadeSpeed) || voltage < 0 || voltage > 6 || fadeSpeed < 0.1 || fadeSpeed > 20) {
        alert('Please enter feedforward values within the allowed ranges');
        return;
    }

    await sendCommand(`SET_FEEDFORWARD:${voltage.toFixed(2)},${fadeSpeed.toFixed(2)}`);
}

async function getFeedforward() {
    await sendCommand('GET_FEEDFORWARD');
}

async function setLowSpeedBias() {
    const voltage = parseFloat(document.getElementById('lowSpeedBiasVoltageInput').value);
    const fadeSpeed = parseFloat(document.getElementById('lowSpeedBiasFadeInput').value);

    if (isNaN(voltage) || isNaN(fadeSpeed) || voltage < 0 || voltage > 6 || fadeSpeed < 0.1 || fadeSpeed > 20) {
        alert('Please enter low-speed bias values within the allowed ranges');
        return;
    }

    await sendCommand(`SET_LOW_SPEED_BIAS:${voltage.toFixed(2)},${fadeSpeed.toFixed(2)}`);
}

async function getLowSpeedBias() {
    await sendCommand('GET_LOW_SPEED_BIAS');
}

async function setMotionProfile() {
    const velocityRampInput = document.getElementById('velocityRampInput');
    const positionVelocityLimitInput = document.getElementById('positionVelocityLimitInput');
    const positionAccelLimitInput = document.getElementById('positionAccelLimitInput');
    const positionDecelLimitInput = document.getElementById('positionDecelLimitInput');
    const velocityRamp = parseFloat(velocityRampInput.value);
    const positionVelocityLimit = parseFloat(positionVelocityLimitInput.value);
    const positionAccelLimit = parseFloat(positionAccelLimitInput.value);
    const positionDecelLimit = parseFloat(positionDecelLimitInput.value);

    if (isNaN(velocityRamp) || velocityRamp < 0.1 || velocityRamp > 200) {
        alert('Please enter a velocity ramp between 0.1 and 200 rad/s²');
        return;
    }

    if (isNaN(positionVelocityLimit) || positionVelocityLimit < 0.2 || positionVelocityLimit > 100) {
        alert('Please enter a position velocity limit between 0.2 and 100 rad/s');
        return;
    }

    if (isNaN(positionAccelLimit) || positionAccelLimit < 0.1 || positionAccelLimit > 400) {
        alert('Please enter a position acceleration limit between 0.1 and 400 rad/s²');
        return;
    }
    if (isNaN(positionDecelLimit) || positionDecelLimit < 0.1 || positionDecelLimit > 400) {
        alert('Please enter a position deceleration limit between 0.1 and 400 rad/s²');
        return;
    }

    state.motionProfileWritePendingUntil = Date.now() + 1500;
    velocityRampInput.value = velocityRamp.toFixed(2);
    positionVelocityLimitInput.value = positionVelocityLimit.toFixed(2);
    positionAccelLimitInput.value = positionAccelLimit.toFixed(2);
    positionDecelLimitInput.value = positionDecelLimit.toFixed(2);
    clearDirty(velocityRampInput);
    clearDirty(positionVelocityLimitInput);
    clearDirty(positionAccelLimitInput);
    clearDirty(positionDecelLimitInput);

    await sendCommand(`SET_VELOCITY_RAMP:${velocityRamp.toFixed(2)}`);
    await sendCommand(`SET_POSITION_VELOCITY_LIMIT:${positionVelocityLimit.toFixed(2)}`);
    await sendCommand(`SET_POSITION_ACCEL_LIMIT:${positionAccelLimit.toFixed(2)}`);
    await sendCommand(`SET_POSITION_DECEL_LIMIT:${positionDecelLimit.toFixed(2)}`);
    await delay(120);
    await getMotionProfile();
}

async function getMotionProfile() {
    await sendCommand('GET_VELOCITY_RAMP');
    await sendCommand('GET_POSITION_VELOCITY_LIMIT');
    await sendCommand('GET_POSITION_ACCEL_LIMIT');
    await sendCommand('GET_POSITION_DECEL_LIMIT');
}

async function setPositionTorqueAssist() {
    const assistVoltage = parseFloat(document.getElementById('positionTorqueAssistInput').value);

    if (isNaN(assistVoltage) || assistVoltage < 0 || assistVoltage > 6) {
        alert('Please enter a position torque assist between 0 and 6 V');
        return;
    }

    await sendCommand(`SET_POSITION_TORQUE_ASSIST:${assistVoltage.toFixed(2)}`);
}

async function getPositionTorqueAssist() {
    await sendCommand('GET_POSITION_TORQUE_ASSIST');
}

/**
 * Disconnect from serial port
 */
async function disconnectSerialPort() {
    try {
        stopTelemetryPolling();
        if (state.reader) {
            await state.reader.cancel();
        }
        if (state.port) {
            await state.port.close();
        }
        state.isConnected = false;
        state.port = null;
        state.reader = null;
        state.telemetryPollCount = 0;
        state.lastDiagCallbackCount = null;
        state.lastDiagCallbackTimestamp = 0;
        state.velocityHistory = [];
        state.positionHistory = [];
        state.targetVelocityHistory = [];
        state.targetPositionHistory = [];
        state.rawAngleHistory = [];
        state.rawMechanicalHistory = [];
        state.previousMechanicalHistory = [];
        state.mechanicalDeltaHistory = [];
        state.previousMultiTurnHistory = [];
        updateConnectionStatus(false);
        drawTracePlot();
        console.log('Disconnected from serial port');
        appendDebugConsole('SYS Disconnected from serial port', 'sys');
    } catch (error) {
        console.error('Disconnect error:', error);
        appendDebugConsole(`ERR Disconnect failed: ${error}`, 'err');
    }
}

/**
 * Initialize connection button listener
 */
document.addEventListener('DOMContentLoaded', function () {
    applyTheme(loadStoredTheme());
    updateBrowserDiagnostics();

    const connectBtn = document.createElement('button');
    connectBtn.textContent = 'Connect Serial Port';
    connectBtn.className = 'connect-button';
    connectBtn.onclick = connectSerialPort;
    if (!webSerialSupported()) {
        connectBtn.textContent = 'Web Serial Unsupported';
        connectBtn.disabled = true;
        connectBtn.title = 'Use Chrome or Edge on desktop to access Web Serial.';
    }

    const themeBtn = document.createElement('button');
    themeBtn.id = 'themeToggleBtn';
    themeBtn.className = 'connect-button toolbar-button';
    themeBtn.onclick = toggleTheme;

    const dfuBtn = document.createElement('button');
    dfuBtn.id = 'dfuModeBtn';
    dfuBtn.textContent = 'DFU';
    dfuBtn.className = 'connect-button toolbar-button dfu-button';
    dfuBtn.onclick = requestDfuMode;
    dfuBtn.style.display = 'none';

    const toolbarSlot = document.getElementById('toolbarSlot');
    if (toolbarSlot) {
        toolbarSlot.appendChild(themeBtn);
        toolbarSlot.appendChild(dfuBtn);
        toolbarSlot.appendChild(connectBtn);
    }
    applyTheme(state.theme);

    syncVelocityControls(parseFloat(document.getElementById('velocityInput')?.value || '1.0'));
    syncPositionControls(parseFloat(document.getElementById('positionInput')?.value || '0.0'));
    ['velocityRampInput', 'positionVelocityLimitInput', 'positionAccelLimitInput', 'positionDecelLimitInput'].forEach((id) => {
        const input = document.getElementById(id);
        if (input) {
            input.addEventListener('change', () => markDirty(input));
            input.addEventListener('blur', () => markDirty(input));
        }
    });
    refreshTelemetryPollingUI();
    drawTracePlot();

    if (webSerialSupported()) {
        console.log('Dashboard initialized. Click "Connect Serial Port" to start.');
    } else {
        console.warn('Dashboard initialized, but Web Serial is unavailable in this browser.');
        appendDebugConsole('SYS Web Serial unavailable. Use Chrome or Edge on desktop.', 'sys');
    }
  });

/**
 * Handle page unload - disconnect gracefully
 */
window.addEventListener('beforeunload', disconnectSerialPort);
