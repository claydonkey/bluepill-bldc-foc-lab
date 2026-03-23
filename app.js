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
const state = {
    port: null,
    reader: null,
    isConnected: false,
    inputBuffer: '',
    velocityHistory: [],
    velocityHistoryLimit: 120,
    positionHistory: [],
    positionHistoryLimit: 120,
    traceMode: 'velocity',
    autotuneResult: null
};

/**
 * Initialize the Web Serial API connection
 */
async function connectSerialPort() {
    try {
        // Request a port from the user
        state.port = await navigator.serial.requestPort();

        // Open the port at 115200 baud (or adjust to your MCU's baud rate)
        await state.port.open({ baudRate: 115200 });

        state.isConnected = true;
        updateConnectionStatus(true);
        console.log('Connected to serial port');

        // Start reading data
        readSerialData();
        await sendCommand('GET_MODULATION');
    } catch (error) {
        console.error('Failed to connect:', error);
        updateConnectionStatus(false);
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
                }
                // Check for alignment offset calibration
                else if (data.align_offset !== undefined) {
                    updateAlignmentDisplay(data);
                }
                // Check for PID parameters
                else if (data.pid !== undefined) {
                    updatePIDDisplay(data);
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
                    updateVelocityDisplay(data.foc);
                    updateFOCDebugDisplay(data.foc);
                }
                // Check for diagnostic (loop count, message status)
                else if (data.diag !== undefined) {
                    updateDiagnosticDisplay(data.diag);
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
                else if (data.autotune !== undefined) {
                    updateAutotuneDisplay(data.autotune);
                } else {
                    console.log('Unknown JSON:', data);
                }
            }
            // Handle text responses
            else {
                console.log('MCU Response:', line);
            }
        } catch (error) {
            // Not JSON, treat as debug message
            if (line.length > 0) {
                console.log('MCU Message:', line);
            }
        }
    }
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
    } else if (data.t !== undefined) {
        const target = parseFloat(data.t).toFixed(2);
        document.getElementById('targetVelocity').textContent = target;
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

/**
 * Update target velocity display
 * @param {Object} data - Parsed JSON object with target_vel property
 */
function updateTargetVelocityDisplay(data) {
    if (data.target_vel !== undefined) {
        const target = parseFloat(data.target_vel).toFixed(2);
        document.getElementById('targetVelocity').textContent = target;
        console.log('Target velocity set to:', target, 'rad/s');
    }
}

function updatePositionDisplay(data) {
    if (data.target_pos !== undefined) {
        document.getElementById('positionInput').value = parseFloat(data.target_pos).toFixed(3);
    } else if (data.tp !== undefined) {
        document.getElementById('positionInput').value = parseFloat(data.tp).toFixed(3);
    }
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
}

/**
 * Send a command to the MCU
 * @param {String} command - Command to send (e.g., 'START', 'STOP')
 */
async function sendCommand(command) {
    if (!state.isConnected || !state.port) {
        alert('Not connected to serial port. Click the connect button first.');
        return;
    }

    try {
        const writer = state.port.writable.getWriter();
        const data = new TextEncoder().encode(command + '\n');
        await writer.write(data);
        writer.releaseLock();
        console.log('Sent command:', command);

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
            state.velocityHistory = [];
            state.positionHistory = [];
            drawTracePlot();
        }
    } catch (error) {
        console.error('Failed to send command:', error);
    }
}

/**
 * Set motor velocity via USB
 */
async function setVelocity() {
    const input = document.getElementById('velocityInput');
    const velocity = parseFloat(input.value);

    if (isNaN(velocity) || velocity < -100 || velocity > 100) {
        alert('Please enter a valid velocity between -100 and 100 rad/s');
        return;
    }

    const command = `SET_VELOCITY:${velocity.toFixed(2)}`;
    await sendCommand(command);
}

async function setPosition() {
    const input = document.getElementById('positionInput');
    const position = parseFloat(input.value);

    if (isNaN(position) || position < -1000 || position > 1000) {
        alert('Please enter a valid position between -1000 and 1000 rad');
        return;
    }

    await sendCommand(`SET_POSITION:${position.toFixed(3)}`);
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

function pushVelocitySample(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    state.velocityHistory.push(value);
    if (state.velocityHistory.length > state.velocityHistoryLimit) {
        state.velocityHistory.shift();
    }
    drawTracePlot();
}

function pushPositionSample(value) {
    if (!Number.isFinite(value)) {
        return;
    }

    state.positionHistory.push(value);
    if (state.positionHistory.length > state.positionHistoryLimit) {
        state.positionHistory.shift();
    }
    drawTracePlot();
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

    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    const paddingX = 14;
    const paddingY = 16;
    const innerWidth = width - (paddingX * 2);
    const innerHeight = height - (paddingY * 2);
    const zeroY = paddingY + (innerHeight * 0.5);
    const history = state.traceMode === 'position' ? state.positionHistory : state.velocityHistory;
    const maxAbs = history.length > 0 ? Math.max(0.5, ...history.map(sample => Math.abs(sample))) : 0.5;

    ctx.clearRect(0, 0, width, height);

    ctx.save();
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.55)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
        const y = paddingY + (innerHeight * i / 4);
        ctx.beginPath();
        ctx.moveTo(paddingX, y);
        ctx.lineTo(width - paddingX, y);
        ctx.stroke();
    }

    ctx.strokeStyle = 'rgba(37, 99, 235, 0.75)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(paddingX, zeroY);
    ctx.lineTo(width - paddingX, zeroY);
    ctx.stroke();

    if (history.length > 1) {
        const gradient = ctx.createLinearGradient(0, 0, width, 0);
        gradient.addColorStop(0, '#14b8a6');
        gradient.addColorStop(1, '#2563eb');

        ctx.strokeStyle = gradient;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.beginPath();

        history.forEach((sample, index) => {
            const x = paddingX + (innerWidth * index / (state.velocityHistoryLimit - 1));
            const y = zeroY - ((sample / maxAbs) * (innerHeight * 0.45));
            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });
        ctx.stroke();

        const lastValue = history[history.length - 1];
        const lastX = paddingX + (innerWidth * (history.length - 1) / (state.velocityHistoryLimit - 1));
        const lastY = zeroY - ((lastValue / maxAbs) * (innerHeight * 0.45));
        ctx.fillStyle = '#0f766e';
        ctx.beginPath();
        ctx.arc(lastX, lastY, 4, 0, Math.PI * 2);
        ctx.fill();
    }

    ctx.restore();
    if (state.traceMode === 'position') {
        meta.textContent = `Window: ${history.length} samples | Scale: ±${maxAbs.toFixed(2)} rad`;
        note.textContent = 'Rolling trace of measured mechanical position from FOC telemetry.';
    } else {
        meta.textContent = `Window: ${history.length} samples | Scale: ±${maxAbs.toFixed(2)} rad/s`;
        note.textContent = 'Rolling trace of measured velocity from FOC telemetry.';
    }
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

/**
 * Disconnect from serial port
 */
async function disconnectSerialPort() {
    try {
        if (state.reader) {
            await state.reader.cancel();
        }
        if (state.port) {
            await state.port.close();
        }
        state.isConnected = false;
        state.port = null;
        state.reader = null;
        updateConnectionStatus(false);
        console.log('Disconnected from serial port');
    } catch (error) {
        console.error('Disconnect error:', error);
    }
}

/**
 * Initialize connection button listener
 */
document.addEventListener('DOMContentLoaded', function () {
    const connectBtn = document.createElement('button');
    connectBtn.textContent = 'Connect Serial Port';
    connectBtn.className = 'connect-button';
    connectBtn.onclick = connectSerialPort;

    const toolbarSlot = document.getElementById('toolbarSlot');
    if (toolbarSlot) {
        toolbarSlot.appendChild(connectBtn);
    }

    drawTracePlot();

    console.log('Dashboard initialized. Click "Connect Serial Port" to start.');
});

/**
 * Handle page unload - disconnect gracefully
 */
window.addEventListener('beforeunload', disconnectSerialPort);
