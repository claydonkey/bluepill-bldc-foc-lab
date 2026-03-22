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
    inputBuffer: ''
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
                // Check for FOC debug information
                else if (data.foc !== undefined) {
                    updateFOCDebugDisplay(data.foc);
                }
                // Check for motor driver status
                else if (data.motor_driver !== undefined) {
                    console.log('Motor driver:', data.motor_driver);
                    document.getElementById('focStatus').textContent = 'Driver Enabled';
                }
                // Check for motor running status
                else if (data.motor_status !== undefined) {
                    console.log('Motor status:', data.motor_status);
                    document.getElementById('focStatus').textContent = 'Running';
                }
                // Check for open-loop test status
                else if (data.open_loop !== undefined) {
                    console.log('Open-loop test:', data.open_loop);
                    document.getElementById('focStatus').textContent = 'Open-Loop Test';
                }
                // Check for diagnostic (loop count, message status)
                else if (data.diag !== undefined) {
                    updateDiagnosticDisplay(data.diag);
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
    } else if (data.v !== undefined) {
        const velocity = parseFloat(data.v).toFixed(2);
        document.getElementById('velocity').textContent = velocity;
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

    // Update FOC status indicator
    document.getElementById('focStatus').textContent = 'Active';
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

/**
 * Update connection status UI
 * @param {Boolean} connected - Current connection state
 */
function updateConnectionStatus(connected) {
    const statusElement = document.getElementById('connectionStatus');
    const statusText = document.getElementById('connectionText');

    if (connected) {
        statusElement.classList.add('connected');
        statusElement.classList.remove('disconnected');
        statusText.textContent = 'Connected';
    } else {
        statusElement.classList.add('disconnected');
        statusElement.classList.remove('connected');
        statusText.textContent = 'Disconnected';
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

    if (isNaN(velocity) || velocity < -2 || velocity > 2) {
        alert('Please enter a valid velocity between -2 and 2 rad/s');
        return;
    }

    const command = `SET_VELOCITY:${velocity.toFixed(2)}`;
    await sendCommand(command);
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
document.addEventListener('DOMContentLoaded', function() {
    // Add connection button
    const header = document.querySelector('h1');
    const connectBtn = document.createElement('button');
    connectBtn.textContent = '🔌 Connect Serial Port';
    connectBtn.style.cssText = `
        position: absolute;
        top: 20px;
        right: 20px;
        padding: 10px 15px;
        background: #667eea;
        color: white;
        border: none;
        border-radius: 8px;
        cursor: pointer;
        font-size: 0.9em;
        font-weight: 600;
    `;
    connectBtn.onclick = connectSerialPort;

    // Insert button in the container
    const container = document.querySelector('.container');
    container.style.position = 'relative';
    container.insertBefore(connectBtn, container.firstChild);

    console.log('Dashboard initialized. Click "Connect Serial Port" to start.');
});

/**
 * Handle page unload - disconnect gracefully
 */
window.addEventListener('beforeunload', disconnectSerialPort);
