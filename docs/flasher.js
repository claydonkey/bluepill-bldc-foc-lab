/*
SPDX-License-Identifier: MIT
Copyright (c) 2026 Anthony Campbell (claydonkey)
*/

const THEME_STORAGE_KEY = 'bluepill_foc_theme';
const DFU_REQUEST = {
    DETACH: 0,
    DNLOAD: 1,
    UPLOAD: 2,
    GETSTATUS: 3,
    CLRSTATUS: 4,
    GETSTATE: 5,
    ABORT: 6
};

const DFU_STATE = {
    APP_IDLE: 0,
    APP_DETACH: 1,
    DFU_IDLE: 2,
    DFU_DNLOAD_SYNC: 3,
    DFU_DNBUSY: 4,
    DFU_DNLOAD_IDLE: 5,
    DFU_MANIFEST_SYNC: 6,
    DFU_MANIFEST: 7,
    DFU_MANIFEST_WAIT_RESET: 8,
    DFU_UPLOAD_IDLE: 9,
    DFU_ERROR: 10
};

const DFU_STATUS_TEXT = {
    0x00: 'OK',
    0x01: 'Target error',
    0x02: 'File error',
    0x03: 'Write error',
    0x04: 'Erase error',
    0x05: 'Check erased failed',
    0x06: 'Program error',
    0x07: 'Verify error',
    0x08: 'Address error',
    0x09: 'Not done',
    0x0A: 'Firmware error',
    0x0B: 'Vendor error',
    0x0C: 'USB reset needed',
    0x0D: 'Power-on reset needed',
    0x0E: 'Unknown error',
    0x0F: 'Stalled packet'
};

const DFU_STATE_TEXT = {
    [DFU_STATE.APP_IDLE]: 'appIDLE',
    [DFU_STATE.APP_DETACH]: 'appDETACH',
    [DFU_STATE.DFU_IDLE]: 'dfuIDLE',
    [DFU_STATE.DFU_DNLOAD_SYNC]: 'dfuDNLOAD-SYNC',
    [DFU_STATE.DFU_DNBUSY]: 'dfuDNBUSY',
    [DFU_STATE.DFU_DNLOAD_IDLE]: 'dfuDNLOAD-IDLE',
    [DFU_STATE.DFU_MANIFEST_SYNC]: 'dfuMANIFEST-SYNC',
    [DFU_STATE.DFU_MANIFEST]: 'dfuMANIFEST',
    [DFU_STATE.DFU_MANIFEST_WAIT_RESET]: 'dfuMANIFEST-WAIT-RESET',
    [DFU_STATE.DFU_UPLOAD_IDLE]: 'dfuUPLOAD-IDLE',
    [DFU_STATE.DFU_ERROR]: 'dfuERROR'
};

const state = {
    theme: 'light',
    device: null,
    interfaceNumber: null,
    alternateSetting: null,
    interfaceName: '',
    file: null,
    firmwareBytes: null,
    busy: false
};

function $(id) {
    return document.getElementById(id);
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
    const themeBtn = $('themeToggleBtn');
    if (themeBtn) {
        themeBtn.textContent = nextTheme === 'dark' ? 'Light Theme' : 'Dark Theme';
    }
}

function loadStoredTheme() {
    try {
        const stored = window.localStorage.getItem(THEME_STORAGE_KEY);
        return stored === 'dark' || stored === 'light' ? stored : 'light';
    } catch (error) {
        return 'light';
    }
}

function toggleTheme() {
    applyTheme(state.theme === 'dark' ? 'light' : 'dark');
}

function webUsbSupported() {
    return typeof navigator !== 'undefined' &&
        !!navigator.usb &&
        typeof navigator.usb.requestDevice === 'function';
}

function log(message, type = 'info') {
    const output = $('logOutput');
    const stamp = new Date().toLocaleTimeString();
    const prefix = type === 'error' ? 'ERR' : type === 'success' ? 'OK ' : 'SYS';
    output.textContent += `[${stamp}] ${prefix} ${message}\n`;
    output.scrollTop = output.scrollHeight;
}

function clearLog() {
    $('logOutput').textContent = '';
}

function formatHex(value) {
    return `0x${Number(value >>> 0).toString(16).toUpperCase().padStart(8, '0')}`;
}

function parseNumericInput(value, fallback) {
    if (typeof value !== 'string') {
        return fallback;
    }
    const trimmed = value.trim();
    if (!trimmed) {
        return fallback;
    }
    const parsed = trimmed.toLowerCase().startsWith('0x')
        ? Number.parseInt(trimmed, 16)
        : Number.parseInt(trimmed, 10);
    return Number.isFinite(parsed) ? parsed : fallback;
}

function delay(ms) {
    return new Promise(resolve => window.setTimeout(resolve, ms));
}

function setProgress(value, label) {
    const next = Math.max(0, Math.min(100, value));
    const progressBar = $('progressBarFill');
    if (progressBar) {
        if ('value' in progressBar) {
            progressBar.value = next;
        } else {
            progressBar.style.width = `${next}%`;
        }
    }
    $('progressValue').textContent = `${Math.round(next)}%`;
    if (label) {
        $('progressLabel').textContent = label;
    }
}

function updateUi() {
    const connected = !!state.device;
    const fileLoaded = !!state.firmwareBytes;
    const webUsb = webUsbSupported();
    $('webUsbStatus').textContent = webUsb ? 'Available' : 'Unavailable';
    $('deviceStatus').textContent = connected ? 'Connected' : 'Disconnected';
    $('fileStatus').textContent = state.file ? state.file.name : 'None';
    $('targetAddressStat').textContent = $('targetAddressInput').value.trim() || '0x08001000';
    $('connectDfuBtn').textContent = connected ? 'Reconnect DFU Device' : 'Connect DFU Device';
    $('flashBtn').style.display = connected ? 'inline' : 'none';
    $('flashBtn').disabled = !connected || !fileLoaded || state.busy;
    $('leaveDfuBtn').disabled = !connected || state.busy;
    $('connectDfuBtn').disabled = state.busy || !webUsb;
    $('firmwareFile').disabled = state.busy;
    $('targetAddressInput').disabled = state.busy;
    $('chunkSizeInput').disabled = state.busy;
}

async function readSelectedFile(file) {
    const buffer = await file.arrayBuffer();
    state.file = file;
    state.firmwareBytes = new Uint8Array(buffer);
    $('fileMeta').textContent = `${file.name} • ${state.firmwareBytes.length} bytes`;
    log(`Loaded ${file.name} (${state.firmwareBytes.length} bytes).`);
    updateUi();
}

function findDfuInterface(device) {
    const configuration = device.configuration;
    if (!configuration) {
        return null;
    }

    for (const iface of configuration.interfaces) {
        for (const alternate of iface.alternates) {
            if (alternate.interfaceClass === 0xfe && alternate.interfaceSubclass === 0x01) {
                return {
                    interfaceNumber: iface.interfaceNumber,
                    alternateSetting: alternate.alternateSetting,
                    interfaceName: alternate.interfaceName || alternate.interfaceProtocol || 'DFU Interface'
                };
            }
        }
    }

    return null;
}

async function connectDfuDevice() {
    if (!webUsbSupported()) {
        log('WebUSB is not available in this browser. Use Chrome or Edge on desktop over HTTPS.', 'error');
        updateUi();
        return;
    }

    try {
        const device = await navigator.usb.requestDevice({
            filters: [
                { classCode: 0xfe, subclassCode: 0x01 },
                { vendorId: 0x0483 },
                { vendorId: 0x1eaf }
            ]
        });

        await device.open();
        if (!device.configuration) {
            await device.selectConfiguration(1);
        }

        const dfuInfo = findDfuInterface(device);
        if (!dfuInfo) {
            throw new Error('No DFU interface was found on the selected device.');
        }

        await device.claimInterface(dfuInfo.interfaceNumber);
        await device.selectAlternateInterface(dfuInfo.interfaceNumber, dfuInfo.alternateSetting);

        state.device = device;
        state.interfaceNumber = dfuInfo.interfaceNumber;
        state.alternateSetting = dfuInfo.alternateSetting;
        state.interfaceName = dfuInfo.interfaceName;
        log(`Connected to ${device.productName || 'DFU device'} on interface ${dfuInfo.interfaceNumber}, alt ${dfuInfo.alternateSetting}.`, 'success');
        if (dfuInfo.interfaceName) {
            log(`DFU target: ${dfuInfo.interfaceName}`);
        }
        await recoverToIdle();
    } catch (error) {
        log(`Failed to connect to DFU device: ${error.message || error}`, 'error');
    }

    updateUi();
}

async function controlTransferOut(request, value, data) {
    const result = await state.device.controlTransferOut({
        requestType: 'class',
        recipient: 'interface',
        request,
        value,
        index: state.interfaceNumber
    }, data);

    if (result.status !== 'ok') {
        throw new Error(`USB OUT request ${request} failed (${result.status}).`);
    }
}

async function controlTransferIn(request, value, length) {
    const result = await state.device.controlTransferIn({
        requestType: 'class',
        recipient: 'interface',
        request,
        value,
        index: state.interfaceNumber
    }, length);

    if (result.status !== 'ok' || !result.data) {
        throw new Error(`USB IN request ${request} failed (${result.status}).`);
    }

    return result.data;
}

async function getStatus() {
    const data = await controlTransferIn(DFU_REQUEST.GETSTATUS, 0, 6);
    return {
        status: data.getUint8(0),
        pollTimeout: data.getUint8(1) | (data.getUint8(2) << 8) | (data.getUint8(3) << 16),
        state: data.getUint8(4),
        iString: data.getUint8(5)
    };
}

async function getState() {
    const data = await controlTransferIn(DFU_REQUEST.GETSTATE, 0, 1);
    return data.getUint8(0);
}

async function clearStatus() {
    await controlTransferOut(DFU_REQUEST.CLRSTATUS, 0, new Uint8Array());
}

async function abortDfu() {
    await controlTransferOut(DFU_REQUEST.ABORT, 0, new Uint8Array());
}

async function pollUntilReady(contextLabel) {
    while (true) {
        const status = await getStatus();
        if (status.status !== 0x00) {
            throw new Error(`${contextLabel} failed: ${DFU_STATUS_TEXT[status.status] || `status 0x${status.status.toString(16)}`}, state ${DFU_STATE_TEXT[status.state] || status.state}`);
        }

        if (status.pollTimeout > 0) {
            await delay(status.pollTimeout);
        }

        if (status.state === DFU_STATE.DFU_IDLE || status.state === DFU_STATE.DFU_DNLOAD_IDLE) {
            return status;
        }

        if (status.state === DFU_STATE.DFU_MANIFEST_WAIT_RESET) {
            return status;
        }
    }
}

async function recoverToIdle() {
    try {
        const stateCode = await getState();
        if (stateCode === DFU_STATE.DFU_ERROR) {
            await clearStatus();
            await delay(20);
            log('Cleared DFU error state.');
            return;
        }
        if (stateCode === DFU_STATE.DFU_DNLOAD_IDLE || stateCode === DFU_STATE.DFU_UPLOAD_IDLE) {
            await abortDfu();
            await delay(20);
            log('Aborted previous DFU transfer to return to idle.');
        }
    } catch (error) {
        log(`DFU state recovery note: ${error.message || error}`, 'error');
    }
}

function createDfuCommand(command, address) {
    const payload = new Uint8Array(5);
    payload[0] = command;
    payload[1] = address & 0xff;
    payload[2] = (address >>> 8) & 0xff;
    payload[3] = (address >>> 16) & 0xff;
    payload[4] = (address >>> 24) & 0xff;
    return payload;
}

async function setAddressPointer(address) {
    await controlTransferOut(DFU_REQUEST.DNLOAD, 0, createDfuCommand(0x21, address));
    await pollUntilReady(`Set address ${formatHex(address)}`);
}

async function erasePage(address) {
    await controlTransferOut(DFU_REQUEST.DNLOAD, 0, createDfuCommand(0x41, address));
    await pollUntilReady(`Erase page ${formatHex(address)}`);
}

async function writeChunk(blockNumber, data) {
    await controlTransferOut(DFU_REQUEST.DNLOAD, blockNumber, data);
    await pollUntilReady(`Write block ${blockNumber}`);
}

async function leaveDfuMode() {
    if (!state.device) {
        return;
    }

    try {
        await controlTransferOut(DFU_REQUEST.DNLOAD, 0, new Uint8Array());
        const status = await getStatus();
        if (status.pollTimeout > 0) {
            await delay(status.pollTimeout);
        }
        log('Sent leave-DFU request. The device may disconnect and reboot.', 'success');
    } catch (error) {
        log(`Leave DFU note: ${error.message || error}`, 'error');
    }
}

async function flashFirmware() {
    if (!state.device || !state.firmwareBytes) {
        return;
    }

    const startAddress = parseNumericInput($('targetAddressInput').value, 0x08001000);
    const chunkSize = parseNumericInput($('chunkSizeInput').value, 1024);
    if (!Number.isFinite(startAddress) || startAddress < 0x08001000) {
        log('Refusing to flash below 0x08001000.', 'error');
        return;
    }
    if (!Number.isFinite(chunkSize) || chunkSize <= 0 || chunkSize > 2048) {
        log('Chunk size must be between 1 and 2048 bytes.', 'error');
        return;
    }

    state.busy = true;
    updateUi();
    setProgress(0, 'Preparing');

    try {
        await recoverToIdle();

        const pageSize = 1024;
        const totalPages = Math.ceil(state.firmwareBytes.length / pageSize);
        const startPageAddress = startAddress - (startAddress % pageSize);

        log(`Starting flash of ${state.firmwareBytes.length} bytes to ${formatHex(startAddress)}.`);
        for (let page = 0; page < totalPages; page += 1) {
            const pageAddress = startPageAddress + (page * pageSize);
            setProgress((page / Math.max(totalPages, 1)) * 25, `Erasing page ${page + 1}/${totalPages}`);
            await erasePage(pageAddress);
            log(`Erased ${formatHex(pageAddress)}.`);
        }

        await setAddressPointer(startAddress);
        log(`Address pointer set to ${formatHex(startAddress)}.`);

        const totalBlocks = Math.ceil(state.firmwareBytes.length / chunkSize);
        for (let block = 0; block < totalBlocks; block += 1) {
            const start = block * chunkSize;
            const end = Math.min(start + chunkSize, state.firmwareBytes.length);
            const chunk = state.firmwareBytes.slice(start, end);
            await writeChunk(2 + block, chunk);
            const progress = 25 + (((block + 1) / Math.max(totalBlocks, 1)) * 70);
            setProgress(progress, `Writing block ${block + 1}/${totalBlocks}`);
        }

        setProgress(96, 'Leaving DFU');
        await leaveDfuMode();
        setProgress(100, 'Flash complete');
        log('Flash sequence completed. If the board does not reboot by itself, power-cycle it once.', 'success');
    } catch (error) {
        setProgress(0, 'Flash failed');
        log(`Flash failed: ${error.message || error}`, 'error');
    } finally {
        state.busy = false;
        updateUi();
    }
}

function handleFileChange(event) {
    const [file] = event.target.files || [];
    if (!file) {
        state.file = null;
        state.firmwareBytes = null;
        $('fileMeta').textContent = 'No file selected.';
        updateUi();
        return;
    }

    readSelectedFile(file).catch(error => {
        log(`Failed to read file: ${error.message || error}`, 'error');
    });
}

function init() {
    applyTheme(loadStoredTheme());
    $('themeToggleBtn').addEventListener('click', toggleTheme);
    $('connectDfuBtn').addEventListener('click', connectDfuDevice);
    $('flashBtn').addEventListener('click', flashFirmware);
    $('leaveDfuBtn').addEventListener('click', leaveDfuMode);
    $('clearLogBtn').addEventListener('click', clearLog);
    $('firmwareFile').addEventListener('change', handleFileChange);
    $('targetAddressInput').addEventListener('input', updateUi);
    updateUi();

    if (webUsbSupported()) {
        log('WebUSB detected. Put the board in DFU mode, then connect it here.');
    } else {
        log('WebUSB is unavailable in this browser. Use Chrome or Edge on desktop over HTTPS.', 'error');
    }
}

window.addEventListener('DOMContentLoaded', init);
