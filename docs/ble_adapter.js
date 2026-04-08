/**
 * ble_adapter.js — unified BLE abstraction
 *
 * Native (Android app):
 *   - Scans Coded + 1M PHY
 *   - Batches packets at 16ms intervals, calls onBLEBatch(streamIdx, ArrayBuffer[])
 *   - JS handler receives an array of ArrayBuffers per stream per tick
 *
 * Browser (Web Bluetooth):
 *   - Standard requestDevice / characteristic subscription
 *   - onBLEBatch called with a single-element array per event
 *     so the page handler always receives an array in both cases
 */

const NativeBLE = (() => {
  const native   = window.AndroidBLE;
  const isNative = !!native;

  let _onDeviceFound  = null;
  let _onConnected    = null;
  let _onDisconnected = null;
  let _onBatch        = null;   // (streamIdx, ArrayBuffer[]) — always an array
  let _onError        = null;

  let _webDevice   = null;
  let _webModeChar = null;

  // ── Native callbacks ───────────────────────────────────────────────────────
  window.onBLEDeviceFound  = (id, name) => _onDeviceFound?.(id, name);
  window.onBLEConnected    = ()         => _onConnected?.();
  window.onBLEDisconnected = ()         => _onDisconnected?.();
  window.onBLEError        = (msg)      => _onError?.(msg);

  // Batch: array of base64 strings → array of ArrayBuffers
  window.onBLEBatch = (streamIdx, b64Array) => {
    if (!_onBatch) return;
    const buffers = b64Array.map(b64 => {
      const bin = atob(b64);
      const buf = new ArrayBuffer(bin.length);
      const u8  = new Uint8Array(buf);
      for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
      return buf;
    });
    _onBatch(streamIdx, buffers);
  };

  // ── Web Bluetooth single-packet wrapper ────────────────────────────────────
  // Wraps single characteristicvaluechanged events as single-element arrays
  // so the page handler is identical for both paths
  function makeWebHandler(streamIdx) {
    return (event) => {
      _onBatch?.(streamIdx, [event.target.value.buffer]);
    };
  }

  // Sequential GATT helper — Chrome queue overflows with concurrent ops
  async function seqMap(arr, fn) {
    const out = [];
    for (const x of arr) out.push(await fn(x));
    return out;
  }

  return {
    isNative,

    async requestDevice(serviceUUID, dataUUIDs, modeUUID, {
      onDeviceFound, onConnected, onDisconnected, onBatch, onError
    }) {
      _onDeviceFound  = onDeviceFound;
      _onConnected    = onConnected;
      _onDisconnected = onDisconnected;
      _onBatch        = onBatch;
      _onError        = onError;

      if (isNative) {
        native.scan(serviceUUID);
        // onBLEDeviceFound → caller calls connect()
      } else {
        if (!navigator.bluetooth) { onError?.('Web Bluetooth not available'); return; }
        try {
          _webDevice = await navigator.bluetooth.requestDevice({
            filters: [{ services: [serviceUUID] }]
          });
          _webDevice.addEventListener('gattserverdisconnected', () => _onDisconnected?.());
          onDeviceFound?.(_webDevice.id, _webDevice.name ?? 'Unknown');
        } catch(e) { onError?.(e.message); }
      }
    },

    async connect(deviceIdOrAddress, serviceUUID, dataUUIDs, modeUUID) {
      if (isNative) {
        native.connect(deviceIdOrAddress);
        // onBLEConnected fires when GATT ready
      } else {
        try {
          const server = await _webDevice.gatt.connect();
          const svc    = await server.getPrimaryService(serviceUUID);
          _webModeChar = await svc.getCharacteristic(modeUUID);
          // Sequential subscription
          const chars = await seqMap(dataUUIDs, u => svc.getCharacteristic(u));
          await seqMap(chars, c => c.startNotifications());
          chars.forEach((c, i) => {
            c.addEventListener('characteristicvaluechanged', makeWebHandler(i));
          });
          _onConnected?.();
        } catch(e) { _onError?.(e.message); }
      }
    },

    // Native: subscribe with stream index so Kotlin knows which queue to use
    subscribe(serviceUUID, charUUID, streamIdx) {
      if (isNative) native.subscribe(serviceUUID, charUUID, streamIdx);
      // Web: already subscribed in connect()
    },

    writeMode(serviceUUID, charUUID, byteValue) {
      if (isNative) {
        native.write(serviceUUID, charUUID, byteValue.toString(16).padStart(2, '0'));
      } else {
        _webModeChar?.writeValue(new Uint8Array([byteValue]));
      }
    },

    disconnect() {
      if (isNative) native.disconnect();
      else if (_webDevice?.gatt?.connected) _webDevice.gatt.disconnect();
      _onDeviceFound = _onConnected = _onDisconnected = _onBatch = _onError = null;
      _webDevice = _webModeChar = null;
    }
  };
})();
