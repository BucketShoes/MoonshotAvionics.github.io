/**
 * ble_adapter.js
 * Drop-in BLE abstraction that uses the native Android bridge when available,
 * and falls back to Web Bluetooth in a regular browser.
 *
 * Usage: replace all navigator.bluetooth calls with NativeBLE calls.
 * The API surface is identical either way.
 *
 * Include this before your main script.
 */

const NativeBLE = (() => {
  const native = window.AndroidBLE;  // set by Android WebView bridge
  const isNative = !!native;

  // ── State ──────────────────────────────────────────────────────────────────
  let _onDeviceFound = null;
  let _onConnected   = null;
  let _onDisconnected = null;
  let _onData        = null;
  let _onError       = null;

  // ── Native callbacks (called from Kotlin via evaluateJavascript) ───────────
  // These are attached to window so Kotlin can call them
  window.onBLEDeviceFound = (id, name) => _onDeviceFound?.(id, name);
  window.onBLEConnected   = ()         => _onConnected?.();
  window.onBLEDisconnected= ()         => _onDisconnected?.();
  window.onBLEError       = (msg)      => _onError?.(msg);
  window.onBLEData        = (arr)      => _onData?.(new Uint8Array(arr).buffer);

  // ── Web Bluetooth fallback internals ──────────────────────────────────────
  let _webDevice = null;
  let _webChar   = null;

  // ── Public API ─────────────────────────────────────────────────────────────
  return {
    isNative,

    /**
     * Scan for and connect to a device with the given service UUID.
     * In native mode: scans on all PHYs including Coded, calls back via
     * onDeviceFound → connect separately.
     * In web mode: opens browser chooser, connects immediately.
     */
    async requestDevice(serviceUUID, {
      onDeviceFound, onConnected, onDisconnected, onData, onError
    }) {
      _onDeviceFound  = onDeviceFound;
      _onConnected    = onConnected;
      _onDisconnected = onDisconnected;
      _onData         = onData;
      _onError        = onError;

      if (isNative) {
        native.scan(serviceUUID);
        // After onDeviceFound fires, caller should call NativeBLE.connect(id)
      } else {
        // Web Bluetooth path
        try {
          _webDevice = await navigator.bluetooth.requestDevice({
            filters: [{ services: [serviceUUID] }]
          });
          _webDevice.addEventListener('gattserverdisconnected', () => {
            _onDisconnected?.();
          });
          // Return device for web path — caller connects manually
          return _webDevice;
        } catch(e) {
          _onError?.(e.message);
        }
      }
    },

    async connect(deviceIdOrWebDevice, serviceUUID, dataCharUUID, modeCharUUID) {
      if (isNative) {
        native.connect(deviceIdOrWebDevice);
        // onBLEConnected fires when GATT is ready, then caller calls subscribe
      } else {
        // Web Bluetooth connect
        try {
          const server = await _webDevice.gatt.connect();
          const svc    = await server.getPrimaryService(serviceUUID);
          _webChar     = await svc.getCharacteristic(dataCharUUID);
          await _webChar.startNotifications();
          _webChar.addEventListener('characteristicvaluechanged', e => {
            _onData?.(e.target.value.buffer);
          });
          _onConnected?.();
          // Store mode char for writes
          this._webModeChar = await svc.getCharacteristic(modeCharUUID);
        } catch(e) {
          _onError?.(e.message);
        }
      }
    },

    subscribe(serviceUUID, charUUID) {
      if (isNative) native.subscribe(serviceUUID, charUUID);
      // Web Bluetooth: already subscribed in connect()
    },

    writeMode(serviceUUID, charUUID, byteValue) {
      if (isNative) {
        const hex = byteValue.toString(16).padStart(2, '0');
        native.write(serviceUUID, charUUID, hex);
      } else {
        // Web Bluetooth
        this._webModeChar?.writeValue(new Uint8Array([byteValue]));
      }
    },

    disconnect() {
      if (isNative) {
        native.disconnect();
      } else {
        if (_webDevice?.gatt.connected) _webDevice.gatt.disconnect();
      }
    }
  };
})();
