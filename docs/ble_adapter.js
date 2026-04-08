/**
 * ble_adapter.js
 * Unified BLE abstraction: native Android bridge or Web Bluetooth fallback.
 * Include before your main script.
 */

const NativeBLE = (() => {
  const native   = window.AndroidBLE;
  const isNative = !!native;

  let _onDeviceFound  = null;
  let _onConnected    = null;
  let _onDisconnected = null;
  let _onData         = null;
  let _onError        = null;

  // Web Bluetooth state
  let _webDevice   = null;
  let _webModeChar = null;

  // ── Native callbacks (Kotlin calls these via evaluateJavascript) ──────────
  window.onBLEDeviceFound  = (id, name) => _onDeviceFound?.(id, name);
  window.onBLEConnected    = ()         => _onConnected?.();
  window.onBLEDisconnected = ()         => _onDisconnected?.();
  window.onBLEError        = (msg)      => _onError?.(msg);

  // Data arrives as base64 from native (avoids huge comma arrays for 500B packets)
  window.onBLEDataB64 = (b64) => {
    const binary = atob(b64);
    const buf    = new ArrayBuffer(binary.length);
    const view   = new Uint8Array(buf);
    for (let i = 0; i < binary.length; i++) view[i] = binary.charCodeAt(i);
    _onData?.(buf);
  };

  return {
    isNative,

    async requestDevice(serviceUUID, { onDeviceFound, onConnected, onDisconnected, onData, onError }) {
      _onDeviceFound  = onDeviceFound;
      _onConnected    = onConnected;
      _onDisconnected = onDisconnected;
      _onData         = onData;
      _onError        = onError;

      if (isNative) {
        native.scan(serviceUUID);
        // onBLEDeviceFound fires → caller calls connect()
      } else {
        if (!navigator.bluetooth) { onError?.('Web Bluetooth not available'); return; }
        try {
          _webDevice = await navigator.bluetooth.requestDevice({
            filters: [{ services: [serviceUUID] }]
          });
          _webDevice.addEventListener('gattserverdisconnected', () => _onDisconnected?.());
          onDeviceFound?.(_webDevice.id, _webDevice.name ?? 'Unknown');
        } catch(e) {
          onError?.(e.message);
        }
      }
    },

    async connect(deviceIdOrAddress, serviceUUID, dataCharUUID, modeCharUUID) {
      if (isNative) {
        native.connect(deviceIdOrAddress);
        // onBLEConnected fires when GATT ready
      } else {
        try {
          const server = await _webDevice.gatt.connect();
          const svc    = await server.getPrimaryService(serviceUUID);
          const dataChar = await svc.getCharacteristic(dataCharUUID);
          _webModeChar   = await svc.getCharacteristic(modeCharUUID);
          await dataChar.startNotifications();
          dataChar.addEventListener('characteristicvaluechanged', e => {
            _onData?.(e.target.value.buffer);
          });
          _onConnected?.();
        } catch(e) {
          _onError?.(e.message);
        }
      }
    },

    subscribe(serviceUUID, charUUID) {
      if (isNative) native.subscribe(serviceUUID, charUUID);
      // Web Bluetooth already subscribed in connect()
    },

    writeMode(serviceUUID, charUUID, byteValue) {
      if (isNative) {
        const hex = byteValue.toString(16).padStart(2, '0');
        native.write(serviceUUID, charUUID, hex);
      } else {
        _webModeChar?.writeValue(new Uint8Array([byteValue]));
      }
    },

    disconnect() {
      if (isNative) {
        native.disconnect();
      } else {
        if (_webDevice?.gatt?.connected) _webDevice.gatt.disconnect();
      }
      _onDeviceFound = _onConnected = _onDisconnected = _onData = _onError = null;
      _webDevice = _webModeChar = null;
    }
  };
})();
