I want to implement ble on the rocket firmware, using nimble; similar to the base station, however some major architecture differences. the main idea is abstraction of transports. wifi we sockets, ble, USB, and Lora, and much of which also goes to log files. however different transports have some differences. for now no USB or wifi. ble allows more bandwidth so might have more data than Lora, but less than logs; somewhat about filtering/relevance, but mostly just how often it's sent over Lora vs ble vs logged (logging full sensor data is intended to be able to be full sensor rate, which could exceed ble ability; and Lora is always too slow)

there are two main use cases:
1: log download. high speed, short range, post flight, while holding the rocket and my phone.
this form of log download is different to the log download command, which is a special signed command. the existing behaviour must be signed and protected because A: its blocking for speed, it stops running the main loop; and B: it puts the Lora radio into a special mode which stops normal telemetry. as opposed to the ble transport for logs which can be requested by anyone. currently it's only intended to be available when armed=false, so it doesn't detract from flight performance guarantees.

2 realtime telemetry: this is only very slow speed, and longer range is more important. this will just push telemetry in realtime as it happens, and will be rate limited by settings (later commands will be able to change these settings). in this mode, the ble log fetch runs independently of any Lora activity. ideally the main loop should keep running, e.g. one nimble notify packet attempt per main loop. (the main loop runs too fast to push a packet every time, so check the return for flow control)

use case 2 is much more closely an equivalent for the existing Lora uses: it should be able to take signed commands like Lora commands, and run them the same way, where ble recieves everything Lora recieves.

log download command is currently very specific to Lora; and should still work over Lora.
the new log fetch functionality, should be available for USB/websocket/ble, but not Lora

implementation plan:
commands (hmac'd , which have major effects) are separate from requests (e.g. http get/post, ble write, USB input, with negligible side effects, or which are only enabled when disarmed), which can control the telemetry delivery
requests include:
-subscribe <rate> - how often you want telemetry, in the normal telemetry format, of data pages (same as Lora builds, or same as logs contain.. TBD exact formatting. may have extra metadata, and/or may exclude the Lora telemetry header). transport may pack multiple data pages. (in future Lora may also have multiple pages in a single packet)
-connection settings - e.g. change ble phy, serial baud, etc.
-historical log request <index> <count> - equivalent to log download command, without blocking, and without effecting Lora radio settings (or effecting any transport other than the one where the request came from). this will be one invitation per main loop, the transport-specific work will limit it to a reasonable impact per loop, and will be armed-flag aware. (e.g. ble will fill one 500-byte pdu, then return. for now, this will be allowed while armed, but testing will be needed to decide if it needs to be disabled while armed). this is a non-blocking implementation, so will need to deliver more over time each loop.
-send command <payload> - treat payload as if it was received over Lora. includes length byte prefix, followed by normal Lora payload, including hmac, etc, which is still checked as normal. this forms a substitute interface, when Lora unavailable. identical implementation of all commands as-is (including the existing blocking Lora log download blocking command -still effects Lora radio and still blocks. that mode may be removed in future, but for now it exists as-is, unrelated to his change)



this changes the existing topology. currently, it's rocket to base and base to phone. also relay to other bases.
anyone can connect their phone to a base station(over wifi or ble, etc), which can show them the recieved Lora packets from the rocket, and send commands (which the browser must make valid hmacs for)
base stations exist mostly because phones don't have built-in Lora transceivers.
however all base stations are also relays, which are functionally different to just a Lora translator.
the new structure bypasses the base stations, so it's intended to be in addition. lora is still needed because none of the other transports realistically have enough good reliable range to work well.


the two main use cases also suggest/recommend, at least for ble, different PHY settings, and pacing. although either mode can use any phy, the default is that historical log fetch (which begins sending  old logs as much as will fit, one packet every loop), will be via PHY 2M, and realtime telemetry (which will be just new data pages, as it happens) will be over coded phy s=8, for good range, although not as good as Lora, it can do better data rate, delivering every page in every packet (later, there may be too many pages, or big pages which can't all fit; flags will be used to select which pages are desired by the request)
. the default set of bitflags for which pages to subscribe to, and which types of historical records request will be the existing pages 1-13. new pages in higher range are to be added later, which will be too big to fit them all, e.g. high detail raw sensor readings, GPS nmea data, etc.
the expectation is the user will subscribe to what they are interested in, those will be packed into every packet, but only if the page is fresh/new since last packet; and only once per subscription rate. e.g. subscribe to 50hz of accelerometer, 10hz of baro... but if no new baro reading has happened, then baro page is skipped; and in the 40 packets between, it's just the Accel packets.

the primary use case is ble to a browser, through web Bluetooth; although apps or other cases may be used. (including possibly ble/websocket between base and rocket, faster than Lora)

gotchas:
this is multi core. use the overload of notify which takes a payload as input, not setvalue+notify()
check the return from notify. we will attempt to queue packets faster than they can be sent, the return value gives us flow rate control when full
for historical requests, fill every packet as many as will fit, then next cycle will add as many as will fit in the next packet. these pages are built as needed, no 4kb buffer, because they will still be available in logs to read next loop. whole records only (each has a length byte, and type, etc and normal params so can be decoded from a ground in a packet). for realtime, they won't necessarily be available next loop, since they aren't necessary logged, and might change by next time., usually they will all fit in a single packet, but if not, save the additional pages as a point in time, (max 4kb fixed buffer. this size may change in future) and send the remainder , drain whole records from the buffer to the next packet on the next loop, or when flow control allows. no additional new pages captured until they all send). overflow should still be whole records per packet.using this buffer implies either slightly bursty, or they have subscribed to more than flow control can fit. reduced rate, but all relevant pages from the same instant

limit one user connected at a time. any new connection assumes the existing connection is broken. disconnect the existing user, clear any buffers/counters, set every page to be marked as fresh data, and start on the new connection. continue ble advertising while connected.default phy should be advertised only on legacy - nimble doesn't properly support dual advertising, so 1m advert, then negotiate to either 2M or coded phy, etc as appropriate.
initial subscription should begin sending 10hz of pages 1-13 (todo: how many pages are implemented? all the existing ones, not unimplemented Kalman.

once a page is sent on a transport, clear the fresh flag. when any sensor updates anything relevant to that page, set the fresh flag. non-fresh pages aren't sent in reliable transport modes (ble counts as reliable. Lora counts as unreliable)

all transports are considered untrusted. anyone can connect to anything. after testing, (and depending on the type of flight/flight plan) if ble is determined a risk by it's mere presence to flight safety/deterministic reliability, the entire ble stack will be stopped when armed; however initially, the impact is expected to be minimal, so should remain available in flight



use a longish advertising period, to save battery, e.g. 1 second or more
todo: do we need the connection controls to choose power level, or will ble do that automatically based on the signal quality?


connection controls for ble should default to 1m, and automatically attempt renegotiate to 2M phy. and mtu 517.

packets should only be filled with length-prefixed whole records, max 502 bytes. request an mtu of 517, but choosing only 502 max allows more LL segments to fit in the intermediate buffers (502 fills 2, 517 spills into a 3rd). use a constant/define for this limit, may be changed later


---

## OTA Firmware Update Characteristic (0x0006)

Both devices expose a sixth GATT characteristic with UUID suffix `0x0006` in their respective service UUID families:

| Device | UUID | Properties |
|--------|------|------------|
| Rocket | `524f434b-4554-5354-424c-000000000006` | WRITE \| WRITE_NR \| NOTIFY |
| Base   | `4d4f4f4e-5348-4f54-4253-000000000006` | WRITE \| WRITE_NR \| NOTIFY |

### Chunk write format

```
[offset u32 LE][data: 1–508 bytes]
```

Max frame: 512 bytes (4-byte offset prefix + 508 bytes data = Web Bluetooth `writeValueWithoutResponse` limit). Chunks are not individually authenticated. Authentication is deferred to the `CMD_OTA_FINALIZE (0x51)` command which verifies HMAC-SHA256 of the entire image.

### Notifications

The device does **not** send a per-chunk ACK (incompatible with target throughput ~120 KB/s). Notifications are only sent on errors or as periodic progress reports:

| Byte(s) | Meaning |
|---------|---------|
| `0x00` | OK (used for explicit success acks, e.g. finalize confirm before reboot) |
| `0x01` | OTA not active — CMD_OTA_BEGIN not sent |
| `0x02` | Write failed / begin/erase failed |
| `0x03` | Offset gap — chunks must be strictly sequential |
| `0x04` | HMAC mismatch on finalize — session aborted |
| `0x05` | esp_ota_end failed |
| `0x06` | In OTA_VERIFYING state — no more chunks allowed |
| `0x07` | Refused: device is armed or rollback is pending |
| `0xA0 [bytesWritten u32 LE]` | Progress report (5 bytes total), sent every 1200 chunks (~600 KB, ~5 s at target throughput) |

JS subscribes to notifications on connect; any non-zero status byte aborts the upload and displays the error. Progress reports (`0xA0` prefix) update the progress bar and reset the stall timer (10 s timeout).

### OTA state machine

```
OTA_LOCKED → (CMD_OTA_BEGIN) → OTA_ERASING → OTA_RECEIVING → (CMD_OTA_FINALIZE) → OTA_VERIFYING → (reboot) → OTA_LOCKED
```

- **CMD_OTA_BEGIN (0x50)**: refused while armed, refused if rollback is pending, refused if not OTA_LOCKED. Pre-erases the entire inactive OTA partition (1.5 MB, ~1–2 s, `vTaskDelay(1)` per 4 KB sector), then calls `esp_ota_begin`.
- **CMD_OTA_FINALIZE (0x51)**: verifies `bytesWritten == fwSize`, reads back entire image and checks HMAC-SHA256 against the provided 32-byte hash, calls `esp_ota_end` + `esp_ota_set_boot_partition`, reboots after 500 ms.
- **CMD_OTA_CONFIRM (0x52)**: after rebooting into the new firmware, cancels rollback via `esp_ota_mark_app_valid_cancel_rollback()`. If never sent, the bootloader rolls back to the previous slot on the next reboot.

All three commands use the standard 0x9A authenticated command format (nonce + HMAC10), sharing the same nonce counter as all other commands.

### OTA/ARM mutual exclusion

- OTA commands and chunks are refused if the device is armed.
- `flightTryArm()` returns `ARM_ERR_OTA_ACTIVE` if `otaGetState() != OTA_LOCKED`.

### Partition layout (both devices, 8 MB flash)

**Rocket:**
```
app0 (ota_0):  0x010000, 1.5 MB  (currently running)
app1 (ota_1):  0x190000, 1.5 MB  ← OTA target (alternates)
log_index:     0x310000, 32 KB
log_data:      0x318000, 4.9 MB
```

**Base station:**
```
app0 (ota_0):  0x010000, 1.5 MB
app1 (ota_1):  0x190000, 1.5 MB  ← OTA target (alternates)
littlefs:      0x310000, 256 KB
log_index:     0x350000, 32 KB
log_data:      0x358000, 4.7 MB
```

The OTA target is always `esp_ota_get_next_update_partition(NULL)` — whichever slot is not currently running. Flashing works in both directions (app0→app1 and app1→app0).



