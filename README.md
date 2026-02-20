# ESPHome AC component for Hisense/Ballu (RS-485)

<img width="1381" height="913" alt="Interface preview" src="https://github.com/user-attachments/assets/5b765094-464f-4cfc-8cbe-eecd8c11de7c" />

This custom component provides full climate control for air conditioners manufactured by Hisense and its OEM brands (Ballu, etc.) that use the RS-485 protocol. It has been tested on **Ballu iGreen Pro (BSAGI-12HN8)** and should work with many other models.

The component exposes a standard Home Assistant Climate entity, along with a set of optional sensors and switches to access all advanced features of the AC (turbo, eco, quiet, sleep, swing, LED, etc.).

## Acknowledgements

We would like to thank the following people for their contributions to reverse engineering the protocol and developing this solution:

- **vins.vins** ([4pda.to forum post](https://4pda.to/forum/index.php?showtopic=1076299&st=200#entry131868776)) – for initial protocol research and sharing findings.
- **straga** ([GitHub](https://github.com/straga/scrivo_project/tree/master/project/ac_xm_hisense_control)) – for providing additional protocol details and wiring diagram.

If you have contributed to the reverse engineering effort or improved the component, feel free to add your name here via a pull request!

## Hardware requirements

-   An ESP32 board
-   An RS-485 transceiver (e.g., MAX485) connected to UART pins
-   Wiring:  
    \- A (RS-485) → A of transceiver  
    \- B (RS-485) → B of transceiver  
    \- Transceiver's RXD to ESP RX pin (e.g., GPIO16)  
    \- Transceiver's TXD to ESP TX pin (e.g., GPIO17)  
    \- Power (3.3V or 5V depending on module) and GND

> ⚠️ The AC uses 5V logic levels on its RS‑485 port. Make sure your transceiver is 3.3V‑tolerant if you power the ESP from 3.3V.

![as6yvpi2AeqCNYRhDz0fXO35MZRrYUvdRQsFswfFAja1](https://github.com/user-attachments/assets/020c53af-f625-4c67-b624-0923f904de1f)

## Installation

### Using `external_components` (recommended)

Add the following to your ESPHome YAML configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/artshevchenko/ac_hi
      ref: main
    components: [ ac_hi ]
```

## Example configuration

A minimal configuration with all optional sensors and switches:

```yaml
uart:
  id: ac_uart
  tx_pin: 16
  rx_pin: 17
  baud_rate: 9600
  stop_bits: 1

external_components:
  - source: github://artshevchenko/AC-Hi
    refresh: 30s

climate:
  - platform: ac_hi
    name: "Ballu AC"
    uart_id: ac_uart
    update_interval: 2s
    enable_presets: true

    set_temperature:
      name: "Temperature Set"
      unit_of_measurement: "°C"
      accuracy_decimals: 0
      icon: "mdi:thermometer-check"

    room_temperature:
      name: "Temperature Current"
      unit_of_measurement: "°C"
      accuracy_decimals: 0
      icon: "mdi:home-thermometer"

    wind:
      name: "Wind Mode Code"
      accuracy_decimals: 0
      icon: "mdi:fan"

    sleep_stage:
      name: "Sleep Mode Code"
      accuracy_decimals: 0
      icon: "mdi:sleep"

    mode_code:
      name: "AC Mode Code"
      accuracy_decimals: 0
      icon: "mdi:air-conditioner"

    quiet:
      name: "Quiet Mode Code"
      accuracy_decimals: 0
      icon: "mdi:fan-off"

    turbo:
      name: "Turbo Mode Code"
      accuracy_decimals: 0
      icon: "mdi:flash"

    economy:
      name: "Economy Mode Code"
      accuracy_decimals: 0
      icon: "mdi:leaf"

    swing_up_down:
      name: "Swing Up-Down"
      accuracy_decimals: 0
      icon: "mdi:unfold-more-horizontal"

    swing_left_right:
      name: "Swing Left-Right"
      accuracy_decimals: 0
      icon: "mdi:unfold-more-vertical"

    power_status:
      name: "Power Status"
      icon: "mdi:power"

    pipe_temperature:
      name: "Temperature Pipe"
      unit_of_measurement: "°C"
      accuracy_decimals: 0
      icon: "mdi:coolant-temperature"

    compressor_frequency_set:
      name: "Compressor Frequency Set"
      unit_of_measurement: "Hz"
      accuracy_decimals: 0
      icon: "mdi:sine-wave"

    compressor_frequency:
      name: "Compressor Frequency"
      unit_of_measurement: "Hz"
      accuracy_decimals: 0
      icon: "mdi:sine-wave"

    outdoor_temperature:
      name: "Temperature Outdoor"
      unit_of_measurement: "°C"
      accuracy_decimals: 0
      icon: "mdi:thermometer"

    outdoor_condenser_temperature:
      name: "Temperature Outdoor Condenser"
      unit_of_measurement: "°C"
      accuracy_decimals: 0
      icon: "mdi:thermometer"

    # --- LED target switch ---
    led_switch:
      name: "AC LED"
      icon: "mdi:lightbulb"

    heap_free:
      name: "ESP Heap Free"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement

    heap_total:
      name: "ESP Heap Total"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement

    heap_used:
      name: "ESP Heap Used"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement

    heap_min_free:
      name: "ESP Heap Min Free"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement

    heap_max_alloc:
      name: "ESP Heap Max Alloc"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement

    heap_fragmentation:
      name: "ESP Heap Fragmentation"
      unit_of_measurement: "%"
      device_class: ""
      state_class: measurement

    psram_total:
      name: "ESP PSRAM Total"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement
      
    psram_free:
      name: "ESP PSRAM Free"
      unit_of_measurement: "B"
      device_class: data_size
      state_class: measurement
```

## Entities provided

<img width="1361" height="812" alt="Interface preview" src="https://github.com/user-attachments/assets/eb84a052-c4aa-41c9-9f14-46581e9eddc6" />


### Climate (`climate`)

-   Modes: `OFF`, `COOL`, `HEAT`, `DRY`, `FAN_ONLY` (auto mode is not supported by the AC)
-   Fan speeds: `AUTO`, `QUIET`, `LOW`, `MEDIUM`, `HIGH`
-   Swing modes: `OFF`, `VERTICAL`, `HORIZONTAL`, `BOTH`
-   Presets (if `enable_presets: true`): `ECO`, `BOOST` (turbo), `SLEEP`
-   Target temperature range: 16–30°C in steps of 1°C
-   Current temperature is read from the AC and displayed

### Sensors (optional)

Most sensors publish raw values received from the AC:

| Sensor | Description |
| --- | --- |
| `set_temperature` | Target temperature setpoint (°C) |
| `room_temperature` | Current indoor temperature (°C) |
| `outdoor_temperature` | Outdoor air temperature (°C, signed) |
| `outdoor_condenser_temperature` | Outdoor condenser temperature (°C, signed) |
| `pipe_temperature` | Indoor pipe temperature (°C) |
| `wind` | Raw fan speed code (0–18) |
| `sleep_stage` | Sleep stage code (0–4) |
| `mode_code` | AC mode code (0–3) |
| `quiet` | Quiet mode active (binary) |
| `turbo` | Turbo mode active (binary) |
| `economy` | Eco mode active (binary) |
| `swing_up_down` | Vertical swing active (binary) |
| `swing_left_right` | Horizontal swing active (binary) |
| `compressor_frequency_set` | Target compressor frequency (Hz) |
| `compressor_frequency` | Actual compressor frequency (Hz) |
| `power_status` | Text sensor showing "ON" or "OFF" |

### Switches (optional)

-   `led_switch`: Controls the indoor unit’s LED backlight.

## How it works

The component communicates with the AC via a simple request/response protocol over RS‑485.

-   **Polling:** Every `update_interval` (default 1s) a short status query (command `0x66`) is sent.
-   **Write commands:** When you change a setting through Home Assistant, the component accumulates changes for 200 ms (debounce) and then sends a full state write (command `0x65`) with a correctly calculated CRC.
-   **Convergence logic:** While HA has priority, the component continues to enforce the desired state until the AC confirms it (by sending a status frame that matches the desired signature). After convergence, remote changes (e.g., from the IR remote) are again accepted and reflected in HA.
-   **CRC validation:** All incoming frames are checked for CRC correctness; invalid frames are discarded.
-   **Write lock timeout:** If the AC does not acknowledge a write within 5 seconds, the lock is released to avoid permanent blocking.

## Protocol specification (reverse engineered)

This section documents the RS‑485 protocol used by Hisense/Ballu ACs. It may be useful for understanding the implementation or for adapting it to other models.

### Frame format

All frames start with header `0xF4 0xF5` and end with tail `0xF4 0xFB`. The length of the frame is variable and is given by `frame[4] + 9` bytes.

| Byte offset | Description |
| --- | --- |
| 0 | 0xF4 (header) |
| 1 | 0xF5 (header) |
| 2 | Unknown (usually 0x00) |
| 3 | Unknown (usually 0x40) |
| 4 | Declared length L – total frame size = L + 9 |
| 5 … 8 | Unknown (often 0x00) |
| 9 … 12 | Unknown (often 0x01 0xFE 0x01 0x00) |
| 13 | Command (0x65 – write, 0x66 – read status, 0x??) |
| 14 … L+4 | Payload |
| L+5 … L+6 | CRC (16‑bit sum of bytes 2 … L+4, big‑endian) |
| L+7 … L+8 | Tail 0xF4 0xFB |

### Status response (command 0x66, byte 13 = 102)

When the AC receives a short query (0x66 frame), it responds with a long status frame (command 102). The payload contains all operational data.

**Relevant bytes (0‑based within the frame):**

| Index | Description | Encoding / Notes |
| --- | --- | --- |
| 16 | Fan speed | Reported values: 0/1/2 = AUTO, 10 = QUIET, 12 = LOW, 14 = MEDIUM, 16 = HIGH |
| 17 | Sleep mode | Raw value. Decode: `(value >> 1)` gives sleep stage: 0 = off, 1 = sleep\_1, 2 = sleep\_2, 4 = sleep\_3, 8 = sleep\_4 |
| 18 | Power + Mode | Bit 3 = power (1=ON). Upper nibble = mode: 0=FAN\_ONLY, 1=HEAT, 2=COOL, 3=DRY |
| 19 | Target temperature | Direct °C value (16–30) |
| 20 | Current indoor temperature | °C |
| 21 | Pipe temperature | °C |
| 32 | Swing | Bits: bit7 = up/down on, bit6 = left/right on |
| 33 | Turbo/Eco flags (RX) | bit1 = turbo, bit2 = eco |
| 35 | Quiet (RX) | bit2 = quiet active |
| 36 | LED (RX) | bit7 = LED on |
| 37 | ... | (unused) |
| 42 | Compressor target frequency | Hz |
| 43 | Compressor actual frequency | Hz |
| 44 | Outdoor air temperature | °C (signed int8\_t) |
| 45 | Outdoor condenser temperature | °C (signed int8\_t) |

### Write command (command 0x65)

To change settings, a full state frame is sent. The payload is built from the desired values. The CRC must be calculated over bytes 2 … L+4.

**Encoding rules (TX):**

| Field | Encoding |
| --- | --- |
| Target temperature | `((c & 0x1F) << 1) | 0x01` where c = 16…30 |
| Fan speed | Base codes: AUTO=1, QUIET=10, LOW=12, MEDIUM=14, HIGH=16. **TX value = base + 1** |
| Sleep mode | `((code) << 1) | 0x01` where code = 0,1,2,4,8 for off,1,2,3,4 |
| Power | low nibble of byte 18: ON = 0b1100, OFF = 0b0100 |
| Mode | high nibble of byte 18: FAN\_ONLY = 0x10, HEAT = 0x30, COOL = 0x50, DRY = 0x70 |
| Turbo | byte 33: ON = 0b1100, OFF = 0b0100 (Turbo overrides Eco) |
| Eco | byte 33: ON = 0b110000, OFF = 0b010000 (combined with Turbo) |
| Quiet | byte 35: ON = 0b110000, OFF = 0b010000 |
| Swing | byte 32: UD ON = 0b11000000, OFF = 0b01000000; LR ON = 0b00110000, OFF = 0b00010000; combined by addition |
| LED | byte 36: ON = 0b11000000, OFF = 0b01000000 |

### CRC calculation

CRC is a simple 16‑bit sum (big‑endian) of bytes starting from index 2 up to index `L+4` (i.e., excluding header, tail, and the CRC bytes themselves). The sum is stored as two bytes: high byte at offset `L+5`, low byte at `L+6`.

Example in C++:

```cpp
uint16_t crc = 0;
for (int i = 2; i < frame.size() - 4; i++) {
    crc += frame[i];
}
frame[frame.size() - 4] = (crc >> 8) & 0xFF;
frame[frame.size() - 3] = crc & 0xFF;
```

## Troubleshooting

-   **No communication:** Check wiring, baud rate (9600), and ensure that the RS‑485 transceiver is powered correctly.
-   **Wrong outdoor temperature:** The value is signed. If you see 244°C, it means –12°C was misinterpreted. This has been fixed in the component.
-   **Commands not working:** Enable verbose logging (`esp_log_level: VERBOSE`) and inspect the TX/RX frames. Compare them with the protocol specification.

## Contributing

Issues and pull requests are welcome. Please ensure your code follows the ESPHome style and includes appropriate logging.
