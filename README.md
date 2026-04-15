# MINIPROJECT
# FPGA and Analog Threshold-Based Channel Occupancy Analyzer with Jammer-Like Anomaly Detection in the 865-867 MHz ISM Band
The proposed system implements a threshold-based energy detection mechanism using an analog signal chain consisting of an envelope detector, averaging stage, logarithmic compression, and a high-speed comparator to generate real-time busy/free channel decisions.

A real-time spectrum monitoring and channel occupancy analyzer using a **Heltec WiFi LoRa 32 V3** as the RF sensor and a **Basys3 (Artix-7)** FPGA for hardware-accelerated signal processing.

## Overview

This system scans 9 LoRa channels in the 865–867 MHz band, computes FFT-based spectral analysis and channel occupancy entirely on the FPGA, and streams results to a MATLAB live dashboard over UART.

```
[Heltec LoRa V3] --UART--> [Basys3 FPGA] --USB--> [MATLAB Dashboard]
      |                        (FFT, Occ)               (Plots)
      |---USB---> [MATLAB] (RSSI / Channel Scan)
```

---

## Repository Structure

```
.
├── fpga/
│   └── top_spectrum.v          # Top-level Verilog — all RTL modules
├── heltec/
│   └── receiver_heltec.ino     # Arduino sketch for Heltec WiFi LoRa 32 V3
|   └── transmitter.ino
├── matlab/
│   └── live_spectrum.m         # MATLAB real-time plotting dashboard
|   └── static_case_script_allcases.m  
|   └── RSSI vs PSD plot.m
|   └── conceptual ROC Curve.m
|   └── full_spectrum_sensing_model.slx
├──CMOS Analysis
└── README.md
```

---

## Hardware Requirements

| Component | Part |
|---|---|
| RF Sensor | Heltec WiFi LoRa 32 V3.2 (ESP32-S3 + SX1262) |
| FPGA Board | Digilent Basys3 (Artix-7 XC7A35T) |
| Vivado IP | Xilinx FFT IP Core (`xfft_512`) — 512-point pipelined streaming |
| Software | MATLAB R2021b+ with Instrument Control Toolbox |
| IDE | Arduino IDE + Heltec ESP32 board package |

---

## Wiring

| Heltec Pin | Basys3 JA Pin | Signal |
|---|---|---|
| GPIO45 (TX) | JA Pin 1 | `esp_rx_pin` — FPGA receives |
| GPIO46 (RX) | JA Pin 2 | FPGA TX back (optional) |
| GND | JA Pin 5 | Common ground (**required**) |


---

## FPGA Architecture (`top_spectrum.v`)

All modules are defined in a single file. The processing pipeline is:

```
uart_rx → frame_parser → occ_counter  → tx_formatter_occ ─┐
                       → fft_feeder   → xfft_512 IP       ├─→ TX FIFO → uart_tx → PC
                                      → fft_output_stage  ─┘
                       heartbeat ─────────────────────────┘
```

### Modules

| Module | Description |
|---|---|
| `uart_rx` | 115200-baud UART receiver with 2-flop synchroniser and framing-error discard |
| `uart_tx` | 115200-baud UART transmitter |
| `frame_parser` | Parses `S` and `E` ASCII frames from Heltec; extracts RSSI, frequency, busy flag, channel index |
| `occ_counter` | Counts busy/total samples per channel over 200-sample windows; emits occupancy percentage |
| `fft_feeder` | Accumulates 512 RSSI samples per channel into BRAM; applies Hann window; streams to FFT IP |
| `xfft_512` | Xilinx FFT IP — 512-point, pipelined streaming, fixed-point, natural-order output |
| `fft_output_stage` | Computes magnitude (alpha-max-beta-min approximation); serialises 256-bin `F` frame |
| `tx_formatter_occ` | Serialises `O` frame: `O,frequency,percent\n` |
| `heartbeat` | Sends `H,ALIVE\n` every 1 second; LED 6 blinks to confirm FPGA→PC link |

### BRAM Usage (Basys3 has 50 × 18Kb = 900 Kb total)

| Buffer | Size | BRAMs |
|---|---|---|
| `sbuf` (9ch × 512 samples) | 72 Kb | 4 |
| `hann` window ROM | 8 Kb | 1 |
| TX FIFO | 32 Kb | 2 |
| `mag_buf` | 4 Kb | 1 |
| **Total** | **116 Kb** | **~7** |

### Output Frame Format

| Frame | Format | Description |
|---|---|---|
| `F` | `F,<freq>,<b0>,<b1>,...,<b255>\n` | 256 FFT magnitude bins for one channel |
| `O` | `O,<freq>,<pct>\n` | Channel occupancy 0–100% |
| `H` | `H,ALIVE\n` | Heartbeat — confirms FPGA→PC link |

---

## Heltec Firmware (`receiver_heltec.ino`)

Scans 9 channels sequentially. For each channel:
- **5 ms RSSI window** — samples RSSI every 1 ms, sends `S` frames to USB and UART1
- **10 ms FFT window** — samples RSSI every 200 µs, sends `E` frames to USB and UART1

`S` and `E` frames go to **both** USB serial (→ MATLAB directly) and UART1/GPIO45 (→ FPGA).

### Channels Scanned

865.1, 865.3, 865.5, 865.7, 865.9, 866.1, 866.3, 866.5, 866.7 MHz

---

## MATLAB Dashboard (`live_spectrum.m`)

Reads from **two COM ports simultaneously**:

| Port | Source | Frames |
|---|---|---|
| `heltec_port` | Heltec USB | `S` frames → RSSI, busy flag, channel scan |
| `fpga_port` | Basys3 USB | `F`, `O`, `H` frames → spectrum, occupancy |

### Plots

| Panel | Data Source |
|---|---|
| Received Spectrogram (waterfall) | FPGA `F` frames |
| Received Spectrum | FPGA `F` frames |
| Average RSSI | Heltec `S` frames |
| Channel Occupancy % | FPGA `O` frames |
| Busy/Free Detection | Heltec `S` frames |
| Channel Scan Power | Heltec `S` frames |

### Configuration

Edit these two lines at the top of `live_spectrum.m`:

```matlab
heltec_port = "COM7";   % Heltec USB COM port
fpga_port   = "COM9";   % FPGA USB COM port
```

Find your COM port numbers in Windows Device Manager under *Ports (COM & LPT)*.

---

## Vivado Setup

1. Create a new Vivado project targeting `xc7a35tcpg236-1` (Basys3).
2. Add `top_spectrum.v` as the design source.
3. Generate the FFT IP core:
   - **IP Catalog** → Fast Fourier Transform
   - Component name: `xfft_512`
   - Transform length: 512
   - Architecture: Pipelined, Streaming I/O
   - Data format: Fixed Point, input width 16, phase factor width 16
   - Output ordering: Natural Order
   - Throttle scheme: Non Real Time
   - Scaling: Unscaled, Truncation
   - Memory: Block RAM for both data and twiddle
4. Add your XDC constraints file mapping `esp_rx_pin` to JA Pin 1.
5. Run Synthesis → Implementation → Generate Bitstream → Program Device.

---

## Diagnostic LEDs (Basys3)

| LED | Meaning |
|---|---|
| 0 | `esp_rx_pin` raw — ON = UART idle (HIGH) |
| 1 | Toggles on every decoded byte |
| 2 | Toggles on every `S` frame parsed |
| 3 | Toggles on every `E` frame parsed |
| 4 | Toggles when FFT output is valid |
| 5 | Toggles on every occupancy frame |
| 6 | Toggles ~1 Hz on heartbeat — confirms FPGA→PC link |
| 15 | `uart_txd_in` raw — ON = TX idle |

---

## Known Issues / Troubleshooting

**LED 6 OFF / no F or O frames in MATLAB**
The FIFO write port is being monopolised by S-frame passthrough. Ensure the passthrough logic is removed from `frame_parser` and the FIFO priority block only contains `occ`, `fft`, and `hb` writers.

**LED 4 never toggles (FFT never fires)**
FFT trigger condition off-by-one. In `fft_feeder` ST_IDLE, check for `scount[j] == 9'd510` not `511`.

**LED 5 flickers with ground movement**
Intermittent common ground. Replace with a solid jumper wire between Heltec GND and Basys3 JA Pin 5.

**Spectrogram stays solid blue**
No `F` frames arriving. Confirm MATLAB `fpga_port` is set to the correct COM number, and that the FPGA bitstream has the passthrough fix applied.

---

## License

MIT License — free to use, modify, and distribute with attribution.
