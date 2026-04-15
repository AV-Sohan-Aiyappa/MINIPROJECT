// =============================================================================
// receiver_heltec.ino
// Hardware-Aware Spectrum Monitoring — Receiver / Main Module
//
// PIN FIX (WiFi LoRa 32 V3.2 datasheet):
//   GPIO17 and GPIO18 are NOT available on V3 — they are used internally
//   by the SX1262 LoRa chip.
//
//   V3.2 uses ESP32-S3. Free GPIOs on headers confirmed from datasheet p.7-9:
//   Header J3: GPIO1-7, GPIO38-42, GPIO45, GPIO46
//   Header J2: GPIO19-21, GPIO26, GPIO33-36, GPIO47-48
//
//   Chosen UART1 pins:
//     TX = GPIO45  (Header J3 pin 6)  -> wire to FPGA JA Pin 1
//     RX = GPIO46  (Header J3 pin 5)  <- wire from FPGA JA Pin 2
//   Both are plain GPIO with no LoRa/OLED/SPI conflicts on V3.2.
//
// WIRING (update physical wires):
//   Heltec GPIO45 -> Basys3 JA Pin 1 (FPGA esp_rx_pin, mapped to J1 in XDC)
//   Heltec GPIO46 <- Basys3 JA Pin 2 (optional, FPGA TX back)
//   Heltec GND    -> Basys3 JA Pin 5 (GND)
//
// CHANGES FROM ORIGINAL:
//   1. Serial1.begin added with correct pins (45=TX, 46=RX)
//   2. Serial1.printf mirrors every frame to FPGA
//   All other logic IDENTICAL to original working code.
// =============================================================================

#include "Arduino.h"
#include "LoRaWan_APP.h"

#define RF_FREQUENCY 866300000
#define RSSI_THRESHOLD -90

static RadioEvents_t RadioEvents;

/* ---------- SETTINGS ---------- */

#define RSSI_WINDOW_MS   5
#define FFT_WINDOW_MS    10

unsigned long lastPrint = 0;

/* ---------- CHANNEL SCAN LIST ---------- */

uint32_t channels[] =
{
  865100000,
  865300000,
  865500000,
  865700000,
  865900000,
  866100000,
  866300000,
  866500000,
  866700000
};

const int NUM_CH = sizeof(channels)/sizeof(channels[0]);

/* ---------- SETUP ---------- */

void setup()
{
  Serial.begin(115200);

  // UART1 to FPGA
  // TX = GPIO45 -> Basys3 JA Pin 1 (esp_rx_pin in FPGA)
  // RX = GPIO46 <- Basys3 JA Pin 2 (optional)
  // Baud must match FPGA uart_rx parameter: 115200
  Serial1.begin(115200, SERIAL_8N1, 46, 45);  // (baud, config, RX_pin, TX_pin)

  delay(2000);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA,
                    0,7,1,0,12,0,
                    false,0,true,
                    0,0,false,true);

  Radio.Rx(0);

  Serial.println("=== SPECTRUM SENSOR STARTED ===");
}

/* ---------- READ RSSI ---------- */

inline int16_t readRSSI()
{
  return Radio.Rssi(MODEM_LORA);
}

/* ---------- LOOP ---------- */

/* ---------- LOOP ---------- */

void loop()
{
  Radio.IrqProcess();

  for(int ch = 0; ch < NUM_CH; ch++)
  {
    /* ---- Tune RF channel ---- */
    Radio.SetChannel(channels[ch]);

    delay(3);   // PLL settling time

    /* ===== RSSI DECISION WINDOW ===== */

    unsigned long t0 = millis();

    while(millis() - t0 < RSSI_WINDOW_MS)
    {
      int16_t rssi = readRSSI();
      bool busy = (rssi > RSSI_THRESHOLD);

      /* MATLAB frame WITH FREQUENCY */
      Serial.printf("S,%lu,%lu,%d,%d\n",
                    millis(),
                    channels[ch],
                    rssi,
                    busy);
      Serial1.printf("S,%lu,%lu,%d,%d\n",   // mirror to FPGA
                    millis(),
                    channels[ch],
                    rssi,
                    busy);

      delay(1);
    }

    /* ===== FFT ENERGY WINDOW ===== */

    unsigned long t1 = millis();

    while(millis() - t1 < FFT_WINDOW_MS)
    {
      int16_t rssi = readRSSI();

      Serial.printf("E,%lu,%lu,%d\n",
                    millis(),
                    channels[ch],
                    rssi);
      Serial1.printf("E,%lu,%lu,%d\n",      // mirror to FPGA
                    millis(),
                    channels[ch],
                    rssi);

      delayMicroseconds(200);
    }
  }
}
