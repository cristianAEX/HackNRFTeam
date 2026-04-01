/* ____________________________
   HackNRF24Team - Sub-GHz CC1101 Expansion
   Universidad Nacional Autónoma de México (UNAM)
   Facultad de Ingeniería — Sistemas de Comunicaciones
   MIT License
   ________________________________________ */

#include "cc1101_module.h"
#include "config.h"

// ─────────────────────────────────────────────
//  Instancias globales
// ─────────────────────────────────────────────
CC1101Driver cc1101A(CC1101_A_CS_PIN, CC1101_A_GDO0_PIN);
CC1101Driver cc1101B(CC1101_B_CS_PIN, CC1101_B_GDO0_PIN);

RawSignal g_capturedSignal = { {}, true, FREQ_433MHZ, false };

// ═════════════════════════════════════════════
//  DRIVER CC1101 — Implementación
// ═════════════════════════════════════════════

CC1101Driver::CC1101Driver(uint8_t cs, uint8_t gdo0)
  : csPin(cs), gdo0Pin(gdo0) {}

void CC1101Driver::_csLow()  { digitalWrite(csPin, LOW);  }
void CC1101Driver::_csHigh() { digitalWrite(csPin, HIGH); }

void CC1101Driver::_waitMISO() {
  uint32_t t = millis();
  while (digitalRead(CC1101_MISO_PIN) && (millis() - t < 50));
}

void CC1101Driver::begin() {
  pinMode(csPin,   OUTPUT);
  pinMode(gdo0Pin, INPUT);
  digitalWrite(csPin, HIGH);

  SPI.begin(CC1101_SCK_PIN, CC1101_MISO_PIN, CC1101_MOSI_PIN);
  SPI.setFrequency(4000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  reset();
}

void CC1101Driver::reset() {
  _csLow();  delayMicroseconds(10);
  _csHigh(); delayMicroseconds(40);
  _csLow();
  _waitMISO();
  SPI.transfer(CC1101_SRES);
  _waitMISO();
  _csHigh();
  delay(5);
}

void CC1101Driver::strobe(uint8_t cmd) {
  _csLow();
  _waitMISO();
  SPI.transfer(cmd);
  _csHigh();
}

uint8_t CC1101Driver::readReg(uint8_t addr) {
  _csLow();
  _waitMISO();
  SPI.transfer(addr | CC1101_READ_BIT);
  uint8_t val = SPI.transfer(0x00);
  _csHigh();
  return val;
}

void CC1101Driver::writeReg(uint8_t addr, uint8_t val) {
  _csLow();
  _waitMISO();
  SPI.transfer(addr);
  SPI.transfer(val);
  _csHigh();
}

bool CC1101Driver::isAlive() {
  uint8_t ver = readReg(0x31 | CC1101_READ_BIT | CC1101_BURST_BIT); // PARTNUM
  // CC1101 devuelve 0x00 en PARTNUM; VERSION es 0x14
  uint8_t vn  = readReg(0x32 | CC1101_READ_BIT | CC1101_BURST_BIT);
  return (vn == 0x14 || vn == 0x04);
}

void CC1101Driver::setIdle() {
  strobe(CC1101_SIDLE);
  delayMicroseconds(100);
}

void CC1101Driver::setRx() {
  strobe(CC1101_SIDLE);
  strobe(CC1101_SFRX);
  strobe(CC1101_SRX);
}

void CC1101Driver::setTx() {
  strobe(CC1101_SIDLE);
  strobe(CC1101_SFTX);
  strobe(CC1101_STX);
}

// ── Frecuencia ──────────────────────────────
// freqMHz100: frecuencia en MHz × 100 (ej. 43392 = 433.92 MHz)
// Formula: FREQ = (freqMHz100 / 100.0 × 1e6) / (26e6 / 65536)
//               = freqMHz100 × 65536 / 2600000
void CC1101Driver::setFrequency(uint32_t freqMHz100) {
  uint32_t freq = ((uint64_t)freqMHz100 * 65536ULL) / 2600000ULL;
  uint8_t f2 = (freq >> 16) & 0xFF;
  uint8_t f1 = (freq >>  8) & 0xFF;
  uint8_t f0 =  freq        & 0xFF;
  writeReg(CC1101_FREQ2, f2);
  writeReg(CC1101_FREQ1, f1);
  writeReg(CC1101_FREQ0, f0);
}

// ── Modo OOK para captura/replay raw ────────
void CC1101Driver::setOOKMode() {
  setIdle();

  // GDO0 → salida de datos seriales demodulados (señal OOK en tiempo real)
  writeReg(CC1101_IOCFG0,   0x0D); // GDO0: carrier sense

  // Longitud de paquete: modo infinito (raw stream)
  writeReg(CC1101_PKTLEN,   0xFF);
  writeReg(CC1101_PKTCTRL0, 0x02); // modo paquete infinito, sin whitening
  writeReg(CC1101_PKTCTRL1, 0x00); // sin status bytes al final

  // Ancho de banda + data rate
  // MDMCFG4: BW = 26MHz / (8 × (4+CHANBW_M) × 2^CHANBW_E)
  // con BW=102kHz: CHANBW_E=2, CHANBW_M=0 → 0x8B
  // DRATE_E=8 (bits 3:0 de MDMCFG4)
  writeReg(CC1101_MDMCFG4,  0x8B); // BW=101.5kHz, DRATE_E=11
  // MDMCFG3: DRATE_M = data rate mantisa
  // Data rate ≈ 3.79 kbaud (suficiente para la mayoría de controles remotos)
  writeReg(CC1101_MDMCFG3,  0x83);
  // MDMCFG2: OOK, sin preámbulo ni sync word
  writeReg(CC1101_MDMCFG2,  0x30); // MOD_FORMAT=OOK, SYNC_MODE=no sync
  writeReg(CC1101_MDMCFG1,  0x00);
  writeReg(CC1101_MDMCFG0,  0xF8);

  // Síntesis de frecuencia
  writeReg(CC1101_FSCTRL1,  0x06);
  writeReg(CC1101_FSCTRL0,  0x00);

  // AGC (control automático de ganancia) — ajustado para OOK
  writeReg(CC1101_AGCCTRL2, 0xC7);
  writeReg(CC1101_AGCCTRL1, 0x00);
  writeReg(CC1101_AGCCTRL0, 0xB2);

  // Calibración del oscilador
  writeReg(CC1101_FSCAL3,   0xEA);
  writeReg(CC1101_FSCAL2,   0x2A);
  writeReg(CC1101_FSCAL1,   0x00);
  writeReg(CC1101_FSCAL0,   0x1F);

  // Misceláneos
  writeReg(CC1101_FREND1,   0x56);
  writeReg(CC1101_FREND0,   0x11); // LODIV_BUF_CURRENT_TX=1 para OOK
  writeReg(CC1101_MCSM0,    0x18); // calibrar al entrar en RX/TX
  writeReg(CC1101_MCSM1,    0x30); // al terminar TX → IDLE

  // Test
  writeReg(CC1101_TEST2,    0x81);
  writeReg(CC1101_TEST1,    0x35);
  writeReg(CC1101_TEST0,    0x09);
}

// ── Tabla de potencias TX ────────────────────
// Valores para 433 MHz: 0x03(min)→0xC0(max)
static const uint8_t PA_TABLE_OOK[] = { 0x00, 0xC0 }; // {off, on} para OOK

void CC1101Driver::setTxPower(uint8_t paLevel) {
  // paLevel: 0-7
  static const uint8_t levels[] = {
    0x03, 0x0E, 0x1E, 0x27, 0x50, 0x81, 0xCB, 0xC2
  };
  if (paLevel > 7) paLevel = 7;
  // Para OOK necesitamos PATABLE[0]=0x00 y PATABLE[1]=potencia
  _csLow();
  _waitMISO();
  SPI.transfer(CC1101_PATABLE | CC1101_BURST_BIT);
  SPI.transfer(0x00);
  SPI.transfer(levels[paLevel]);
  _csHigh();
}

// ═════════════════════════════════════════════
//  CAPTURA RAW SUB-GHz
// ═════════════════════════════════════════════
namespace SubGhzCapture {

// ── Estado del menú ──────────────────────────
static int     selectedFreqIdx   = 0;
static bool    capturing         = false;
static bool    captureComplete   = false;
static uint32_t captureStartTime = 0;

static const uint32_t freqOptions[] = {
  FREQ_300MHZ, FREQ_315MHZ, FREQ_433MHZ, FREQ_868MHZ, FREQ_915MHZ
};
static const char* freqLabels[] = {
  "300 MHz", "315 MHz", "433.92 MHz", "868 MHz", "915 MHz"
};
static const int NUM_FREQS = 5;

static bool btnUpPrev    = false;
static bool btnDownPrev  = false;
static bool btnRightPrev = false;

// ── Inicialización ───────────────────────────
void subghzCaptureSetup() {
  capturing       = false;
  captureComplete = false;

  cc1101A.begin();
  cc1101A.setFrequency(freqOptions[selectedFreqIdx]);
  cc1101A.setOOKMode();
  cc1101A.setIdle();

  if (neoPixelActive) setNeoPixelColour("blue");

  // Pantalla inicial
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "SubGHz Captura");
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 24, "UP/DN: frecuencia");
  u8g2.drawStr(0, 34, "RIGHT: iniciar");
  u8g2.drawStr(0, 50, freqLabels[selectedFreqIdx]);

  bool alive = cc1101A.isAlive();
  u8g2.drawStr(0, 62, alive ? "CC1101-A: OK" : "CC1101-A: N/C");
  u8g2.sendBuffer();
  delay(1500);
}

// ── Función de captura por polling de GDO0 ──
static void doCapture() {
  g_capturedSignal.timings.clear();
  g_capturedSignal.valid     = false;
  g_capturedSignal.freqMHz100 = freqOptions[selectedFreqIdx];

  cc1101A.setFrequency(freqOptions[selectedFreqIdx]);
  cc1101A.setOOKMode();
  cc1101A.setRx();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "SubGHz Captura");
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 24, "Esperando señal...");
  u8g2.drawStr(0, 36, freqLabels[selectedFreqIdx]);
  u8g2.sendBuffer();

  if (neoPixelActive) setNeoPixelColour("cyan");

  // Esperar primer flanco (timeout 5 s)
  uint32_t waitStart = millis();
  while (digitalRead(CC1101_A_GDO0_PIN) == LOW) {
    if (millis() - waitStart > 5000) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_5x8_tr);
      u8g2.drawStr(0, 32, "Timeout: no signal");
      u8g2.sendBuffer();
      if (neoPixelActive) setNeoPixelColour("red");
      cc1101A.setIdle();
      delay(1500);
      return;
    }
  }

  // Señal detectada — medir pulsos
  if (neoPixelActive) setNeoPixelColour("green");
  g_capturedSignal.startHigh = true;

  uint32_t pulseStart = micros();
  bool     lastLevel  = HIGH;
  uint32_t lastEdge   = micros();
  uint32_t now;

  g_capturedSignal.timings.reserve(RAW_CAPTURE_MAX_PULSES);

  while (g_capturedSignal.timings.size() < RAW_CAPTURE_MAX_PULSES) {
    bool level = digitalRead(CC1101_A_GDO0_PIN);
    now = micros();

    if (level != lastLevel) {
      uint32_t duration = now - lastEdge;
      if (duration >= RAW_CAPTURE_MIN_US) {
        g_capturedSignal.timings.push_back(duration);
      }
      lastEdge  = now;
      lastLevel = level;
    }

    // Timeout entre pulsos → fin de trama
    if ((now - lastEdge) > RAW_CAPTURE_TIMEOUT_US &&
        !g_capturedSignal.timings.empty()) {
      break;
    }
  }

  cc1101A.setIdle();

  if (g_capturedSignal.timings.size() >= 4) {
    g_capturedSignal.valid = true;
    captureComplete = true;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Captura OK!");
    u8g2.setFont(u8g2_font_5x8_tr);

    char buf[32];
    snprintf(buf, sizeof(buf), "Pulsos: %d",
             (int)g_capturedSignal.timings.size());
    u8g2.drawStr(0, 24, buf);

    uint32_t minT = 0xFFFFFFFF, maxT = 0;
    for (auto t : g_capturedSignal.timings) {
      if (t < minT) minT = t;
      if (t > maxT) maxT = t;
    }
    snprintf(buf, sizeof(buf), "Min:%uus Max:%uus",
             (unsigned)minT, (unsigned)maxT);
    u8g2.drawStr(0, 36, buf);
    snprintf(buf, sizeof(buf), "Freq: %s",
             freqLabels[selectedFreqIdx]);
    u8g2.drawStr(0, 48, buf);
    u8g2.drawStr(0, 60, "Listo para Replay");
    u8g2.sendBuffer();
    if (neoPixelActive) setNeoPixelColour("green");
    delay(2500);
  } else {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 32, "Captura invalida");
    u8g2.sendBuffer();
    if (neoPixelActive) setNeoPixelColour("red");
    delay(1500);
  }

  capturing = false;
}

// ── Loop principal ───────────────────────────
void subghzCaptureLoop() {
  bool upNow    = !digitalRead(BUTTON_UP_PIN);
  bool downNow  = !digitalRead(BUTTON_DOWN_PIN);
  bool rightNow = !digitalRead(BTN_PIN_RIGHT);

  // UP — frecuencia anterior
  if (upNow && !btnUpPrev) {
    selectedFreqIdx = (selectedFreqIdx - 1 + NUM_FREQS) % NUM_FREQS;
  }
  // DOWN — frecuencia siguiente
  if (downNow && !btnDownPrev) {
    selectedFreqIdx = (selectedFreqIdx + 1) % NUM_FREQS;
  }
  // RIGHT — iniciar captura
  if (rightNow && !btnRightPrev && !capturing) {
    capturing = true;
    doCapture();
  }

  btnUpPrev    = upNow;
  btnDownPrev  = downNow;
  btnRightPrev = rightNow;

  // Actualizar pantalla
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "SubGHz Captura");

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 22, "Frec:");
  u8g2.drawStr(30, 22, freqLabels[selectedFreqIdx]);

  if (captureComplete && g_capturedSignal.valid) {
    char buf[24];
    snprintf(buf, sizeof(buf), "Pulsos: %d",
             (int)g_capturedSignal.timings.size());
    u8g2.drawStr(0, 36, buf);
    u8g2.drawStr(0, 48, "Estado: Capturado");
  } else {
    u8g2.drawStr(0, 36, "Sin datos aun");
  }

  u8g2.drawStr(0, 60, "RIGHT=capturar");
  u8g2.sendBuffer();

  delay(50);
}

} // namespace SubGhzCapture


// ═════════════════════════════════════════════
//  REPLAY SUB-GHz
// ═════════════════════════════════════════════
namespace SubGhzReplay {

static int  replayRepeats   = RAW_REPLAY_REPEATS;
static bool replaying       = false;
static bool btnRightPrev    = false;
static bool btnUpPrev       = false;
static bool btnDownPrev     = false;

// ── Transmitir una trama raw por OOK ────────
// Estrategia: cuantizar los timings al período mínimo (1 bit),
// generar un buffer de bytes y enviarlo al TXFIFO del CC1101.
static void transmitRaw(const RawSignal& sig) {
  if (!sig.valid || sig.timings.empty()) return;

  // Encontrar período mínimo (= duración de 1 bit)
  uint32_t minPulse = 0xFFFFFFFF;
  for (auto t : sig.timings) {
    if (t < minPulse) minPulse = t;
  }
  if (minPulse < RAW_CAPTURE_MIN_US) minPulse = RAW_CAPTURE_MIN_US;

  // Construir bitstream: cada timing = N bits del mismo nivel
  std::vector<uint8_t> txBuf;
  txBuf.reserve(256);
  uint8_t  currentByte = 0;
  int      bitPos      = 7;
  bool     level       = sig.startHigh;

  for (uint32_t dur : sig.timings) {
    int numBits = (int)((dur + minPulse / 2) / minPulse);
    if (numBits < 1) numBits = 1;
    if (numBits > 64) numBits = 64; // sanity cap

    for (int b = 0; b < numBits; b++) {
      if (level) currentByte |= (1 << bitPos);
      bitPos--;
      if (bitPos < 0) {
        txBuf.push_back(currentByte);
        currentByte = 0;
        bitPos = 7;
      }
    }
    level = !level;
  }
  // Último byte parcial
  if (bitPos < 7) txBuf.push_back(currentByte);

  if (txBuf.empty()) return;

  // Calcular data rate para que el período de bit coincida con minPulse
  // DataRate (baud) = 1 / (minPulse_s) = 1e6 / minPulse_us
  // DataRate = 26e6 × (256 + DRATE_M) × 2^DRATE_E / 2^28
  // Calcular DRATE_E y DRATE_M:
  float targetBaud = 1e6f / (float)minPulse;
  // Clamp a rango CC1101 (0.6 kbaud – 600 kbaud)
  if (targetBaud <   600.0f) targetBaud =   600.0f;
  if (targetBaud > 250000.0f) targetBaud = 250000.0f;

  uint8_t drateE = 0;
  float   tmp    = targetBaud / 26e6f * (1 << 28);
  while (tmp > 511.0f && drateE < 15) { tmp /= 2.0f; drateE++; }
  uint8_t drateM = (uint8_t)(tmp - 256.0f);

  cc1101B.setFrequency(sig.freqMHz100);
  cc1101B.setOOKMode();
  cc1101B.setTxPower(7);   // potencia máxima

  // Ajustar data rate calculado
  cc1101B.writeReg(CC1101_MDMCFG4,
    (cc1101B.readReg(CC1101_MDMCFG4) & 0xF0) | (drateE & 0x0F));
  cc1101B.writeReg(CC1101_MDMCFG3, drateM);

  // Modo paquete de longitud fija para la transmisión
  cc1101B.writeReg(CC1101_PKTCTRL0, 0x00); // fixed length
  cc1101B.writeReg(CC1101_PKTLEN,
    (uint8_t)min((int)txBuf.size(), 255));

  cc1101B.setTx();
  delayMicroseconds(200);

  // Enviar al TXFIFO en bloques de 64 bytes
  size_t sent = 0;
  while (sent < txBuf.size()) {
    // Esperar espacio en FIFO (TX_FIFO_UNDERFLOW check)
    size_t chunk = min((size_t)64, txBuf.size() - sent);

    // Burst write al TXFIFO
    digitalWrite(cc1101B.csPin, LOW);
    while (digitalRead(CC1101_MISO_PIN));
    SPI.transfer(CC1101_TXFIFO | CC1101_BURST_BIT);
    for (size_t i = 0; i < chunk; i++) {
      SPI.transfer(txBuf[sent + i]);
    }
    digitalWrite(cc1101B.csPin, HIGH);

    sent += chunk;
    delayMicroseconds(500);
  }

  // Esperar fin de transmisión (max 500 ms)
  uint32_t tStart = millis();
  while ((cc1101B.readReg(0xF5 /*TXBYTES status*/) & 0x7F) > 0) {
    if (millis() - tStart > 500) break;
    delayMicroseconds(100);
  }

  cc1101B.setIdle();
}

// ── Inicialización ───────────────────────────
void subghzReplaySetup() {
  replaying = false;

  cc1101B.begin();
  cc1101B.setOOKMode();
  cc1101B.setIdle();

  if (neoPixelActive) setNeoPixelColour("purple");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "SubGHz Replay");
  u8g2.setFont(u8g2_font_5x8_tr);

  if (g_capturedSignal.valid) {
    char buf[32];
    snprintf(buf, sizeof(buf), "Pulsos: %d",
             (int)g_capturedSignal.timings.size());
    u8g2.drawStr(0, 26, buf);
    u8g2.drawStr(0, 38, "RIGHT: transmitir");
    u8g2.drawStr(0, 50, "UP/DN: repeticiones");
  } else {
    u8g2.drawStr(0, 30, "Sin captura previa");
    u8g2.drawStr(0, 44, "Usa SubGHz Captura");
  }

  bool alive = cc1101B.isAlive();
  u8g2.drawStr(0, 62, alive ? "CC1101-B: OK" : "CC1101-B: N/C");
  u8g2.sendBuffer();
  delay(1500);
}

// ── Loop principal ───────────────────────────
void subghzReplayLoop() {
  bool upNow    = !digitalRead(BUTTON_UP_PIN);
  bool downNow  = !digitalRead(BUTTON_DOWN_PIN);
  bool rightNow = !digitalRead(BTN_PIN_RIGHT);

  if (upNow && !btnUpPrev) {
    if (replayRepeats < 20) replayRepeats++;
  }
  if (downNow && !btnDownPrev) {
    if (replayRepeats > 1) replayRepeats--;
  }

  if (rightNow && !btnRightPrev && !replaying) {
    if (!g_capturedSignal.valid) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_5x8_tr);
      u8g2.drawStr(0, 32, "No hay señal");
      u8g2.drawStr(0, 44, "Captura primero");
      u8g2.sendBuffer();
      delay(1500);
    } else {
      replaying = true;
      if (neoPixelActive) setNeoPixelColour("yellow");

      for (int r = 0; r < replayRepeats; r++) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "Transmitiendo...");
        u8g2.setFont(u8g2_font_5x8_tr);
        char buf[24];
        snprintf(buf, sizeof(buf), "Intento %d/%d", r + 1, replayRepeats);
        u8g2.drawStr(0, 26, buf);
        u8g2.sendBuffer();

        transmitRaw(g_capturedSignal);
        delay(50); // pausa entre repeticiones
      }

      if (neoPixelActive) setNeoPixelColour("green");
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 10, "Replay OK!");
      u8g2.setFont(u8g2_font_5x8_tr);
      char buf[32];
      snprintf(buf, sizeof(buf), "%d envios completados", replayRepeats);
      u8g2.drawStr(0, 30, buf);
      u8g2.sendBuffer();
      delay(2000);
      replaying = false;
    }
  }

  btnUpPrev    = upNow;
  btnDownPrev  = downNow;
  btnRightPrev = rightNow;

  // Pantalla de estado
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "SubGHz Replay");
  u8g2.setFont(u8g2_font_5x8_tr);

  if (g_capturedSignal.valid) {
    char buf[32];
    snprintf(buf, sizeof(buf), "Pulsos: %d",
             (int)g_capturedSignal.timings.size());
    u8g2.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "Repetir x%d", replayRepeats);
    u8g2.drawStr(0, 36, buf);
    u8g2.drawStr(0, 48, "RIGHT=enviar");
    u8g2.drawStr(0, 60, "UP/DN=reps");
  } else {
    u8g2.drawStr(0, 32, "Sin datos");
    u8g2.drawStr(0, 44, "Captura primero");
  }

  u8g2.sendBuffer();
  delay(50);
}

} // namespace SubGhzReplay
