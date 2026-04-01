/* ____________________________
   HackNRF24Team - Sub-GHz CC1101 Expansion
   Universidad Nacional Autónoma de México (UNAM)
   Facultad de Ingeniería — Sistemas de Comunicaciones
   MIT License
   ________________________________________ */

#ifndef CC1101_MODULE_H
#define CC1101_MODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <string>
#include "setting.h"

// ─────────────────────────────────────────────
//  Pin Definitions — CC1101 en bus VSPI
//  VSPI compartido:  SCK=18  MISO=19  MOSI=23
// ─────────────────────────────────────────────
#define CC1101_SCK_PIN   18
#define CC1101_MISO_PIN  19
#define CC1101_MOSI_PIN  23

// Módulo A → Captura (RX)
#define CC1101_A_CS_PIN   12
#define CC1101_A_GDO0_PIN 34   // solo entrada (ESP32 input-only GPIO)

// Módulo B → Replay (TX)
#define CC1101_B_CS_PIN   13
#define CC1101_B_GDO0_PIN 35   // solo entrada (ESP32 input-only GPIO)

// ─────────────────────────────────────────────
//  Frecuencias soportadas (MHz × 100)
// ─────────────────────────────────────────────
#define FREQ_300MHZ   30000
#define FREQ_315MHZ   31500
#define FREQ_433MHZ   43392   // 433.92 MHz  ← default
#define FREQ_868MHZ   86800
#define FREQ_915MHZ   91500

// ─────────────────────────────────────────────
//  Registros de configuración CC1101
// ─────────────────────────────────────────────
#define CC1101_IOCFG2    0x00
#define CC1101_IOCFG1    0x01
#define CC1101_IOCFG0    0x02
#define CC1101_FIFOTHR   0x03
#define CC1101_SYNC1     0x04
#define CC1101_SYNC0     0x05
#define CC1101_PKTLEN    0x06
#define CC1101_PKTCTRL1  0x07
#define CC1101_PKTCTRL0  0x08
#define CC1101_ADDR      0x09
#define CC1101_CHANNR    0x0A
#define CC1101_FSCTRL1   0x0B
#define CC1101_FSCTRL0   0x0C
#define CC1101_FREQ2     0x0D
#define CC1101_FREQ1     0x0E
#define CC1101_FREQ0     0x0F
#define CC1101_MDMCFG4   0x10
#define CC1101_MDMCFG3   0x11
#define CC1101_MDMCFG2   0x12
#define CC1101_MDMCFG1   0x13
#define CC1101_MDMCFG0   0x14
#define CC1101_DEVIATN   0x15
#define CC1101_MCSM2     0x16
#define CC1101_MCSM1     0x17
#define CC1101_MCSM0     0x18
#define CC1101_FOCCFG    0x19
#define CC1101_BSCFG     0x1A
#define CC1101_AGCCTRL2  0x1B
#define CC1101_AGCCTRL1  0x1C
#define CC1101_AGCCTRL0  0x1D
#define CC1101_WOREVT1   0x1E
#define CC1101_WOREVT0   0x1F
#define CC1101_WORCTRL   0x20
#define CC1101_FREND1    0x21
#define CC1101_FREND0    0x22
#define CC1101_FSCAL3    0x23
#define CC1101_FSCAL2    0x24
#define CC1101_FSCAL1    0x25
#define CC1101_FSCAL0    0x26
#define CC1101_RCCTRL1   0x27
#define CC1101_RCCTRL0   0x28
#define CC1101_FSTEST    0x29
#define CC1101_PTEST     0x2A
#define CC1101_AGCTEST   0x2B
#define CC1101_TEST2     0x2C
#define CC1101_TEST1     0x2D
#define CC1101_TEST0     0x2E

// Strobes
#define CC1101_SRES      0x30
#define CC1101_SFSTXON   0x31
#define CC1101_SXOFF     0x32
#define CC1101_SCAL      0x33
#define CC1101_SRX       0x34
#define CC1101_STX       0x35
#define CC1101_SIDLE     0x36
#define CC1101_SWOR      0x38
#define CC1101_SPWD      0x39
#define CC1101_SFRX      0x3A
#define CC1101_SFTX      0x3B
#define CC1101_SWORRST   0x3C
#define CC1101_SNOP      0x3D
#define CC1101_PATABLE   0x3E
#define CC1101_TXFIFO    0x3F
#define CC1101_RXFIFO    0x3F

// Bits de acceso SPI
#define CC1101_READ_BIT  0x80
#define CC1101_BURST_BIT 0x40

// ─────────────────────────────────────────────
//  Parámetros de captura raw
// ─────────────────────────────────────────────
#define RAW_CAPTURE_MAX_PULSES  1024   // máximo de pulsos almacenados
#define RAW_CAPTURE_TIMEOUT_US  15000  // timeout entre pulsos (µs)
#define RAW_CAPTURE_MIN_US      80     // pulso mínimo válido (µs)
#define RAW_REPLAY_REPEATS      5      // repeticiones en replay

// ─────────────────────────────────────────────
//  Estructura de señal capturada
// ─────────────────────────────────────────────
struct RawSignal {
  std::vector<uint32_t> timings;  // duraciones en µs (alternando HIGH/LOW)
  bool     startHigh;             // si el primer pulso es HIGH
  uint32_t freqMHz100;            // frecuencia × 100 (ej. 43392 = 433.92 MHz)
  bool     valid;                 // captura con datos válidos
};

// ─────────────────────────────────────────────
//  Driver low-level CC1101
// ─────────────────────────────────────────────
class CC1101Driver {
public:
  CC1101Driver(uint8_t csPin, uint8_t gdo0Pin);

  void     begin();
  void     reset();
  void     setIdle();
  void     setRx();
  void     setTx();
  void     setFrequency(uint32_t freqMHz100);
  void     setOOKMode();
  void     setTxPower(uint8_t paLevel);
  uint8_t  readReg(uint8_t addr);
  void     writeReg(uint8_t addr, uint8_t val);
  void     strobe(uint8_t cmd);
  bool     isAlive();

  uint8_t  csPin;
  uint8_t  gdo0Pin;

private:
  void     _csLow();
  void     _csHigh();
  void     _waitMISO();
};

// ─────────────────────────────────────────────
//  Namespaces públicos
// ─────────────────────────────────────────────
namespace SubGhzCapture {
  void subghzCaptureSetup();
  void subghzCaptureLoop();
}

namespace SubGhzReplay {
  void subghzReplaySetup();
  void subghzReplayLoop();
}

// Señal compartida entre captura y replay
extern RawSignal g_capturedSignal;

// Instancias globales de los dos módulos
extern CC1101Driver cc1101A;  // captura
extern CC1101Driver cc1101B;  // replay

#endif // CC1101_MODULE_H
