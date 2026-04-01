/* ____________________________
   HackNRF24Team - Sub-GHz CC1101 Expansion
   Universidad Nacional Autónoma de México (UNAM)
   Facultad de Ingeniería — Sistemas de Comunicaciones
   Fork basado en: https://github.com/cifertech/nrfbox
   MIT License
   ________________________________________ */

#ifndef CONFIG_H
#define CONFIG_H

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64

// Push Buttons
#define BUTTON_UP_PIN       26
#define BUTTON_SELECT_PIN   33
#define BUTTON_DOWN_PIN     32
#define BTN_PIN_RIGHT       27
#define BTN_PIN_LEFT        25

// SD Card
#define SD_CS_PIN      5
#define FIRMWARE_FILE  "/firmware.bin"

// nRF24L01 — bus HSPI
#define NRF_CE_PIN_A   5
#define NRF_CSN_PIN_A  17
#define NRF_CE_PIN_B   16
#define NRF_CSN_PIN_B  4
#define NRF_CE_PIN_C   15
#define NRF_CSN_PIN_C  2

// CC1101 — bus VSPI (SCK=18  MISO=19  MOSI=23)
// Módulo A (Captura RX) : CS=12  GDO0=34
// Módulo B (Replay TX)  : CS=13  GDO0=35
#define CC1101_SCK_PIN    18
#define CC1101_MISO_PIN   19
#define CC1101_MOSI_PIN   23
#define CC1101_A_CS_PIN   12
#define CC1101_A_GDO0_PIN 34
#define CC1101_B_CS_PIN   13
#define CC1101_B_GDO0_PIN 35

// Dependencias comunes
#include "setting.h"
#include "cc1101_module.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <vector>
#include <string>
#include <SD.h>
#include <Update.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_system.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_bt.h>

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern Adafruit_NeoPixel pixels;

// Namespaces BLE
namespace BleJammer  { void blejammerSetup();  void blejammerLoop();  }
namespace BleScan    { void blescanSetup();    void blescanLoop();    }
namespace SourApple  { void sourappleSetup();  void sourappleLoop();  }
namespace Spoofer    { void spooferSetup();    void spooferLoop();    }

// Namespaces nRF24 / 2.4 GHz
namespace Analyzer   { void analyzerSetup();   void analyzerLoop();   }
namespace ProtoKill  { void blackoutSetup();   void blackoutLoop();   }
namespace Scanner    { void scannerSetup();    void scannerLoop();    }
namespace Jammer     { void jammerSetup();     void jammerLoop();     }

// Namespaces WiFi
namespace WifiScan   { void wifiscanSetup();   void wifiscanLoop();   }
namespace Deauther   { void deautherSetup();   void deautherLoop();   }

// Namespaces Sub-GHz CC1101
namespace SubGhzCapture { void subghzCaptureSetup(); void subghzCaptureLoop(); }
namespace SubGhzReplay  { void subghzReplaySetup();  void subghzReplayLoop();  }

#endif // CONFIG_H
