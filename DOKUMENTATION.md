# RB-Avionics ESP32-S3 - Technische Dokumentation

## Projektübersicht

**RB-Avionics** ist eine Open-Source Aviation Instruments Suite für ESP32-S3 basierte Displays. Dies ist die RB-02 Variante mit Fokus auf "SixPack" Fluginstrumente, GPS-Integration, IMU-basierter Lagenanzeige und Stratux BLE-Verkehrsempfang.

### Ziel-Hardware

- **Display:** ESP32-S3-Touch-LCD-2.8C oder 2.1" (Waveshare)
- **Sensoren:**
  - BMP280 Barometer (Höhenmessung)
  - QMI8658 IMU (6-Achsen Gyro/Beschleunigungsmesser)
  - GPS-Modul (NMEA, z.B. uBlox NEO M6N)
  - PCF85063 Echtzeituhr (RTC)
- **Peripherie:**
  - GT911 (2.8") oder CST820 (2.1") Touch Controller
  - SD-Karte für Karten, Checklisten, Datenaufzeichnung
  - Batteriespannungsüberwachung
  - Summer (Buzzer)

### Technische Spezifikationen

- **MCU:** ESP32-S3 (Dual-Core, 240MHz)
- **Flash:** 16MB (DIO Mode)
- **PSRAM:** Oktal-SPI, 80MHz (erforderlich)
- **Display:** 480×480 RGB-LCD (ST7701S)
- **Framework:** ESP-IDF 5.5.1+
- **UI-Library:** LVGL ~8.2.0

---

## Architektur

### Schichtenmodell

```
┌─────────────────────────────────────────┐
│   Anwendungsschicht (RB02.c)            │
│   • Instrumenten-Implementierungen      │
│   • UI-Workflow & Zustandsverwaltung    │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│   LVGL Grafik-Layer                      │
│   • Display-Rendering                    │
│   • Touch-Eingabeverarbeitung           │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│   Hardware-Abstraktionsschicht          │
│   • LCD-Treiber (ST7701S)               │
│   • Touch-Treiber (GT911/CST820)        │
│   • Sensor-Treiber (QMI8658, BMP280)    │
│   • I2C, UART, SD/MMC Schnittstellen    │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│   ESP-IDF (FreeRTOS)                     │
└─────────────────────────────────────────┘
```

### Build-Konfigurationssystem

Das Projekt nutzt ein Template-System für verschiedene Hardware/Feature-Kombinationen via `main/RB/BuildMachine.h`:

**Template-Dateien:**
- `BuildMachine-Template-RB-02-TOUCH-30hz-GPS-2.1.h` - 2.1" Touch, GPS
- `BuildMachine-Template-RB-02-NONTOUCH-30hz-EMS-2.8.h` - 2.8" ohne Touch, EMS
- `BuildMachine-Template-RB-02-TOUCH-30hz-TRAFFIC-2.8.h` - Verkehrsempfänger

**Wichtige Compile-Flags:**
```c
#define RB_02_DISPLAY_SIZE RB_02_DISPLAY_21    // Display-Größe
#define RB_02_DISPLAY_TOUCH 1                   // Touch an/aus
#define RB_ENABLE_AAT 1                         // Advanced Attitude
#define RB_ENABLE_ALT 1                         // Höhenmesser
#define RB_ENABLE_MAP 1                         // GPS-Karte
#define RB_ENABLE_GPS 1                         // GPS NMEA
#define RB02_ESP_BLUETOOTH 1                    // BLE Verkehr
```

---

## Hauptkomponenten

### 1. Anwendungseinstieg (main.c)

**Initialisierungssequenz:**

1. **NVS Flash** (Zeile 134-139) - Persistente Konfiguration
2. **Driver_Loop Task** (Zeile 140) - Sensor-Polling auf Core 0
3. **UART Setup** (Zeile 142-148) - GPS-Empfang (Pin 44)
4. **Display & Touch** (Zeile 150-154) - ST7701S LCD + Touch-Init
5. **LVGL** (Zeile 156) - Framebuffer-Allokation, Grafik-Setup
6. **Anwendung** (Zeile 168) - Start RB02_Main(), LVGL Event-Loop

### 2. Sensor-Task (Driver_Loop)

**Datei:** `main/main.c` (Zeilen 70-93)

**Strategie:**
- **IMU (QMI8658):** Wird jeden Zyklus gepollt (~20-50Hz)
- **RTC, Barometer, Batterie:** Nur 1× pro Sekunde
- **Kalibrierungs-Gate:** Erst nach `workflow > 100` werden Sensoren ausgelesen

```c
void Driver_Loop(void *parameter) {
    int loopThreshold = 10;
    while (1) {
        QMI8658_Loop();  // IMU @ Loop-Rate

        if (loopThreshold == 0) {
            RTC_Loop();
            if (workflow > 100) {
                BAT_Get_Volts();
                Get_BMP280();
            }
            loopThreshold = 1000 / (10 + DriverLoopMilliseconds);
        }
        loopThreshold--;
        vTaskDelay(pdMS_TO_TICKS(10 + DriverLoopMilliseconds));
    }
}
```

**Abstimmbar:** `DriverLoopMilliseconds` (Standard 40ms = 20Hz)

### 3. Kernanwendung (RB02.c)

**Datei:** `main/RB/RB02.c` (78.267 Zeilen - sehr groß!)

**Hauptbestandteile:**

- **Tab-basiertes UI** (Zeilen 250-330)
  - Enum-gesteuerte Navigation
  - Bedingte Kompilierung pro Instrument
  - Beispiele: `RB02_TAB_SPD`, `RB02_TAB_AAT`, `RB02_TAB_MAP`

- **Touch-Zonen-Erkennung** (Zeilen 468-474)
  ```c
  touchLocation getTouchLocation(lv_coord_t x, lv_coord_t y) {
      int8_t sx = x / RB02_TOUCH_SECTION_SIZE;  // 160px Sektionen
      int8_t sy = y / RB02_TOUCH_SECTION_SIZE;
      return sy * RB02_TOUCH_SECTION + sx;  // 3×3 Raster
  }
  ```
  - Bildschirm in 3×3 Raster unterteilt
  - Ermöglicht Gestennavigation

- **Globale Zustandsvariablen** (Zeilen 332-452)
  - Status-Flags: `Operative_GPS`, `Operative_BMP280`, `Operative_Attitude`
  - UI-Referenzen für alle Instrumente
  - Konfigurationsparameter (QNH, Geschwindigkeitsbereiche, G-Meter)

### 4. Workflow-Management

**Datei:** `main/RB/RB02_Workflow.c`

**Workflow-Zustände:**
- `workflow = 0-100`: Initialisierung & Kalibrierung
- `workflow > 100`: Normalbetrieb (aktiviert Sensor-Polling)

**Singleton-Pattern:**
```c
RB02_Status *singletonConfig();  // Globale Status-Struktur
```

---

## Instrumenten-Implementierungen

### Advanced Attitude Indicator (Künstlicher Horizont)

**Dateien:** `main/RB/RB02_AAttitude.c`, `RB02_AAttitude.h`

**Features:**
- GPS-gestützte Lageberechnung
- Ball-Indikator für Koordination
- Integriertes Variometer & Höhenmesser
- Sky-Tiles für 3D-ähnliche Darstellung (10×10 Kachelmatrix)
- Bogen-Indikatoren für Nick/Roll-Grenzen

**Struktur:** `RB02_AdvancedAttitude_Status`
- Enthält alle LVGL-Objektreferenzen
- Nick/Roll/Gier-Winkel
- Geschwindigkeit, Track, Höhenmesser-Integration

### Höhenmesser (Altimeter)

**Dateien:** `main/RB/RB02_Altimeter.c`, `RB02_Altimeter.h`

**Features:**
- 7-Segment-Display-Rendering
- QNH-Einstellung (Druckhöhe)
- Druckhöhenberechnung aus BMP280
- GPS-Höhenabgleich

### GPS-Karte

**Dateien:** `main/RB/RB02_GPSMap.c`, `RB02_GPSMap.h`

**Kartensystem:**
- MapBox-kompatibles BMP-Kachelformat
- Externe SD-Karten-Speicherung
- 9-Kachel-Anzeige (3×3 Raster) für flüssiges Schwenken
- Mercator-Projektion

**Struktur:** `RB02_GpsMapStatus`
```c
typedef struct {
    lv_obj_t *tiles[9];           // 3×3 Kachelanzeige
    uint8_t zoomLevel;
    int32_t latitude100;          // Festkomma-Lat/Lon
    int32_t longitude100;
    TileXY lastTile;
} RB02_GpsMapStatus;
```

### Richtungskreisel/Track-Anzeige

**Dateien:** `main/RB/RB02_Gyro.c`, `RB02_Gyro.h`

**Features:**
- GPS-Track-basierter Kurs
- Kompass-Rose mit 12 Himmelsrichtungen
- Gier-Korrektur aus GPS
- Track-Quellen-Indikator (GPS vs. Gyro)

### Weitere Instrumente

- **Geschwindigkeitsanzeige:** `RB02_Speed.c` (7-Segment)
- **Wendezeiger:** `RB02_TurnCoordinator.c`
- **Vertikaler Geschwindigkeitsmesser:** Variometer in AAT integriert
- **Motorüberwachung:** `RB04_EMS.c` (Engine Monitoring System)
- **Verkehrsanzeige:** `RB05_Traffic.c`, `RB05_Radar.c` (Stratux BLE)

---

## Sensorfusion & Navigation

### Madgwick AHRS Filter

**Datei:** `main/RB/madgwick.c`

**Algorithmus:**
- Quaternion-basierte Orientierungsschätzung
- Fusioniert 6-Achsen-IMU-Daten (Gyro + Beschleunigungsmesser)
- Adaptiver Beta-Gain basierend auf `AttitudeBalanceAlpha`
- Sample-Frequenz: 20Hz (konfigurierbar)

**Ablauf:**
```c
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az) {
    1. Normalisiere Beschleunigungsdaten
    2. Berechne Quaternion-Rate aus Gyroskop
    3. Gradientenabstiegs-Korrektur mit Beschleunigungsmesser
    4. Feedback mit Beta-Gain anwenden
    5. Quaternion integrieren
    6. Quaternion normalisieren
}
```

### GPS-Integration

**Datei:** `main/RB/RB02_NMEA.h`

**NMEA-Parsing:**
- UART1-Empfang (1024-Byte-Ringpuffer, Pin 44)
- Unterstützte Sätze: RMC, GGA, GSA, GSV
- Globale Struktur `NMEA_DATA`

**GPS-Struktur:**
```c
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    gps_fix_t fix;
    uint8_t sats_in_use;
    float speed;          // m/s
    float cog;            // Course over Ground
    float variation;      // Magnetische Deklination
    gps_satellite_t sats_desc_in_view[16];
} gps_t;
```

**GPS-gestützte Lage:**
- GPS-Track korrigiert Gier-Drift
- GPS-Beschleunigung kompensiert anhaltende Kurven
- Lateralbeschleunigung aus GPS-Geschwindigkeit

### IMU-Datenfluss (QMI8658)

**Datei:** `main/QMI8658/QMI8658.c`

**Initialisierung:**
1. I2C-Erkennung (Adresse 0x6B)
2. NVS-Wiederherstellung (gespeicherte Konfiguration)
3. Skalen-Konfiguration:
   - Beschleunigungsmesser: 2G, 4G, 8G, 16G
   - Gyroskop: 16-1024 DPS
4. ODR (Ausgabedatenrate): 8000Hz bis 3Hz
5. Tiefpassfilter: 4 Modi
6. Madgwick-Initialisierung (20Hz)

**Loop-Verarbeitung:**
```c
void QMI8658_Loop(void) {
    getAccelerometer();  // 3-Achsen-Beschleunigung
    getGyroscope();      // 3-Achsen-Winkelgeschwindigkeit
    getAttitude();       // Madgwick-Filter aktualisieren
    getGFactor();        // G-Kraft berechnen
}
```

**Kalibrierung:**
- Hardware-Gyro-Kalibrierung
- Bias-Kompensation in NVS gespeichert
- Laufzeit-Bias-Anpassung
- Vibrationsfilterung (konfigurierbarer LPF)

### Barometer (BMP280)

**Implementierung:** In `main/RB/RB02.c` (Zeile 1816)

**Verarbeitung:**
1. I2C-Auslesen
2. Kalibrierung (12 Kalibrierkonstanten)
3. Höhenberechnung (QNH-Einstellung)
4. Variometer (Druckänderungsrate)
5. Anzeige-Update (1Hz)

---

## Display & Rendering

### LVGL-Integration

**Datei:** `main/LVGL_Driver/LVGL_Driver.c`

**Framebuffer-Architekturen:**

**Option 1: Doppelpufferung** (CONFIG_EXAMPLE_DOUBLE_FB)
```c
esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2);
lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 480 * 480);
```
- Zwei Hardware-Framebuffer
- Tearing-freies Rendering
- Vollbild-Refresh

**Option 2: PSRAM-Puffer** (Standard)
```c
buf1 = heap_caps_malloc(480 * 480, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
buf2 = heap_caps_malloc(480 * 480, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
```
- Aus PSRAM allokiert
- DMA-fähiger Speicher
- Doppelpufferung in PSRAM

**Option 3: Semaphore Tearing-Prävention**
- Software-Vsync-Synchronisierung
- Geringerer Speicherverbrauch

**Touch-Eingabe:**
```c
void example_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    esp_lcd_touch_read_data(drv->user_data);
    esp_lcd_touch_get_coordinates(...);
    // Aktualisiert data->point.x, data->point.y, data->state
}
```

### LCD-Treiber

**Datei:** `main/LCD_Driver/ST7701S.c`

**Display-spezifisch:**
- `ST7701S_28.c` für 2.8" (480×480)
- `ST7701S_21.c` für 2.1" (480×480)
- Bedingte Kompilierung via `RB_02_DISPLAY_SIZE`

**RGB-Schnittstelle:**
- ESP32-S3 RGB-LCD-Peripherie
- 16-Bit-Paralleldatenbus
- Hardware-beschleunigtes DMA

### Touch-Controller

**Datei:** `main/Touch_Driver/Touch_Driver.c`

- **GT911** für 2.8" (I2C 0x5D/0x14)
- **CST820** für 2.1"
- Einheitliche `esp_lcd_touch`-Abstraktion

---

## Speicherverwaltung

### PSRAM-Konfiguration

**Datei:** `sdkconfig.defaults.esp32s3`

**Kritische Einstellungen:**
```
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y              # Oktal-SPI-Modus
CONFIG_SPIRAM_SPEED_80M=y             # 80MHz PSRAM
CONFIG_SPIRAM_FETCH_INSTRUCTIONS=y    # Code aus PSRAM
CONFIG_SPIRAM_RODATA=y                # Konstanten in PSRAM
```

**Vorteile:**
- Ermöglicht höheren PCLK für Display
- Reduziert IRAM-Druck
- Erlaubt große Framebuffer

### Speicher-Allokationsstrategie

**Framebuffer:**
- Größe: 480 × 480 × 2 Bytes = 460.800 Bytes pro Puffer
- Doppelpufferung: ~921.600 Bytes gesamt
- Aus PSRAM mit DMA-Fähigkeit

**Bild-Assets:**
- In Binary kompiliert (`.c`-Dateien mit Image-Arrays)
- Beispiele: `att_aircraft.c`, `arcGreen.c`, `RB02Images.c`
- In Flash gespeichert, via LVGL-Deskriptoren zugegriffen

**Heap-Nutzung:**
- FreeRTOS-Heaps für dynamische Allokation
- PSRAM-Heap für große Objekte
- Interner SRAM für kritische Echtzeit-Daten

---

## Konfiguration & Persistenz

### NVS (Non-Volatile Storage)

**Datei:** `main/RB/RB02_Config.h`

**Gespeicherte Parameter:**
- Gyro-Hardware-Kalibrierung
- Bluetooth-Einstellungen
- IMU-Konfiguration (Skalen, Filter)
- Geschwindigkeitsbereich-Einstellungen
- G-Meter Min/Max
- QNH-Druckeinstellung
- Datenlogger-Aktivierung

**Muster:**
```c
nvs_handle_t my_handle;
nvs_open("storage", NVS_READWRITE, &my_handle);
nvs_set_u8(my_handle, "key", value);
nvs_commit(my_handle);
nvs_close(my_handle);
```

### SD-Karten-Nutzung

**Datei:** `main/SD_Card/SD_MMC.c`

**Gespeicherte Daten:**
1. **Kartenkacheln:** `/maps/zoom/x/y.bmp` (MapBox-Format)
2. **Checklisten:** Textdateien
3. **Flugdaten-Logs:** CSV-Format (Timestamp, Position, Höhe, Geschwindigkeit)
4. **Konfigurationsdateien:** Optional externe Config

---

## Build-System

### CMake-Struktur

**Root:** `CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-S3-Touch-LCD-2.8C-Test)
```

**Main-Komponente:** `main/CMakeLists.txt`
- Alle `.c`-Dateien explizit gelistet
- Include-Verzeichnisse spezifiziert
- Keine Wildcards (ESP-IDF Best Practice)

### Kconfig

**Datei:** `main/Kconfig.projbuild`

**Menü-Optionen:**
- `EXAMPLE_DOUBLE_FB`: Doppel-Framebuffer aktivieren
- `EXAMPLE_USE_BOUNCE_BUFFER`: Bounce-Buffer-Modus
- `EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM`: Semaphore Tearing-Prävention
- `BT_ENABLED`: Bluetooth aktivieren
- Schriftarten-Auswahl (Montserrat 12, 14, 16)

---

## Performance-Charakteristiken

### Update-Raten

- **IMU-Polling:** 20-50Hz (einstellbar via `DriverLoopMilliseconds`)
- **Display-Refresh:** 30Hz typisch (~33ms Framezeit)
- **Sensorfusion:** 20Hz (Madgwick-Update-Rate)
- **GPS-Update:** 1Hz (NMEA-Satzrate)
- **Barometer-Update:** 1Hz
- **LVGL-Timer:** Alle 10ms (Hauptschleife)

### Echtzeit-Anforderungen

- Lagenanzeige: <50ms Latenz (IMU → Display)
- Touch-Antwort: <100ms
- GPS-Positions-Update: <1s
- Höhenmesser-Update: <1s

### Task-Prioritäten

- Driver_Loop: Priorität 3, Core 0
- LVGL-Hauptschleife: Priorität 1 (implizit), Core 1

---

## Abhängigkeiten zwischen Komponenten

### Abhängigkeitsgraph

```
RB02.c (Hauptanwendung)
  ├─→ RB02_Workflow.c (Zustandsmaschine)
  ├─→ RB02_Config.c (NVS-Persistenz)
  ├─→ RB02_AAttitude.c ──→ madgwick.c (Sensorfusion)
  │                     └─→ RB02_NMEA.h (GPS)
  ├─→ RB02_Altimeter.c ──→ BMP280
  ├─→ RB02_GPSMap.c ────→ RB02_NMEA.h
  │                     └─→ SD_MMC.c (Kacheln)
  ├─→ RB02_Gyro.c ──────→ RB02_NMEA.h
  ├─→ RB02_Checklist.c ─→ SD_MMC.c
  ├─→ RB05_Traffic.c ───→ RB05_ESP_Bluetooth.c
  │                     └─→ RB05_Radar.c
  └─→ RB04_EMS.c

LVGL_Driver.c
  ├─→ ST7701S.c (LCD)
  ├─→ Touch_Driver.c (GT911/CST820)
  └─→ lvgl (Bibliothek)

QMI8658.c (IMU)
  ├─→ I2C_Driver.c
  ├─→ madgwick.c
  └─→ NVS (Kalibrierung)

main.c
  ├─→ Driver_Init()
  ├─→ LCD_Init()
  ├─→ Touch_Init()
  ├─→ LVGL_Init()
  └─→ RB02_Main()
```

### Kritische Datenteilung

**Globale Zustandsvariablen:**
- `workflow`: Kalibrierstatus (main.c → RB02.c)
- `NMEA_DATA`: GPS-Daten (UART → RB02.c → Instrumente)
- `Accel`, `Gyro`: IMU-Daten (QMI8658.c → RB02.c)
- `bmp280Pressure`: Barometer (RB02.c → Höhenmesser)
- `q0, q1, q2, q3`: Quaternion (madgwick.c → RB02.c)

**Gemeinsame Ressourcen:**
- I2C-Bus: QMI8658, BMP280, PCF85063, TCA9554
- UART1: GPS NMEA
- SD/MMC: Kartenkacheln, Checklisten, Logs
- NVS: Persistente Konfiguration

---

## Wichtige Dateien (Referenz)

### Kernanwendung
- `main/main.c` - Einstiegspunkt, Initialisierung
- `main/RB/RB02.c` - Hauptanwendung, Instrumentenimplementierungen
- `main/RB/RB02_Workflow.c` - Zustandsmaschine
- `main/RB/BuildMachine.h` - Build-Konfiguration

### Instrumente
- `main/RB/RB02_AAttitude.c/h` - Künstlicher Horizont
- `main/RB/RB02_Altimeter.c/h` - Höhenmesser
- `main/RB/RB02_GPSMap.c/h` - GPS-Karte
- `main/RB/RB02_Gyro.c/h` - Richtungskreisel

### Sensorfusion
- `main/RB/madgwick.c/h` - Madgwick AHRS Filter
- `main/QMI8658/QMI8658.c/h` - IMU-Treiber

### Display
- `main/LVGL_Driver/LVGL_Driver.c` - LVGL-Integration
- `main/LCD_Driver/ST7701S.c` - LCD-Treiber
- `main/Touch_Driver/Touch_Driver.c` - Touch-Eingabe

### Konfiguration
- `main/RB/RB02_Config.c/h` - NVS-Persistenz
- `main/RB/RB02_Defines.h` - Compile-Zeit-Defines
- `main/Kconfig.projbuild` - menuconfig-Optionen
- `sdkconfig.defaults.esp32s3` - PSRAM-Konfiguration

---

## Einzigartige Features

### 1. Multi-Display-Unterstützung
Eine einzige Codebasis für:
- 2.8" runde Displays (480×480, GT911 Touch)
- 2.1" runde Displays (480×480, CST820 Touch)
- Touch- und Nicht-Touch-Varianten

### 2. GPS-gestützte Lagenanzeige
Kombiniert IMU und GPS:
- GPS-Track korrigiert Gier-Drift
- GPS-Beschleunigung kompensiert anhaltende Kurven
- Lateralbeschleunigung aus GPS-Geschwindigkeit

### 3. Operationsstatus-Indikatoren
Rote "X"-Overlays für inoperative Instrumente:
- `Operative_GPS`: GPS-Fix-Status
- `Operative_BMP280`: Barometer-Gesundheit
- `Operative_Attitude`: IMU-Kalibrierstatus

### 4. Modulares Instrumenten-Design
Jedes Instrument ist in sich geschlossen:
- `CreateScreen()`: UI-Konstruktion
- `Tick()`: Periodisches Update
- `Touch_X()`: Touch-Handler
- Eigene Status-Struktur

### 5. Singleton-Konfigurationsmuster
Globaler Zustand via `singletonConfig()`:
- Einzige Wahrheitsquelle
- Einfache Serialisierung zu NVS
- Reduziert Parameter-Übergabe

### 6. Hardware-Kalibrierungssystem
Fortgeschrittene IMU-Kalibrierung:
- Hardware-Gyro-Bias-Kompensation
- Laufzeit-Beschleunigungsmesser-Ausrichtung
- Panel-Montagewinkel-Korrektur
- Persistente Speicherung in NVS

---

## Build-Befehle

```bash
# Projekt konfigurieren
idf.py menuconfig

# Projekt bauen
idf.py build

# Auf Gerät flashen
idf.py -p <PORT> flash

# Seriellen Monitor starten
idf.py -p <PORT> monitor

# Bauen und Flashen in einem
idf.py -p <PORT> flash monitor

# Clean Build
idf.py fullclean
```

### Flash-Konfiguration
- Chip: esp32s3
- Flash-Modus: DIO
- Flash-Größe: 16MB
- Bootloader: 0x0
- Partitionstabelle: 0x8000
- Anwendung: 0x10000

---

## Lizenz

GNU AGPLv3 / Duale kommerzielle Lizenzierung

---

## Zusammenfassung

Das RB-Avionics ESP32-S3-Projekt ist ein ausgereiftes Embedded-System mit:

**Technische Erfolge:**
- Echtzeit-Sensorfusion @ 20Hz
- Hochleistungsgrafik auf ressourcenbeschränkter Hardware
- Multi-Display-Unterstützung aus einer Codebasis
- Luftfahrttaugliche Instrumentenimplementierungen
- GPS/IMU-Integration für erhöhte Genauigkeit

**Empfohlen für:**
- Luftfahrt-Instrumentenentwicklung
- ESP32-S3-Grafikanwendungen
- Echtzeit-Sensorfusionssysteme
- Multi-Varianten-Produktlinien-Entwicklung
