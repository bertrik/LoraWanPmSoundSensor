// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// Copyright (c) 2023 Marcel Meek
// Copyright (c) 2023 Bas van Drunen

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <Arduino.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <ElegantOTA.h>

#include "lmic.h"
#include <hal/hal.h>
#include "arduino_lmic_hal_boards.h"

#include <SPI.h>
#include <SSD1306.h>
#include <SparkFunBME280.h>
#include "soc/efuse_reg.h"
#include "HardwareSerial.h"
#include "lwqrcode.h"

// OTA
#include <WiFi.h>
#include <DNSServer.h>

#include "sds011.h"
#include "sps30.h"
#include "dnms.h"
#include "types.h"

#include "printf.h"
#include "hexutil.h"
#include "editline.h"
#include "cmdproc.h"
#include "aggregator.h"
#include "item.h"

#include "soundsensor.h"
#include "measurement.h"

#include "SPIFFS.h"

#include "webserver_html.h"

// This EUI must be in BIG-ENDIAN format, most-significant byte (MSB).
// For TTN issued EUIs the first bytes should be 0x70, 0xB3, 0xD5.
//static const uint8_t APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA0, 0x9B };

// voor bas
static const uint8_t APPEUI[8] = { 0x58, 0xEE, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format as well, see above.
// static const uint8_t APPKEY[] = {
//     0xAA, 0x9F, 0x12, 0x45, 0x7F, 0x06, 0x64, 0xDF, 0x4C, 0x1E, 0x9F, 0xC9, 0x5E, 0xDA, 0x1A, 0x8A
// };

// voor bas
static const uint8_t APPKEY[] = {
0x83, 0x87, 0x80, 0xE4, 0x99, 0x4C, 0xEE, 0x7F, 0xF6, 0xB9, 0xD3, 0xDA, 0x4B, 0x88, 0xC8, 0xC9
};

#define DEFAULT_WIFI_PASSWORD   "adminsensor"

#if defined(heltec_wifi_lora_32_V2)
    #define OLED_I2C_ADDR 0x3C

    #define PIN_OLED_RESET  16
    #define PIN_OLED_SDA    4
    #define PIN_OLED_SCL    15
    #define PIN_BUTTON      0
    #define PIN_SDS_RX      22 // PM sensor
    #define PIN_SDS_TX      23 // PM sensor
    #define PIN_VEXT        21

    #define PIN_MEMS_MIC_BCK 13
    #define PIN_MEMS_MIC_WS  12
    #define PIN_MEMS_MIC_DATA -1
    #define PIN_MEMS_MIC_IN 35

    #define PIN_UART_RECORDER 17 // Pin which can be used to output value to serial port
#endif

#if defined(ARDUINO_TTGO_LoRa32_V1)
    #define LED_BUILTIN     25

    #define OLED_I2C_ADDR 0x3C

    #define PIN_OLED_RESET  NOT_A_PIN
    #define PIN_OLED_SDA    21
    #define PIN_OLED_SCL    22
    #define PIN_BUTTON      NOT_A_PIN // TTgo has no button
    #define PIN_SDS_RX      12 // PM sensor
    #define PIN_SDS_TX      13 // PM sensor
    #define PIN_VEXT        NOT_A_PIN // TTgo has no Vext

    #define PIN_MEMS_MIC_BCK 13
    #define PIN_MEMS_MIC_WS  12
    #define PIN_MEMS_MIC_DATA -1
    #define PIN_MEMS_MIC_IN 35

    #define PIN_UART_RECORDER 34 // Check which pin is available
#endif

#define UG_PER_M3       "\u00B5g/m\u00B3"
#define DBA "dBA"

// duration of warmup (seconds)
#define TIME_WARMUP     20    
// duration of measurement (seconds)
#define TIME_MEASURE    10
// time to show version info
#define TIME_VERSION    5
// reboot interval (seconds)
#define REBOOT_INTERVAL 2592000UL
// wifi Access Point period (seconds)
#define WIFIAP_PERIOD   300
// time to keep display on (ms)
#define TIME_OLED_ENABLED   10000UL

// how we know the non-volatile storage contains meaningful data
#define NVDATA_MAGIC    "magic2"

// structure of non-volatile data
typedef struct {
    uint8_t deveui[8];
    uint8_t appeui[8];
    uint8_t appkey[16];
    char wifipass[64];
    char magic[8];
    uint8_t sensortype[1]; // Undefine, PM or DNMS
    uint8_t dnmssensor[1]; // dnmssensor_t
    uint8_t mic_offset[4]; // sign, units, tenth, hundredth
} nvdata_t;

typedef enum {
    E_DISPLAYMODE_HWINFO,
    E_DISPLAYMODE_MEASUREMENTS,
    E_DISPLAYMODE_QRCODE,
    E_DISPLAYMODE_OFF
} displaymode_t;

// data structures related to information shown on screen
typedef struct {
    bool enabled;
    bool update;
    char loraDevEui[32];
    char loraStatus[32];
    displaymode_t displaymode = E_DISPLAYMODE_HWINFO;
    char pmsensor_name[32];
    char rhsensor_name[32];
    char dnmssensor_name[32];
} screen_t;

// main state machine
typedef enum {
    E_INIT = 0,
    E_IDLE,
    E_WARMUP,
    E_MEASURE,
    E_SEND,
    E_LAST
} fsm_state_t;

// Pin mapping
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_ThisBoard();

// each measurement cycle takes 5 minutes, this table specifies how many cycles there are per transmission
static const int interval_table[] = {
    1,  // SF6
    1,  // SF7
    1,  // SF8
    2,  // SF9
    4,  // SF10
    8,  // SF11
    16  // SF12
};

static DNSServer dnsServer;
static char board_name[32];
static fsm_state_t main_state;
static WebServer webServer(80);
static SSD1306 display(OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
static bool has_external_display = false;
static BME280 bme280;
static bool bmeFound = false;
static HardwareSerial serial(1);
static SDS011 sds(&serial);
static SPS30 sps(&serial);
static pmsensor_t pmsensor = E_PMSENSOR_NONE;
static screen_t screen;
static unsigned long button_last_pressed = 0;
static nvdata_t nvdata;
static char cmdline[100];
static rps_t last_tx_rps = 0;
static bool has_joined_otaa = false;
static DNMS dnms(&serial);
static Aggregator aggregator(E_ITEM_MAX);
static float MIC_OFFSET = 0.0;
static bool wifiAPActive;
static unsigned long wifiapStartSecond;

static bool calibration_mode = false;
HardwareSerial Recorder_Serial(2); // Use port Serial2

// total measurement cycle time (seconds)
// For sensor.community measurements must be sent at least each 5 mins.
// Due to the FUP of LORAWAN, 4 minutes interval is choosen.
static int Time_Cycle = 240;

// Task 1 is the default ESP core 1, this one handles the LoRa TTN messages
// Task 0 is the added ESP core 0, this one handles the audio, (read MEMS, FFT process and compose message)
TaskHandle_t Task0;

// task semaphores
volatile bool audioRequest = false;
volatile bool audioReady = false;

// create soundsensor
static SoundSensor soundSensor;

// Create webcontent to host
static webcontent mypage;

//Task0code: handle sound measurements
void Task0code( void * pvParameters ){
  Serial.print("Task0 running on core ");
  Serial.println(xPortGetCoreID());

  // Weighting lists
  static float aweighting[] = A_WEIGHTING;

  // measurement buffers
  static Measurement aMeasurement(aweighting);

  MIC_OFFSET = get_Mic_Offset(nvdata.mic_offset);
  printf("Starting soundsensor with microphone offset %f \n",MIC_OFFSET);
  soundSensor.offset( MIC_OFFSET);   // set microphone dependent correction in dB
  soundSensor.begin();

  // main loop task 0
  while (true) {
    // read chunk form MEMS and perform FFT, and sum energy in octave bins
    float* energy = soundSensor.readSamples();
    // update
    aMeasurement.update(energy);
    // calculate average and compose message
    if (audioRequest) {
      audioRequest = false;
      aMeasurement.calculate();

      aggregator.add(E_ITEM_LAMIN, aMeasurement.min);
      aggregator.add(E_ITEM_LAEQ, aMeasurement.avg);      
      aggregator.add(E_ITEM_LAMAX, aMeasurement.max);

      if (calibration_mode) {
        printf("LAmin %f \n", aMeasurement.min);
        printf("LAavg %f \n", aMeasurement.avg);
        printf("LAmax %f \n", aMeasurement.max);
        aMeasurement.print();      
        // Output maximum in dBA to serial port for sound recorder.
        char MaxdBA[10];
        sprintf(MaxdBA, "<%.1f>\r\n", aMeasurement.max);
        printf("Sending to soundrecorder %s \n",MaxdBA);
        Recorder_Serial.write(MaxdBA);  
      }
      printf("Next measurement in %d seconds\n", Time_Cycle);

      // reset counters etc.
      aMeasurement.reset();
      //printf("end compose message core=%d\n", xPortGetCoreID());
      audioReady = true;    // signal LoRa worker task that audio report is ready
    }
  }
}

void os_getDevEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = nvdata.deveui[7 - i];
    }
}

void os_getArtEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = nvdata.appeui[7 - i];
    }
}

void os_getDevKey(u1_t * buf)
{
    memcpy(buf, nvdata.appkey, 16);
}

float get_Mic_Offset(u1_t * buf)
{
    float my_Mic_Offset = 0.0;
    my_Mic_Offset = buf[1];
    float decimal = 0.0;
    decimal = buf[2];
    my_Mic_Offset += (decimal / 10);
    float hundreth = 0.0;
    hundreth = buf[3];
    my_Mic_Offset += (hundreth / 100);
    if (buf[0] == 1) {
        my_Mic_Offset = my_Mic_Offset * -1;
    }
    return my_Mic_Offset;
}
// saves settings to EEPROM
static void nvdata_save(void)
{
    strcpy(nvdata.magic, NVDATA_MAGIC);
    EEPROM.put(0, nvdata);
    EEPROM.commit();
}

// restores settings from EEPROM, restoring default if no valid settings were found 
static void nvdata_load(void)
{
    EEPROM.get(0, nvdata);
    if (strcmp(nvdata.magic, NVDATA_MAGIC) != 0) {
        memset(&nvdata, 0, sizeof(nvdata));

        // default OTAA settings
        uint64_t chipid = ESP.getEfuseMac();
        nvdata.deveui[0] = (chipid >> 56) & 0xFF;
        nvdata.deveui[1] = (chipid >> 48) & 0xFF;
        nvdata.deveui[2] = (chipid >> 40) & 0xFF;
        nvdata.deveui[3] = (chipid >> 32) & 0xFF;
        nvdata.deveui[4] = (chipid >> 24) & 0xFF;
        nvdata.deveui[5] = (chipid >> 16) & 0xFF;
        nvdata.deveui[6] = (chipid >> 8) & 0xFF;
        nvdata.deveui[7] = (chipid >> 0) & 0xFF;
        memcpy(&nvdata.appeui, APPEUI, 8);
        memcpy(&nvdata.appkey, APPKEY, 16);
        nvdata.sensortype[0] = (int) E_SENSOR_UNDEFINED;
        // As most will be equiped with a ICS43434, set this as default mic.
        // nvdata.dnmssensor[0] = E_DNMS_NONE;
        set_mic_ics43434();
     
        // default WiFi settings
        strcpy(nvdata.wifipass, DEFAULT_WIFI_PASSWORD);
        nvdata_save();
    }
}

static void set_mic_ics43434()
{
    nvdata.dnmssensor[0] = E_DNMS_ICS43434;
    // MIC_OFFSET = 2.5;
    nvdata.mic_offset[0] = 0; // +
    nvdata.mic_offset[1] = 2; // 2
    nvdata.mic_offset[2] = 5; // 5
    nvdata.mic_offset[3] = 0; // 0
}

static void set_mic_sph0645()
{
    nvdata.dnmssensor[0] = E_DNMS_SPH0645;
    // MIC_OFFSET = -1.6;
    nvdata.mic_offset[0] = 1; // -
    nvdata.mic_offset[1] = 1; // 1
    nvdata.mic_offset[2] = 6; // 6
    nvdata.mic_offset[3] = 0; // 0
}

static void setLoraStatus(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(screen.loraStatus, sizeof(screen.loraStatus), fmt, args);
    va_end(args);
    screen.update = true;
}

const char *event_names[] = { LMIC_EVENT_NAME_TABLE__INIT };

static void onEventCallback(void *user, ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(event_names[ev]);

    switch (ev) {
    case EV_JOINING:
        setLoraStatus("OTAA JOIN...");
        has_joined_otaa = false;
        break;
    case EV_JOINED:
        setLoraStatus("JOIN OK!");
        has_joined_otaa = true;
        // Disable the screen once connected.
        screen.enabled = false;
        break;
    case EV_JOIN_FAILED:
        setLoraStatus("JOIN failed!");
        has_joined_otaa = false;
        break;
    case EV_TXCOMPLETE:
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println("Received ack");
        if (LMIC.dataLen) {
            Serial.println("Received downlink");
            Serial.println(LMIC.dataLen);
            Serial.println(" bytes of payload.");
            int firstbyte = LMIC.frame[LMIC.dataBeg];
            Serial.println(firstbyte);
            if ((LMIC.dataLen == 1) && (firstbyte == 1)) {
                Serial.println("Downlink requested reboot.");    
                ESP.restart();
            }
            if ((LMIC.dataLen == 1) && (firstbyte == 2)) {
                Serial.println("Downlink requested screen ON.");    
                screen.enabled = true;
            }
            if ((LMIC.dataLen == 1) && (firstbyte == 3)) {
                Serial.println("Downlink requested screen OFF.");    
                screen.enabled = false;
            }
            if ((LMIC.dataLen == 1) && (firstbyte == 4)) {
                Serial.println("Downlink requested wifiAP ON.");    
                softAPEnable();
            }
            if ((LMIC.dataLen == 1) && (firstbyte == 5)) {
                Serial.println("Downlink requested wifiAP OFF.");    
                WiFi.softAPdisconnect(true);
            }
        }
        setLoraStatus("%08X-%d", LMIC.devaddr, LMIC.seqnoUp);
        break;
    case EV_TXSTART:
        setLoraStatus("Transmit SF%d", getSf(LMIC.rps) + 6);
        last_tx_rps = LMIC.rps;
        break;
    case EV_RXSTART:
        setLoraStatus("Receive SF%d", getSf(LMIC.rps) + 6);
        break;
    case EV_JOIN_TXCOMPLETE:
        setLoraStatus("JOIN sent");
        break;
    default:
        Serial.print("Unknown event: ");
        Serial.println((unsigned) ev);
        break;
    }
}

static void add_cayenne_u16(uint8_t *buf, int &index, item_t item, int channel, int type, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        intval = constrain(intval, 0, 32767);
        buf[index++] = channel;
        buf[index++] = type;
        buf[index++] = highByte(intval);
        buf[index++] = lowByte(intval);
    }
}

static void add_cayenne_s16(uint8_t *buf, int &index, item_t item, int channel, int type, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        intval = constrain(intval, -32768, 32767);
        buf[index++] = channel;
        buf[index++] = type;
        buf[index++] = highByte(intval);
        buf[index++] = lowByte(intval);
    }
}

static void add_cayenne_8bit(uint8_t *buf, int &index, item_t item, int channel, int type,  double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        buf[index++] = channel;
        buf[index++] = type;
        buf[index++] = lowByte(intval);
    }
}

static void add_be_16bit(uint8_t *buf, int &index, item_t item, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        buf[index++] = (intval >> 8) & 0xFF;
        buf[index++] = (intval >> 0) & 0xFF;
    }
}

static bool send_data(void)
{
    uint8_t buf[32];
    int idx = 0;

    if ((LMIC.opmode & (OP_TXDATA | OP_TXRXPEND)) != 0) {
        return false;
    }

    int port = 1;
    if (nvdata.sensortype[0] == (int) E_SENSOR_PMSENSOR) {
        if (pmsensor == E_PMSENSOR_SPS30) {
            // encode as custom SPS30
            port = 30;
            add_be_16bit(buf, idx, E_ITEM_PM1_0, 0.1);
            add_be_16bit(buf, idx, E_ITEM_PM2_5, 0.1);
            add_be_16bit(buf, idx, E_ITEM_PM4_0, 0.1);
            add_be_16bit(buf, idx, E_ITEM_PM10, 0.1);
            add_be_16bit(buf, idx, E_ITEM_N0_5, 1.0);
            add_be_16bit(buf, idx, E_ITEM_N1_0, 1.0);
            add_be_16bit(buf, idx, E_ITEM_N2_5, 1.0);
            add_be_16bit(buf, idx, E_ITEM_N4_0, 1.0);
            add_be_16bit(buf, idx, E_ITEM_N10, 1.0);
            add_be_16bit(buf, idx, E_ITEM_TPS, 0.001);
        } 
        if (pmsensor == E_PMSENSOR_SDS011) {
            // encode as Cayenne
            printf("Sending PM data to ttn\n");
            //add_cayenne_u16(buf, idx, E_ITEM_PM1_0, 0, 2, 0.01);
            add_cayenne_u16(buf, idx, E_ITEM_PM10, 1, 2, 0.01);
            add_cayenne_u16(buf, idx, E_ITEM_PM2_5, 2, 2, 0.01);
            //add_cayenne_u16(buf, idx, E_ITEM_PM4_0, 4, 2, 0.01);
            add_cayenne_8bit(buf, idx, E_ITEM_HUMIDITY, 10, 104, 0.5);
            add_cayenne_s16(buf, idx, E_ITEM_TEMPERATURE, 11, 103, 0.1);
            add_cayenne_u16(buf, idx, E_ITEM_PRESSURE, 12, 115, 10.0);
        }
    }
    if (nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) {
        if ((nvdata.dnmssensor[0] == (int) E_DNMS_ICS43434) or (nvdata.dnmssensor[0] == (int) E_DNMS_SPH0645)) {
            // encode as Cayenne
            printf("Sending sound data to ttn\n");
            add_cayenne_u16(buf, idx, E_ITEM_LAMIN, 15, 2, 0.01);
            add_cayenne_u16(buf, idx, E_ITEM_LAEQ, 16, 2, 0.01);
            add_cayenne_u16(buf, idx, E_ITEM_LAMAX, 17, 2, 0.01);
            add_cayenne_8bit(buf, idx, E_ITEM_HUMIDITY, 10, 104, 0.5);
            add_cayenne_s16(buf, idx, E_ITEM_TEMPERATURE, 11, 103, 0.1);
            add_cayenne_u16(buf, idx, E_ITEM_PRESSURE, 12, 115, 10.0);
        }
    }
    printf("Sending on port %d, ", port);
    hexprint("data ", buf, idx);
    LMIC_setTxData2(port, buf, idx, 0);
    return true;
}

static void screen_update(unsigned long int second)
{
    char line[16];
    double value;

    if (!screen.enabled) {
        display.displayOff();
        return;
    }
    // do nothing if nothing to do, or already done
    if (!screen.update) {
        return;
    }

    if (calibration_mode) {
        display.displayOn();
        display.clear();
        display.setColor(WHITE);
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, "Calibration Mode");
        display.setFont(ArialMT_Plain_10);
        if (aggregator.get(E_ITEM_LAMIN, value)) {
            snprintf(line, sizeof(line), "LAmin:%2.2f ", value);
            display.drawString(0, 30, String(line) + DBA);
        }
        if (aggregator.get(E_ITEM_LAEQ, value)) {
            snprintf(line, sizeof(line), "LAeq:%2.2f ", value);
            display.drawString(0, 40, String(line) + DBA);
        }
        if (aggregator.get(E_ITEM_LAMAX, value)) {
            snprintf(line, sizeof(line), "LAmax:%2.2f ", value);
            display.drawString(0, 50, String(line) + DBA);
        }
        display.display();
        return;
    }

    switch (screen.displaymode) {
    case E_DISPLAYMODE_HWINFO:
        display.displayOn();
        display.clear();
        display.setColor(WHITE);
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 14, board_name);
        display.setFont(ArialMT_Plain_10);
        switch (nvdata.sensortype[0]) {
            case E_SENSOR_UNDEFINED:
                display.drawString(0, 30, "Unknown sensor");
                break;
            case E_SENSOR_PMSENSOR:
                display.drawString(0, 30, "PM sensor");
                display.drawString(0, 40, screen.pmsensor_name);
                display.drawString(0, 50, screen.rhsensor_name);
                break;
            case E_SENSOR_DNMSSENSOR:
                display.drawString(0, 30, "DNMS sensor");
                display.drawString(0, 40, screen.dnmssensor_name);
                display.drawString(0, 50, screen.rhsensor_name);
                break;
            default:
                break;
        };
        display.display();
        break;
    case E_DISPLAYMODE_MEASUREMENTS:
        display.displayOn();
        display.clear();
        display.setColor(WHITE);
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 14, screen.loraStatus);
        display.setFont(ArialMT_Plain_10);
        if (aggregator.get(E_ITEM_PM10, value)) {
            snprintf(line, sizeof(line), "PM 10:%3d ", (int) round(value));
            display.drawString(0, 30, String(line) + UG_PER_M3);
        }
        if (aggregator.get(E_ITEM_PM2_5, value)) {
            snprintf(line, sizeof(line), "PM2.5:%3d ", (int) round(value));
            display.drawString(0, 40, String(line) + UG_PER_M3);
        }
        if (aggregator.get(E_ITEM_LAEQ, value)) {
            snprintf(line, sizeof(line), "LAeq:%2.2f ", value);
            display.drawString(0, 50, String(line) + DBA);
        }
        display.display();
        break;
    case E_DISPLAYMODE_QRCODE:
        if (has_joined_otaa) {
            display.displayOff();
        } else {
            display.displayOn();
            qrcode_show(&display, nvdata.appeui, nvdata.deveui, nvdata.appkey);
        }
        break;
    case E_DISPLAYMODE_OFF:
        display.displayOff();
        break;
    default:
        break;
    }
    screen.update = false;
}

static void set_display_mode(displaymode_t new_mode)
{
    screen.displaymode = new_mode;
    screen.update = true;
}

static void set_fsm_state(fsm_state_t newstate)
{
    if (newstate < E_LAST) {
        static const char* states[] = { "E_INIT", "E_IDLE", "E_WARMUP", "E_MEASURE", "E_SEND" };
        printf(">>> %s\n", states[newstate]);
    }
    main_state = newstate;
}

static bool pmsensor_on_off(boolean on)
{
    switch (pmsensor) {
    case E_PMSENSOR_SDS011:
        return sds.fan(on);
    case E_PMSENSOR_SPS30:
        return on ? sps.start() : sps.stop();
    default:
        break;
    }
    return false;
}

// return true if new measurement available
static bool dnmssensor_measure(void)
{
    unsigned long ms = millis();
    unsigned long start = ms / 1000UL;
    unsigned long now = ms / 1000UL;
    unsigned long duration = 0;

    audioReady = false;
    audioRequest = true; // Set semaphores for task running ESP core 0 to calculate sound measurement
    printf("audio request from core=%d\n", xPortGetCoreID());
    while(!audioReady && duration <= 2) {
        for (int i =1; i <= 1000; i++) {
            asm("nop");
        };
        ms = millis();
        now = ms / 1000UL;
        duration = now - start;
    };
    if (audioReady) {
        return true;
    } else {
        return false;
    }
}

// return true if new measurement available
static bool pmsensor_measure(void)
{
    sds_meas_t sds_meas;
    sps_meas_t sps_meas;

    switch (pmsensor) {
    case E_PMSENSOR_SDS011:
        if (sds.poll(&sds_meas)) {
            aggregator.add(E_ITEM_PM2_5, sds_meas.pm2_5);
            aggregator.add(E_ITEM_PM10, sds_meas.pm10);
            return true;
        }
        break;
    case E_PMSENSOR_SPS30:
        if (sps.read_measurement(&sps_meas)) {
            aggregator.add(E_ITEM_PM1_0, sps_meas.pm1_0);
            aggregator.add(E_ITEM_PM2_5, sps_meas.pm2_5);
            aggregator.add(E_ITEM_PM4_0, sps_meas.pm4_0);
            aggregator.add(E_ITEM_PM10, sps_meas.pm10);
            aggregator.add(E_ITEM_N0_5, sps_meas.n0_5);
            aggregator.add(E_ITEM_N1_0, sps_meas.n1_0);
            aggregator.add(E_ITEM_N2_5, sps_meas.n2_5);
            aggregator.add(E_ITEM_N4_0, sps_meas.n4_0);
            aggregator.add(E_ITEM_N10, sps_meas.n10);
            aggregator.add(E_ITEM_TPS, sps_meas.tps);
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}

static boolean detect_SPS303(void) {
    // detect SPS30
    printf("Detecting SPS30 sensor ...\n");
    serial.begin(115200, SERIAL_8N1, PIN_SDS_RX, PIN_SDS_TX);
    if (sps.wakeup()) {
        printf("Found SPS30\n");
        return true;
    }
    else {
        return false;
    }
}

static boolean detect_SDS011(void) {
    printf("Detecting SDS011 sensor ...\n");
    serial.begin(9600, SERIAL_8N1, PIN_SDS_RX, PIN_SDS_TX);
    if (sds.fan(true) || sds.fan(true)) {
        printf("Found SDS011\n");
        return true;
    }
    else {
        printf("Not found SDS011\n");
        return false;
    }        
}

static void fsm_run(unsigned long int seconds)
{
    static int cycle = 0;
    boolean doSend = false;
    unsigned long int sec = seconds % Time_Cycle;

    switch (main_state) {
    case E_INIT:
        if (nvdata.sensortype[0] == (int) E_SENSOR_PMSENSOR) {
            if (pmsensor == E_PMSENSOR_NONE) {
                if (detect_SPS303()) {
                    pmsensor = E_PMSENSOR_SPS30;
                    snprintf(screen.pmsensor_name, sizeof(screen.pmsensor_name), "SPS30");
                    break;
                }
                // detect SDS011
                if (detect_SDS011()) {
                    char serial[16], date[16];
                    if (sds.version(serial, date)) {
                        printf("SDS011: %s, %s\n", serial, date);
                    }
                    pmsensor = E_PMSENSOR_SDS011;
                    snprintf(screen.pmsensor_name, sizeof(screen.pmsensor_name), "SDS011: %s", serial);
                    break;
                }
            } 
        }
        if (nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) {
            switch (nvdata.dnmssensor[0]) {
                case E_DNMS_NONE:
                    snprintf(screen.dnmssensor_name, sizeof(screen.dnmssensor_name), "Unknown mic");
                    break;
                case E_DNMS_ICS43434:
                    snprintf(screen.dnmssensor_name, sizeof(screen.dnmssensor_name), "ICS43434");
                    break;
                case E_DNMS_SPH0645:
                    snprintf(screen.dnmssensor_name, sizeof(screen.dnmssensor_name), "SPH0645");
                    break;
            };
        }
        if (nvdata.sensortype[0] == (int) E_SENSOR_UNDEFINED) {
            // There must be an PM sensor OR an DNMS sensor available
        }
        else {
            set_display_mode(E_DISPLAYMODE_HWINFO);
            set_fsm_state(E_IDLE);
        }
        break;

    case E_IDLE:
        if (calibration_mode) {
            if (sec == (Time_Cycle - 1)) {
                delay(1010); // sleep 1 second
                aggregator.reset();
                set_fsm_state(E_MEASURE);
                break;
            }
        } else {
            if (sec < TIME_WARMUP) {
                // turn fan on
                pmsensor_on_off(true);
                aggregator.reset();
                set_fsm_state(E_WARMUP);
            }
        }
        break;

    case E_WARMUP:
        if (sec > TIME_WARMUP) {
            // reset sum
            aggregator.reset();
            set_display_mode(has_joined_otaa ? E_DISPLAYMODE_MEASUREMENTS : E_DISPLAYMODE_QRCODE);
            set_fsm_state(E_MEASURE);
        }
        break;

    case E_MEASURE:
        // take temperature/humidity sample
        if (bmeFound) {
            aggregator.add(E_ITEM_TEMPERATURE, bme280.readTempC());    
            aggregator.add(E_ITEM_HUMIDITY, bme280.readFloatHumidity());
            aggregator.add(E_ITEM_PRESSURE, bme280.readFloatPressure());
        }
        if (nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) {
            printf("start dnms measurement reading");
            if (dnmssensor_measure()) {
                printf("dnms measurement ready");
                screen.update = true;
            } else {
                printf("dnms measurement NOT ready");
            }
            set_fsm_state(E_SEND);
        } else {
            if (sec < (TIME_WARMUP + TIME_MEASURE)) {
                if (pmsensor_measure()) {
                    screen.update = true;
                }
            } else {
                // turn the fan off
                pmsensor_on_off(false);
                set_fsm_state(E_SEND);
            }
        }
        break;
        
    case E_SEND:
        if (nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) {
            if (calibration_mode) {
                printf("Calibration mode active, no data sent to LORA");
                set_fsm_state(E_IDLE);
            } else {
                doSend = true;
            }
        } else {
            if (sec >= (TIME_WARMUP + TIME_MEASURE)) {
                doSend = true;
            }
        }        
        if (doSend) {
            // send data with an interval depending on the current spreading factor
            int sf = getSf(last_tx_rps);
            int interval = interval_table[sf];
            printf("SF %d, cycle %d / interval %d\n", sf + 6, cycle, interval);
            if ((cycle % interval) == 0) {
                if (send_data()) {
                    printf("send_data OK");
                } else {
                    printf("send_data NOK");
                }
            }
            cycle++;
            set_fsm_state(E_IDLE);
        }
        break;

    default:
        set_fsm_state(E_INIT);
        break;
    }

    // when measuring, light the LED
    digitalWrite(LED_BUILTIN, main_state == E_MEASURE);
}

static bool findBME280(char *version)
{
    bme280.setI2CAddress(0x76);
    if (bme280.beginI2C()) {
        strcpy(version, "0x76");
        return true;
    }
    bme280.setI2CAddress(0x77);
    if (bme280.beginI2C()) {
        strcpy(version, "0x77");
        return true;
    }
    return false;
}

static bool findDisplay(TwoWire *wire, int pinSda, int pinScl, uint8_t address)
{
    wire->begin(pinSda, pinScl);
    wire->beginTransmission(address);
    return (wire->endTransmission() == 0);
}

static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        printf("%10s: %s\n", cmd->name, cmd->help);
    }
}

static int do_help(int argc, char *argv[]);

static int do_reboot(int argc, char *argv[])
{
    ESP.restart();
    return CMD_OK;
}

static int do_mic(int argc, char *argv[])
{
    // reset mic
    if ((argc == 2) && (strcmp(argv[1], "reset") == 0)) {
        printf("Reset microphone to unknown\n");
        nvdata.dnmssensor[0] = E_DNMS_NONE;
        nvdata_save();
        set_fsm_state(E_INIT);
        return CMD_OK;
    }

    if ((argc == 2) && (strcmp(argv[1], "SPH0645") == 0)) {
        printf("Set microphone to SPH0645\n");
        set_mic_sph0645();
        nvdata_save();
        set_fsm_state(E_INIT);
        return CMD_OK;
    }

    if ((argc == 2) && (strcmp(argv[1], "ICS43434") == 0)) {
        printf("Set microphone to ICS43434\n");
        set_mic_ics43434();
        nvdata_save();
        set_fsm_state(E_INIT);
        return CMD_OK;
    }
    return CMD_ARG;
}

static int do_type(int argc, char *argv[])
{
    // reset type
    if ((argc == 2) && (strcmp(argv[1], "reset") == 0)) {
        printf("Reset sensor type\n");
        nvdata.sensortype[0] = E_SENSOR_UNDEFINED;
        nvdata.dnmssensor[0] = E_DNMS_NONE;
        nvdata_save();
        return CMD_OK;
    }

    if ((argc == 2) && (strcmp(argv[1], "PM") == 0)) {
        printf("Set sensor to be a PM sensor\n");
        nvdata.sensortype[0] = E_SENSOR_PMSENSOR;
        nvdata.dnmssensor[0] = E_DNMS_NONE;
        nvdata_save();
        return CMD_OK;
    }

    if ((argc == 2) && (strcmp(argv[1], "DNMS") == 0)) {
        printf("Set sensor to be a DNMS sensor\n");
        nvdata.sensortype[0] = E_SENSOR_DNMSSENSOR;
        nvdata_save();
        return CMD_OK;
    }
    return CMD_ARG;
}

static int do_screen(int argc, char *argv[])
{
    if ((argc == 2) && (strcmp(argv[1], "ON") == 0)) {
        printf("Switch display of sensor ON\n");
        screen.enabled = true;
        return CMD_OK;
    }if ((argc == 2) && (strcmp(argv[1], "OFF") == 0)) {
        printf("Switch display of sensor OFF\n");
        screen.enabled = false;
        return CMD_OK;
    }
    return CMD_ARG;
}

static int do_wifiAP(int argc, char *argv[])
{
    if ((argc == 2) && (strcmp(argv[1], "ON") == 0)) {
        printf("Switch wifi Access point ON\n");
        softAPEnable();
        return CMD_OK;
    }if ((argc == 2) && (strcmp(argv[1], "OFF") == 0)) {
        printf("Switch wifi Access point OFF\n");
        WiFi.softAPdisconnect(true);
        wifiAPActive = false;
        return CMD_OK;
    }
    return CMD_ARG;
}

static int do_calibrate(int argc, char *argv[])
{
    if ((argc == 2) && (strcmp(argv[1], "ON") == 0)) {
        printf("Switch calibration mode ON\n");
        calibration_mode = true;
        if (calibration_mode) {
            // Cycle time must be 2 second for quick response.
            Time_Cycle = 2;
            // Config serial port for output to sound recorder.
            Recorder_Serial.begin(9600, SERIAL_8N1, -1, PIN_UART_RECORDER); // Only transmit
        }
        // Make sure screen is on in calibration mode.
        screen.enabled = true;
        return CMD_OK;
    }if ((argc == 2) && (strcmp(argv[1], "OFF") == 0)) {
        printf("Switch calibration mode OFF, do a restart.\n");
        ESP.restart();
        return CMD_OK;
    }
    return CMD_ARG;
}

static int do_mic_offset(int argc, char *argv[])
{
    printf("Current microphone offset is %f\n",get_Mic_Offset(nvdata.mic_offset));
    if (argc == 2) {
        float new_offset = atof(argv[1]);
        if ((new_offset >= 10) || (new_offset <= -10)) {
            printf("Only from -10 to 10dB correction is allowed\n");
            return CMD_ARG;
        } else {
            printf("Set microphone offset\n");
            if (new_offset < 0) {
                nvdata.mic_offset[0] = 1; // +
            } else {
                nvdata.mic_offset[0] = 0; // -
            }
            new_offset= abs(new_offset);
            nvdata.mic_offset[1] = int(new_offset);
            new_offset -= int(new_offset);
            new_offset = new_offset * 10;
            nvdata.mic_offset[2] = int(new_offset);
            new_offset -= int(new_offset);
            new_offset = new_offset * 10;
            nvdata.mic_offset[3] = int(new_offset);
            printf("New microphone offset is %f\n",get_Mic_Offset(nvdata.mic_offset));
            nvdata_save();
            printf("Device will reboot");
            delay(1000); // sleep 1 second
            ESP.restart();
            return CMD_OK;            
        }
    } else {
        printf("Not a number as offset value\n");
        return CMD_ARG;
    }
}

static int do_otaa(int argc, char *argv[])
{
    // reset OTAA
    if ((argc == 2) && (strcmp(argv[1], "reset") == 0)) {
        printf("Resetting OTAA to defaults\n");
        memset(&nvdata, 0, sizeof(nvdata));
        EEPROM.put(0, nvdata);
        nvdata_load();
    }

    // save OTAA parameters
    if (argc == 4) {
        printf("Setting OTAA parameters\n");
        char *deveui_hex = argv[1];
        char *appeui_hex = argv[2];
        char *appkey_hex = argv[3];
        if ((strlen(deveui_hex) != 16) || (strlen(appeui_hex) != 16) || (strlen(appkey_hex) != 32)) {
            return CMD_ARG;
        }

        hexparse(deveui_hex, nvdata.deveui, 8);
        hexparse(appeui_hex, nvdata.appeui, 8);
        hexparse(appkey_hex, nvdata.appkey, 16);
        nvdata_save();
    }

    // show current OTAA parameters
    hexprint("Dev EUI: ", nvdata.deveui, 8);
    hexprint("App EUI: ", nvdata.appeui, 8);
    hexprint("App key: ", nvdata.appkey, 16);
    return CMD_OK;
}

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "reboot", do_reboot, "Reboot ESP" },
    { "otaa", do_otaa, "[reset|<[deveui] [appeui] [appkey]>] Query/reset/set OTAA parameters" },
    { "mic", do_mic, "[reset|<SPH0645>|<ICS43434>] Reset SPH0645 ICS43434"},
    { "type", do_type, "[reset|<PM>|<DNMS>] Reset PM DNMS"},
    { "mic_offset", do_mic_offset, "Set microphone offset in dB"},
    { "screen", do_screen, "[ON|OFF] Set screen on or off"},
    { "wifiAP", do_wifiAP, "[ON|OFF] Set wifi Access Point on or off"},
    { "calibrate", do_calibrate, "[ON|OFF] Set calibration mode on or off"},
    { NULL, NULL, NULL }
};

static int do_help(int argc, char *argv[])
{
    show_help(commands);
    return CMD_OK;
}

void softAPEnable()
{
    uint64_t chipid = ESP.getEfuseMac();
    char ssid[32];
    sprintf(ssid, "%s-%04X%08X", board_name, (uint32_t)(chipid >> 32), (uint32_t)chipid);
    printf("Starting AP on 192.168.4.1 with SSID '%s', pass '%s'\n", ssid, nvdata.wifipass); 
    WiFi.softAP(ssid, nvdata.wifipass);
    wifiAPActive = true;
    unsigned long ms = millis();
    unsigned long second = ms / 1000UL;    
    wifiapStartSecond = second;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Starting...");
    webcontent mypage = webcontent();

    if(!SPIFFS.begin(true)){
        printf("An Error has occurred while mounting SPIFFS");
    }
    // Let LMIC compensate for +/- 10% clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

    EditInit(cmdline, sizeof(cmdline));

    // VEXT config: 0 = enable Vext
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, 0);

    // LED config
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

    // button config
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // detect the display
    pinMode(PIN_OLED_RESET, OUTPUT);
    digitalWrite(PIN_OLED_RESET, LOW);
    has_external_display = findDisplay(&Wire, PIN_OLED_SDA, PIN_OLED_SCL, OLED_I2C_ADDR);
    printf("Found external display: %s\n", has_external_display ? "yes" : "no");
    if (!has_external_display) {
        // no external display found, use the internal one
        digitalWrite(PIN_OLED_RESET, HIGH);
        delay(100);
    }

    // init the display
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    if (!has_external_display) {
        // reduce brightness of internal display to mitigate burn-in
        display.setBrightness(127);
    }
    screen.enabled = true;

    // restore setting (LoRaWAN keys, etc) from EEPROM, or use a default
    EEPROM.begin(sizeof(nvdata));
    nvdata_load();
    printf("nvdata loaded ...\n");
    if (nvdata.sensortype[0] == (int) E_SENSOR_UNDEFINED) {
        printf("Sensor type is unknown, searching for sensors ...\n");
        if (detect_SPS303() or detect_SDS011()) {
            // A pm sensor is detected
            nvdata.sensortype[0] = (int) E_SENSOR_PMSENSOR;
        } else {
            printf("No PM sensor detected, activating DNMS ...\n");
            // No PM sensor, so use dnms
            nvdata.sensortype[0] = (int) E_SENSOR_DNMSSENSOR;
        }
        nvdata_save();
    } else {
        printf("Sensor type is known, ");
        if (nvdata.sensortype[0] == (int) E_SENSOR_PMSENSOR) {
            printf("activating PM sensor ...\n");
        }
        if (nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) {
            printf("activating DNMS sensor ...\n");
        }
    }

    if (nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) {
        if (nvdata.dnmssensor[0] == (int) E_DNMS_NONE) {
            printf("No microphone type known for DNMS sensor ...\n");
        }
    }

    if ((nvdata.sensortype[0] == (int) E_SENSOR_DNMSSENSOR) && ((nvdata.dnmssensor[0] != (int) E_DNMS_NONE))) {
// Only create task for DNMS sensor with a known microphone.
//create a task that will be executed in the Task0code() function, with priority 1 and executed on core 0
        xTaskCreatePinnedToCore(
                        Task0code,   /* Task function. */
                        "Task0",     /* name of task. */
                        40000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        &Task0,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */                  
    }

    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", nvdata.deveui[0], nvdata.deveui[1], nvdata.deveui[2],
             nvdata.deveui[3], nvdata.deveui[4], nvdata.deveui[5], nvdata.deveui[6], nvdata.deveui[7]);

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_registerEventCb(onEventCallback, NULL);
    LMIC_startJoining();

    // detect hardware
    #if defined(ARDUINO_TTGO_LoRa32_V1)
        snprintf(board_name, sizeof(board_name), "ESP32-%s", "TTGO");
    #endif
    #if defined(heltec_wifi_lora_32_V2)
        snprintf(board_name, sizeof(board_name), "ESP32-%s", "Heltec");
    #endif
    printf("Detecting BME280 ...\n");
    strcpy(screen.pmsensor_name, "");
    strcpy(screen.dnmssensor_name,"");
    strcpy(screen.rhsensor_name, "");
    char bmeVersion[8];
    bmeFound = findBME280(bmeVersion);
    if (bmeFound) {
        printf("Found BME280, i2c=%s\n", bmeVersion);
        snprintf(screen.rhsensor_name, sizeof(screen.rhsensor_name), "BME280: %s", bmeVersion);
    }

    // OTA init
    softAPEnable();

    // Set software version of webpage
    mypage.setSWVersion(FIRMWARE_VERSION);
    // Set sensor type of webpage
    mypage.setSensorType((sensor_t) nvdata.sensortype[0]);
    // Set microphone type of webpage
    mypage.setDNMSSensor((dnmssensor_t) nvdata.dnmssensor[0]);
    // Set microphone offset
    mypage.setDNMSMicOffset(get_Mic_Offset(nvdata.mic_offset));
    // Set LORA Information
    char loraAppEui[40];
    snprintf(loraAppEui, sizeof(loraAppEui),
             "%02X %02X %02X %02X %02X %02X %02X %02X",
             nvdata.appeui[0], nvdata.appeui[1], nvdata.appeui[2],nvdata.appeui[3],
             nvdata.appeui[4], nvdata.appeui[5], nvdata.appeui[6], nvdata.appeui[7]);
    char loraAppKey[80];             
    snprintf(loraAppKey, sizeof(loraAppKey),
             "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             nvdata.appkey[0], nvdata.appkey[1], nvdata.appkey[2], nvdata.appkey[3],
             nvdata.appkey[4], nvdata.appkey[5], nvdata.appkey[6], nvdata.appkey[7],
             nvdata.appkey[8], nvdata.appkey[9], nvdata.appkey[10], nvdata.appkey[11],
             nvdata.appkey[12], nvdata.appkey[13], nvdata.appkey[14], nvdata.appkey[15]);
    mypage.setLORAInfo(loraAppEui,loraAppKey, screen.loraDevEui);

    webServer.on("/",handle_Home);
    webServer.on("/setup",handle_Setup);
    webServer.on("/setupreset",handle_SetupReset);
    webServer.on("/save_ICS43434", handle_OnSave_ICS43434);
    webServer.on("/save_SPH0645", handle_OnSave_SPH0645);
    webServer.on("/reset", handle_Reset);
    webServer.on("/NoiceOrDust.png", handle_Logo);
    ElegantOTA.begin(&webServer);
    webServer.begin();
    dnsServer.start(53, "*", WiFi.softAPIP());
}

void handle_Home() {
    webServer.send(200,"text/html", mypage.getHomePage());
}

void handle_Setup() {
    webServer.send(200,"text/html", mypage.makeSetupPage());
}

void handle_SetupReset() {
    webServer.send(200,"text/html", mypage.makeSetupReset());
}
void handle_OnSave_ICS43434() {
  printf("we have a connection for mic ICS43434 on dnms site ...\n");
  set_mic_ics43434();
  nvdata_save();
  webServer.send(200, "text/html", "Settings saved"); 
  printf("Reboot ...\n");
  ESP.restart();
}

void handle_OnSave_SPH0645() {
  printf("we have a connection for mic SPH0645 on dnms site ...\n");
  set_mic_sph0645();
  nvdata_save();
  webServer.send(200, "text/html", "Settings saved"); 
  printf("Reboot ...\n");
  ESP.restart();
}

void handle_Reset() {
  printf("we have received a reset ...\n");
  nvdata.sensortype[0] = E_SENSOR_UNDEFINED;
  nvdata_save();
  webServer.send(200, "text/html", "Reset done"); 
  printf("Reboot ...\n");
  ESP.restart();
}

void handle_Logo() {
    File file = SPIFFS.open("/NoiceOrDust.png", "r");
    webServer.streamFile(file,"image/png");
}

void loop(void)
{
    unsigned long ms = millis();
    unsigned long second = ms / 1000UL;

    // parse command line
    while (Serial.available()) {
        char c;
        bool haveLine = EditLine(Serial.read(), &c);
        Serial.write(c);
        if (haveLine) {
            int result = cmd_process(commands, cmdline);
            switch (result) {
            case CMD_OK:
                printf("OK\n");
                break;
            case CMD_NO_CMD:
                break;
            case CMD_ARG:
                printf("Invalid arguments\n");
                break;
            case CMD_UNKNOWN:
                printf("Unknown command, available commands:\n");
                show_help(commands);
                break;
            default:
                printf("%d\n", result);
                break;
            }
            printf(">");
        }
    }

    // button press cycles through display states
    int timebetweenclick = 500;
    if (PIN_BUTTON == NOT_A_PIN) {
        timebetweenclick = 5000;
    }
    if ((ms - button_last_pressed) > timebetweenclick) {
        if (digitalRead(PIN_BUTTON) == 0) {
            button_last_pressed = ms;
            // cycle display mode
            switch (screen.displaymode) {
            case E_DISPLAYMODE_HWINFO:
                set_display_mode(E_DISPLAYMODE_MEASUREMENTS);
                break;
            case E_DISPLAYMODE_MEASUREMENTS:
                set_display_mode(E_DISPLAYMODE_QRCODE);
                break;
            case E_DISPLAYMODE_QRCODE:
                set_display_mode(E_DISPLAYMODE_OFF);
                break;
            case E_DISPLAYMODE_OFF:
                set_display_mode(E_DISPLAYMODE_HWINFO);
                break;
            }
        }
    }
    if ((ms - button_last_pressed) > TIME_OLED_ENABLED) {
        set_display_mode(E_DISPLAYMODE_OFF);
    }

    // run the measurement state machine
    fsm_run(second);

    // update screen
    screen_update(second);

    // run LoRa process
    os_runloop_once();

    // reboot every 30 days
    if (second > REBOOT_INTERVAL) {
        printf("Reboot ...\n");
        ESP.restart();
        while (true);
    }

    // Stop wifi na 5 minuten
    if ((second - wifiapStartSecond) > WIFIAP_PERIOD) {
        if (wifiAPActive) {
            printf("Disabling wifi AP ...\n");
            WiFi.softAPdisconnect(true);
            wifiAPActive = false;
        }
    }

    // run the OTA process
    webServer.handleClient();
    dnsServer.processNextRequest();
}

