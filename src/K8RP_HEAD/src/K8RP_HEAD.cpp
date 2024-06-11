#include <Arduino.h>
#include <FastLED.h>
#include <TFMPI2C.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

#include "RunningAverage.h"

////////////////////////////////////////////////////////////////////////////////////
/*                                     Battery                                    */
////////////////////////////////////////////////////////////////////////////////////
#define BAT_PIN 36
#define VIN_VOLTAGE 3.3
#define R1 10000.0
#define R2 10000.0
#define BATTV_MAX 4.2  // maximum battery voltage
#define BATTV_MIN 3.2  // empty battery

RunningAverage RA(1000);
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
/*                                      LIDAR                                     */
////////////////////////////////////////////////////////////////////////////////////
TFMPI2C tfmp;
int16_t dist = 0;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
/*                                     ESP-NOW                                    */
////////////////////////////////////////////////////////////////////////////////////
esp_now_peer_info_t peerInfo;
uint8_t esp2_mac_addr[] = {0xA0, 0xB7, 0x65, 0x4E, 0xFC, 0xC8};  // ESP2 MAC address

struct ESPNOW_MSG_SEND {
    int16_t distance;
};
ESPNOW_MSG_SEND sendEspnowData;

struct ESPNOW_MSG_RCV {
    bool play;
    bool showHeadBat;
    bool redMode;
};
ESPNOW_MSG_RCV recEspnowData;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
/*                                       LED                                      */
////////////////////////////////////////////////////////////////////////////////////
#define LED_DATA_PIN 13
#define NUM_LEDS_TOTAL 9
#define NUM_LEDS_EYE 6

CRGB leds[NUM_LEDS_TOTAL];

#define COLOR_BLUE CRGB::Blue
#define COLOR_YELLOW CRGB::Yellow
#define COLOR_GREEN CRGB::Green
#define COLOR_PURPLE CRGB::Purple
#define COLOR_RED CRGB::Red
#define COLOR_OFF CRGB::Black

#define COLOR_RED_GRB CRGB::Green
#define COLOR_GREEN_GRB CRGB::Red
#define COLOR_YELLOW_GRB CRGB::Yellow
#define COLOR_BLUE_GRB CRGB::Blue

#define MIN_BLINK_PAUSE_TIME 40000
#define MAX_BLINK_PAUSE_TIME 60000
#define MIN_BLINK_COUNT 60
#define MAX_BLINK_COUNT 200

#define MIN_LED_BLINK_ON_TIME 100
#define MAX_LED_BLINK_ON_TIME 300
#define MIN_LED_BLINK_OFF_TIME 100
#define MAX_LED_BLINK_OFF_TIME 300

static int ledIndex1 = 0;  // LED 0
static int ledIndex2 = 1;  // LED 1
static int ledIndex3 = 2;  // LED 2
static int ledIndex4 = 3;  // LED 3, 4, 5, 6, 7, 8

static unsigned long lastBlinkOnTime = 0;
static unsigned long lastBlinkOffTime = 0;
static unsigned long lastFullCycleTime = 0;
static unsigned long eyeAnimationDelay = 60;
static unsigned long eyeLedForwardLastOn = 0;
static unsigned long eyeLedBackwardLastOn = 0;
static int randomEyeAnimation = 0;
static int blinkCount = 0;
static int blinkPauseTime = 0;
static int blinkCountTotal = 0;
static int blinkOnTime = 0;
static int blinkOffTime = 0;
bool eyeGoBack = false;
bool blinking = true;
bool saveAnimation = false;
unsigned long lastBatteryShowTime = 0;
////////////////////////////////////////////////////////////////////////////////////

void initFastLed();
void initTFMini();
void initESPNOW();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void saveAnimationBegin();
void saveAnimationEnd();
void led1Blink();
void eyeRunningDot();
void eyeStaticColor();
void eyeMirror();
void displayBatteryLevel();

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_MODE_STA);
    initESPNOW();
    initTFMini();
    initFastLed();
}

void initFastLed() {
    FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS_TOTAL);
    FastLED.setBrightness(80);
    FastLED.clearData();
}

void initTFMini() {
    tfmp.recoverI2CBus();
    tfmp.sendCommand(SET_FRAME_RATE, FRAME_500);
}

void initESPNOW() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    } else {
        Serial.println("ESP-NOW INIT OK");
    }

    // register peer
    memcpy(peerInfo.peer_addr, esp2_mac_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&recEspnowData, incomingData, sizeof(recEspnowData));
}

void loop() {
    saveAnimation = recEspnowData.redMode;

    tfmp.getData(dist);

    if (tfmp.status == TFMP_READY) {
        sendEspnowData.distance = dist;
    }

    // ESP-NOW - send message to ESP2
    esp_err_t result = esp_now_send(esp2_mac_addr, (const uint8_t *)&sendEspnowData, sizeof(sendEspnowData));

    if (!saveAnimation) {
        if (recEspnowData.play == true) {
            if (randomEyeAnimation == 0) {
                randomEyeAnimation = random(1, 3);
            }
        } else {
            randomEyeAnimation = 0;
        }

        // LED 1 blink
        led1Blink();

        // LED 2 static
        leds[ledIndex2] = COLOR_BLUE;
        FastLED.show();

        // LED 3 static
        leds[ledIndex3] = COLOR_PURPLE;
        FastLED.show();

        // Eye
        if (recEspnowData.showHeadBat) {
            displayBatteryLevel();
        } else {
            switch (randomEyeAnimation) {
                case 1:
                    eyeMirror();
                    break;
                case 2:
                    eyeRunningDot();
                    break;
                default:
                    eyeStaticColor();
            }
        }
    }

    saveAnimationBegin();
}

void saveAnimationBegin() {
    if (saveAnimation) {
        for (int i = 0; i < 3; i++) {
            leds[i] = COLOR_RED;
        }
        for (int i = 3; i < NUM_LEDS_TOTAL; i++) {
            leds[i] = COLOR_RED_GRB;
        }

        FastLED.show();

        if (!recEspnowData.redMode) {
            saveAnimationEnd();
        }
    }
}

void saveAnimationEnd() {
    for (int i = 0; i < 3; i++) {
        leds[i] = COLOR_BLUE;
    }
    for (int i = 3; i < NUM_LEDS_TOTAL; i++) {
        leds[i] = COLOR_BLUE_GRB;
    }

    FastLED.show();
    delay(1000);
    FastLED.clearData();

    saveAnimation = false;
}

void led1Blink() {
    blinkOnTime = random(MIN_LED_BLINK_ON_TIME, MAX_LED_BLINK_ON_TIME + 1);
    blinkOffTime = random(MIN_LED_BLINK_OFF_TIME, MAX_LED_BLINK_OFF_TIME + 1);

    if (!blinking) {
        leds[ledIndex1] = COLOR_YELLOW;
        FastLED.show();

        if (millis() - lastFullCycleTime >= blinkPauseTime) {
            blinking = true;
            blinkCountTotal = random(MIN_BLINK_COUNT, MAX_BLINK_COUNT + 1);
            return;
        }
    }

    if (millis() - lastBlinkOffTime >= blinkOffTime && blinking) {
        lastBlinkOffTime = millis();

        leds[ledIndex1] = COLOR_YELLOW;
        FastLED.show();

        blinkCount++;

        if (blinkCount >= blinkCountTotal) {
            blinking = false;
            blinkPauseTime = random(MIN_BLINK_PAUSE_TIME, MAX_BLINK_PAUSE_TIME + 1);
            lastFullCycleTime = millis();
            blinkCount = 0;
            return;
        }
    }

    if (millis() - lastBlinkOnTime >= blinkOnTime && blinking) {
        lastBlinkOnTime = millis();
        leds[ledIndex1] = COLOR_OFF;
        FastLED.show();
    }
}

void eyeStaticColor() {
    eyeGoBack = false;
    ledIndex4 = 3;
    leds[3] = CRGB::Red;
    leds[4] = COLOR_OFF;
    leds[5] = COLOR_OFF;
    leds[6] = COLOR_OFF;
    leds[7] = COLOR_OFF;
    leds[8] = COLOR_OFF;
}

void eyeRunningDot() {
    unsigned long currentMillis = millis();

    if (currentMillis - eyeLedForwardLastOn >= eyeAnimationDelay && !eyeGoBack) {
        eyeLedForwardLastOn = currentMillis;

        if (ledIndex4 >= 3 && ledIndex4 < NUM_LEDS_TOTAL - 1) {
            leds[ledIndex4] = CRGB::Red;
        }

        if (ledIndex4 > 3 && ledIndex4 < NUM_LEDS_TOTAL - 1) {
            leds[ledIndex4 - 1] = COLOR_OFF;
        }

        FastLED.show();
        ledIndex4++;

        if (ledIndex4 > NUM_LEDS_TOTAL - 1) {
            ledIndex4 = NUM_LEDS_TOTAL - 1;
            eyeGoBack = true;
        }
    }

    if (currentMillis - eyeLedBackwardLastOn >= eyeAnimationDelay && eyeGoBack) {
        eyeLedBackwardLastOn = currentMillis;

        if (ledIndex4 >= 3 && ledIndex4 < NUM_LEDS_TOTAL) {
            leds[ledIndex4] = CRGB::Red;
        }

        if (ledIndex4 >= 3 && ledIndex4 < NUM_LEDS_TOTAL - 1) {
            leds[ledIndex4 + 1] = COLOR_OFF;
        }

        FastLED.show();
        ledIndex4--;

        if (ledIndex4 < 3) {
            ledIndex4 = 3;
            eyeGoBack = false;
            return;
        }
    }
}

void eyeMirror() {
    unsigned long currentMillis = millis();

    if (currentMillis - eyeLedForwardLastOn >= eyeAnimationDelay && !eyeGoBack) {
        eyeLedForwardLastOn = currentMillis;

        if (ledIndex4 >= 6 && ledIndex4 < NUM_LEDS_TOTAL) {
            leds[ledIndex4] = CRGB::Red;
            leds[11 - ledIndex4] = CRGB::Red;
            if (ledIndex4 > 6) {
                leds[ledIndex4 - 1] = COLOR_OFF;
                leds[11 - ledIndex4 + 1] = COLOR_OFF;
            }
        }

        ledIndex4++;
        FastLED.show();

        if (ledIndex4 > NUM_LEDS_TOTAL) {
            ledIndex4 = NUM_LEDS_TOTAL - 1;
            eyeGoBack = true;
        }
    }

    if (currentMillis - eyeLedBackwardLastOn >= eyeAnimationDelay && eyeGoBack) {
        eyeLedBackwardLastOn = currentMillis;
        if (ledIndex4 <= NUM_LEDS_TOTAL - 1 && ledIndex4 >= 6) {
            leds[ledIndex4] = CRGB::Red;
            leds[11 - ledIndex4] = CRGB::Red;
            if (ledIndex4 < NUM_LEDS_TOTAL - 1) {
                leds[ledIndex4 + 1] = COLOR_OFF;
                leds[11 - ledIndex4 - 1] = COLOR_OFF;
            }
        }

        ledIndex4--;
        FastLED.show();

        if (ledIndex4 < 6) {
            ledIndex4 = 6;
            eyeGoBack = false;
            return;
        }
    }
}

void displayBatteryLevel() {
    float bat = ((analogRead(BAT_PIN) * VIN_VOLTAGE) / 4095.0 * 1.08) / (R2 / (R1 + R2));

    int batCharge = 100 * ((bat - BATTV_MIN) / (BATTV_MAX - BATTV_MIN));
    batCharge = constrain(batCharge, 0, 100);

    RA.addValue(batCharge);
    int batAvg = RA.getAverage();

    int ledsCount = NUM_LEDS_EYE * batAvg / 100;

    for (int i = 0; i < ledsCount; i++) {
        if (batAvg > 0 && batAvg < 30) {
            leds[i + 3] = COLOR_RED_GRB;
        }
        if (batAvg >= 30 && batAvg < 50) {
            leds[i + 3] = COLOR_YELLOW_GRB;
        }
        if (batAvg >= 50 && batAvg <= 100) {
            leds[i + 3] = COLOR_GREEN_GRB;
        }
    }

    FastLED.show();
}