#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <DFPlayerMini_Fast.h>
#include <EasyTransfer.h>
#include <FastLED.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>

#include "RunningAverage.h"

Adafruit_ADS1115 ads;

//==================================================================================
/*                                   EspSerial1                                   */
//==================================================================================
#define RXD1 18
#define TXD1 19
HardwareSerial Mp3Serial(1);  // ESP2 <-> DFPlayer

EasyTransfer RecESP;
EasyTransfer SendESP;

struct RECEIVE_DATA_STRUCTURE_ESP1 {
    int8_t play_num = 0;
    uint8_t ps4_bat = 0;
    bool ps4_chrg = false;
    bool showHeadBat = false;
};
RECEIVE_DATA_STRUCTURE_ESP1 recESPData;

struct SEND_DATA_STRUCTURE_ESP1 {
    int16_t distance;
    uint16_t pot_s2s;
    uint16_t pot_dome;
};
SEND_DATA_STRUCTURE_ESP1 sendESPData;

int batteryBody = 100;
int batteryPS4 = 0;

unsigned long lastLoopMillis;
//==================================================================================

//==================================================================================
/*                                    Battery                                     */
//==================================================================================
RunningAverage RA(5000);

#define BATTV_MAX 16.6  // maximum battery voltage
#define BATTV_MIN 12    // empty battery
#define DIVIDER 6.9     // voltage divider value
//==================================================================================

//==================================================================================
/*                                    ESP-NOW                                     */
//==================================================================================
uint8_t espnowBroadcastAddress[] = {0xA0, 0xB7, 0x65, 0x4A, 0xF4, 0x34};  // Head Esp32 address

struct ESPNOW_MSG_RCV {
    int16_t distance;
};
ESPNOW_MSG_RCV rcvEspnowData;

struct ESPNOW_MSG {
    bool play;
    bool showHeadBat;
    bool redMode;
};
ESPNOW_MSG sendEspnowData;

esp_now_peer_info_t peerInfo;
//==================================================================================

//==================================================================================
/*                                    MP3 PLAYER                                  */
//==================================================================================
#define MP3_BUSY 13  // LOW if playing
DFPlayerMini_Fast dfPlayer;
unsigned long lastPlayTime = 0;
//==================================================================================

//==================================================================================
/*                                       LED                                      */
//==================================================================================
#define INTERNAL_LED_PIN 2
#define LED_DATA_PIN 25

#define NUM_LEDS 13
CRGB leds[NUM_LEDS];

#define COLOR_BLUE CRGB::Blue
#define COLOR_YELLOW CRGB::Orange
#define COLOR_GREEN CRGB::DarkGreen
#define COLOR_PURPLE CRGB::Purple
#define COLOR_RED CRGB::Red
#define COLOR_DARK_ORANGE CRGB::OrangeRed
#define COLOR_OFF CRGB::Black

#define INTERNAL_LED_BLINK_PERIOD 15
#define MIN_ANIMATION_DURATION 30000
#define MAX_ANIMATION_DURATION 70000
#define MIN_ANIMATIONS_TOTAL_COUNT 10
#define MAX_ANIMATIONS_TOTAL_COUNT 20

bool firstRun = true;
bool isAnimating = true;
bool endAnimation = false;
bool prepareToEndAnimation = false;
bool saveAnimation = false;

static int ledIndex1 = 0;   // LED 0, 1, 2
static int ledIndex2 = 3;   // LED 3, 4, 5, 6
static int ledIndex3 = 11;  // LED 11
static int ledIndex4 = 12;  // LED 12

static int animationsPlayed = 0;
static int endAnimationLedIndex = 6;
static int animationsTotalCount = 0;

static unsigned long animationPeriod = 0;
static unsigned long lastLedOnTime = 0;
static unsigned long lastAnimationTime = 0;
static unsigned long lastInternalLedBlinkTime = 0;
//==================================================================================

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void onRXReceive(void);
void blinkInternalLed();
void turnOnAnimation();
void allLedsOn();
void allLedsOff();
void saveAnimationBegin();
void saveAnimationEnd();
void loadingAnimation();
void sendEspNow();
void updateSerialData();
void showBatteryLevel(float voltage);
void playMP3();

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&rcvEspnowData, incomingData, sizeof(rcvEspnowData));
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(460800);
    Serial2.onReceive(onRXReceive);
    Mp3Serial.begin(9600, SERIAL_8N1, RXD1, TXD1);

    RecESP.begin(details(recESPData), &Serial2);
    SendESP.begin(details(sendESPData), &Serial2);

    pinMode(INTERNAL_LED_PIN, OUTPUT);
    pinMode(MP3_BUSY, INPUT_PULLUP);

    WiFi.mode(WIFI_MODE_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    } else {
        Serial.println("ESP-NOW INIT OK");
    }

    // Register peer
    memcpy(peerInfo.peer_addr, espnowBroadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    dfPlayer.begin(Mp3Serial);

    ads.setGain(GAIN_TWOTHIRDS);

    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS.");
        while (1)
            ;
    }

    FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    turnOnAnimation();
    FastLED.clearData();
    FastLED.setBrightness(80);

    randomSeed(analogRead(0));
}

void onRXReceive(void) {
    blinkInternalLed();
}

void blinkInternalLed() {
    long int currentMillis = millis();
    if (currentMillis - lastInternalLedBlinkTime >= INTERNAL_LED_BLINK_PERIOD) {
        lastInternalLedBlinkTime = currentMillis;
        digitalWrite(INTERNAL_LED_PIN, !digitalRead(INTERNAL_LED_PIN));
    }
}

void turnOnAnimation() {
    FastLED.setBrightness(8);

    for (int i = 0; i <= 3; i++) {
        allLedsOn();
        delay(random(50, 300));
        allLedsOff();
        delay(random(50, 300));
    }

    delay(700);
}

void allLedsOn() {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::COLOR_DARK_ORANGE;
    }

    FastLED.show();
}

void allLedsOff() {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = COLOR_OFF;
    }

    FastLED.show();
}

void saveAnimationBegin() {
    saveAnimation = true;
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = COLOR_RED;
    }
    FastLED.show();
}

void saveAnimationEnd() {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = COLOR_BLUE;
    }
    FastLED.show();
    saveAnimation = false;
    sendEspNow();
    delay(1000);
    FastLED.clearData();
}

void loadingAnimation() {
    unsigned long currentMillis = millis();

    // static color
    if (!isAnimating) {
        if (ledIndex2 > 6) {
            ledIndex2 = 3;
        }

        leds[ledIndex2] = COLOR_YELLOW;
        ledIndex2++;
        FastLED.show();

        if (currentMillis - lastAnimationTime >= animationPeriod) {
            isAnimating = true;
            ledIndex2 = 3;
            animationsTotalCount = random(MIN_ANIMATIONS_TOTAL_COUNT, MAX_ANIMATIONS_TOTAL_COUNT + 1);
            return;
        }
    }

    // animation
    if ((currentMillis - lastLedOnTime >= 200) && isAnimating) {
        lastLedOnTime = currentMillis;

        if (!prepareToEndAnimation) {
            if (ledIndex2 > 6) {
                leds[ledIndex2 - 1] = COLOR_OFF;
                ledIndex2 = 3;
                animationsPlayed++;
            }
        }

        if (animationsPlayed >= animationsTotalCount) {
            prepareToEndAnimation = true;
        }

        if (prepareToEndAnimation) {
            if (ledIndex2 > endAnimationLedIndex) {
                ledIndex2 = 3;
                endAnimationLedIndex--;
            }

            if (endAnimationLedIndex < 3) {
                endAnimation = true;
            }
        }

        if (endAnimation) {
            lastAnimationTime = currentMillis;
            endAnimation = false;
            isAnimating = false;
            animationPeriod = random(MIN_ANIMATION_DURATION, MAX_ANIMATION_DURATION + 1);
            animationsPlayed = 0;
            prepareToEndAnimation = false;
            endAnimationLedIndex = 6;
            return;
        }

        leds[ledIndex2] = COLOR_YELLOW;

        if (ledIndex2 > 3) {
            leds[ledIndex2 - 1] = COLOR_OFF;
        }

        FastLED.show();
        ledIndex2++;
    }
}

void loop() {
    sendESPData.distance = rcvEspnowData.distance;

    int res = ads.readADC_SingleEnded(0);
    batteryBody = ads.computeVolts(res) * 100;

    sendESPData.pot_dome = ads.readADC_SingleEnded(1);
    sendESPData.pot_s2s = ads.readADC_SingleEnded(2);

    SendESP.sendData();

    if (RecESP.receiveData()) {
        updateSerialData();
    }

    if (!saveAnimation) {
        // LED 0 - 2 static
        if (ledIndex1 > 2) {
            ledIndex1 = 0;
        }

        leds[ledIndex1] = COLOR_BLUE;
        ledIndex1++;
        FastLED.show();

        // LED 11 static
        leds[ledIndex3] = COLOR_PURPLE;
        FastLED.show();

        // LED 12 static
        leds[ledIndex4] = COLOR_RED;
        FastLED.show();

        // LED 3 - 6
        loadingAnimation();
    }

    sendEspNow();
}

void sendEspNow() {
    // sound playing state
    sendEspnowData.play = !digitalRead(MP3_BUSY);
    sendEspnowData.redMode = saveAnimation;
    sendEspnowData.showHeadBat = recESPData.showHeadBat;

    // ESP-NOW - send message to Head
    esp_err_t result = esp_now_send(espnowBroadcastAddress, (const uint8_t *)&sendEspnowData, sizeof(sendEspnowData));
}

void updateSerialData() {
    batteryPS4 = recESPData.ps4_bat;
    playMP3();
    showBatteryLevel(batteryBody);
}

void showBatteryLevel(float voltage) {
    unsigned long currentMillis = millis();

    float fullVoltage = voltage * DIVIDER / 100.0;
    int batCharge = 100 * ((fullVoltage - BATTV_MIN) / (BATTV_MAX - BATTV_MIN));

    batCharge = constrain(batCharge, 0, 100);

    RA.addValue(batCharge);
    int batAvg = RA.getAverage();

    if (batAvg > 0 && batAvg < 25) {
        leds[7] = COLOR_GREEN;
        leds[8] = COLOR_OFF;
        leds[9] = COLOR_OFF;
        leds[10] = COLOR_OFF;
    }

    if (batAvg >= 25 && batAvg < 50) {
        leds[7] = COLOR_GREEN;
        leds[8] = COLOR_GREEN;
        leds[9] = COLOR_OFF;
        leds[10] = COLOR_OFF;
    }

    if (batAvg >= 50 && batAvg < 75) {
        leds[7] = COLOR_GREEN;
        leds[8] = COLOR_GREEN;
        leds[9] = COLOR_GREEN;
        leds[10] = COLOR_OFF;
    }

    if (batAvg >= 75 && batAvg < 100) {
        leds[7] = COLOR_GREEN;
        leds[8] = COLOR_GREEN;
        leds[9] = COLOR_GREEN;
        leds[10] = COLOR_GREEN;
    }

    FastLED.show();
}

void playMP3() {
    if (recESPData.play_num > 0) {
        if (millis() - lastPlayTime >= 500) {
            lastPlayTime = millis();

            if (dfPlayer.isPlaying()) {
                dfPlayer.stop();
            }
            dfPlayer.play(recESPData.play_num);
        }

        if (recESPData.play_num == 50) {
            saveAnimationBegin();
        }
        if (recESPData.play_num == 51) {
            saveAnimationEnd();
        }
    }
}
