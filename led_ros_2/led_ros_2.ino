#include <M5Stack.h>
#undef ESP32
#include <ros.h>
#define ESP32
#include <std_msgs/UInt16.h>

#include "FastLED.h"

#define Neopixel_PIN_C 17
#define Neopixel_PIN_B 26
#define NUM_LEDS 37


CRGB leds_strip_B[NUM_LEDS];
CRGB leds_strip_C[NUM_LEDS];

ros::NodeHandle nh;
int eye_status_num = 0;

void messageCb(const std_msgs::UInt16& eye_status) {
    Serial.print("Received eye_status: ");
    Serial.println(eye_status.data);
    eye_status_num = eye_status.data;
}

ros::Subscriber<std_msgs::UInt16> sub("eye_status", &messageCb);

void setup() {
    M5.begin();
    M5.Speaker.begin();
    M5.Speaker.mute();

    M5.Lcd.println("Display");

    FastLED.addLeds<WS2811, Neopixel_PIN_B, GRB>(leds_strip_B, NUM_LEDS)
        .setCorrection(TypicalLEDStrip);
    FastLED.addLeds<WS2811, Neopixel_PIN_C, GRB>(leds_strip_C, NUM_LEDS)
        .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(2);

    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
}

void loop() {
    if (eye_status_num == 6) {
        fill_solid(leds_strip_B, NUM_LEDS, CRGB::Orange);
        fill_solid(leds_strip_C, NUM_LEDS, CRGB::Orange);
    } else if (eye_status_num == 3) {
        fill_solid(leds_strip_B, NUM_LEDS, CRGB::Green);
        fill_solid(leds_strip_C, NUM_LEDS, CRGB::Green);
    } else if (eye_status_num == 5) {
        fill_solid(leds_strip_B, NUM_LEDS, CRGB::Blue);
        fill_solid(leds_strip_C, NUM_LEDS, CRGB::Blue);
    } else if (eye_status_num == 7) {
        fill_solid(leds_strip_B, NUM_LEDS, CRGB::MediumVioletRed);
        fill_solid(leds_strip_C, NUM_LEDS, CRGB::MediumVioletRed);
    } else if (eye_status_num == 4) {
        fill_solid(leds_strip_B, NUM_LEDS, CRGB::Red);
        fill_solid(leds_strip_C, NUM_LEDS, CRGB::Red);
    } else {
        fill_solid(leds_strip_B, NUM_LEDS, CRGB::Black);
        fill_solid(leds_strip_C, NUM_LEDS, CRGB::Black);
    }

    nh.spinOnce();
    FastLED.show();
    delay(10);
}
