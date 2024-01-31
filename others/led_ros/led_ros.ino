#include <M5Stack.h>
#undef ESP32
#include <ros.h>
#define ESP32
#include <std_msgs/UInt16.h>

#include "FastLED.h"

#define Neopixel_PIN_C 17
//#define Neopixel_PIN_B 26
#define NUM_LEDS 37

CRGB leds[NUM_LEDS];
uint8_t gHue = 0;

ros::NodeHandle nh;
int eye_status_num = 0;

void messageCb( const std_msgs::UInt16& eye_status) {
    Serial.print("Received eye_status: ");
    Serial.println(eye_status.data);
    eye_status_num = eye_status.data;
   
}

ros::Subscriber<std_msgs::UInt16> sub("eye_status", &messageCb );

void setup(){
    M5.begin();
    M5.Speaker.begin();
    M5.Speaker.mute();  

    //M5.Power.begin();
    //M5.Lcd.setTextSize(2);
    //M5.Lcd.println("Example");
    M5.Lcd.println("Display");

    FastLED.addLeds<WS2811, Neopixel_PIN_C, GRB>(leds, NUM_LEDS)
        .setCorrection(TypicalLEDStrip);
    //FastLED.addLeds<WS2811, Neopixel_PIN_B, GRB>(leds, NUM_LEDS)
    //    .setCorrection(TypicalLEDStrip);    
    FastLED.setBrightness(4);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
}

void loop() {
    //nh.spinOnce();
    if (eye_status_num == 1) {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
    } else if (eye_status_num == 2) {
        fill_solid(leds, NUM_LEDS, CRGB::Green);
    } else {
        fill_solid(leds, NUM_LEDS, CRGB::Blue);
    }
    nh.spinOnce();
    FastLED.show();
    delay(10);
}
