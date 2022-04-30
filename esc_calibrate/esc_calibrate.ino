#include "I2Cdev.h"
#include "MPU6050.h"
#include <WiFi.h>
#include "soc/rtc_wdt.h"

// motor pins and chanels
#define FRONT_LEFT_PIN 25 //CW
#define FRONT_RIGHT_PIN 26 //CCW
#define REAR_LEFT_PIN 27 //CW
#define REAR_RIGHT_PIN 14 //CCW

#define FRONT_LEFT 0 //CW
#define FRONT_RIGHT 1 //CCW
#define REAR_LEFT 2 //CW
#define REAR_RIGHT 3 //CCW

#define PWM_FREQ 50

// util function

inline uint16_t throttle_to_pwm(double ms)
{
  return uint16_t((ms + 1.0) / 1000.0 * double(PWM_FREQ) * 65535.0);
}

void setup()
{
  Serial.begin(38400);
  ledcSetup(FRONT_LEFT, 55, 16); // canal 0, frecventa 55, 16 biti rezolutie
  ledcSetup(FRONT_RIGHT, 55, 16); // canal 1, frecventa 55, 16 biti rezolutie
  ledcSetup(REAR_LEFT, 55, 16); // canal 2, frecventa 55, 16 biti rezolutie
  ledcSetup(REAR_RIGHT, 55, 16); // canal 3, frecventa 55, 16 biti rezolutie
    
  ledcAttachPin(FRONT_LEFT_PIN, FRONT_LEFT);
  ledcAttachPin(FRONT_RIGHT_PIN, FRONT_RIGHT);
  ledcAttachPin(REAR_LEFT_PIN, REAR_LEFT);
  ledcAttachPin(REAR_RIGHT_PIN, REAR_RIGHT);
  
  uint16_t startup_freq = throttle_to_pwm(0.25);
  uint16_t zero_freq = throttle_to_pwm(0.0);
  ledcWrite(FRONT_LEFT, startup_freq);
  ledcWrite(FRONT_RIGHT, startup_freq);
  ledcWrite(REAR_LEFT, startup_freq);
  ledcWrite(REAR_RIGHT, startup_freq);
}

void loop()
{
  if (Serial.available()) {
    String s = Serial.readString();
    float throttle = s.toFloat();
    Serial.print("Setting throttle to ");
    Serial.println(throttle);
    float throttle_freq = throttle_to_pwm(throttle);
    ledcWrite(FRONT_LEFT, throttle_freq);
    ledcWrite(FRONT_RIGHT, throttle_freq);
    ledcWrite(REAR_LEFT, throttle_freq);
    ledcWrite(REAR_RIGHT, throttle_freq);
  }
}
