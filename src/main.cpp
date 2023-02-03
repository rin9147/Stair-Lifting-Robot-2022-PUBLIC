// Define
//------------------------------------------------------------------//
#define BLYNK_TEMPLATE_ID "YOUR_BLTNK_TEMPLATE_ID"
#define BLYNK_DEVICE_NAME "M5stack"
#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"
#define JOYSTICK_DEADZONE 5
#define TIMER_INTERRUPT 1 // Timer Interrupt Period
#define EXTENDABLE_WHEELS_GEAR_RATIO 3
#define EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING 4.6
#define EXTENDABLE_WHEELS_ANGLE_DEG 110
#define SERIAL_RX 3
#define SERIAL_TX 1
#define RS485_EN 19
#define LGFX_M5STACK_CORE2 // M5Stack Core2
#define LGFX_AUTODETECT
#define LGFX_USE_V1

// Include
//------------------------------------------------------------------//
#include <M5Core2.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ArduinoOTA.h>
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include "GYEMS.h"

// Global
//------------------------------------------------------------------//
// LovyanGFX
static LGFX lcd;
static LGFX_Sprite sprite(&lcd);

// WiFi, Blynk
const char auth[] = BLYNK_AUTH_TOKEN;
const char ssid1[] = "YOUR_SSID";
const char pass1[] = "YOUR_SSID_PASS";
const char server[] = "blynk.cloud";
const uint16_t port = 8080;
bool result;
bool wifi_flag;

bool OTA_flag = false;

// Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
byte iTimer50;

// GYEMS
const byte Ex_L = 1; // motorID 1-4
const byte Ex_R = 2;
const byte MOVE_L = 3;
const byte MOVE_R = 4;

GYEMS GS1(Ex_L, SERIAL_RX, SERIAL_TX, RS485_EN);   // Expanding wheel - Left
GYEMS GS2(Ex_R, SERIAL_RX, SERIAL_TX, RS485_EN);   // Expanding wheel - Right
GYEMS GS3(MOVE_L, SERIAL_RX, SERIAL_TX, RS485_EN); // Move wheel - Left
GYEMS GS4(MOVE_R, SERIAL_RX, SERIAL_TX, RS485_EN); // Move wheel - Right

bool emergencyStop_toggleFlag = false;
bool expand_toggleFlag = false;
bool move_forwardFlag = false;
bool move_backwardFlag = false;
bool move_rightFlag = false;
bool move_leftFlag = false;

int R_moveValue = 300;
int L_moveValue = 300;

int raw_expanding_percent_value = 0;
int expanding_dps_value = 0;
int moving_angle_value = 0;
int previous_expanding_value = 0;
int expanding_value = 0;
int deviation_expanding_value = 0;

int16_t replyData1[4] = {0};
int16_t replyData2[4] = {0};
int16_t replyData3[4] = {0};
int16_t replyData4[4] = {0};

bool GS1_F_flag = false;
bool GS2_F_flag = false;
bool GS3_F_flag = false;
bool GS4_F_flag = false;

bool GS1_B_flag = false;
bool GS2_B_flag = false;
bool GS3_B_flag = false;
bool GS4_B_flag = false;

bool GS1_L_flag = false;
bool GS2_L_flag = false;
bool GS3_L_flag = false;
bool GS4_L_flag = false;

bool GS1_R_flag = false;
bool GS2_R_flag = false;
bool GS3_R_flag = false;
bool GS4_R_flag = false;

bool GS1_E_flag = false;
bool GS2_E_flag = false;

// Prototype
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void);
void timerInterrupt(void);

// Blynk
//------------------------------------------------------------------//
BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

BLYNK_WRITE(V0) // EMERGENCY STOP BUTTON
{
  emergencyStop_toggleFlag = param.asInt();
}

// EXPAND / RETRACT BUTTON
BLYNK_WRITE(V1)
{
  // Expand button
  expand_toggleFlag = param.asInt();
}

// NUMETRIC INPUT FOR EXPANDING DPS
BLYNK_WRITE(V2)
{
  expanding_dps_value = param.asInt();
}

// SLIDER FOR EXPANDING PERCENT(EXPANDING AMOUNT)
BLYNK_WRITE(V3)
{
  raw_expanding_percent_value = param.asInt();
}

BLYNK_WRITE(V4)
{
  move_backwardFlag = param.asInt();
}

// NUMETRIC INPUT FOR EXPANDING DPS
BLYNK_WRITE(V12)
{
  moving_angle_value = param.asInt();
}

BLYNK_WRITE(V13)
{
  move_forwardFlag = param.asInt();
}

BLYNK_WRITE(V5)
{
  move_leftFlag = param.asInt();
}

BLYNK_WRITE(V6)
{
  move_rightFlag = param.asInt();
}

// Setup
//------------------------------------------------------------------//
void setup()
{
  M5.begin(false, true, false, false, kMBusModeOutput);
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setFont(&fonts::Font4);
  lcd.setCursor(0, 0);
  lcd.println("Blynk Connection...");
  Blynk.begin(auth, ssid1, pass1, "blynk.cloud", 8080);
  result = Blynk.connect();
  if (result != true)
  {
    lcd.printf("No1 Blynk FAIL");
    WiFi.disconnect();
    wifi_flag = false;
  }
  else
  {
    lcd.printf("No1 Blynk SUCCESS");
    wifi_flag = true;
  }
  delay(2000);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer);

  Blynk.syncVirtual(V2, V12);

  // Initialize Arduino OTA
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("Hold BtnA to switch OTA");
  delay(2000);
  M5.update();
  lcd.clear();
  if (M5.BtnA.read() && wifi_flag == true)
  {
    lcd.setCursor(0, 30);
    lcd.printf("OTA_MODE");
    ArduinoOTA
        .setHostname("M5Core2")
        .onStart([]() {})
        .onEnd([]() {})
        .onProgress([](unsigned int progress, unsigned int total) {})
        .onError([](ota_error_t error) {});
    ArduinoOTA.begin();
    lcd.setCursor(60, 135);
    lcd.printf("OTA MODE ACTIVE");
    OTA_flag = true;
  }
}

// loop
//------------------------------------------------------------------//
void loop()
{
  // Arduino OTA
  while (OTA_flag)
    ArduinoOTA.handle();

  // RECONNECT BLYNK when lost connection
  while (WiFi.status() != WL_CONNECTED)
  {
    lcd.setCursor(0, 0);
    lcd.printf("Reconnecting Blynk");
    Blynk.begin(auth, ssid1, pass1, "blynk.cloud", 8080);
    lcd.clear();
    delay(1000);
  }

  // MAIN
  Blynk.run();
  timerInterrupt();

  // expand/retract command
  if (expand_toggleFlag)
  {
    expanding_value = EXTENDABLE_WHEELS_ANGLE_DEG * EXTENDABLE_WHEELS_GEAR_RATIO * raw_expanding_percent_value / 100;
    deviation_expanding_value = expanding_value - previous_expanding_value;
    GS1_E_flag = true;
    GS2_E_flag = true;
    expand_toggleFlag = false;
    previous_expanding_value = expanding_value;
  }

  if (move_forwardFlag)
  {
    GS2_F_flag = true;
    GS1_F_flag = true;
    move_forwardFlag = false;
  }

  if (move_backwardFlag)
  {
    GS2_B_flag = true;
    GS1_B_flag = true;
    move_backwardFlag = false;
  }

  if (move_rightFlag)
  {
    GS2_R_flag = true;
    GS1_R_flag = true;
    move_rightFlag = false;
  }

  if (move_leftFlag)
  {
    GS2_L_flag = true;
    GS1_L_flag = true;
    move_leftFlag = false;
  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void timerInterrupt(void)
{
  if (interruptCounter > 0)
  {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    iTimer50++;
    // 50ms timerInterrupt
    switch (iTimer50)
    {
    case 10:
      if (GS2_E_flag)
      {
        GS2.IncrementalControlMode2((int32_t)deviation_expanding_value, (int32_t)expanding_dps_value, replyData2, false);
        GS2_E_flag = false;
      }
      else if (GS2_F_flag)
      {
        GS2.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, (uint32_t)R_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData4, false);
        GS2_F_flag = false;
        GS4_F_flag = true;
      }
      else if (GS2_B_flag)
      {
        GS2.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING * -1, (uint32_t)R_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData4, false);
        GS2_B_flag = false;
        GS4_B_flag = true;
      }
      else if (GS2_R_flag)
      {
        GS2.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, (uint32_t)R_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData2, false);
        GS2_R_flag = false;
        GS4_R_flag = true;
      }
      else if (GS2_L_flag)
      {
        GS2.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING * -1, (uint32_t)R_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData2, false);
        GS2_L_flag = false;
        GS4_L_flag = true;
      }
      else
      {
      }
      break;
    case 20:
      if (GS4_F_flag)
      {
        GS4.IncrementalControlMode2((int32_t)moving_angle_value * -1, (uint32_t)R_moveValue, replyData4, false);
        GS4_F_flag = false;
      }
      else if (GS4_B_flag)
      {
        GS4.IncrementalControlMode2((int32_t)moving_angle_value, (uint32_t)R_moveValue, replyData4, false);
        GS4_B_flag = false;
      }
      else if (GS4_R_flag)
      {
        GS4.IncrementalControlMode2((int32_t)moving_angle_value * -1, (uint32_t)R_moveValue, replyData4, false);
        GS4_R_flag = false;
      }
      else if (GS4_L_flag)
      {
        GS4.IncrementalControlMode2((int32_t)moving_angle_value, (uint32_t)R_moveValue, replyData4, false);
        GS4_L_flag = false;
      }
      else
      {
      }
      break;
    case 30:
      if (GS1_E_flag)
      {
        GS1.IncrementalControlMode2((int32_t)deviation_expanding_value, (int32_t)expanding_dps_value, replyData1, false);
        GS1_E_flag = false;
      }
      else if (GS1_F_flag)
      {
        GS1.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, (uint32_t)L_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData1, false);
        GS1_F_flag = false;
        GS3_F_flag = true;
      }
      else if (GS1_B_flag)
      {
        GS1.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING * -1, (uint32_t)L_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData1, false);
        GS1_B_flag = false;
        GS3_B_flag = true;
      }
      else if (GS1_R_flag)
      {
        GS1.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING * -1, (uint32_t)L_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData1, false);
        GS1_R_flag = false;
        GS3_R_flag = true;
      }
      else if (GS1_L_flag)
      {
        GS1.IncrementalControlMode2((int32_t)moving_angle_value / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, (uint32_t)L_moveValue / EXTENDABLE_WHEELS_GEAR_RATIO_FOR_MOVING, replyData1, false);
        GS1_L_flag = false;
        GS3_L_flag = true;
      }
      else
      {
      }
      break;
    case 40:
      if (GS3_F_flag)
      {
        GS3.IncrementalControlMode2((int32_t)moving_angle_value * -1, (uint32_t)L_moveValue, replyData3, false);
        GS3_F_flag = false;
      }
      else if (GS3_B_flag)
      {
        GS3.IncrementalControlMode2((int32_t)moving_angle_value, (uint32_t)L_moveValue, replyData3, false);
        GS3_B_flag = false;
      }
      else if (GS3_R_flag)
      {
        GS3.IncrementalControlMode2((int32_t)moving_angle_value, (uint32_t)L_moveValue, replyData3, false);
        GS3_R_flag = false;
      }
      else if (GS3_L_flag)
      {
        GS3.IncrementalControlMode2((int32_t)moving_angle_value * -1, (uint32_t)L_moveValue, replyData3, false);
        GS3_L_flag = false;
      }
      else
      {
      }
      break;
    case 50:
      Blynk.syncVirtual(V4, V5, V6, V13);
      iTimer50 = 0;
      break;
    }
  }
}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void)
{
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
}
