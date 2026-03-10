// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee temperature and humidity sensor Sleepy device.
 *
 * The example demonstrates how to use Zigbee library to create an end device temperature and humidity sensor.
 * The sensor is a Zigbee end device, which is reporting data to the Zigbee network.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

#define USE_GLOBAL_ON_RESPONSE_CALLBACK 0  // Set to 0 to use local callback specified directly for the endpoint.

#define USE_LED_ON_BATTERY 0 // when 1 LED blink shortly when reporting

/* Zigbee temperature + humidity sensor configuration */
#define ENDPOINT_NUMBER 1

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define WAKEUP_GPIO              GPIO_NUM_2 

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP_SHORT  900         /* Sleep for 15min */
#define TIME_TO_SLEEP_LONG  21500     /* Sleep for 6h (after 6h ZHA set device as unavailable) */
#define TIME_TO_SLEEP_TRESHOLD 45 // moisture percentage below witch is period of reporting more frequent
#define REPORT_TIMEOUT 1000       /* Timeout for response from coordinator in ms */

#define WET_VOLTAGE 1600
#define DRY_VOLTAGE 2750
#define BATTERY_FACTOR 2 // how much battery charge 

const uint8_t buttonPin = 2;
const uint8_t pwmPin = 21;
const uint8_t sensorPin = 1;
const uint8_t batteryPin = 0;
const uint8_t ledRedPin = 20;
const uint8_t ledGreenPin = 19;
const uint8_t ledYellowPin = 18;

ZigbeeAnalog zbMoisture = ZigbeeAnalog(ENDPOINT_NUMBER);

uint8_t dataToSend = 1;  // Temperature and humidity values are reported in same endpoint, so 2 values are reported
bool resend = false;

/************************ Callbacks *****************************/
#if USE_GLOBAL_ON_RESPONSE_CALLBACK
void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
  //Serial.printf("Global response command: %d, status: %s, endpoint: %d, cluster: 0x%04x\r\n", command, esp_zb_zcl_status_to_name(status), endpoint, cluster);
  if ((command == ZB_CMD_REPORT_ATTRIBUTE) && (endpoint == ENDPOINT_NUMBER)) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--; break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
      default:                        break;  // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE, ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}
#else
void onResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status) {
  //Serial.printf("Response command: %d, status: %s\r\n", command, esp_zb_zcl_status_to_name(status));
  if (command == ZB_CMD_REPORT_ATTRIBUTE) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--; break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
      default:                        break;  // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE, ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}
#endif

uint8_t measureBatteryVoltage(){
  uint8_t voltage = analogRead(batteryPin)/100;
  return voltage;
}
uint8_t measureBatteryPercentage(){
  uint8_t percentage;
  int voltage = analogRead(batteryPin);
  if( (voltage < 1100) && (voltage > 100) ) percentage = 0;
  else if(voltage < 1500 && (voltage > 100) ) percentage = (voltage-1100)*100/400;
  else percentage = 100;
  if(percentage == 0){
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_deep_sleep_start();
  }
  return percentage;
}
/************************ Temp sensor *****************************/
static void measureAndSleep(void *arg) {
  // Measure temperature sensor value
  uint moisture = 0;
  bool sent;

  for(uint8_t m = 0; m<5; m++){
    moisture = moisture + ( ( DRY_VOLTAGE - analogRead(sensorPin) )*100 / ( DRY_VOLTAGE - (WET_VOLTAGE - BATTERY_FACTOR*(100-measureBatteryPercentage()) ) ) );  //s nízkou baterkou napětí ve vodě klesne až na 1350
  }
  moisture = moisture /5;
  analogWrite(pwmPin, 0);

  // Update temperature and humidity values in Temperature sensor EP
  zbMoisture.setAnalogInput(moisture);

  zbMoisture.setBatteryVoltage(measureBatteryVoltage());
  zbMoisture.setBatteryPercentage(measureBatteryPercentage());
  zbMoisture.reportBatteryPercentage();

  // Report temperature and humidity values
  sent = zbMoisture.reportAnalogInput();  // reports temperature and humidity values (if humidity sensor is not added, only temperature is reported)

#if USE_LED_ON_BATTERY
  if(moisture > 50) digitalWrite(ledGreenPin, 1);
  else if(moisture > 40) digitalWrite(ledYellowPin, 1);
  else digitalWrite(ledRedPin, 1);
#endif

  uint8_t tryies = 3;
  while(!sent && tryies != 0){
    sent = zbMoisture.reportAnalogInput();
    tryies--;
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);

#if USE_LED_ON_BATTERY
  digitalWrite(ledRedPin, 0);
  digitalWrite(ledGreenPin, 0);
  digitalWrite(ledYellowPin, 0);
#endif
  
  // Put device to deep sleep after data was sent successfully or timeout
  if(moisture < TIME_TO_SLEEP_TRESHOLD) esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_SHORT * uS_TO_S_FACTOR);
  else esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_LONG * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

static void measure(void *arg) {
  // Measure temperature sensor value
  while(true){
    uint moisture = 0;

    for(uint8_t m = 0; m<5; m++){
      moisture = moisture + ( ( DRY_VOLTAGE - analogRead(sensorPin) )*100 / ( DRY_VOLTAGE - WET_VOLTAGE ) );  //s nízkou baterkou napětí ve vodě klesne až na 1350
    }
    moisture = moisture /5;

    // Update temperature and humidity values in Temperature sensor EP
    zbMoisture.setAnalogInput(moisture);
    
    digitalWrite(ledRedPin, 0);
    digitalWrite(ledGreenPin, 0);
    digitalWrite(ledYellowPin, 0);

    if(moisture > 50) digitalWrite(ledGreenPin, 1);
    else if(moisture > 40) digitalWrite(ledYellowPin, 1);
    else digitalWrite(ledRedPin, 1);

    // Report temperature and humidity values
    zbMoisture.reportAnalogInput();  // reports temperature and humidity values (if humidity sensor is not added, only temperature is reported)

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/********************* Arduino functions **************************/
void setup() {
  
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledYellowPin, OUTPUT);

  analogWriteFrequency(pwmPin, 200000);
  analogWriteResolution(pwmPin, 2);
  analogWrite(pwmPin, 2);

  // Init button switch
  pinMode(buttonPin, INPUT);

  // Optional: set Zigbee device name and model
  zbMoisture.setManufacturerAndModel("Espressif", "Soil Moisture Sensor");
  // Set default (initial) value for the temperature sensor to 10.0°C to match the minimum temperature measurement value (default value is 0.0°C)
  zbMoisture.addAnalogInput();
  zbMoisture.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER);
  zbMoisture.setAnalogInputDescription("Soil Moisture");
  zbMoisture.setAnalogInputResolution(1);

  // Set power source to battery, battery percentage and battery voltage (now 100% and 3.5V for demonstration)
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
  zbMoisture.setPowerSource(ZB_POWER_SOURCE_BATTERY, measureBatteryPercentage(), measureBatteryVoltage());

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbMoisture);

  // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

  // For battery powered devices, it can be better to set timeout for Zigbee Begin to lower value to save battery
  // If the timeout has been reached, the network channel mask will be reset and the device will try to connect again after reset (scanning all channels)
  Zigbee.setTimeout(10000);  // Set timeout for Zigbee Begin to 10s (default is 30s)

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    digitalWrite(ledRedPin, 1);
    delay(500);
    digitalWrite(ledRedPin, 0);
    delay(500);
    digitalWrite(ledRedPin, 1);
    delay(500);
    digitalWrite(ledRedPin, 0);
    delay(500);
    digitalWrite(ledRedPin, 1);
    delay(500);
    ESP.restart();  // If Zigbee failed to start, reboot the device and try again
  }
  while (!Zigbee.connected()) {
    delay(100);
  }

  // Start Temperature sensor reading task

  if(measureBatteryVoltage() > 1){
    esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_LOW);
    // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
    // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    // No need to keep that power domain explicitly, unlike EXT1.
    gpio_pullup_dis(WAKEUP_GPIO);
    gpio_pulldown_dis(WAKEUP_GPIO);

    xTaskCreate(measureAndSleep, "sensor_update", 2048, NULL, 10, NULL);
  }
  else xTaskCreate(measure, "sensor_update", 2048, NULL, 10, NULL);
}

void loop() {
  // Checking button for factory reset
  if (digitalRead(buttonPin) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(buttonPin) == LOW) {
      delay(50);
      if ((millis() - startTime) > 10000) {
        // If key pressed for more than 10secs, factory reset Zigbee and reboot
        delay(1000);
        // Optional set reset in factoryReset to false, to not restart device after erasing nvram, but set it to endless sleep manually instead
        Zigbee.factoryReset(false);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
    }
  }
  delay(100);
}