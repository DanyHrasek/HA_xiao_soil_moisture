// This is firmaware for Seed Studio Xiao Soil Mosture Sensor based on ESP32C6 communicating via Zigbee in End Device mode. 
// The device measures soil moisture and reports it to the coordinator, then goes to deep sleep to save battery. 
// When the device is plugged in, it reports more frequently and indicates the moisture level by LED. 
// The device can be woken up by a button press, which can be used for factory reset if the button is pressed for more than 10 seconds.

#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <Adafruit_NeoPixel.h>

#define USE_LED_ON_BATTERY 0 // when 1 LED blink shortly when reporting

#define ENDPOINT_NUMBER 1

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define WAKEUP_GPIO              GPIO_NUM_2 

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP_SHORT  900         /* Sleep for 15min */
#define TIME_TO_SLEEP_LONG  21500     /* Sleep for 6h (after 6h ZHA set device as unavailable) */
#define TIME_TO_SLEEP_TRESHOLD 45 /* moisture percentage below witch is period of reporting more frequent */
#define REPORT_TIMEOUT 1000       /* Timeout for response from coordinator in ms */

#define WET_VOLTAGE 1600 // voltage at full water saturation
#define DRY_VOLTAGE 2550 // voltage at dry soil
#define BATTERY_FACTOR 0 // how much battery charge affects the voltage measurement in water, 0 means no effect

#define PIXELS_NUM 33

const uint8_t buttonPin = 2;
const uint8_t pwmPin = 21;
const uint8_t sensorPin = 1;
const uint8_t batteryPin = 0;
const uint8_t ledRedPin = 20;
const uint8_t ledGreenPin = 19;
const uint8_t ledYellowPin = 18;
const uint8_t neopixelPin = 17; //D7

TaskHandle_t lightTaskHandle = NULL;

ZigbeeAnalog zbMoisture = ZigbeeAnalog(ENDPOINT_NUMBER);
ZigbeeColorDimmableLight zbColorLight = ZigbeeColorDimmableLight(ENDPOINT_NUMBER+1);
ZigbeeBinary zbWave = ZigbeeBinary(ENDPOINT_NUMBER+2);
ZigbeeBinary zbToggleAnim = ZigbeeBinary(ENDPOINT_NUMBER+3);
ZigbeeAnalog zbSpeed = ZigbeeAnalog(ENDPOINT_NUMBER+4);
ZigbeeAnalog zbAmplitude = ZigbeeAnalog(ENDPOINT_NUMBER+5);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(PIXELS_NUM, neopixelPin, NEO_GRB + NEO_KHZ800);

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
/************************ function called when on battery *****************************/
static void measureAndSleep(void *arg) {
  // Measure temperature sensor value
  uint moisture = 0;
  bool sent;

  // measure moisture 5 times and calculate average to get more stable value
  for(uint8_t m = 0; m<5; m++){
    if ( (DRY_VOLTAGE - analogRead(sensorPin)) >= 0 ) moisture = moisture + ( ( DRY_VOLTAGE - analogRead(sensorPin) )*100 / ( DRY_VOLTAGE - (WET_VOLTAGE - BATTERY_FACTOR*(100-measureBatteryPercentage()) ) ) );  //s nízkou baterkou napětí ve vodě klesne až na 1350
  }
  moisture = moisture /5;
  analogWrite(pwmPin, 0);

  zbMoisture.setAnalogInput(moisture);

  zbMoisture.setBatteryVoltage(measureBatteryVoltage());
  zbMoisture.setBatteryPercentage(measureBatteryPercentage());
  zbMoisture.reportBatteryPercentage();

  sent = zbMoisture.reportAnalogInput();  // report moisture as percentage value

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
  vTaskDelay(500 / portTICK_PERIOD_MS); // wait for a while to let the data be sent before going to sleep, otherwise the device may go to sleep before sending the data and the data will be lost

#if USE_LED_ON_BATTERY
  digitalWrite(ledRedPin, 0);
  digitalWrite(ledGreenPin, 0);
  digitalWrite(ledYellowPin, 0);
#endif
  
  // Put device to deep sleep based on moisture level, if the moisture is below the threshold, the device will sleep for shorter time to report more frequently, otherwise it will sleep for longer time to save battery
  if(moisture < TIME_TO_SLEEP_TRESHOLD) esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_SHORT * uS_TO_S_FACTOR);
  else esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_LONG * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

/************************ function called when plugged in *****************************/
static void measure(void *arg) {
  while(true){
    uint moisture = 0;

    for(uint8_t m = 0; m<10; m++){
      if ( (DRY_VOLTAGE - analogRead(sensorPin)) >= 0 ) moisture = moisture + ( ( DRY_VOLTAGE - analogRead(sensorPin) )*100 / ( DRY_VOLTAGE - WET_VOLTAGE ) );
    }
    moisture = moisture /10;

    zbMoisture.setAnalogInput(moisture);
    
    // indicate moisture level by LED when plugged in (all the time on, not only when reporting)
    digitalWrite(ledRedPin, 0);
    digitalWrite(ledGreenPin, 0);
    digitalWrite(ledYellowPin, 0);

    if(moisture > 50) digitalWrite(ledGreenPin, 1);
    else if(moisture > 40) digitalWrite(ledYellowPin, 1);
    else digitalWrite(ledRedPin, 1); 

    zbMoisture.reportAnalogInput();

    vTaskDelay(5000 / portTICK_PERIOD_MS); // wait for 5s before measuring again, the reporting interval is not critical when the device is plugged in, so we can afford to report more frequently to get more accurate data
  }
}

void fadePixel(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t r_current = pixels.getPixelColor(n) >> 16 & 0xFF;
  uint8_t g_current = pixels.getPixelColor(n) >> 8 & 0xFF;
  uint8_t b_current = pixels.getPixelColor(n) & 0xFF;
  float brightness = (float)zbColorLight.getLightLevel() / 255.0;
  float rstep = (r*brightness - r_current) / 10.0;
  float gstep = (g*brightness - g_current) / 10.0;
  float bstep = (b*brightness - b_current) / 10.0;
  for(uint8_t i=0; i<10; i++){
    pixels.setPixelColor(n, (r_current + rstep*i), (g_current + gstep*i), (b_current + bstep*i));
    pixels.show();
  }
  pixels.setPixelColor(n, r*brightness, g*brightness, b*brightness);
  pixels.show();
}

void setNeopixels(void *arg) {
  if(zbToggleAnim.getBinaryOutput()){
    for(uint8_t i=0; i<PIXELS_NUM; i++){
      fadePixel(i, zbColorLight.getLightRed(), zbColorLight.getLightGreen(), zbColorLight.getLightBlue());
      pixels.show();
    }
  }
  else {
    pixels.fill(pixels.Color(zbColorLight.getLightRed(), zbColorLight.getLightGreen(), zbColorLight.getLightBlue()));
    pixels.show();
  }
  while(zbColorLight.getLightState()){
    if(zbWave.getBinaryOutput()) {
      float brightness = (float)zbColorLight.getLightLevel() / 255.0;
      float amplitude = zbAmplitude.getAnalogOutput();
      for(uint8_t m=0; m<PIXELS_NUM && zbColorLight.getLightState(); m++){
        for(uint8_t n=0; n<PIXELS_NUM; n++){
          if(n < PIXELS_NUM/2) {
            if(m+n < PIXELS_NUM) {
              pixels.setPixelColor(m+n, zbColorLight.getLightRed()*brightness*(1-(n*amplitude)), zbColorLight.getLightGreen()*brightness*(1-(n*amplitude)), zbColorLight.getLightBlue()*brightness*(1-(n*amplitude)));
            }
            else pixels.setPixelColor(m+n-PIXELS_NUM, zbColorLight.getLightRed()*brightness*(1-(n*amplitude)), zbColorLight.getLightGreen()*brightness*(1-(n*amplitude)), zbColorLight.getLightBlue()*brightness*(1-(n*amplitude)));
          }
          else {
            if(m+n < PIXELS_NUM) {
              pixels.setPixelColor(m+n, zbColorLight.getLightRed()*brightness*(1-((PIXELS_NUM-n)*amplitude)), zbColorLight.getLightGreen()*brightness*(1-((PIXELS_NUM-n)*amplitude)), zbColorLight.getLightBlue()*brightness*(1-((PIXELS_NUM-n)*amplitude)));
            }
            else pixels.setPixelColor(m+n-PIXELS_NUM, zbColorLight.getLightRed()*brightness*(1-((PIXELS_NUM-n)*amplitude)), zbColorLight.getLightGreen()*brightness*(1-((PIXELS_NUM-n)*amplitude)), zbColorLight.getLightBlue()*brightness*(1-((PIXELS_NUM-n)*amplitude)));
          }
        }
        pixels.show();
        vTaskDelay(zbSpeed.getAnalogOutput() / portTICK_PERIOD_MS); // delay based on speed value from zigbee
      }
    }
    else vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  if(zbToggleAnim.getBinaryOutput()){
    for(uint8_t i=0; i<PIXELS_NUM; i++){
      fadePixel(i, 0, 0, 0);
      pixels.show();
    }
  }
  else {
    pixels.clear();
    pixels.show();
  }
  lightTaskHandle = NULL;
  vTaskDelete(NULL);
}

void createLight(bool state, uint8_t red, uint8_t green, uint8_t blue, uint8_t level) {
  if (state && lightTaskHandle == NULL) xTaskCreate(setNeopixels, "update_light", 2048, NULL, 11, &lightTaskHandle);
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

  pixels.begin();
  pixels.setBrightness(255);
  pixels.show(); // Initialize all pixels to 'off'

  // set Zigbee device name and model + add cluster for moisture sensor
  zbMoisture.setManufacturerAndModel("Seed Studio", "Xiao Soil Moisture Sensor");
  zbMoisture.addAnalogInput();
  zbMoisture.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER);
  zbMoisture.setAnalogInputDescription("Soil Moisture");
  zbMoisture.setAnalogInputResolution(1);

  zbColorLight.setLightColorCapabilities(ZIGBEE_COLOR_CAPABILITY_X_Y);
  zbColorLight.onLightChangeRgb(createLight);

  zbWave.addBinaryOutput();
  zbWave.setBinaryOutputDescription("Wave animation");
  zbToggleAnim.addBinaryOutput();
  zbToggleAnim.setBinaryOutputDescription("Toggle animation");
  zbSpeed.addAnalogOutput();
  zbSpeed.setAnalogOutputDescription("Animation speed");
  zbSpeed.setAnalogOutputResolution(1);
  zbSpeed.setAnalogOutputMinMax(1, 100);
  zbAmplitude.addAnalogOutput();
  zbAmplitude.setAnalogOutputDescription("Animation amplitude");
  zbAmplitude.setAnalogOutputResolution(0.01);
  zbAmplitude.setAnalogOutputMinMax(0.01, 0.10);

  // Set power source to battery, battery percentage and battery voltage
  if(measureBatteryVoltage() > 1) zbMoisture.setPowerSource(ZB_POWER_SOURCE_BATTERY, measureBatteryPercentage(), measureBatteryVoltage());
  else zbMoisture.setPowerSource(ZB_POWER_SOURCE_MAINS);

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbMoisture);
  Zigbee.addEndpoint(&zbColorLight);
  Zigbee.addEndpoint(&zbWave);
  Zigbee.addEndpoint(&zbToggleAnim);
  Zigbee.addEndpoint(&zbSpeed);
  Zigbee.addEndpoint(&zbAmplitude);

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
    digitalWrite(ledRedPin, 0);
    ESP.restart();  // If Zigbee failed to start, reboot the device and try again after couple blinks of red LED to indicate the error
  }
  while (!Zigbee.connected()) {
    delay(100);
  }

  // setup wakeup button for battery powered device and start measurement task according to power source
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
  zbColorLight.setLight(false, 50, 255, 117, 58);
  zbWave.setBinaryOutput(true);
  zbToggleAnim.setBinaryOutput(true);
  zbSpeed.setAnalogOutput(60);
  zbAmplitude.setAnalogOutput(0.05);
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
        Zigbee.factoryReset(true);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
    }
  }
  delay(100);
}