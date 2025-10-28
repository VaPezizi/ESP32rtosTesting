#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define INITBLINKTASK(y, a) xTaskCreate( \
                    blinkTask,              /* Function that implements the task */ \
                    "blinkTask",            /* Name of the task */ \
                    2048,           /* Stack size */ \
                    y,           /* Parameter */ \
                    1,              /* Priority */ \
                    a)           /* Task handle */

const uint8_t LED_PIN0 = 5;
const uint8_t LED_PIN1 = 18;
const uint8_t LED_PIN2 = 19;
const uint8_t POT_PIN = 34;

static volatile int LED_SPEED = 500; // milliseconds
static volatile int potValue = 0;

typedef struct blinkArgs{
  int ledPin;
  int speed;
}blinkArgs;

void blinkTask(void *pvParameters) {
  blinkArgs *args = (blinkArgs *)pvParameters;
  pinMode(args->ledPin, OUTPUT);

  for (;;) {
    int wait = map(potValue, 0 , 4095, 100, args->speed);
    digitalWrite(args->ledPin, HIGH); 
    vTaskDelay(wait / portTICK_PERIOD_MS); 

    digitalWrite(args->ledPin, LOW);  
    vTaskDelay(wait / portTICK_PERIOD_MS); 
  }
}

void readPotTask(void * pvParameters){
  pinMode(POT_PIN, INPUT);
  while(1){
    potValue = analogRead(POT_PIN);
    Serial.println("Potentiometer Value: " + String(potValue));
    //potValue = map(potValue, 0, 4095, 100, 1000);
    Serial.println("LED_SPEED value: " + String(potValue));
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("FreeRTOS Example Starting");

  static const blinkArgs args0 = {LED_PIN0, LED_SPEED << 1};
  static const blinkArgs args1 = {LED_PIN1, LED_SPEED};
  static const blinkArgs args2 = {LED_PIN2, LED_SPEED >> 1};

  // Create a simple FreeRTOS task
  /*xTaskCreate(
    blinkTask,      // Function that implements the task
    "blinkTask",    // Name of the task
    2048,           // Stack size
    NULL,           // Parameter
    1,              // Priority
                // Task handle
  );*/
  TaskHandle_t blinkTaskHandle0;
  TaskHandle_t blinkTaskHandle1;
  TaskHandle_t blinkTaskHandle2;
  TaskHandle_t readPotTaskHandle;
  
  INITBLINKTASK((void *)&args0, &blinkTaskHandle0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  INITBLINKTASK((void *)&args1, &blinkTaskHandle1);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  INITBLINKTASK((void *)&args2, &blinkTaskHandle2);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  xTaskCreate(
    readPotTask,      // Function that implements the task
    "readPotTask",    // Name of the task
    2048,           // Stack size
    NULL,           // Parameter
    1,              // Priority
    &readPotTaskHandle
  );

}

void loop() {
}