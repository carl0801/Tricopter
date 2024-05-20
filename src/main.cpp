#include <Arduino.h>
#include <FlightControllerMPC.hpp>
#include "freertos/task.h"

const double STEP_TIME = 100; // timeInterval for flight controller

motorData moterValues;

FlightController flightController(STEP_TIME);

void updateMotor(motorData moterValues) {
  Serial.println(moterValues.force_1);
  Serial.println(moterValues.force_2);
  Serial.println(moterValues.force_3);
  Serial.println(moterValues.tilt);
}

void control(){
  //flightController.calculate();
  //motorData data = flightController.getMotorData();
  updateMotor(flightController.calculate());
}

void controlTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(STEP_TIME);
  // Initialise the xLastWakeTime variable with the current time.
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    xTaskDelayUntil( &xLastWakeTime, xFrequency );
    control();
  }
  vTaskDelete(NULL);
}

void setup() {
  delay(1000);
  Serial.begin(115200);

  xTaskCreate(
    controlTask,          // Task function
    "Control", // Name of task
    10000,            // Stack size of task
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL             // Task handle.
  );
}

void loop() {
  // Empty, since we are using FreeRTOS tasks
}
