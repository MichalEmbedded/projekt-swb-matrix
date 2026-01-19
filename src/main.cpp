// ZAD 1
// #include <Arduino.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/timers.h"

// const int LED_PIN    = 10;  
// const int BUTTON_PIN = 11;  

// TimerHandle_t blinkTimer;

// void vBlinkCallback(TimerHandle_t xTimer)
// {
//   (void)xTimer; 
//   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
// }

// void setup()
// {
//   pinMode(LED_PIN, OUTPUT);
//   pinMode(BUTTON_PIN, INPUT_PULLUP); 

//   blinkTimer = xTimerCreate(
//       "Blink",
//       pdMS_TO_TICKS(500),
//       pdTRUE,
//       nullptr,
//       vBlinkCallback
//   );

// }

// void loop()
// {
//   static int lastButton = HIGH;

//   int current = digitalRead(BUTTON_PIN);

//   if (lastButton == HIGH && current == LOW)
//   {
    
//     if (xTimerIsTimerActive(blinkTimer) == pdFALSE)
//     {
//       xTimerStart(blinkTimer, 0);
//     }
//     else
//     {
//       xTimerStop(blinkTimer, 0);
//       digitalWrite(LED_PIN, LOW); 
//     }
//   }

//   lastButton = current;
//   delay(20); 
// }

//ZAD 2 

// #include <Arduino.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/timers.h"

// const int LED_PIN    = 10;  
// const int BUTTON_PIN = 11;  

// TimerHandle_t blinkTimer;

// volatile bool buttonPressed = false;

// void IRAM_ATTR buttonISR()
// {
//     xTimerResetFromISR(blinkTimer, NULL);
// }

// void vBlinkCallback(TimerHandle_t xTimer)
// {
//   int state = digitalRead(BUTTON_PIN);
//   if (state == LOW){
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//   }
// }

// void setup()
// {
//   pinMode(LED_PIN, OUTPUT);
//   pinMode(BUTTON_PIN, INPUT_PULLUP); 
//   attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

//   blinkTimer = xTimerCreate(
//       "Blink",
//       pdMS_TO_TICKS(50),
//       pdFALSE,
//       nullptr,
//       vBlinkCallback
//   );

// }

// void loop()
// {}

// #include <Arduino.h>
// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "timers.h"

// const int LED_PIN = 10; 

// QueueHandle_t xQueueAB;       
// TaskHandle_t taskAHandle;     
// TimerHandle_t timeoutTimer;

// volatile bool waitingForResponse = false;

// void vTimeoutCallback(TimerHandle_t xTimer)
// {
//     if (waitingForResponse)
//     {
//       waitingForResponse = false;
//       xTaskNotifyGive(taskAHandle);
//     }
// }

// void TaskA(void *pvParameters)
// {
//     int zlecenie = 67;

//     for (;;)
//     {
//         Serial.println("Task A: wysyłam zlecenie do Task B");
//         xQueueSend(xQueueAB, &zlecenie, portMAX_DELAY);

//         waitingForResponse = true;
//         xTimerStart(timeoutTimer, 0);

//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//         if (waitingForResponse == true)
//         {
            
//             waitingForResponse = false;
//             xTimerStop(timeoutTimer, 0);
//             Serial.println("Task A: otrzymano odpowiedź od Task B!");
//         }
//         else
//         {
            
//             Serial.println("Task A: TIMEOUT → tryb błąd!");
//             digitalWrite(LED_PIN, HIGH);
//         }

//         vTaskDelay(pdMS_TO_TICKS(3000));
//     }
// }



// void TaskB(void *pvParameters)
// {
//     int odebrane;

//     for (;;)
//     {
//         if (xQueueReceive(xQueueAB, &odebrane, portMAX_DELAY) == pdPASS)
//         {
//             Serial.printf("Task B: otrzymałem zlecenie %d\n", odebrane);

//             vTaskDelay(pdMS_TO_TICKS(1000)); 

//             Serial.println("Task B: wysyłam odpowiedź do Task A");
//             xTaskNotifyGive(taskAHandle);
//         }
//     }
// }

// void setup()
// {
//     Serial.begin(115200);

//     pinMode(LED_PIN, OUTPUT);
//     digitalWrite(LED_PIN, LOW);

//     xQueueAB = xQueueCreate(5, sizeof(int));

//     timeoutTimer = xTimerCreate(
//         "Timeout",
//         pdMS_TO_TICKS(2000),
//         pdFALSE,
//         NULL,
//         vTimeoutCallback
//     );

//     xTaskCreate(TaskA, "TaskA", 4096, NULL, 2, &taskAHandle);
//     xTaskCreate(TaskB, "TaskB", 4096, NULL, 1, NULL);

//     // vTaskStartScheduler();
// }

// void loop() {}
//////////////////////////////////////////////////

#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include <stdio.h>

#define PIN_SER    4
#define PIN_RCLK   5
#define PIN_SRCLK  6

#define TRIG_PIN 17
#define ECHO_PIN 18

#define TRIG_PINX 13
#define ECHO_PINX 12

#define POT_PIN 9

QueueHandle_t sonarXQueue;
QueueHandle_t sonarYQueue;


void shiftOutByte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_SRCLK, LOW);
    digitalWrite(PIN_SER, (data >> i) & 1);
    digitalWrite(PIN_SRCLK, HIGH);
  }
}

void write2(uint8_t rows, uint8_t cols) {
  digitalWrite(PIN_RCLK, LOW);
  shiftOutByte(rows); 
  shiftOutByte(cols);
  digitalWrite(PIN_RCLK, HIGH);
}

uint8_t sonarToCols(float sonarVal) {
  sonarVal = constrain(sonarVal, 10, 100);
  int n = map(sonarVal, 100, 10, 0, 8); 

  uint8_t cols = 0x00;

  for (int i = 0; i < n; i++) {
    cols |= (1 << (7 - i)); 
  }

  return cols;
}

uint8_t sonarToRows(float sonarVal) {
  sonarVal = constrain(sonarVal, 10, 100);
  int n = map(sonarVal, 100, 10, 0, 8); 

  uint8_t rows = 0xFF;

  for (int i = 0; i < n; i++) {
    rows |= (1 << (7 - i)); 
  }

  return rows;
}

void matrixTask(void *pv) {
  int c = 0;
  float sonarXVal;
  float sonarYVal;

  uint8_t cols = 0x00;
  uint8_t rows = 0xFF;
  for (;;) {
    if (xQueueReceive(sonarXQueue, &sonarYVal, portMAX_DELAY) == pdTRUE)
    {
      printf("Odebrano: %d\n", sonarYVal);
        uint8_t rows = 0x00;

        uint8_t cols = sonarToCols(sonarYVal);
        
        for (int k = 0; k < 150; k++) {
          write2(rows, cols);
          delayMicroseconds(800);
        }
     
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    else
    {
        printf("Brak danych w kolejce!\n");
    }
    
  }
}

void sonarTask(void *pv) {
  long duration;
  float sonarYVal;
  for (;;) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0) {
      Serial.println("Brak echa");
    }
    else {
      sonarYVal = duration * 0.0343 / 2.0;

      Serial.print("Odleglosc y: ");
      Serial.print(sonarYVal);
      Serial.println(" cm");
      xQueueOverwrite(sonarXQueue, &sonarYVal);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void sonarXTask(void *pv) {
  long duration;
  float sonarXVal;
  // int potVal;
  for (;;) {
    digitalWrite(TRIG_PINX, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PINX, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINX, LOW);

    duration = pulseIn(ECHO_PINX, HIGH, 30000);

    if (duration == 0) {
      Serial.println("Brak echa");
    }
    else {
      sonarXVal = duration * 0.0343 / 2.0;

      Serial.print("Odleglosc x: ");
      Serial.print(sonarXVal);
      Serial.println(" cm");
      xQueueOverwrite(sonarXQueue, &sonarXVal);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println("HC-SR04 start");
  pinMode(PIN_SER, OUTPUT);
  pinMode(PIN_RCLK, OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  sonarXQueue = xQueueCreate(1, sizeof(float));
  sonarYQueue = xQueueCreate(1, sizeof(float));
  if (sonarXQueue != NULL){
    xTaskCreatePinnedToCore(matrixTask, "TaskA", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sonarTask, "TaskB", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sonarXTask, "TaskB", 2048, NULL, 1, NULL, 1);
  } else {
    Serial.print("Nie udało się  utworzyć kolejki");
  }
}

void loop() {

}

