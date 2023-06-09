/*
 * Example to demonstrate thread definition, semaphores, and thread sleep.
 */
#include <FreeRTOS_TEENSY4.h>

// The LED is attached to pin 13 on the Teensy 4.0
const uint8_t LED_PIN = 13;

// Declare a semaphore handle.
SemaphoreHandle_t sem;
//------------------------------------------------------------------------------
/*
 * Thread 1, turn the LED off when signalled by thread 2.
 */
// Declare the thread function for thread 1.
static void Thread1(void* arg) {
  while (1) {

    Serial.println("Thread 1 : Waiting on Thread 2 to turn LED OFF");

    // Wait for signal from thread 2.
    xSemaphoreTake(sem, portMAX_DELAY);

    Serial.println("Thread 1 : Turning LED OFF");

    // Turn LED off.
    digitalWrite(LED_PIN, LOW);
  }
}
//------------------------------------------------------------------------------
/*
 * Thread 2, turn the LED on and signal thread 1 to turn the LED off.
 */
// Declare the thread function for thread 2.
static void Thread2(void* arg) {

  pinMode(LED_PIN, OUTPUT);

  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);

    Serial.println("Thread 2 : Turning LED ON");

    // Sleep for 200 milliseconds.
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

    Serial.println("Thread 2 : Asking Thread 1 to turn LED OFF");

    // Signal thread 1 to turn LED off.
    xSemaphoreGive(sem);

    // Sleep for 200 milliseconds.
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
  }
}
//------------------------------------------------------------------------------
void setup() {
  portBASE_TYPE s1, s2;

  Serial.begin(9600);

  // initialize semaphore
  sem = xSemaphoreCreateCounting(1, 0);

  // create task at priority two
  s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // create task at priority one
  s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // check for creation errors
  if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
    Serial.println("Creation problem");
    while(1);
  }

  Serial.println("Starting the scheduler !");

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
  // Not used.
}