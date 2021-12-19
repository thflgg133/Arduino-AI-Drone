TaskHandle_t TaskAutoAviHandle;

void auto_setup() {
  xTaskCreatePinnedToCore(
    TaskAutoAviMain,   /* Task function. */
    "TaskAutoAvi",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &TaskAutoAviHandle,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */

  delay(2000); 
}

extern double tAngleX, tAngleY;
extern int throttle;
int data_sampling_en = 0;
int data_streaming_en = 0;
void TaskAutoAviMain( void * pvParameters ){

  pinMode(0, INPUT);

  while(true) {
    int pinState = digitalRead(0);
    if(pinState == LOW) break;
  }  
  
  data_sampling_en = 1;

  for(int thr = 0; thr < 700; thr++) {
    throttle = thr;
    delay(4);
  }

  for(int thr = 700; thr > 400; thr--) {
    throttle = thr;
    delay(9);
  }

  for(int thr = 400; thr > 0; thr--) {
    throttle = thr;
    delay(5);
  }

  throttle = 0;
  
  data_sampling_en = 0;
  
  data_streaming_en = 1;

  vTaskDelete( TaskAutoAviHandle );
  
}
