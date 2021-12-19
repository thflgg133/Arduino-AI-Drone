void setup() {
  Serial.begin(115200);
  auto_setup();

  drone_setup();
    
}

void loop() {
//  long t1 = micros();
  
  drone_loop();

//  long t2 = micros();
//  Serial.println((t2-t1)/1000.0);

}
