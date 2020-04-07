  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);
}


void loop() {
//Serial.println(Serial.available());
  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
}
