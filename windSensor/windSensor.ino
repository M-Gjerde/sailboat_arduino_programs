int WSPin = A0;


void setup() {
    pinMode(A0, OUTPUT);
    Serial.begin(9600);
}


void loop() {

    Serial.print(A0);

}
