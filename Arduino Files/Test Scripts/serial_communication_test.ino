void setup() {
    Serial.begin(9600);

    while (!serial){
        ; // wait for serial port to connect. Needed for native USB port only
    }
}

const char TERMINATOR = '|';

void loop() {
    if (Serial.available() > 0) {
        //chat messageBuffer[32];
        //int size = Serial.readBytesUntil('\n', messageBuffer, 32);
        String commandFromJetson = Serial.readStringUntil(TERMINATOR);

        // confirm
        String ackMsg = "Hello Jetson! This is what I got from you: " + commandFromJetson + TERMINATOR;
        Serial.print(ackMsg);
        //Serial.flush();
    }
    delay(500);

}