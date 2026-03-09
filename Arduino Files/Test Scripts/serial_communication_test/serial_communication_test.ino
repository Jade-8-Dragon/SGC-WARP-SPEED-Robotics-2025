void setup() {
    Serial.begin(115200);

    while (!Serial){
        ; // wait for serial port to connect. Needed for native USB port only
    }
}

const char TERMINATOR = '|';

void loop() {
    if (Serial.available() > 0) {
        //chat messageBuffer[32];
        //int size = Serial.readBytesUntil('\n', messageBuffer, 32);
        uint16_t commandFromJetson = Serial.read());
        uint8_t 1111111

        // confirm
        String ackMsg = "Hello Jetson! This is what I got from you: " + commandFromJetson;
        Serial.print(ackMsg);
        //Serial.flush();


        fwardamount = 
    }
    delay(500);

}
