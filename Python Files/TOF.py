

def main():
    """Main entry point of the program."""
    import serial 
    import time
    import board
    import RPi.GPIO as GPIO
    import busio
    import adafruit_vl53l0x
    from hcsr04sensor import sensor
    
    #arduino = serial.Serial('/dev/ttyUSB0', 9600)
    
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO_TRIGGER = 17
    GPIO_ECHO = 27
    
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
    GPIO.setup(GPIO_ECHO,GPIO.IN)
    
    GPIO.output(GPIO_TRIGGER, False)
    
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    
    print("initializing ultrasonic sensor")
    time.sleep(2)
    while True:
       
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(.00001)
        GPIO.output(GPIO_TRIGGER, False)
      
        while GPIO.input(GPIO_ECHO) ==0:
            start = time.time()
            
        while GPIO.input(GPIO_ECHO) ==1:
            stop = time.time()
            
            
        elapsed = stop-start
        #print(elapsed, "s")
        print( "Ultrasonic Measurement: ",elapsed*17150*.393701, "in")
        print(f"LIDAR range: {vl53.range/25.4}in")    

        time.sleep(1)
    
            
          
    pass
    


if __name__ == "__main__":
    main()
pass

def movementParameters(forwardAmount, turnAmount, liftHeight):
    # Convert parameters to bytes and send to Arduino
    forwardAmountBytes = forwardAmount.to_bytes(2, byteorder='big', signed=True)
    turnAmountBytes = turnAmount.to_bytes(2, byteorder='big', signed=True)
    liftHeightBytes = liftHeight.to_bytes(2, byteorder='big', signed=True)
    arduino.write( 0x3C + forwardAmountBytes + turnAmountBytes + liftHeightBytes + 0x3E)
    pass
