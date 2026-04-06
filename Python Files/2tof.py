

def main():
    """Main entry point of the program."""
    import serial 
    import time
    import board
    import RPi.GPIO as GPIO
    import busio
    import digitalio
    import adafruit_vl53l0x
    from hcsr04sensor import sensor
    
    #arduino = serial.Serial('/dev/ttyUSB0', 9600)
    
    
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
   
    i2c = busio.I2C(board.SCL, board.SDA)
    xshut1 = digitalio.DigitalInOut(board.D23)
    xshut2 = digitalio.DigitalInOut(board.D24)
    xshut1.direction=digitalio.Direction.OUTPUT
    xshut2.direction=digitalio.Direction.OUTPUT
    
    xshut1.value = False
    xshut2.value = False
    time.sleep(0.1)
    
    xshut1.value = True
    time.sleep(0.1)
    sensor1 = adafruit_vl53l0x.VL53L0X(i2c)
    sensor1.set_address(0x30)
  
  
    
    xshut2.value = True
    time.sleep(0.1)
    sensor2 = adafruit_vl53l0x.VL53L0X(i2c)
    sensor2.set_address(0x31)
  
    
    
 
    
    
    while True:
       
     
        print(f"1st LIDAR range: {sensor1.range/25.4}in") 
        
        print(f"2nd LIDAR range: {sensor2.range/25.4}in")    

        time.sleep(1)
    
            
          
    pass
if __name__ == "__main__":
    main()
    pass
