

def main():
    """Main entry point of the program."""
    import time
    import RPi.GPIO as GPIO
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO_TRIGGER = 17
    GPIO_ECHO = 27
    
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
    GPIO.setup(GPIO_ECHO,GPIO.IN)
    
    GPIO.output(GPIO_TRIGGER, False)
    

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
            

        time.sleep(.1)
    
if __name__ == "__main__":
    main()
