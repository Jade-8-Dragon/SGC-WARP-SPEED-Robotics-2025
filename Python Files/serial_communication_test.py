import serial
import time 
#arduino = serial.Serial('/dev/ttyUSB0', 9600)  # For Linux

arduino = serial.Serial(
port = '/dev/ttyUSB0',  # Update this to your port
baudrate = 115200,
#bytesize = serial.EIGHTBITS,
#Stopbits - serial.STOPBITS_ONE,
timeout = 5,
#xonxoff = False,
#rtscts = False,
#dsrdtr = False,
#writeTimeout = 2,
)

while True:
    try:
        arduino.write('Command from Jetson |'.encode())
        data = arduino.readline()
        if data:
            print(data.decode('ascii'))
            time.sleep(.1)
    except Exception as e:
        print(e)
        arduino.close()
