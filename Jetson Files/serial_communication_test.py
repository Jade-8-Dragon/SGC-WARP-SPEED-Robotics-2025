import serial
import time 

arduino = serial.Serial('/dev/ttyACM0', 9600)  # For Linux

arduino = serial.Serial(
port = '/dev/ttyACM0',  # Update this to your port
baud = 9600,
bytesize = serial.EIGHTBITS,
stopbits - serial.STOPBITS_ONE,
timeout = 5,
xonxoff = False,
rtscts = False,
dsrdtr = False,
writeTimeout = 2,
)

while True:
    try:
        arduino.write('Command from Jetson |'.encode())
        data = arduino.readline()
        if data:
            print(data)
            time.sleep(1)
    except Exception as e:
        print(e)
        arduino.close()
