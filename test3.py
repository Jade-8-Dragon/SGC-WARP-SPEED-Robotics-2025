
import time
import math
import board
import adafruit_icm20x
import adafruit_vl53l0x

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)
#defining tolerance
tolerance = 30 #in mm
#sensor 1
sensor1 = adafruit_vl53l0x.VL53L0X(i2c)
sensor1.measurement_timing_budget(200000) #in ms
#sensor 2
sensor2 = adafruit_vl53l0x.VL53L0X(i2c)
sensor2.measurement_timing_budget(200000) #in ms

print("test3")

y_a = tan(45)*sensor1.range
y_s = sensor2.range

#if abs(y_a-y_s) > tolerance:
    #plan movement/find new path
#else:
    #keep on current movement plan

print(y_a)



    