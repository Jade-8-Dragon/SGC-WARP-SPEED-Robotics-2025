
import time
import board
import adafruit_icm20x

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)

print("test")

while True:

    #print("Acell: X: %.2f, Y: %.2f, Z:%.2f m/s^2" % (icm.acceleration))
    #print("Gyro: X: %.2f, Y: %.2f, Z:%.2f m/s^2" % (icm.gyro))
    #print("Magnet: X: %.2f, Y: %.2f, Z:%.2f m/s^2" % (icm.magnetic))

    oldXAcell = icm.acceleration[0]
    oldYAcell = icm.acceleration[1]
    oldZAcell = icm.acceleration[2]
    time.sleep(1)
    #print(icm.gyro[2])

    #yaw
    turningTolerance = .1
    if(abs(icm.gyro[2] > turningTolerance )):
        print("Yawing left!")
    if(icm.gyro[2] < -turningTolerance ):
        print("Yawing right!")

    #pitch
    if(abs(icm.gyro[1] > turningTolerance )):
        print("Pitching Up!")
    if(icm.gyro[1] < -turningTolerance ):
        print("Pitching Down!")

    #roll
        
    if(abs(icm.gyro[0] > turningTolerance )):
        print("Rolling left!")
    if(icm.gyro[0] < -turningTolerance ):
        print("Rolling right!")

    #movement analysis
    
    newXAcell = icm.acceleration[0]
    newYAcell = icm.acceleration[1]
    newZAcell = icm.acceleration[2]

    movementTolerance = .1
    if(abs(newXAcell - oldXAcell) > movementTolerance) and (newXAcell > oldXAcell):
        print("Moving backwards!")


    
        
    
