

import os
import time

def main():
    pipe_path = "/tmp/camera_pipe"
    
    if not os.path.exists(pipe_path):
        os.mkfifo(pipe_path)

    while True:
        #opens the pipe and waits for C++ to send something
        with open(pipe_path, 'r') as fifo:
            for line in fifo:
                try:
                    angle_val = int(line.strip())
                    
                    sendAngle(angle_val)
                except ValueError:
                    continue
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

def sendAngle(angle):
    # Convert angle to 2-byte signed big-endian
    angleBytes = int(angle).to_bytes(2, byteorder='big', signed=True)
    packet = b'\x3C' + angleBytes + b'\x3E'
    arduino.write(packet)
    print(f"Angle Packet Sent: {packet.hex()} ({angle} degrees)")
    pass