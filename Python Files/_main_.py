

def main():
    """Main entry point of the program."""
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