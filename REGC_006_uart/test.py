import serial

#find the connected serial port otomaticly
import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p)
    if "USB Serial Port" in p.description:
        ser = serial.Serial(p.device, 9600)
        print("Connected to " + p.device)
        break
   
def detect_auto_baudrate(port):
    baudrates = [9600, 19200, 38400, 57600, 115200]  # List of possible baudrates to test

    for baudrate in baudrates:
        ser = serial.Serial(port, baudrate)
        ser.timeout = 1  # Set a timeout value for reading from the serial port

        # Send a test message
        ser.write(b'U')  # You can modify this message according to your needs

        # Try to read a response
        response = ser.read(1)

        if response == b'A':  # You can modify this condition based on the expected response
            print(f"Detected baudrate: {baudrate}")
            ser.close()
            return baudrate

        ser.close()

    #print("Baudrate detection failed")
    return None


# Usage example
port = '/dev/cu.usbmodem1103'  # Replace with the actual port name
baudrate = detect_auto_baudrate(port)
while baudrate ==None:
    baudrate = detect_auto_baudrate(port)
if baudrate:
    print(f"Auto-detected baudrate: {baudrate}")
