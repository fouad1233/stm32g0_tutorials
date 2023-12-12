import serial
import serial.tools.list_ports
import time
def find_stm32_serial_port():
    available_ports = list(serial.tools.list_ports.comports())

    for port in available_ports:
        if "STM32" in port.description or "VID:PID" in port.hwid:
            return port.device

    return None

def main():
    serial_port = find_stm32_serial_port()

    if serial_port is None:
        print("STM32F4 serial port not found. Please check the connection.")
        return

    print(f"Found STM32F4 serial port at {serial_port}")

    baud_rate = 9600  # Should match the baud rate configured on the STM32F4

    try:
        ser = serial.Serial(serial_port, baud_rate)
    except serial.SerialException:
        print(f"Failed to open {serial_port}. Make sure it's available and not in use.")
        return

    try:
        while True:
            #send a character to the microcontroller
            sendedData = input("Enter a character: ")
            #send the data
            isSended = ser.write(sendedData.encode())
            time.sleep(0.01)
            print(f"Sent: {sendedData}")
            while(ser.in_waiting > 0):
                #0.1 ms delay
                time.sleep(0.001)
                if readLine == False:
                    # Read one byte from the serial port
                    received_byte = ser.read()
                    if len(received_byte) > 0:
                        # Interpret the received byte as an 8-bit integer
                        received_value = ord(received_byte)

                        # Print the received value
                        print(f"Received: {chr(received_value)}")
                else:
                    received_data = ser.readline().decode('utf-8').rstrip()
                    print(f"Received: {received_data}")
                    
                
                
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        ser.close()

if __name__ == "__main__":
    readLine = False
    main()