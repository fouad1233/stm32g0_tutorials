import serial
import serial.tools.list_ports
import time
import numpy as np
import matplotlib.pyplot as plt
def find_stm32_serial_port():
    available_ports = list(serial.tools.list_ports.comports())

    for port in available_ports:
        if "STM32" in port.description or "VID:PID" in port.hwid:
            return port.device

    return None

def main():
    adcValues = []
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
            while(ser.in_waiting > 0):
                data = ser.readline()
                data = int(ser.readline().decode('utf-8').rstrip())
                print(f"Received: {data}")
                #data = ser.read(2)
                ser.reset_input_buffer()
                
                #print(data_int)
                
                #plot the received data with system time on x-axis and data(Sound sensor adc) on y-axis
                #make it as a real time plot like animation with 1 ms interval
                #use matplotlib.pyplot and numpy libraries
                #every time plot the last 100 data points
                #in every new data received delete the first data point and append the new data point to the end of the array
                #use plt.plot() and plt.draw() functions
                
                #please write for me the code 
                
                adcValues.append(data)
                plt.plot(adcValues)
                plt.draw()
                plt.pause(0.000000001)
                plt.clf()
                
                if len(adcValues) > 100:
                    adcValues.pop(0)
                #clear serial buffer
                
                
                
                
                
                
                
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        ser.close()

if __name__ == "__main__":
    readLine = False
    main()