import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('COM6', 9600, timeout=1)  # Replace 'COM5' with your Arduino's port
time.sleep(2)  # Allow time for the connection to establish

# Send some commands to Arduino and read the response
arduino.write(b'Y')  # Send a sample command to Arduino
time.sleep(1)  # Wait for Arduino to process
response = arduino.readline().decode('utf-8').strip()  # Read the response from Arduino

print("Arduino response: ", response)

arduino.write(b'Y')  # Send another command to Arduino
time.sleep(1)
response = arduino.readline().decode('utf-8').strip()

print("Arduino response: ", response)

# Close the serial connection
arduino.close()
