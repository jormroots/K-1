#from sensor_msgs import NavSatFix, NavSatStatus, TimeRefrence
import serial
import pynmea2

# Open a serial port
ser = serial.Serial('/dev/ttyACM0', 115200)  # Change 'COM1' to the appropriate port and 9600 to the baud rate

try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        #print(ser)
        
        if line.startswith('$GNGGA'):
            try:
                msg = pynmea2.parse(line)
                
                if isinstance(msg, pynmea2.types.talker.GGA):
                    print("Latitude:", msg.latitude)
                    print("Longitude:", msg.longitude)
                    print("Altitude:", msg.altitude)
                    print("Fix Quality:", msg.gps_qual)
                    print("Number of Satellites:", msg.num_sats)
                    
            except pynmea2.ParseError as e:
                print("Parse error: ", e)
except KeyboardInterrupt:
    ser.close()

