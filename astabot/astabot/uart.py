import serial
import struct

# Configure the serial connection
serial_port = '/dev/ttyUSB0'  # Replace with your device path
baud_rate = 115200  # Match the baud rate
bytesize=serial.EIGHTBITS,  # Data bits
parity=serial.PARITY_NONE,  # Parity
stopbits=serial.STOPBITS_ONE,  # Stop bits
timeout=1,            # Timeout in seconds
xonxoff=False,        # Software flow control
rtscts=True          # Hardware flow control
timeout = 0  # Timeout in seconds for reading

# Define the packet format
PACKET_FORMAT = '<3sBIddddddBB'  # Little-endian: 3-char string, 1-byte, 4-byte uint, 4 doubles, 2 bytes
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # Calculate the size of the packet (42 bytes)

try:
    # Open the serial port
    with serial.Serial(
        port=serial_port,
        baudrate=baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        xonxoff=False,  # Software flow control
        rtscts=False    # Hardware flow control
    ) as ser:
        print(f"Connected to {serial_port} at {baud_rate} baud.")
        
        buffer = b''  # Buffer to store incoming data
        
        while True:
            # Read the exact number of bytes for one packet
            raw_data = ser.read(PACKET_SIZE)
            
            if len(raw_data) == PACKET_SIZE:  # Ensure we received a full packet
                # Unpack the binary data into fields
                prefix, length, packet_number, time, latitude, longitude, angle_direct, v1, v2, chksum_a, chksum_b = struct.unpack(PACKET_FORMAT, raw_data)
                
                # Decode the prefix (it's a byte string)
                prefix = prefix.decode('ascii')
                
                # Print the parsed data

                print(f"Prefix: {prefix}, Length: {length}, PacketNumber: {packet_number}, Time: {time}")
                # print(f"Latitude: {latitude}, Longitude: {longitude}, AngleDirect: {angle_direct}")
                # print(f"Checksum A: {chksum_a}, Checksum B: {chksum_b}")
                
                # Modify the AngleDirect field
                angle_direct += 1
                
                # Pack the modified data back into binary format
                modified_packet = struct.pack(
                    PACKET_FORMAT,
                    prefix.encode('ascii'),  # Prefix as bytes
                    length,
                    packet_number,
                    time,
                    latitude,
                    longitude,
                    angle_direct,
                    chksum_a,
                    chksum_b
                )
                
                # Send the modified packet back to the device
                ser.write(modified_packet)
                print(f"Sent Modified Packet with AngleDirect: {angle_direct}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nExiting...")