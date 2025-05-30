# ##############################################################################
# # Imports
# ##############################################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
import serial
import struct
import threading
import math
import time
import matplotlib.pyplot as plt
from datetime import datetime
import signal
import sys
import os


# ##############################################################################
# # Configuration
# ##############################################################################

# ............................................................................
# Configure the serial connection ............................................

serial_port = '/dev/ttyUSB0'  # Replace with your device path
baud_rate = 115200  # Match the baud rate
bytesize=serial.EIGHTBITS,  # Data bits
parity=serial.PARITY_NONE,  # Parity
stopbits=serial.STOPBITS_ONE,  # Stop bits
xonxoff=False,        # Software flow control
rtscts=True           # Hardware flow control
timeout = 0.0  # Timeout in seconds for reading

# ............................................................................
# Define the packet format ...................................................

# Little-endian: 3-char string, 1-byte, 4-byte uint, 4 doubles, 2 bytes
PACKET_FORMAT = '<3sBIdddddd'

# Calculate the size of the packet (42 bytes)
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  


##############################################################################
# Classes
##############################################################################

# ............................................................................
# Coordinate logging and Graph drawing .......................................

# class CoordinateLogger:
#     def __init__(self, source_filenames=['odom', 'amcl', 'tf']):
#         self.source_filenames = source_filenames
#         self.coordinates = {source: [] for source in source_filenames}
#         self.running = True

#         # Get the current date and time
#         now = datetime.now()
#         formatted_time = now.strftime("%Y.%m.%d-%H:%M")

#         # Create the directory
#         try:
#             os.mkdir("/data/coordinate_log/")
#             print(f"Directory 'coordinate_log' created successfully.")
#         except FileExistsError:
#             print(f"Directory 'coordinate_log' already exists.")
#         try:
#             os.mkdir("/data/coordinate_log/"+formatted_time)
#             print(f"Directory '{formatted_time}' created successfully.")
#         except FileExistsError:
#             print(f"Directory '{formatted_time}' already exists.")

#         # Create new files for each source
#         self.path = "/data/coordinate_log/"+formatted_time+"/"
#         for filename in self.source_filenames:
#             with open(self.path+filename+".csv", 'w') as f:
#                 f.write("Time, X, Y, Angle\n")

#         # Set up signal handler for Ctrl+C                  # Doesn't work from docker...
#         signal.signal(signal.SIGINT, self.signal_handler)   # Doesn't work from docker...


#     def signal_handler(self, sig, frame):
#         self.running = False
#         self.plot_graphs()


#     def add_coordinates(self, source, x, y, angle):
        
#         # Take time
#         current_time = time.time()

#         # Store coordinates
#         # self.coordinates[source].append((current_time, x, y, angle))

#         # Write to the corresponding file
#         with open(self.path+source+'.csv', 'a') as f:
#             f.write(f"{current_time}, {x}, {y}, {angle}\n")


#     def plot_graphs(self):
#         print(f"Drawing graphs...")
#         plt.figure(figsize=(12, 8))

#         for source in self.source_filenames:
#             times = [coord[0] for coord in self.coordinates[source]]
#             x_coords = [coord[1] for coord in self.coordinates[source]]
#             y_coords = [coord[2] for coord in self.coordinates[source]]
#             angle = [coord[3] for coord in self.coordinates[source]]

#             # Plot X vs Time
#             plt.subplot(3, 1, 1)
#             plt.plot(times, x_coords, label=f'{source} X')
        
#             # Plot Y vs Time
#             plt.subplot(3, 1, 2)
#             plt.plot(times, y_coords, label=f'{source} Y')

#             # Plot X vs Y
#             plt.subplot(3, 1, 3)
#             plt.plot(x_coords, y_coords, label=f'{source}')

#         plt.subplot(3, 1, 1)
#         plt.title('X Coordinates Over Time')
#         plt.xlabel('Time (s)')
#         plt.ylabel('X Coordinate')
#         plt.legend()

#         plt.subplot(3, 1, 2)
#         plt.title('Y Coordinates Over Time')
#         plt.xlabel('Time (s)')
#         plt.ylabel('Y Coordinate')
#         plt.legend()

#         plt.subplot(3, 1, 3)
#         plt.title('X vs Y Coordinates')
#         plt.xlabel('X Coordinate')
#         plt.ylabel('Y Coordinate')
#         plt.legend()

#         plt.tight_layout()

#         # Plot graphs doesn't works from docker now (need to do smth)
#         # plt.show()

#         # Save the plot to a file instead of showing it
#         plt.savefig(self.path + 'coordinates_plot.png')  # Save as PNG file
#         plt.close()  # Close the figure to free memory

#         print(f"END")


# ............................................................................
# SerialROSNode ..............................................................

class SerialROSNode(Node):
    def __init__(self):
        super().__init__('serial_ros_node')

        # # Pub and sub
        # self.pub_odom = self.create_publisher(Odometry, 'odom', 1)
        # self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_listener_callback, 1)
        # self.sub_tf = self.create_subscription(PoseWithCovarianceStamped, 'tf', self.tf_listener_callback, 1)
        # self.tf_broadcaster = TransformBroadcaster(self)
        # self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        # self.joint_state = JointState()
        # self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']  # Replace with your joint names
        # self.joint_state.position = [0.0, 0.0]  # Initial positions
        # self.joint_state.velocity = [0.0, 0.0]  # Initial velocity

        # # To set odom to zero
        # self.save_odom = False
        # self.lat = 0.0
        # self.lon = 0.0

        # # Coordinate Logger
        # self.coordinate_logger = CoordinateLogger()
        # # self.i = 0  # each 'i' msgs it save pragh.png
        
        # # Serial msg variables
        # self.prefix = 'Slm'
        # self.length = 42
        # self.packet_number = 0
        # self.time = 0.0

        # Serial connection
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                xonxoff=xonxoff,
                rtscts=rtscts
            )
            self.get_logger().info(f"Connected to {serial_port} at {baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            raise
        
        # Thread locks and flags
        self.running = True
        # self.serial_lock = threading.Lock()

    # ........................................................................
    # Read ...................................................................

    def read(self):
        """Thread 1: Read from serial, convert to ROS Odometry, and publish."""
        while self.running:


            # Set a timeout (in seconds) if not already set
            self.ser.timeout = 2.0  # 2 seconds timeout, adjust as needed
            
            start_time = time.time()
            raw_data = self.ser.read(PACKET_SIZE)
            # raw_data = ser.read(ser.in_waiting)  # or use ser.read(size) for specific size

            packet_size = len(raw_data)  # Size in bytes
            print(f"packet_size: {packet_size}, PACKET_SIZE: {PACKET_SIZE}")
            
            elapsed_time = time.time() - start_time
            if elapsed_time > 1.5:  # Threshold in seconds
                print(f"Warning: Serial read took too long: {elapsed_time:.2f} seconds")

            # if len(raw_data) == PACKET_SIZE:

            # Unpack the binary data. Decode the prefix (it's a byte string)
            prefix, length, packet_number, time_, latitude, longitude, angle, v1, v2 = struct.unpack(PACKET_FORMAT, raw_data)
            prefix = prefix.decode('ascii')
            

            print(f"prefix: {prefix}, time delta: {round((time.time()-time_), 2)}, X={round(latitude, 2)}, Y={round(longitude, 2)}, Angle={round(angle, 1)}")

            # try:
                

            #     # with self.serial_lock:

            #     # Set a timeout (in seconds) if not already set
            #     self.ser.timeout = 2.0  # 2 seconds timeout, adjust as needed
                
            #     start_time = time.time()
            #     raw_data = self.ser.read(PACKET_SIZE)

            #     packet_size = len(raw_data)  # Size in bytes
            #     print(f"packet_size: {packet_size}, PACKET_SIZE: {PACKET_SIZE}")
                
            #     elapsed_time = time.time() - start_time
            #     if elapsed_time > 1.5:  # Threshold in seconds
            #         print(f"Warning: Serial read took too long: {elapsed_time:.2f} seconds")

                    

            #     if len(raw_data) == PACKET_SIZE:
            #         # Unpack the binary data. Decode the prefix (it's a byte string)
            #         # PACKET_FORMAT = '<3sBIdddd'
            #         prefix, length, packet_number, time_, latitude, longitude, angle_direct, v1, v2 = struct.unpack(PACKET_FORMAT, raw_data)
            #         # prefix, length, packet_number, time_, latitude, longitude, angle_direct = struct.unpack(PACKET_FORMAT, raw_data)
            #         prefix = prefix.decode('ascii')

            #         if (prefix == 'Slm'):
                    
            #             if(not self.save_odom):
            #                 self.lat = -latitude
            #                 self.lon = longitude
            #                 self.angle = angle_direct
            #                 self.save_odom = True
            #             else:
            #                 latitude = -latitude
            #                 latitude -= self.lat
            #                 longitude -= self.lon
            #                 # angle_direct -= self.angle
            #                 angle_direct += 90.0 # correction

                            

            #                 # Create Odometry message
            #                 msg = Odometry()
            #                 msg.header.stamp = self.get_clock().now().to_msg()
            #                 msg.header.frame_id = "odom"
            #                 msg.child_frame_id = "base_footprint"
            #                 msg.pose.pose.position.x = round(latitude, 3)
            #                 msg.pose.pose.position.y = round(longitude, 3)
            #                 msg.pose.pose.position.z = 0.0
            #                 # Simple conversion of angle_direct to quaternion (assuming angle in radians)
            #                 angle_deg = round(angle_direct, 3)

                            

            #                 qx, qy, qz, qw = self.angle_to_quaternion(math.radians(angle_deg))
            #                 msg.pose.pose.orientation.z = qz
            #                 msg.pose.pose.orientation.w = qw

                            

            #                 self.coordinate_logger.add_coordinates("odom", msg.pose.pose.position.x, msg.pose.pose.position.y, angle_deg)

                            

            #                 # Publish to odom topic
            #                 self.pub_odom.publish(msg)
            #                 self.get_logger().info(f"prefix: {prefix}, time delta: {round((time.time()-time_), 2)}, X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}, Angle={round(angle_deg, 1)}")
                            

                            


            #                 # Create and publish the transform
            #                 t = TransformStamped()
            #                 t.header.stamp = msg.header.stamp
            #                 t.header.frame_id = 'odom'
            #                 t.child_frame_id = 'base_footprint'
            #                 t.transform.translation.x = msg.pose.pose.position.x
            #                 t.transform.translation.y = msg.pose.pose.position.y
            #                 t.transform.translation.z = 0.0
            #                 t.transform.rotation = msg.pose.pose.orientation
            #                 self.tf_broadcaster.sendTransform(t)
                            
            #                 # Update joint positions here (e.g., based on sensor data or control commands)
            #                 #self.joint_state.position[0] += 0.01  # Example increment
            #                 #self.joint_state.position[1] += 0.02  # Example increment
            #                 self.joint_state.header.stamp = msg.header.stamp
            #                 self.publisher_.publish(self.joint_state)
            #                 #self.get_logger().info(f'Publishing: {self.joint_state}')

            #         else:
            #             self.get_logger().warning(f"Prefix: '{prefix}'")

                

            # except serial.SerialException as e:
            #     self.get_logger().error(f"Serial read error: {e}")
            #     break
            # except struct.error as e:
            #     self.get_logger().error(f"Unpacking error: {e}")
            #     continue
            # except UnicodeDecodeError as e:
            #     self.get_logger().error(f"UnicodeDecodeError: {e}")
            #     self.get_logger().error(f"Raw data: {raw_data.hex()}")
            #     # print(f"Raw data: {raw_data.hex()}")
            #     continue

            # # time.sleep(0.01)  # Small delay to prevent CPU overload


    # ........................................................................
    # Transmit ...............................................................

    def pose_listener_callback(self, msg):
        """Read from ros, convert, and write back."""

        #self.get_logger().info('I heard: "%s"' % msg)

        if self.running:
            try:
                self.packet_number += 1

                latitude = msg.pose.pose.position.x
                longitude = msg.pose.pose.position.y

                # Converting quaternion back to angle
                angle_direct = self.quaternion_to_angle(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                )

                # Pack without checksums first to calculate them
                temp_packet = struct.pack(
                    PACKET_FORMAT[:-2],  # Exclude checksum fields
                    self.prefix.encode('ascii'),
                    self.length,
                    self.packet_number,
                    self.time,
                    latitude,
                    longitude,
                    angle_direct
                )
                chksum_a, chksum_b = self.calculate_checksum(temp_packet)
                
                # Pack the modified data
                modified_packet = struct.pack(
                    PACKET_FORMAT,
                    self.prefix.encode('ascii'),
                    self.length,
                    self.packet_number,
                    self.time,
                    latitude,
                    longitude,
                    angle_direct,
                    chksum_a,
                    chksum_b
                )

                # Write back to serial
                with self.serial_lock:
                    self.ser.write(modified_packet)
                #self.get_logger().info(f"Sent Modified Packet with AngleDirect: {angle_direct}")

                # Coordinate logger
                self.coordinate_logger.add_coordinates("amcl", latitude, longitude, angle_direct)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
            except struct.error as e:
                self.get_logger().error(f"Unpacking error: {e}")


    def tf_listener_callback(self, msg):

        x = msg.transform.translation
        y = msg.transform.translation
        quaternion = msg.transform.rotation

        # Converting quaternion back to angle
        angle_direct = self.quaternion_to_angle(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        )

        # Coordinate logger
        self.coordinate_logger.add_coordinates("tf", x, y, angle_direct)


    def calculate_checksum(self, data):
        """Simple example: sum of all bytes modulo 256."""
        return sum(data) % 256, (sum(data) >> 8) % 256  # Two 8-bit checksums


    def angle_to_quaternion(self, yaw):
        """
        Convert an angle in radians (yaw, around Z-axis) to a quaternion.
        
        Args:
            yaw (float): Angle in radians.
        
        Returns:
            tuple: (x, y, z, w) representing the quaternion.
        """
        # Quaternion components for rotation around Z-axis
        x = 0.0
        y = 0.0
        z = math.sin(yaw / 2.0)
        w = math.cos(yaw / 2.0)
        
        return (x, y, z, w)
    

    def quaternion_to_angle(self, x, y, z, w):
        """
        Convert a quaternion to an angle in radians (yaw, around Z-axis).
        
        Args:
            x (float): Quaternion x component.
            y (float): Quaternion y component.
            z (float): Quaternion z component.
            w (float): Quaternion w component.
        
        Returns:
            float: Yaw angle in radians.
        """
        # Normalize quaternion to avoid numerical errors (optional but recommended)
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return 0.0  # Avoid division by zero
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        
        # Extract yaw (assuming rotation is only around Z-axis)
        yaw = 2.0 * math.atan2(z, w)
        
        # Ensure yaw is in range [-pi, pi]
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi
        
        return yaw

    
########################################################
# Shutdown
########################################################

    def shutdown(self):
        """Clean up resources."""
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.destroy_node()


########################################################
########################################################
# Main
########################################################
########################################################


def main(args=None):
    rclpy.init(args=args)
    node = SerialROSNode()

    # Start thread for parallel processing
    # read_thread = threading.Thread(target=node.read, daemon=True)
    # read_thread.start()

    node.read()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.get_logger().info("Shutting down...")
    # finally:
    #     node.shutdown()
    #     rclpy.shutdown()

    # Ensure threads are joined properly
    # read_thread.join()

if __name__ == '__main__':
    main()















# import serial
# import struct
# import time

# # Configure the serial connection
# serial_port = '/dev/ttyUSB0'  # Replace with your device path
# baud_rate = 115200  # Match the baud rate
# bytesize=serial.EIGHTBITS,  # Data bits
# parity=serial.PARITY_NONE,  # Parity
# stopbits=serial.STOPBITS_ONE,  # Stop bits
# timeout=1,            # Timeout in seconds
# xonxoff=False,        # Software flow control
# rtscts=True          # Hardware flow control
# timeout = 0  # Timeout in seconds for reading

# # Define the packet format
# PACKET_FORMAT = '<3sBIdddddd'  # Little-endian: 3-char string, 1-byte, 4-byte uint, 4 doubles, 2 bytes
# PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # Calculate the size of the packet (42 bytes)


# def main(args=None):
    
#     try:
#         # Open the serial port
#         with serial.Serial(
#             port=serial_port,
#             baudrate=baud_rate,
#             bytesize=serial.EIGHTBITS,
#             parity=serial.PARITY_NONE,
#             stopbits=serial.STOPBITS_ONE,
#             timeout=timeout,
#             xonxoff=False,  # Software flow control
#             rtscts=False    # Hardware flow control
#         ) as ser:
#             print(f"Connected to {serial_port} at {baud_rate} baud.")
            
#             while True:

#                 loop_start = time.time()

#                 # Set a timeout (in seconds) if not already set
#                 ser.timeout = 2.0  # 2 seconds timeout, adjust as needed
                
#                 start_time = time.time()
#                 raw_data = ser.read(PACKET_SIZE)
#                 # raw_data = ser.read(ser.in_waiting)  # or use ser.read(size) for specific size

#                 packet_size = len(raw_data)  # Size in bytes
#                 print(f"packet_size: {packet_size}, PACKET_SIZE: {PACKET_SIZE}")
                
#                 elapsed_time = time.time() - start_time
#                 if elapsed_time > 1.5:  # Threshold in seconds
#                     print(f"Warning: Serial read took too long: {elapsed_time:.2f} seconds")

#                 # if len(raw_data) == PACKET_SIZE:

#                 # Unpack the binary data. Decode the prefix (it's a byte string)
#                 prefix, length, packet_number, time_, latitude, longitude, angle, v1, v2 = struct.unpack(PACKET_FORMAT, raw_data)
#                 prefix = prefix.decode('ascii')
                

#                 print(f"prefix: {prefix}, time delta: {round((time.time()-time_), 2)}, X={round(latitude, 2)}, Y={round(longitude, 2)}, Angle={round(angle, 1)}")

#                 # time.sleep(0.05)


#                 loop_time = time.time() - loop_start
#                 print(f"Loop time: {loop_time:.3f} seconds")
                                                

#     except serial.SerialException as e:
#         print(f"Serial error: {e}")
#     except KeyboardInterrupt:
#         print("\nExiting...")



# if __name__ == '__main__':
#     main()