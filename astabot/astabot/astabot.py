# ##############################################################################
# # Imports
# ##############################################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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

serial_port = '/dev/ttyUSB0'    # Replace with your device path
baud_rate = 115200              # Match the baud rate
bytesize=serial.EIGHTBITS       # Data bits
parity=serial.PARITY_NONE       # Parity
stopbits=serial.STOPBITS_ONE    # Stop bits
xonxoff=False                   # Software flow control
rtscts=False                    # Hardware flow control
timeout = 0                     # Timeout in seconds for reading

# ............................................................................
# Define the packet format ...................................................

# Little-endian: 3-char string, 1-byte, 4-byte uint, 6 doubles
# PACKET_FORMAT = '<3sBIdddddd'
# PACKET_FORMAT_RETURN = '<3sBIdddd'

serial_port = os.environ.get('SERIAL_PORT', '/dev/ttyUSB0')
baud_rate = os.environ.get('BAUD_RATE', 115200)
PACKET_FORMAT = os.environ.get('PACKET_FORMAT', '<3sBIdddddd')
PACKET_FORMAT_RETURN = os.environ.get('PACKET_FORMAT_RETURN', '<3sBIdddd')

# Calculate the size of the packet (56 bytes)
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  

# ............................................................................
# Axes transform parameters ..................................................

# X_DIRECTION = -1    # 1 or -1
# Y_DIRECTION = 1     # 1 or -1
# X_DELTA = 2.4 # 2.0       # meters
# Y_DELTA = 5.6 # 21.59     # meters
# ANGLE_DELTA = 270   # in degrees

def str_to_bool(value):
    if isinstance(value, str):
        return value.lower() in ('True', 'true', '1', 't', 'y', 'yes')
    return bool(value)

X_Y_FLIP = str_to_bool(os.environ.get('X_Y_FLIP', False))    # True or False
X_DIRECTION = float(os.environ.get('X_DIRECTION', 1))    # 1 or -1
Y_DIRECTION = float(os.environ.get('Y_DIRECTION', 1))    # 1 or -1
X_DELTA = float(os.environ.get('X_DELTA', 0))            # meters
Y_DELTA = float(os.environ.get('Y_DELTA', 0))            # meters
ANGLE_DELTA = float(os.environ.get('ANGLE_DELTA', 270))  # degrees

# ............................................................................
# Other parameters ...........................................................

# SEND_LOCALIZATION = False
# VALID_DELTA_X = 1.0
# VALID_DELTA_Y = 1.0

SEND_LOCALIZATION = str_to_bool(os.environ.get('SEND_LOCALIZATION', True))
VALID_DELTA_X = float(os.environ.get('VALID_DELTA_X', 1.0))  # meters
VALID_DELTA_Y = float(os.environ.get('VALID_DELTA_Y', 1.0))  # meters

# ............................................................................
# Print configuration ........................................................

print(f"##############################################################################")
print(f"# Configuration")
print(f"##############################################################################")
print(f"")

print(f"# Serial settings")
print(f"PACKET_FORMAT: {PACKET_FORMAT}")
print(f"PACKET_FORMAT_RETURN: {PACKET_FORMAT_RETURN}")
print(f"")

print(f"# Axes transform")
print(f"X_Y_FLIP: {X_Y_FLIP}")
print(f"X_DIRECTION: {X_DIRECTION}")
print(f"Y_DIRECTION: {Y_DIRECTION}")
print(f"X_DELTA: {X_DELTA}")
print(f"Y_DELTA: {Y_DELTA}")
print(f"ANGLE_DELTA: {ANGLE_DELTA}")
print(f"")

print(f"# Sending localization settings")
print(f"SEND_LOCALIZATION: {SEND_LOCALIZATION}")
print(f"VALID_DELTA_X: {VALID_DELTA_X}")
print(f"VALID_DELTA_Y: {VALID_DELTA_Y}")
print(f"")
print(f"##############################################################################")


##############################################################################
# Classes
##############################################################################

# ............................................................................
# Coordinate logging and Graph drawing .......................................

class CoordinateLogger:
    def __init__(self, source_filenames=['odom', 'amcl', 'tf']):
        self.source_filenames = source_filenames
        self.coordinates = {source: [] for source in source_filenames}
        self.running = True

        # Get the current date and time
        now = datetime.now()
        formatted_time = now.strftime("%Y.%m.%d-%H:%M")

        # Create the directory
        try:
            os.mkdir("/data/coordinate_log/")
            print(f"Directory 'coordinate_log' created successfully.")
        except FileExistsError:
            print(f"Directory 'coordinate_log' already exists.")
        try:
            os.mkdir("/data/coordinate_log/"+formatted_time)
            print(f"Directory '{formatted_time}' created successfully.")
        except FileExistsError:
            print(f"Directory '{formatted_time}' already exists.")

        # Create new files for each source
        self.path = "/data/coordinate_log/"+formatted_time+"/"
        for filename in self.source_filenames:
            with open(self.path+filename+".csv", 'w') as f:
                f.write("Time, X, Y, Angle\n")

        # Set up signal handler for Ctrl+C                  # Doesn't work from docker...
        signal.signal(signal.SIGINT, self.signal_handler)   # Doesn't work from docker...


    def signal_handler(self, sig, frame):
        self.running = False
        self.plot_graphs()


    def add_coordinates(self, source, x, y, angle):
        
        # Take time
        current_time = time.time()

        # Store coordinates
        # self.coordinates[source].append((current_time, x, y, angle))

        # Write to the corresponding file
        with open(self.path+source+'.csv', 'a') as f:
            f.write(f"{current_time}, {x}, {y}, {angle}\n")


    def plot_graphs(self):
        print(f"Drawing graphs...")
        plt.figure(figsize=(12, 8))

        for source in self.source_filenames:
            times = [coord[0] for coord in self.coordinates[source]]
            x_coords = [coord[1] for coord in self.coordinates[source]]
            y_coords = [coord[2] for coord in self.coordinates[source]]
            angle = [coord[3] for coord in self.coordinates[source]]

            # Plot X vs Time
            plt.subplot(3, 1, 1)
            plt.plot(times, x_coords, label=f'{source} X')
        
            # Plot Y vs Time
            plt.subplot(3, 1, 2)
            plt.plot(times, y_coords, label=f'{source} Y')

            # Plot X vs Y
            plt.subplot(3, 1, 3)
            plt.plot(x_coords, y_coords, label=f'{source}')

        plt.subplot(3, 1, 1)
        plt.title('X Coordinates Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('X Coordinate')
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.title('Y Coordinates Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Y Coordinate')
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.title('X vs Y Coordinates')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend()

        plt.tight_layout()

        # Plot graphs doesn't works from docker now (need to do smth)
        # plt.show()

        # Save the plot to a file instead of showing it
        plt.savefig(self.path + 'coordinates_plot.png')  # Save as PNG file
        plt.close()  # Close the figure to free memory

        print(f"END")


# ............................................................................
# SerialROSNode ..............................................................

class SerialROSNode(Node):
    def __init__(self):
        super().__init__('serial_ros_node')

        # Pub and sub
        self.pub_odom = self.create_publisher(Odometry, 'odom', 1)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_listener_callback, 1)
        # self.sub_tf = self.create_subscription(TFMessage, 'tf', self.tf_listener_callback, 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Read TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state = JointState()
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']  # Replace with your joint names
        self.joint_state.position = [0.0, 0.0]  # Initial positions
        self.joint_state.velocity = [0.0, 0.0]  # Initial velocity

        # Initial localization - initial pose publisher
        self.pub_init_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        # self.localization_rinning = False
        self.do_localization = True
        self.time_send_init_pose = 0.0

        # To send initial robot pose (localization)
        self.to_init_x = -99.9
        self.to_init_y = -99.9
        self.to_init_angle = 0.0

        # To check is data valid (delta btw odom and localization)
        self.last_x = 0.0
        self.last_y = 0.0

        # Coordinate Logger
        self.coordinate_logger = CoordinateLogger()
        
        # Serial msg variables
        self.prefix = 'Slm'
        self.length = 56
        self.packet_number = 0
        self.time = 0.0

        self.last_msg_time = 0.0
        self.time_delta = 0.0


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
        self.serial_lock = threading.Lock()


    # ........................................................................
    # Initial pose publisher .................................................

    def send_initial_pose(self, x_, y_, angle_):
        
        if ((time.time() - self.time_send_init_pose) > 2.5):

            if ((not x_ == -99.9) or (not y_ == -99.9)):
            
                x = (x_ - X_DELTA) * X_DIRECTION
                y = (y_ - Y_DELTA) * Y_DIRECTION
                angle_deg = angle_ - ANGLE_DELTA
                self.publish_initial_pose(x, y, angle_deg)
                self.time_send_init_pose = time.time()
            
    def publish_initial_pose(self, x: float, y: float, angle_deg: float):
        """
        Publish an initial pose estimate to /initialpose topic.
        
        Args:
            x (float): X coordinate in meters
            y (float): Y coordinate in meters
            orientation (Quaternion): Orientation as a Quaternion message
        """
        # Create PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()
        
        # Set header
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Typically 'map' frame for localization
        
        # Set position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0  # 2D pose, z = 0
        
        # Set orientation
        qx, qy, qz, qw = self.angle_to_quaternion(math.radians(angle_deg))
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        
        # Set covariance (example values, adjust as needed)
        # Diagonal covariance matrix for x, y, and yaw
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068  # yaw variance
        ]
        
        # Publish the message
        self.pub_init_pose.publish(pose_msg)
        self.get_logger().info(f'Published initial pose: x={round(x, 2)}, y={round(y, 2)}')

    # ........................................................................
    # Read ...................................................................

    def read(self):
        """Thread 1: Read from serial, convert to ROS Odometry, and publish."""
        while self.running:
            try:
                with self.serial_lock:
                    # Set a timeout (in seconds) if not already set
                    self.ser.timeout = 1.0  # 2 seconds timeout, adjust as needed
                    start_time = time.time()

                    # Read
                    raw_data = self.ser.read(PACKET_SIZE)
                    
                    time_plg_get = time.time()

                    # Chech time
                    elapsed_time = time.time() - start_time
                    if elapsed_time > 0.9:  # Threshold in seconds
                        print(f"Warning: Serial read took too long: {elapsed_time:.2f} seconds")

                    # Unpack the binary data
                    if len(raw_data) == PACKET_SIZE:
                        prefix, length, packet_number, time_, latitude_, longitude_, angle_direct, v1, v2 = struct.unpack(PACKET_FORMAT, raw_data)
                        
                        # Decode the prefix (it's a byte string)
                        prefix = prefix.decode('ascii')

                        if (X_Y_FLIP):
                            latitude = longitude_
                            longitude = latitude_
                        else:
                            latitude = latitude_
                            longitude = longitude_

                        print(f"got X={round(latitude, 2)}, Y={round(longitude, 2)}, Angle={round(angle_direct, 1)}")

                        self.to_init_x = latitude
                        self.to_init_y = longitude
                        self.to_init_angle = angle_direct

                        # If data is valid - process data
                        if (prefix == 'Slm'):
                            
                            # Send robot initial pose
                            if (False):
                            # if (self.localization_rinning == False):

                            # if (self.do_localization == True):
                                
                            #     if ((time.time() - self.time_send_init_pose) > 2.5):
                                    
                            #         x = (latitude - X_DELTA) * X_DIRECTION
                            #         y = (longitude - Y_DELTA) * Y_DIRECTION
                            #         angle_deg = angle_direct - ANGLE_DELTA
                            #         self.publish_initial_pose(x, y, angle_deg)
                            #         self.time_send_init_pose = time.time()
                                pass
                            
                            # If initial pose already sent
                            else:
                                
                                # Transform and shift axes
                                latitude *= X_DIRECTION
                                latitude -= (X_DELTA*X_DIRECTION)
                                longitude *= Y_DIRECTION
                                longitude -= (Y_DELTA*Y_DIRECTION)
                                angle_direct -= ANGLE_DELTA # constant correction

                                
                                # Create Odometry message
                                msg = Odometry()
                                msg.header.stamp = self.get_clock().now().to_msg()
                                msg.header.frame_id = "odom"
                                msg.child_frame_id = "base_footprint"
                                msg.pose.pose.position.x = round(latitude, 3)
                                msg.pose.pose.position.y = round(longitude, 3)
                                msg.pose.pose.position.z = 0.0
                                # Simple conversion of angle_direct to quaternion (assuming angle in radians)
                                angle_deg = round(angle_direct, 3)
                                qx, qy, qz, qw = self.angle_to_quaternion(math.radians(angle_deg))
                                msg.pose.pose.orientation.z = qz
                                msg.pose.pose.orientation.w = qw

                                # Time delta
                                self.last_msg_time = time_
                                self.time_delta = time_plg_get-time_

                                self.coordinate_logger.add_coordinates("odom", msg.pose.pose.position.x, msg.pose.pose.position.y, angle_deg)

                                # Publish to odom topic
                                self.pub_odom.publish(msg)
                                
                                # Create and publish the transform
                                t = TransformStamped()
                                t.header.stamp = msg.header.stamp
                                t.header.frame_id = 'odom'
                                t.child_frame_id = 'base_footprint'
                                t.transform.translation.x = msg.pose.pose.position.x
                                t.transform.translation.y = msg.pose.pose.position.y
                                t.transform.translation.z = 0.0
                                t.transform.rotation = msg.pose.pose.orientation
                                self.tf_broadcaster.sendTransform(t)
                                self.joint_state.header.stamp = msg.header.stamp
                                self.publisher_.publish(self.joint_state)

                                # Save last odom
                                self.last_x = msg.pose.pose.position.x
                                self.last_y = msg.pose.pose.position.y


                        else:
                            self.get_logger().error(f"Prefix: '{prefix}'")

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                break
            except struct.error as e:
                self.get_logger().error(f"Unpacking error: {e}")
                continue
            except UnicodeDecodeError as e:
                self.get_logger().error(f"UnicodeDecodeError: {e}")
                self.get_logger().error(f"Raw data: {raw_data.hex()}")
                continue

            time.sleep(0.01)  # Small delay to prevent CPU overload


    # ........................................................................
    # Transmit ...............................................................

    def pose_listener_callback(self, msg):
        """Read from ros, convert, and write back."""

        #self.get_logger().info('I heard: "%s"' % msg)
        if (self.do_localization == True):
            self.do_localization = False

        if self.running:
            try:
                
                latitude = msg.pose.pose.position.x
                longitude = msg.pose.pose.position.y

                # Converting quaternion back to angle
                angle_direct = self.quaternion_to_angle(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                )
                
                # # Pack the modified data
                # modified_packet = struct.pack(
                #     PACKET_FORMAT,
                #     self.prefix.encode('ascii'),
                #     self.length,
                #     self.packet_number,
                #     self.time,
                #     latitude,
                #     longitude,
                #     angle_direct,
                # )

                # # Write back to serial
                # with self.serial_lock:
                #     self.ser.write(modified_packet)
                # self.get_logger().info(f"Sent Modified Packet with AngleDirect: {angle_direct}")

                # Coordinate logger
                self.coordinate_logger.add_coordinates("amcl", latitude, longitude, angle_direct)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
            except struct.error as e:
                self.get_logger().error(f"Unpacking error: {e}")

    
    def timer_callback(self):

        if (self.do_localization == True):
            self.send_initial_pose(self.to_init_x, self.to_init_y, self.to_init_angle)

        try:
            # Lookup transform from map to base_footprint
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            
            # Converting quaternion back to angle
            angle_direct = self.quaternion_to_angle(
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            
            # Our time
            stamp = trans.header.stamp
            time_ = float(stamp.sec) + float(stamp.nanosec) * 1e-9

            # Coordinate logger
            self.coordinate_logger.add_coordinates("tf", trans.transform.translation.x, trans.transform.translation.y, angle_direct)

            # Send localization data or not ?
            if (SEND_LOCALIZATION):

                # Check if data is valid
                is_valid = self.is_data_valid(trans.transform.translation.x, trans.transform.translation.y)
                if (is_valid):
                    self.send_localization(time_, trans.transform.translation.x, trans.transform.translation.y, angle_direct)
            
            
        except TransformException as e:
            # self.get_logger().warn(f'Could not transform map to base_footprint: {e}')
            pass

    


    def is_data_valid(self, x, y) -> bool:
        delta_x = abs(self.last_x - x)
        delta_y = abs(self.last_y - y)
        if ((delta_x <= VALID_DELTA_X) and (delta_y <= VALID_DELTA_Y)):
            is_valid = True
        else:
            is_valid = False

        return is_valid

    
    def send_localization(self, time_, latitude, longitude, angle_direct):
        try:
            # Increment packet number
            self.packet_number += 1

            latitude *= X_DIRECTION
            latitude += X_DELTA
            longitude *= Y_DIRECTION
            longitude += Y_DELTA
            angle_direct += ANGLE_DELTA

            angle_direct = self.normalize_angle(angle_direct)
            
            # print(f"p={self.prefix}, length={self.length}, num={self.packet_number}, X={round(latitude, 2)}, Y={round(longitude, 2)}, Angle={round(angle_direct, 1)}")

            # Pack data
            packet = struct.pack(
                PACKET_FORMAT_RETURN,
                self.prefix.encode('ascii'),
                self.length,
                self.packet_number,
                time_,
                latitude,
                longitude,
                angle_direct
            )

            # Write back to serial
            with self.serial_lock:
                self.ser.write(packet)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in func: send_localization(): {e}")


    def tf_listener_callback(self, msg):

        for tf in msg.transforms:

            # odom -> base_footprint (odometry)
            if (tf.child_frame_id == 'base_footprint'):
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                quaternion = tf.transform.rotation

                # Converting quaternion back to angle
                angle_direct = self.quaternion_to_angle(
                    quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w
                )

            # # map -> odom (correction)
            # if (tf.child_frame_id == 'odom'):
            #     x = tf.transform.translation.x
            #     y = tf.transform.translation.y
            #     quaternion = tf.transform.rotation

            #     # Converting quaternion back to angle
            #     angle_direct = self.quaternion_to_angle(
            #         quaternion.x,
            #         quaternion.y,
            #         quaternion.z,
            #         quaternion.w
            #     )

                # Coordinate logger
                # self.coordinate_logger.add_coordinates("tf", x, y, angle_direct)


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
        z = math.sin(yaw / 2.0) * (-1)
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

        # print(f"x: {round(x, 2)} | y: {round(y, 2)} | z: {round(z, 2)} | w: {round(w, 2)}")

        z = -z

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
        
        yaw_degrees = math.degrees(yaw)
        return yaw_degrees


    def normalize_angle(self, angle: float) -> float:
        """
        Normalize an angle to be within 0-360 degrees.
        
        Args:
            angle (float): Input angle in degrees
            
        Returns:
            float: Equivalent angle between 0 and 360 degrees
        """
        # Use modulo operator to wrap angle and handle negative values
        normalized = angle % 360
        
        # Ensure the result is non-negative
        if normalized < 0:
            normalized += 360
            
        return normalized


    
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
    read_thread = threading.Thread(target=node.read, daemon=True)
    read_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.shutdown()
        rclpy.shutdown()

    # Ensure threads are joined properly
    read_thread.join()

if __name__ == '__main__':
    main()
