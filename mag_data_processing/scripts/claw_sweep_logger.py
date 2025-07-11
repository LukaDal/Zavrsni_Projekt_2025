#!/usr/bin/env python3

# This node is used to gather magnetic sensor data from the full sweep of
# robotic gripper (fully open -> fully closed). Sweep data is given through
# ros parameter server, and the output is ien in form of .csv file.

import math
import rospy
import csv
from std_msgs.msg import String
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from datetime import datetime

#--------------------------------------------------------------------------------------------------

class ClawSweepLogger:
    def __init__(self):

        # Sweep parameters
        self.baud         = rospy.get_param('~dynamixel_baud', 57600)
        self.dxl_id       = rospy.get_param('~dynamixel_id', 1)
        self.angle_min    = rospy.get_param('~angle_min', 700)
        self.angle_max    = rospy.get_param('~angle_max', 1400)
        self.angle_step   = rospy.get_param('~angle_step', 10)
        self.angle_toler  = rospy.get_param('~angle_tolerance', 1)
        self.sample_count = rospy.get_param('~samples_per_angle', 300)
        self.claw_empty   = rospy.get_param('~claw_empty', True)
        self.device       = rospy.get_param('~dxl_device', '/dev/ttyUSB0')
        self.sensor_topic = rospy.get_param('~sensor_topic', '/sensor_data/senzor0')
        self.folder       = rospy.get_param('~output_folder', '/home/luka/Documents/sweep_data')

        # Initilize - Sensor
        self.x_fields = []
        self.y_fields = []
        self.z_fields = []
        self.r_fields = []
        self.enable_reading = False
        rospy.Subscriber(self.sensor_topic, String, self.sensor_callback)

        # Initilize - Output file
        self.object = input("Enter the test object name: ")
        self.init_csv()

        # Initilize - Claw Motor
        self.init_dynamixel()

        # Final warnings
        rospy.logwarn("Make sure that 'mag_sensor_publisher' is turned on and publishing!")
        rospy.logwarn("Place the object inside the claw (or leave empty).")
        rospy.logwarn("Do NOT move the claw or anything attached to it during the sweep cycle!")
        input("Press ENTER to start the Sweep Cycle.\n")

        # Shutdown
        rospy.on_shutdown(self.cleanup)

#-INITS--------------------------------------------------------------------------------------------

    def init_dynamixel(self):
        # Establish contact with Dynamixel
        self.port_handler = PortHandler(self.device)
        self.packet_handler = PacketHandler(2.0)
        if not self.port_handler.openPort():
            rospy.logerr("Failed to open port")
            exit(1)
        if not self.port_handler.setBaudRate(self.baud):
            rospy.logerr("Failed to set baudrate")
            exit(1)

        # Disable Torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, 64, 0)
        if dxl_comm_result != 0 or dxl_error != 0:
            rospy.logwarn(f"Failed to Disable Torque (result: {dxl_comm_result}, error: {dxl_error})")

        # Set Operating Mode to Position Control (3)
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, 11, 3)
        if dxl_comm_result != 0 or dxl_error != 0:
            rospy.logwarn(f"Failed to set Operating Mode (result: {dxl_comm_result}, error: {dxl_error})")

        # Set Profile Velocity to 10 (2.29 rpm)
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, 112, 10)
        if dxl_comm_result != 0 or dxl_error != 0:
            rospy.logwarn(f"Failed to set Profile Velocity (result: {dxl_comm_result}, error: {dxl_error})")

        # Enable Torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, 64, 1)
        if dxl_comm_result != 0 or dxl_error != 0:
            rospy.logwarn(f"Failed to Enable Torque (result: {dxl_comm_result}, error: {dxl_error})")

        rospy.loginfo("Dynamixel initilization completed. Please check for any warnings!")
        input("Press ENTER to move the claw to its initial position.")

        # Move to start position
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, 116, self.angle_min)
        if dxl_comm_result != 0 or dxl_error != 0:
            rospy.logwarn(f"Failed to reach start position (result: {dxl_comm_result}, error: {dxl_error})")


    def init_csv(self):
        timestamp = datetime.now().strftime("%Y_%d_%m_%H%M%S")
        filename = f"{self.folder}/{self.object}_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['angle_dec', 'avg_x', 'avg_y', 'avg_z', 'avg_r', 'label'])
        self.csv_file.flush()
        rospy.loginfo(f"Logging to CSV: {filename}")

#-CUSTOM-FUNCTIONS---------------------------------------------------------------------------------

    # Close the .csv file
    def cleanup(self):
        if self.csv_file:
            self.csv_file.close()
            rospy.loginfo("CSV file closed cleanly.")

    # Function for moving the claw
    def set_angle(self, angle : int): # Angle is given in dec.
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, 116, angle)
        if result != 0 or error != 0:
            rospy.logwarn(f"Failed to set angle {angle} — result: {result}, error: {error}")

#-CALLBACKS----------------------------------------------------------------------------------------

    # Extract data from incoming String topic
    def sensor_callback(self, msg : String):
        if not self.enable_reading:
            return
        try:
            clean_str = msg.data.strip()                            # "-13328,5586,-6174"
            values = list(map(int, clean_str.strip().split(",")))   # [-13328, 5586, -6174]

            if len(values) == 3:
                x, y, z = values
                r = math.sqrt(x**2 + y**2 + z**2)
                self.x_fields.append(x)
                self.y_fields.append(y)
                self.z_fields.append(z)
                self.r_fields.append(r)

        except ValueError as e:
            rospy.logwarn(f"Failed to parse message: {msg.data} — {e}")

#-MAIN-PROGRAM-------------------------------------------------------------------------------------

    def run_sweep(self):
        for angle in range(self.angle_min, self.angle_max + 1, self.angle_step):

            self.x_fields.clear()
            self.y_fields.clear()
            self.z_fields.clear()
            self.r_fields.clear()

            # Move the claw
            self.set_angle(angle)
            rospy.sleep(1.0) # Wiggle room for moving
            # IMPORTANT: There is no check if we actually reached the specifed angle,
            # because the objects in the claw can prevent us from doing so

            # Begin taking samples
            rospy.loginfo(f"Collecting samples at angle {angle} dec.")
            self.enable_reading = True

            # Wait until we have gathered all samples
            while len(self.r_fields) < self.sample_count and not rospy.is_shutdown():
                rospy.sleep(0.01)
            self.enable_reading = False

            try:
                # Data to write into .csv
                avg_x = sum(self.x_fields) / len(self.x_fields)
                avg_y = sum(self.y_fields) / len(self.y_fields)
                avg_z = sum(self.z_fields) / len(self.z_fields)
                avg_r = sum(self.r_fields) / len(self.r_fields)
                if self.claw_empty:
                    label = 0
                # Write to .csv
                self.writer.writerow([angle, avg_x, avg_y, avg_z, avg_r, label])

            except ZeroDivisionError:
                rospy.logerr(f"No samples collected at angle {angle} — skipping.")
            
            self.csv_file.flush()
            rospy.loginfo(f"Angle {angle} done\n")

        self.csv_file.close()
        rospy.loginfo("Sweep complete")

#--------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('claw_sweep_logger')
    node = ClawSweepLogger()
    rospy.sleep(1.0)
    node.run_sweep()