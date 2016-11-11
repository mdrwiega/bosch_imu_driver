#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Michal Drwiega
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import serial
import rospy
import sys
import struct as st
import binascii

from time import time
from sensor_msgs.msg import Imu, Temperature, MagneticField

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# BOSCH BNO055 IMU Registers map and other information
# Page 0 registers
CHIP_ID = 0x00
PAGE_ID = 0x07
ACCEL_DATA = 0x08
MAG_DATA = 0x0e
GYRO_DATA = 0x14
FUSED_EULER = 0x1a
FUSED_QUAT = 0x20
LIA_DATA = 0x28
GRAVITY_DATA = 0x2e
TEMP_DATA = 0x34
CALIB_STAT = 0x35
SYS_STATUS = 0x39
SYS_ERR = 0x3a
UNIT_SEL = 0x3b
OPER_MODE = 0x3d
PWR_MODE = 0x3e
SYS_TRIGGER = 0x3f
TEMP_SOURCE = 0x440
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

ACC_OFFSET = 0x55
MAG_OFFSET = 0x5b
GYR_OFFSET = 0x61
ACC_RADIUS = 0x68
MAG_RADIUS = 0x69

# Page 1 registers
ACC_CONFIG = 0x08
MAG_CONFIG = 0x09
GYR_CONFIG0 = 0x0a
GYR_CONFIG1 = 0x0b

#  Operation modes
OPER_MODE_CONFIG = 0x00
OPER_MODE_ACCONLY = 0x01
OPER_MODE_MAGONLY = 0x02
OPER_MODE_GYROONLY = 0x03
OPER_MODE_ACCMAG = 0x04
OPER_MODE_ACCGYRO = 0x05
OPER_MODE_MAGGYRO = 0x06
OPER_MODE_AMG = 0x07
OPER_MODE_IMU = 0x08
OPER_MODE_COMPASS = 0x09
OPER_MODE_M4G = 0x0a
OPER_MODE_NDOF_FMC_OFF = 0x0b
OPER_MODE_NDOF = 0x0C

#  Power modes
PWR_MODE_NORMAL = 0x00
PWR_MODE_LOW = 0x01
PWR_MODE_SUSPEND  = 0x02

# Communication constants
BNO055_ID = 0xa0
START_BYTE_WR = 0xaa
START_BYTE_RESP = 0xbb
READ = 0x01
WRITE = 0x00

# Read data from IMU
def read_from_dev(ser, reg_addr, length):
    buf_out = bytearray()
    buf_out.append(START_BYTE_WR)
    buf_out.append(READ)
    buf_out.append(reg_addr)
    buf_out.append(length)

    try:
        ser.write(buf_out)
        buf_in = bytearray(ser.read(2 + length))

        # print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
    except:
        return 0

    # Check if response is correct
    if (buf_in.__len__() != (2 + length)) or (buf_in[0] != START_BYTE_RESP):
        #rospy.logerr("Incorrect Bosh IMU device response.")
        return 0
    buf_in.pop(0)
    buf_in.pop(0)
    return buf_in


# Write data to IMU
def write_to_dev(ser, reg_addr, length, data):
    buf_out = bytearray()
    buf_out.append(START_BYTE_WR)
    buf_out.append(WRITE)
    buf_out.append(reg_addr)
    buf_out.append(length)
    buf_out.append(data)

    try:
        ser.write(buf_out)
        buf_in = bytearray(ser.read(2))
        # print("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
    except:
        return False

    if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
        #rospy.logerr("Incorrect Bosh IMU device response.")
        return False
    return True


imu_data = Imu()            # Filtered data
imu_raw = Imu()             # Raw IMU data
temperature_msg = Temperature() # Temperature
mag_msg = MagneticField()       # Magnetometer data

# Main function
if __name__ == '__main__':
    rospy.init_node("bosch_imu_node")

    # Sensor measurements publishers
    pub_data = rospy.Publisher('imu/data', Imu, queue_size=1)
    pub_raw = rospy.Publisher('imu/raw', Imu, queue_size=1)
    pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=1)
    pub_temp = rospy.Publisher('imu/temp', Temperature, queue_size=1)

    # srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback

    # Get parameters values
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    frame_id = rospy.get_param('~frame_id', 'imu_link')
    frequency = rospy.get_param('frequency', 100)
    operation_mode = rospy.get_param('operation_mode', OPER_MODE_NDOF)

    # Open serial port
    rospy.loginfo("Opening serial port: %s...", port)
    try:
        ser = serial.Serial(port, 115200, timeout=0.02)
    except serial.serialutil.SerialException:
        rospy.logerr("IMU not found at port " + port + ". Check the port in the launch file.")
        sys.exit(0)

    # Check if IMU ID is correct
    buf = read_from_dev(ser, CHIP_ID, 1)
    if buf == 0 or buf[0] != BNO055_ID:
        #rospy.logerr("Device ID is incorrect. Shutdown.")
        sys.exit(0)

    # IMU Configuration
    if not(write_to_dev(ser, OPER_MODE, 1, OPER_MODE_CONFIG)):
        rospy.logerr("Unable to set IMU into config mode.")

    if not(write_to_dev(ser, PWR_MODE, 1, PWR_MODE_NORMAL)):
        rospy.logerr("Unable to set IMU normal power mode.")

    if not(write_to_dev(ser, PAGE_ID, 1, 0x00)):
        rospy.logerr("Unable to set IMU register page 0.")

    if not(write_to_dev(ser, SYS_TRIGGER, 1, 0x00)):
        rospy.logerr("Unable to start IMU.")

    if not(write_to_dev(ser, UNIT_SEL, 1, 0x83)):
        rospy.logerr("Unable to set IMU units.")

    if not(write_to_dev(ser, AXIS_MAP_CONFIG, 1, 0x24)):
        rospy.logerr("Unable to remap IMU axis.")

    if not(write_to_dev(ser, AXIS_MAP_SIGN, 1, 0x06)):
        rospy.logerr("Unable to set IMU axis signs.")

    if not(write_to_dev(ser, OPER_MODE, 1, OPER_MODE_NDOF)):
        rospy.logerr("Unable to set IMU operation mode into operation mode.")

    rospy.loginfo("Bosch BNO055 IMU configuration complete.")

    rate = rospy.Rate(frequency)

    # Factors for unit conversions
    acc_fact = 100.0
    mag_fact = 16.0
    gyr_fact = 900.0
    seq = 0

    while not rospy.is_shutdown():
        buf = read_from_dev(ser, ACCEL_DATA, 45)
        if buf != 0:
            # Publish raw data
            imu_raw.header.stamp = rospy.Time.now()
            imu_raw.header.frame_id = frame_id
            imu_raw.header.seq = seq
            imu_raw.orientation_covariance[0] = -1
            imu_raw.linear_acceleration.x = float(st.unpack('h', st.pack('BB', buf[0], buf[1]))[0]) / acc_fact
            imu_raw.linear_acceleration.y = float(st.unpack('h', st.pack('BB', buf[2], buf[3]))[0]) / acc_fact
            imu_raw.linear_acceleration.z = float(st.unpack('h', st.pack('BB', buf[4], buf[5]))[0]) / acc_fact
            imu_raw.linear_acceleration_covariance[0] = -1
            imu_raw.angular_velocity.x = float(st.unpack('h', st.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
            imu_raw.angular_velocity.y = float(st.unpack('h', st.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
            imu_raw.angular_velocity.z = float(st.unpack('h', st.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
            imu_raw.angular_velocity_covariance[0] = -1
            pub_raw.publish(imu_raw)

            #            print("read: ", binascii.hexlify(buf), "acc = (",imu_data.linear_acceleration.x,
            #                  imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, ")")

            # Publish filtered data
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = frame_id
            imu_data.header.seq = seq
            imu_data.orientation.w = float(st.unpack('h', st.pack('BB', buf[24], buf[25]))[0])
            imu_data.orientation.x = float(st.unpack('h', st.pack('BB', buf[26], buf[27]))[0])
            imu_data.orientation.y = float(st.unpack('h', st.pack('BB', buf[28], buf[29]))[0])
            imu_data.orientation.z = float(st.unpack('h', st.pack('BB', buf[30], buf[31]))[0])
            imu_data.linear_acceleration.x = float(st.unpack('h', st.pack('BB', buf[32], buf[33]))[0]) / acc_fact
            imu_data.linear_acceleration.y = float(st.unpack('h', st.pack('BB', buf[34], buf[35]))[0]) / acc_fact
            imu_data.linear_acceleration.z = float(st.unpack('h', st.pack('BB', buf[36], buf[37]))[0]) / acc_fact
            imu_data.linear_acceleration_covariance[0] = -1
            imu_data.angular_velocity.x = float(st.unpack('h', st.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
            imu_data.angular_velocity.y = float(st.unpack('h', st.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
            imu_data.angular_velocity.z = float(st.unpack('h', st.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
            imu_data.angular_velocity_covariance[0] = -1
            pub_data.publish(imu_data)

            # Publish magnetometer data
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.header.frame_id = frame_id
            mag_msg.header.seq = seq
            mag_msg.magnetic_field.x = float(st.unpack('h', st.pack('BB', buf[6], buf[7]))[0]) / mag_fact
            mag_msg.magnetic_field.y = float(st.unpack('h', st.pack('BB', buf[8], buf[9]))[0]) / mag_fact
            mag_msg.magnetic_field.z = float(st.unpack('h', st.pack('BB', buf[10], buf[11]))[0]) / mag_fact
            pub_mag.publish(mag_msg)

            # Publish temperature
            temperature_msg.header.stamp = rospy.Time.now()
            temperature_msg.header.frame_id = frame_id
            temperature_msg.header.seq = seq
            temperature_msg.temperature = buf[44]
            pub_temp.publish(temperature_msg)

            #yaw = float(st.unpack('h', st.pack('BB', buf[18], buf[19]))[0]) / 16.0
            #roll = float(st.unpack('h', st.pack('BB', buf[20], buf[21]))[0]) / 16.0
            #pitch = float(st.unpack('h', st.pack('BB', buf[22], buf[23]))[0]) / 16.0
            #           print "RPY=(%.2f %.2f %.2f)" %(roll, pitch, yaw)

            seq = seq + 1
        rate.sleep()
    ser.close()
