#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import utm
import rospy
import serial
from imu_driver.msg import imu_msg
import numpy as np
import math
import time

def euler2quaternion(yaw,pitch,roll):
    # DCM=np.zeros((3,3))
    # DCM[0][0] = (np.cos(p))*(np.cos(y))
    # DCM[0][1] = (np.cos(p))*(np.sin(y))
    # DCM[0][2] = -(np.sin(p))
    # DCM[1][0] = (np.sin(r))*(np.sin(p))*(np.cos(y))-(np.cos(r))*(np.sin(y))
    # DCM[1][1] = (np.sin(r))*(np.sin(p))*(np.sin(y))+(np.cos(r))*(np.cos(y))
    # DCM[1][2] = (np.sin(r))*(np.cos(p))
    # DCM[2][0] = (np.cos(r))*(np.sin(p))*(np.cos(y))+(np.sin(r))*(np.sin(y))
    # DCM[2][1] = (np.cos(r))*(np.sin(p))*(np.sin(y))-(np.sin(r))*(np.cos(y))
    # DCM[2][2] = (np.cos(r))*(np.cos(p))
    # Q1 = (0.25*(1+DCM[0][0]-DCM[1][1]-DCM[2][2]))
    # Q2 = (0.25*(1-DCM[0][0]+DCM[1][1]-DCM[2][2]))
    # Q3 = (0.25*(1-DCM[0][0]-DCM[1][1]+DCM[2][2]))
    # Q4 = (0.25*(1+DCM[0][0]+DCM[1][1]+DCM[2][2]))
    # A = [Q1,Q2,Q3,Q4]
    # num = [0,1,2,3]
    # max_q = max(A)
    # index = A.index(max_q)
    # num.remove(index)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]
    

def imu_publisher():
    SENSOR_NAME = "imu"
    rospy.init_node("imu_driver",anonymous=False)
    port = rospy.get_param("/imu_driver/port_imu",default="/dev/ttyUSB0")
    #port = "/dev/ttyUSB0"
    pub = rospy.Publisher("/imu",imu_msg,queue_size=50)
    serial_port = (port)
    baud_rate = 115200
    #port = serial.Serial(serial_port, baud_rate, bytesize=8,timeout=2, stopbits=serial.STOPBITS_ONE)
    port = serial.Serial(serial_port, baud_rate, timeout=2)
    port.write(b'$VNWRG,07,40*XX\r')
    port.write(b'$VNWRG,06,14*XX\r')
    data = imu_msg()
    data.Header.frame_id = "IMU1_Frame"
    data.Header.seq = 0
    encoding = 'UTF-8'
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            line_str = str(line, encoding)
            data.msgstring = line_str
            line_str = line_str.replace('\r','')
            line_str = line_str.replace('\n','')
            imu_data = line_str.split(",")
            curr_time = time.time_ns()
            t_s = curr_time//(1000000000)
            t_ns = curr_time-(t_s*1000000000)
            data.Header.stamp.secs = t_s
            data.Header.stamp.nsecs = t_ns
            if(imu_data[0]=="$VNYMR"):
                y = float(imu_data[1])*(np.pi)/180
                p = float(imu_data[2])*(np.pi)/180
                r = float(imu_data[3])*(np.pi)/180
                magx = float(imu_data[4])
                magy = float(imu_data[5])
                magz = float(imu_data[6])
                accx = float(imu_data[7])
                accy = float(imu_data[8])
                accz = float(imu_data[9])
                gyrox = float(imu_data[10])
                gyroy = float(imu_data[11])
                gyroz = imu_data[12]
                size = len(gyroz)
                gyroz = float(gyroz[:size-3])
                quatx,quaty,quatz,quatw = euler2quaternion(y,p,r)
                data.IMU.orientation.x = quatx
                data.IMU.orientation.y = quaty
                data.IMU.orientation.z = quatz
                data.IMU.orientation.w = quatw
                data.IMU.angular_velocity.x = gyrox
                data.IMU.angular_velocity.y = gyroy
                data.IMU.angular_velocity.z = gyroz
                data.IMU.linear_acceleration.x = accx
                data.IMU.linear_acceleration.y = accy
                data.IMU.linear_acceleration.z = accz
                data.MagField.magnetic_field.x = magx
                data.MagField.magnetic_field.y = magy
                data.MagField.magnetic_field.z = magz
            rospy.loginfo(data)
            pub.publish(data)
            data.Header.seq += 1
    except rospy.ROSInterruptException:
        port.close()

if __name__ == '__main__':
    imu_publisher()
