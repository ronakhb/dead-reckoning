#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import utm
import rospy
import serial
from gps_driver.msg import gps_msg
from std_msgs.msg import Float64
from std_msgs.msg import Header

def convert_dd(lat,lat_dir,lon,lon_dir):
    if lat[:2]!="" and lat[2:]!="" and lon[:3]!="" and lon[3:]!="":
        lat_DD = float(lat[:2])
        lat_mmmm=float(lat[2:])
        lon_DDD = float(lon[:3])
        lon_mmmm = float(lon[3:])
        latitude = lat_DD+lat_mmmm/60
        longitude = lon_DDD+lon_mmmm/60
        if lat_dir == 'S':
            latitude = -latitude
        elif lat_dir != 'N' and 'S':
            print("Invalid latitude direction, Please check device")
            latitude = "Null"
        if lon_dir == 'W':
            longitude = -longitude
        elif lon_dir != 'E' and 'W':
            print("Invalid longitude direction, Please check device")
            longitude = "Null"
        return latitude,longitude
    else:
        print("NO GPS SIGNAL FOUND")
        return 0,0


def sensor_init():
    SENSOR_NAME = "gps_module"
    rospy.init_node("gps_driver",anonymous=False)
    port = rospy.get_param("/gps_ros_driver/port_gps",default="/dev/ttyUSB0")
    pub = rospy.Publisher("/gps",gps_msg,queue_size=50)
    rate = rospy.Rate(120)
    #serial_port = ("/dev/ttyUSB0")
    serial_port = (port)
    baud_rate = 4800
    sampling_rate = 5
    #port = serial.Serial(serial_port, baud_rate, bytesize=8,timeout=2, stopbits=serial.STOPBITS_ONE)
    port = serial.Serial(serial_port, baud_rate, timeout=2)
    gps_reading = gps_msg()
    gps_reading.Header.frame_id = "GPS1_Frame"
    gps_reading.Header.seq = 0
    sleep_time = 1/sampling_rate - 0.025
    encoding = 'UTF-8'
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            line_str = str(line, encoding)
            line_str = line_str.replace('\r','')
            line_str = line_str.replace('\n','')
            gps_data = line_str.split(",")
            if(gps_data[0]=="$GPGGA"):
                print(gps_data)
                gpgga_time = gps_data[1]
                gpgga_time_h = int(gpgga_time[:2])
                gpgga_time_m = int(gpgga_time[2:4])
                gpgga_time_s = int(gpgga_time[4:6])
                gpgga_time_ns = int(gpgga_time[7:])*1000000
                gps_reading.Header.stamp.secs = (gpgga_time_h*3600)+(gpgga_time_m*60)+gpgga_time_s
                gps_reading.Header.stamp.nsecs = gpgga_time_ns
                latitude,longitude = convert_dd(gps_data[2],gps_data[3],gps_data[4],gps_data[5])
                gps_reading.Latitude = latitude
                gps_reading.Longitude = longitude
                gps_reading.Altitude = float(gps_data[9])
                utm_data = utm.from_latlon(latitude,longitude)
                gps_reading.UTM_easting = utm_data[0]
                gps_reading.UTM_northing = utm_data[1]
                gps_reading.Zone = utm_data[2]
                gps_reading.Letter = utm_data[3]
                pub.publish(gps_reading)
                gps_reading.Header.seq += 1
                rate.sleep()
    except rospy.ROSInterruptException:
        port.close()
if __name__ == '__main__':
    sensor_init()
