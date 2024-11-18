import numpy as np
import quaternion
from ellipse import LsqEllipse
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from bagpy import bagreader
import math
import pandas as pd
from scipy.signal import butter, filtfilt
plt.rcParams.update({'font.size': 22})


class calibrate():

    def __init__(self):
        #Read bag file
        #bag = bagreader('/home/ronak/backup/LAB3/calibration.bag', "r")
        #data = bag.message_by_topic('/imu')
        df=pd.read_csv('/home/ronak/backup/LAB3/calibration/imu.csv')

        #Read Magnetometer data
        self.x_cal = np.array(df["MagField.magnetic_field.x"])
        self.y_cal = np.array(df["MagField.magnetic_field.y"])


    def euler_from_quaternion(self,quatern):
        roll = []
        pitch = []
        yaw = []
        for i in range(0,quatern.shape[0]):
            x, y, z, w = quatern[i,0],quatern[i,1],quatern[i,2],quatern[i,3]

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            roll.append(roll_x)
            pitch.append(pitch_y)
            yaw.append(yaw_z)
        
        theta = np.column_stack([roll,pitch,yaw])
        return theta # in radians

    def calibration(self):

        raw_mag_data = np.column_stack((self.x_cal,self.y_cal))

        #Fit Ellipse
        fit_ellipse = LsqEllipse().fit(raw_mag_data)
        center, width, height, phi = fit_ellipse.as_parameters()
        self.x_center,self.y_center = center
        self.ellipse = Ellipse(
            xy=center, width=2*width, height=2*height, angle=np.rad2deg(phi),
            edgecolor='b', fc='None', lw=2, label='Fit', zorder=2)

        print(np.rad2deg(phi))

        #Rotation Matrix
        rotation_matrix = np.array([[math.cos(phi),math.sin(phi)],[-math.sin(phi),math.cos(phi)]])
        
        #Scaling Matrix
        sigma = height/width
        print(sigma)
        scale_mat = np.array([[sigma,0],[0,1]])

        #Correcting magnetometer points
        self.corrected_points = np.zeros((2,self.x_cal.shape[0]))
        self.corrected_points[0:,] = self.x_cal-self.x_center
        self.corrected_points[1:,] = self.y_cal-self.y_center
        self.final_correction_matrix = np.matmul(scale_mat,rotation_matrix)
        self.corrected_points = np.matmul(np.matmul(scale_mat,rotation_matrix),self.corrected_points)


    def calibration_plots(self):
        # fig = plt.figure(figsize=(6, 6))
        # ax = plt.subplot()
        # ax.axis('equal')
        # ax.plot(self.x_cal, self.y_cal, '.',markersize=1,color='r')
        # ax.add_patch(self.ellipse)
        # plt.plot(self.x_center,self.y_center, marker="o", markersize=10,  markerfacecolor="g")
        # plt.text(self.x_center,self.y_center+0.01,'({},{})'.format(round(self.x_center,4),round(self.y_center,4)))
        # plt.xlabel('$Magnetometer X (Gauss)$')
        # plt.ylabel('$Magnetometer Y (Gauss)$')
        # plt.legend(['Points','Best Fit Ellipse'])
        # plt.title('Raw Magnetometer data')
        # plt.grid(True)
        # plt.show()
        fig = plt.figure(figsize=(6, 6))
        plt.plot(self.x_cal, self.y_cal, '.',markersize=2,color='r')
        plt.plot(self.corrected_points[0,:],self.corrected_points[1,:],'.', markersize=2, color="b")
        plt.plot(0, 0, marker="o", markersize=10,  markerfacecolor="g")
        plt.text(0,0.01,'({},{})'.format(0,0))
        plt.plot(self.x_center,self.y_center, marker="o", markersize=10,  markerfacecolor="black")
        plt.text(self.x_center,self.y_center+0.01,'({},{})'.format(round(self.x_center,4),round(self.y_center,4)))
        plt.xlabel('$Magnetometer X (Gauss)$')
        plt.ylabel('$Magnetometer Y (Gauss)$')
        plt.legend(['Raw Data','Corrected Data'])
        plt.title('Raw vs Corrected Magnetometer data')
        plt.axis('equal')
        plt.grid(True)
        plt.savefig('Calibration.png')


    def moving_data_correction(self):
        # bag = bagreader('/home/ronak/backup/LAB3/driving_data_1.bag', "r")
        # data = bag.message_by_topic('/imu')
        # df_imu=pd.read_csv(data)
        df_gps = pd.read_csv("/home/ronak/backup/LAB3/driving_data_1/gps.csv")
        df_imu=pd.read_csv('/home/ronak/backup/LAB3/driving_data_1/imu.csv')
        self.x_mag_driv_raw = np.array(df_imu["MagField.magnetic_field.x"])
        self.y_mag_driv_raw = np.array(df_imu["MagField.magnetic_field.y"])
        self.gps_x = np.array(df_gps["UTM_easting"])
        self.gps_y = np.array(df_gps["UTM_northing"])
        self.acc_x = np.array(df_imu["IMU.linear_acceleration.x"])
        self.acc_y = np.array(df_imu["IMU.linear_acceleration.y"])
        self.acc_z = np.array(df_imu["IMU.linear_acceleration.z"])
        self.ang_vel_z = np.array(df_imu["IMU.angular_velocity.z"])
        gps_time_s = df_gps["Header.stamp.secs"]
        gps_time_ns = df_gps["Header.stamp.nsecs"]
        imu_time_s = df_imu["Header.stamp.secs"]
        imu_time_ns = df_imu["Header.stamp.nsecs"]
        self.gps_time = gps_time_s + gps_time_ns/1000000000
        self.imu_time = imu_time_s + imu_time_ns/1000000000
        self.gps_time = self.gps_time - np.min(self.gps_time)
        self.imu_time = self.imu_time - np.min(self.imu_time)
        corrected_points = np.zeros((2,self.x_mag_driv_raw.shape[0]))
        corrected_points[0:,] = self.x_mag_driv_raw-self.x_center
        corrected_points[1:,] = self.y_mag_driv_raw-self.y_center
        self.mag_driv_corr = np.matmul(self.final_correction_matrix,corrected_points)
        theta_x = df_imu["IMU.orientation.x"]
        theta_y = df_imu["IMU.orientation.y"]
        theta_z = df_imu["IMU.orientation.z"]
        theta_w = df_imu["IMU.orientation.w"]
        self.theta_quat = np.column_stack([theta_x,theta_y,theta_z,theta_w])
        self.theta_rad = self.euler_from_quaternion(self.theta_quat)

    def wrap(self,theta):
        n = theta.shape[0]
        for i in range(0,n):
            if np.pi< theta[i] < 2*np.pi:
                theta[i] = theta[i]-2*np.pi
            elif 2*np.pi< theta[i] < 4*np.pi:
                theta[i] = theta[i]-4*np.pi
            elif 4*np.pi< theta[i] < 6*np.pi:
                theta[i] = theta[i]-6*np.pi

        return theta

    def unwrap(self,theta):
        yaw_mag_temp = np.copy(theta)
        n = yaw_mag_temp.shape[0]
        for i in range(0,n):
            if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>340):
                yaw_mag_temp[i] = yaw_mag_temp[i-1]
        for i in range(0,int(3*n/4)):
            if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
                yaw_mag_temp[i] = yaw_mag_temp[i] + 360
        for i in range(int(3*n/4),n):
            if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
                yaw_mag_temp[i:] = yaw_mag_temp[i:] + 360
                break
        for i in range(int(5*n/6),n):
            if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
                yaw_mag_temp[i] = yaw_mag_temp[i] + 360
        
        return yaw_mag_temp

    def unwrap2(self,theta):
        yaw_mag_temp = np.copy(theta)
        yaw_mag_temp = yaw_mag_temp+360
        n = yaw_mag_temp.shape[0]
        for i in range(0,n):
            if (yaw_mag_temp[i]-yaw_mag_temp[i-1])>359:
                yaw_mag_temp[i:] = yaw_mag_temp[i:]-yaw_mag_temp[i]+yaw_mag_temp[i-1]
            elif (yaw_mag_temp[i]-yaw_mag_temp[i-1])<-359:
                yaw_mag_temp[i:] = yaw_mag_temp[i:]-yaw_mag_temp[i]+yaw_mag_temp[i-1]
        # for i in range(0,int(3*n/4)):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
        #         yaw_mag_temp[i] = yaw_mag_temp[i] + 360
        # for i in range(int(3*n/4),n):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
        #         yaw_mag_temp[i:] = yaw_mag_temp[i:] + 360
        #         break
        # for i in range(int(5*n/6),n):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
        #         yaw_mag_temp[i] = yaw_mag_temp[i] + 360
        
        return yaw_mag_temp

    def moving_data_yaw(self):

        #Magnetometer Yaw Calculation
        yaw_mag = np.arctan2(-self.mag_driv_corr[1,:],self.mag_driv_corr[0,:])*(180/np.pi)
        yaw_mag_raw = np.arctan2(-self.x_mag_driv_raw,self.y_mag_driv_raw)*(180/np.pi)
        yaw_mag_temp = self.unwrap(yaw_mag)
        yaw_mag_raw2 = self.unwrap2(yaw_mag_raw)
        n = yaw_mag_raw.shape[0]
        for i in range(0,n):
            if (abs(yaw_mag_raw[i]-yaw_mag_raw[i-1])>300):
                yaw_mag_raw[i] = yaw_mag_raw[i-1]
        # sq = np.square(self.mag_driv_corr)
        # magn = sq[1,:]+sq[0,:]
        # print(self.mag_driv_corr)
        # sine = np.divide(-self.mag_driv_corr[1,:],magn)
        # sine = np.where(sine<-1,-1,sine)
        # sine = np.where(sine>1,1,sine)
        # m = yaw_mag.shape[0]
        # for i in range(0,m):
        #     if ((-170>yaw_mag[i]>-180) or (180>yaw_mag[i]>170)) and ((-170>yaw_mag[i-1]>-180) or (180>yaw_mag[i-1]>170)):
        #          yaw_mag[i] = yaw_mag[i-1]

        # yaw_mag = np.arcsin(sine)*(180/np.pi)

        #gyro Yaw Calculation
        prev_yaw_angle = yaw_mag[0]*(np.pi/180)
        #prev_yaw_angle = 0
        yaw_gyro = []
        n = self.ang_vel_z.shape[0]
        for i in range(0,n):
            if i == n-1:
                break
            yaw_gyro_temp = prev_yaw_angle+np.trapz([self.ang_vel_z[i],self.ang_vel_z[i+1]],dx=1)*0.025
            # if yaw_gyro_temp > np.pi:
            #     yaw_gyro_temp = yaw_gyro_temp -2*np.pi
            # elif yaw_gyro_temp < -np.pi:
            #     yaw_gyro_temp=yaw_gyro_temp+2*np.pi

            # if ((-np.pi+0.01>yaw_gyro_temp>-np.pi) or (np.pi>yaw_gyro_temp>np.pi-0.01)) and ((-np.pi+0.01>prev_yaw_angle>-np.pi) or (np.pi>prev_yaw_angle>np.pi-0.01)):
            #     yaw_gyro_temp = np.pi
            prev_yaw_angle = yaw_gyro_temp
            yaw_gyro.append(yaw_gyro_temp)
        yaw_gyro.append(yaw_gyro[-1]) #repeating the last value as we have 1 less data point due to integration
        yaw_gyro = np.array(yaw_gyro)*180/np.pi

        # yaw_mag_temp = np.copy(yaw_mag)
        # for i in range(0,n):
        #     # if ((-165>yaw_mag[i]>-180) or (180>yaw_mag[i]>165)) and ((-165>yaw_mag[i-1]>-180) or (180>yaw_mag[i-1]>165)):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>340):
        #         yaw_mag_temp[i] = yaw_mag_temp[i-1]

        # n = yaw_mag.shape[0]
        # for i in range(0,int(3*n/4)):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
        #         yaw_mag_temp[i] = yaw_mag_temp[i] + 360
        # for i in range(int(3*n/4),n):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
        #         yaw_mag_temp[i:] = yaw_mag_temp[i:] + 360
        #         break
        # for i in range(int(5*n/6),n):
        #     if (abs(yaw_mag_temp[i]-yaw_mag_temp[i-1])>180):
        #         yaw_mag_temp[i] = yaw_mag_temp[i] + 360
                
        #Apply Lowpass and Highpass Filter
        # Filter requirements.
        T = 0.5        # Sample Period
        fs = 40       # sample rate, Hz
        low_cutoff = 0.08   # desired cutoff frequency of the filter, Hz ,
        nyq = 0.5 * fs  # Nyquist Frequency
        order = 1       # sin wave can be approx represented as quadratic
        n = int(T * fs) # total number of samples
        low_normal_cutoff = low_cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, low_normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, yaw_mag_temp)
        high_cutoff = 0.05
        high_normal_cutoff = high_cutoff / nyq
        b, a = butter(order, high_normal_cutoff, btype='high', analog=False)
        z  = filtfilt(b, a, yaw_gyro)
        final_yaw = z+y
        self.heading_angle = final_yaw
        # # plt.plot(self.imu_time,yaw_gyro,color= 'b')
        # plt.plot(self.imu_time,yaw_gyro,color= 'r')
        # plt.plot(self.imu_time,z,color= 'b')
        # plt.xlabel('$Time (s)$')
        # plt.ylabel('$Yaw Angle (Degrees)$')
        # #plt.legend(['Gyroscope','Magnetometer'])
        # plt.title('Gyroscope vs Magnetometer Yaw Data')
        # plt.axis('equal')
        # plt.grid(True)
        # plt.show()

        #Magnetometer Raw vs Calibrated
        plt.figure("Magnetometer Raw vs Calibrated Yaw Plot")
        plt.plot(self.imu_time,yaw_mag_raw2,color= 'r')
        plt.plot(self.imu_time,yaw_mag_temp,color= 'y')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Yaw Angle (Degrees)$')
        plt.legend(['Raw','Calibrated'])
        plt.title('Magnetometer Raw vs Calibrated Yaw')
        # plt.axis('equal')
        plt.grid(True)

        plt.figure('Gyroscope vs Magnetometer Yaw Data')
        plt.plot(self.imu_time,yaw_gyro,color= 'r')
        plt.plot(self.imu_time,yaw_mag_temp,color = 'b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Yaw Angle (Degrees)$')
        plt.legend(['Gyroscope Yaw','Magnetometer Yaw'])
        plt.title('Gyroscope vs Magnetometer Yaw Data')
        plt.grid(True)
        #plt.show()

        plt.figure('LPF HPF and Complementary Filter Yaw')
        plt.plot(self.imu_time,y,color= 'r')
        plt.plot(self.imu_time,z,color='g')
        plt.plot(self.imu_time,final_yaw,color='b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Yaw Angle (Degrees)$')
        plt.legend(['Low Pass','High Pass','Complementary Filter'])
        plt.title('LPF CPF and Complementary Filter Yaw')
        plt.grid(True)
        # plt.close()
        #plt.show()

        plt.figure("Complementary Filter vs IMU Yaw")
        plt.plot(self.imu_time,self.unwrap2(np.rad2deg(self.theta_rad[:,2])-360),color= 'y')
        plt.plot(self.imu_time,final_yaw,color= 'r')
        #plt.plot(self.imu_time,final_yaw,color= 'b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Yaw Angle (Degrees)$')
        plt.legend(['IMU','Complementary Filter'])
        plt.title("Complementary Filter vs IMU Yaw")
        # plt.axis('equal')
        plt.grid(True)
        #plt.close()

    
    def moving_data_vel(self):
        acc_raw_x = self.acc_x
        self.acc_x = self.acc_x - np.mean(self.acc_x)
        acc_int_x = self.acc_x

        #GPS velocity Calculation
        vel = []
        for i in range(0,self.gps_x.shape[0]):
            if i == self.gps_x.shape[0]-1:
                vel.append(vel[-1])
                break
            y1 = self.gps_y[i]
            x1 = self.gps_x[i]
            y2 = self.gps_y[i+1]
            x2 = self.gps_x[i+1]

            vel_temp = ((y2-y1)**2+(x2-x1)**2)**0.5

            vel.append(vel_temp)
        
        #Jerk Calculation
        acc_diff = np.ediff1d(self.acc_x)
        acc_corrected = np.copy(self.acc_x)
        n = acc_diff.shape[0]
        j=150
        i=0
        tol = 0.3
        tol2 = 0.7
        while i<n:
            if np.mean(acc_diff[i:i+j])<tol and (abs(max(acc_corrected[i:i+j])-min(acc_corrected[i:i+j])))<tol2:
                acc_corrected[i:i+j] = acc_corrected[i:i+j] - np.mean(acc_corrected[i:i+j])
            i=i+int(j/2)

        
        # n = self.acc_z.shape[0]
        # i=0
        # while i<n:
        #     self.acc_z[i:i+200] = np.mean(self.acc_z[i:i+200])
        #     i=i+200

        int_velocity = 0
        acc_vel_list=[]
        lin_vel_list = []
        #acc_raw_x = acc_raw_x - np.mean(acc_raw_x)
        for i in range(0,acc_raw_x.shape[0]):
            if i == acc_raw_x.shape[0]-1:
                lin_vel_list.append(lin_vel_list[-1])
                break
            acc_vel_list = np.array([acc_raw_x[i],acc_raw_x[i+1]])
            lin_vel = int_velocity+np.trapz(acc_vel_list,dx=1)*0.025
            #lin_vel = integrate(acc_vel_list)
            int_velocity = lin_vel
            lin_vel_list.append(lin_vel)

        vel_acc_raw = lin_vel_list

        int_velocity = 0
        acc_vel_list=[]
        lin_vel_list = []
        #acc_int_x = acc_int_x - np.mean(acc_int_x)
        for i in range(0,acc_int_x.shape[0]):
            if i == acc_int_x.shape[0]-1:
                lin_vel_list.append(lin_vel_list[-1])
                break
            acc_vel_list = np.array([acc_int_x[i],acc_int_x[i+1]])
            lin_vel = int_velocity+np.trapz(acc_vel_list,dx=1)*0.025
            #lin_vel = integrate(acc_vel_list)
            int_velocity = lin_vel
            lin_vel_list.append(lin_vel)

        vel_int_raw = lin_vel_list

        #Velocity Calculation From Acceleration
        int_velocity = 0
        acc_vel_list=[]
        lin_vel_list = []
        #acc_corrected = acc_corrected - np.mean(acc_corrected)
        for i in range(0,acc_corrected.shape[0]):
            if i == acc_corrected.shape[0]-1:
                lin_vel_list.append(lin_vel_list[-1])
                break
            acc_vel_list = np.array([acc_corrected[i],acc_corrected[i+1]])
            lin_vel = int_velocity+np.trapz(acc_vel_list,dx=1)*0.025
            #lin_vel = integrate(acc_vel_list)
            if lin_vel < 0:
                lin_vel = 0
            int_velocity = lin_vel
            lin_vel_list.append(lin_vel)
        
        plt.figure("GPS and Accelerometer Velocities Raw")
        plt.plot(self.imu_time,vel_acc_raw,color= 'r')
        plt.plot(self.gps_time,vel,color= 'b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Velocity m/s$')
        plt.legend(['Accelerometer Velocity','GPS Velocity'])
        plt.title('GPS vs Accelerometer velocity before Adjustment')
        #plt.axis('equal')
        plt.grid(True)
        #plt.show()

        plt.figure("GPS and Accelerometer Velocities Intermediate")
        plt.plot(self.imu_time,vel_int_raw,color= 'r')
        plt.plot(self.gps_time,vel,color= 'b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Velocity m/s$')
        plt.legend(['Accelerometer Velocity','GPS Velocity'])
        plt.title('GPS vs Accelerometer velocity Intermediate')
        #plt.axis('equal')
        plt.grid(True)
        #plt.show()

        plt.figure("GPS and Accelerometer Velocities Corrected")
        plt.plot(self.imu_time,lin_vel_list,color= 'r')
        plt.plot(self.gps_time,vel,color= 'b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Velocity m/s$')
        plt.legend(['Accelerometer Velocity','GPS Velocity'])
        plt.title('GPS vs Accelerometer velocity after Adjustment')
        #plt.axis('equal')
        plt.grid(True)
        #plt.show()

        # plt.figure('Accelerations')
        # plt.plot(self.imu_time,self.acc_x,color= 'b')
        # plt.plot(self.imu_time,acc_corrected,color= 'r')
        # plt.xlabel('$Time (s)$')
        # plt.ylabel('$Velocity m/s$')
        # #plt.legend(['Gyroscope','Magnetometer'])
        # plt.title('Rate of change of Acceleration')
        # #plt.axis('equal')
        # plt.grid(True)
        # plt.close()

        #f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)

        self.vel_gps = np.array(vel)
        self.vel_acc = np.array(lin_vel_list)

        self.y_acc_obs = self.ang_vel_z * self.vel_acc

        plt.figure("Y Acceleration")
        plt.plot(self.imu_time,self.acc_y,color= 'b')
        plt.plot(self.imu_time,self.y_acc_obs,'r')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Acceleration m/s^2 $')
        plt.legend(['Y observed','Calculated (wX)'])
        plt.title('Y acceleration Comparison')
        #plt.axis('equal')
        plt.grid(True)

    def displacement_comparison(self):

        #IMU Distance Estimate
        prev_x_disp = 0
        x_displacement_acc = []
        n = self.vel_acc.shape[0]
        for i in range(0,n):
            if i == n-1:
                break
            x_displacement_acc_temp = prev_x_disp+np.trapz([self.vel_acc[i],self.vel_acc[i+1]],dx=1)*0.025
            prev_x_disp = x_displacement_acc_temp
            x_displacement_acc.append(x_displacement_acc_temp)
        x_displacement_acc.append(x_displacement_acc[-1]) #repeating the last value as we have 1 less data point due to integration

        #GPS Distance Estimate
        prev_x_disp = 0
        x_displacement_gps = []
        n = self.vel_gps.shape[0]
        for i in range(0,n):
            if i == n-1:
                break
            x_displacement_gps_temp = prev_x_disp+np.trapz([self.vel_gps[i],self.vel_gps[i+1]],dx=1)
            prev_x_disp = x_displacement_gps_temp
            x_displacement_gps.append(x_displacement_gps_temp)
        x_displacement_gps.append(x_displacement_gps[-1]) #repeating the last value as we have 1 less data point due to integration

        plt.figure("Distance")
        plt.plot(self.imu_time,x_displacement_acc,color= 'r')
        plt.plot(self.gps_time,x_displacement_gps,color= 'b')
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Distance m$')
        plt.legend(['Accelerometer','GPS'])
        plt.title('GPS vs IMU Distance')
        #plt.axis('equal')
        plt.grid(True)

        #Starting Point Adjustment and plot Rotation
        self.gps_x = self.gps_x - self.gps_x[0]
        self.gps_y = self.gps_y - self.gps_y[0]
        # plt.figure("X vs Y")
        # plt.plot(self.gps_x,self.gps_y,'.',color = 'b')
        # plt.plot(x_disp_acc,y_disp_acc,'.',color = 'r')
        # plt.xlabel('$Time (s)$')
        # plt.ylabel('$Distance m$')
        # plt.legend(['GPS','IMU Estimate'])
        # plt.title('GPS vs Accelerometer Distance')
        # #plt.axis('equal')
        # plt.grid(True)
        # plt.close()
        # print(np.arctan2(self.gps_x[4]-self.gps_x[1],self.gps_y[4]-self.gps_y[1])*180/np.pi)
        self.heading_angle = self.wrap(-np.deg2rad(self.heading_angle)) + np.arctan2(self.gps_y[4]-self.gps_y[1],self.gps_x[4]-self.gps_x[1])
        #self.heading_angle = self.wrap(np.deg2rad(self.heading_angle))
        vel_x = np.multiply(self.vel_acc,np.cos(self.heading_angle))
        vel_y = np.multiply(self.vel_acc,np.sin(self.heading_angle))

        # #X Displacement
        # prev_x_disp = 0
        # x_disp_acc = []
        # n = vel_x.shape[0]
        # for i in range(0,n):
        #     if i == n-1:
        #         break
        #     x_disp_acc_temp = prev_x_disp+np.trapz([vel_x[i],vel_x[i+1]],dx=1)
        #     prev_x_disp = x_disp_acc_temp
        #     x_disp_acc.append(x_disp_acc_temp)
        # x_disp_acc.append(x_disp_acc[-1]) #repeating the last value as we have 1 less data point due to integration

        #X Displacement
        prev_x_disp = 0
        x_disp_acc = []
        n = vel_x.shape[0]
        for i in range(0,n):
            if i == n-1:
                break
            x_disp_acc_temp = prev_x_disp+np.trapz([vel_x[i],vel_x[i+1]],dx=1)*0.025
            prev_x_disp = x_disp_acc_temp
            x_disp_acc.append(x_disp_acc_temp)
        x_disp_acc.append(x_disp_acc[-1]) #repeating the last value as we have 1 less data point due to integration
        x_disp_acc = np.array(x_disp_acc)

        #Y Displacement
        prev_y_disp = 0
        y_disp_acc = []
        n = vel_y.shape[0]
        for i in range(0,n):
            if i == n-1:
                break
            y_disp_acc_temp = prev_y_disp+np.trapz([vel_y[i],vel_y[i+1]],dx=1)*0.025
            prev_y_disp = y_disp_acc_temp
            y_disp_acc.append(y_disp_acc_temp)
        y_disp_acc.append(y_disp_acc[-1]) #repeating the last value as we have 1 less data point due to integration
        y_disp_acc = np.array(y_disp_acc)
        
        diff_x = []
        diff_y = []
        j=0
        for i in range(0,self.gps_x.shape[0]):

            diff_x.append(self.gps_x[i] - x_disp_acc[j])
            diff_y.append(self.gps_y[i] - y_disp_acc[j])
            j = j+40
            if(j>x_disp_acc.shape[0]):
                break
        
        diff_x = np.array(diff_x)
        diff_y = np.array(diff_y)

        diff = np.sqrt(np.square(diff_x)+np.square(diff_y))
        

        plt.figure("Difference")
        
        plt.xlabel('$Time (s)$')
        plt.ylabel('$Distance m$')
        plt.plot(self.gps_time,diff,'.')
        plt.legend(['Accelerometer','Gyroscope'])
        plt.title('GPS vs Accelerometer Distance')
        #plt.axis('equal')
        plt.grid(True)
        plt.show()

        plt.figure("X vs Y")
        plt.plot(self.gps_x,self.gps_y,'.',color = 'b')
        plt.plot(x_disp_acc,y_disp_acc,'.',color = 'r')
        plt.xlabel('$X Displacement (m)$')
        plt.ylabel('$Y Displacement (m) $')
        plt.legend(['GPS','IMU'])
        plt.title('GPS vs IMU Position Estimate')
        #plt.axis('equal')
        plt.grid(True)
        plt.show()

        # plt.figure("Corrected Angles")
        # #plt.plot(self.imu_time,self.wrap(np.deg2rad(self.heading_angle))*180/np.pi,color = 'b')
        # plt.plot(self.imu_time,self.heading_angle*180/np.pi,'.',color = 'r')
        # plt.xlabel('$Time (s)$')
        # plt.ylabel('$Distance m$')
        # plt.legend(['Accelerometer','Gyroscope'])
        # plt.title('GPS vs Accelerometer Distance')
        # #plt.axis('equal')
        # plt.grid(True)
        # plt.show()


    def filter_accel_y(self):
        T = 0.5        # Sample Period
        fs = 40       # sample rate, Hz
        low_cutoff = 0.5   # desired cutoff frequency of the filter, Hz ,
        nyq = 0.5 * fs  # Nyquist Frequency
        order = 1       # sin wave can be approx represented as quadratic
        n = int(T * fs) # total number of samples
        low_normal_cutoff = low_cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, low_normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, self.acc_y)
        plt.figure("X vs Y")
        plt.plot(self.imu_time,y,color = 'b')
        plt.plot(self.imu_time,self.y_acc_obs,color = 'r')
        plt.xlabel('$Time (s) $')
        plt.ylabel('$Acceleration (m/s^2)$')
        plt.legend(['Filtered Y Acceleration Observed','Calculated Acceleration'])
        plt.title('Filtered Acceleration')
        #plt.axis('equal')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    obj = calibrate()
    obj.calibration()
    obj.calibration_plots()
    plt.show()
    obj.moving_data_correction()
    obj.moving_data_yaw()
    obj.moving_data_vel()
    obj.displacement_comparison()
    obj.filter_accel_y()