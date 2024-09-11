import sys
import time
import math
import IMU
import datetime
import os

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070                  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40                      # Complementary filter constant
MAG_LPF_FACTOR = 0.4            # Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4            # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay

class Compass:
    def __init__(self):
        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            print(" No BerryIMU found... exiting ")
            sys.exit()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

        ################# Compass Calibration values ############
        self.magXmin =  -3768
        self.magYmin =  392
        self.magZmin =  -2212
        self.magXmax =  -155
        self.magYmax =  1882
        self.magZmax =  -1319

        self.Q_angle = 0.02
        self.Q_gyro = 0.0015
        self.R_angle = 0.005
        self.y_bias = 0.0
        self.x_bias = 0.0
        self.XP_00 = 0.0
        self.XP_01 = 0.0
        self.XP_10 = 0.0
        self.XP_11 = 0.0
        self.YP_00 = 0.0
        self.YP_01 = 0.0
        self.YP_10 = 0.0
        self.YP_11 = 0.0
        self.KFangleX = 0.0
        self.KFangleY = 0.0

        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0
        self.CFangleX = 0.0
        self.CFangleY = 0.0
        self.CFangleXFiltered = 0.0
        self.CFangleYFiltered = 0.0
        self.kalmanX = 0.0
        self.kalmanY = 0.0
        self.oldXMagRawValue = 0
        self.oldYMagRawValue = 0
        self.oldZMagRawValue = 0
        self.oldXAccRawValue = 0
        self.oldYAccRawValue = 0
        self.oldZAccRawValue = 0
        self.velocityX = 0.0
        self.velocityY = 0.0
        self.velocityZ = 0.0
        self.prev_velocityX = 0.0
        self.prev_velocityY = 0.0
        self.prev_velocityZ = 0.0
        self.prev_timestamp = datetime.datetime.now()

        #Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
        self.acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
        self.mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE
    
    def kalmanFilterX(self, accAngle, gyroRate, DT):
        x = 0.0
        S = 0.0

        # Update the KFangleX using class variable
        self.KFangleX = self.KFangleX + DT * (gyroRate - self.x_bias)

        # Update covariance matrix elements using class variables
        self.XP_00 = self.XP_00 + (-DT * (self.XP_10 + self.XP_01) + self.Q_angle * DT)
        self.XP_01 = self.XP_01 + (-DT * self.XP_11)
        self.XP_10 = self.XP_10 + (-DT * self.XP_11)
        self.XP_11 = self.XP_11 + (self.Q_gyro * DT)

        # Calculate the angle and the Kalman gain
        x = accAngle - self.KFangleX
        S = self.XP_00 + self.R_angle
        K_0 = self.XP_00 / S
        K_1 = self.XP_10 / S

        # Update KFangleX and bias using the Kalman gain
        self.KFangleX = self.KFangleX + (K_0 * x)
        self.x_bias = self.x_bias + (K_1 * x)

        # Update the covariance matrix
        self.XP_00 = self.XP_00 - (K_0 * self.XP_00)
        self.XP_01 = self.XP_01 - (K_0 * self.XP_01)
        self.XP_10 = self.XP_10 - (K_1 * self.XP_00)
        self.XP_11 = self.XP_11 - (K_1 * self.XP_01)

        return self.KFangleX
    
    def kalmanFilterY(self, accAngle, gyroRate, DT):
        y = 0.0
        S = 0.0

        # Update KFangleY using class variable
        self.KFangleY = self.KFangleY + DT * (gyroRate - self.y_bias)

        # Update covariance matrix elements using class variables
        self.YP_00 = self.YP_00 + (-DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT)
        self.YP_01 = self.YP_01 + (-DT * self.YP_11)
        self.YP_10 = self.YP_10 + (-DT * self.YP_11)
        self.YP_11 = self.YP_11 + (self.Q_gyro * DT)

        # Calculate the angle and the Kalman gain
        y = accAngle - self.KFangleY
        S = self.YP_00 + self.R_angle
        K_0 = self.YP_00 / S
        K_1 = self.YP_10 / S

        # Update KFangleY and bias using the Kalman gain
        self.KFangleY = self.KFangleY + (K_0 * y)
        self.y_bias = self.y_bias + (K_1 * y)

        # Update the covariance matrix
        self.YP_00 = self.YP_00 - (K_0 * self.YP_00)
        self.YP_01 = self.YP_01 - (K_0 * self.YP_01)
        self.YP_10 = self.YP_10 - (K_1 * self.YP_00)
        self.YP_11 = self.YP_11 - (K_1 * self.YP_01)

        return self.KFangleY

    
    def getHeading(self):

        # Read the accelerometer, gyroscope, and magnetometer values
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        # Apply compass calibration
        MAGx -= (self.magXmin + self.magXmax) / 2
        MAGy -= (self.magYmin + self.magYmax) / 2
        MAGz -= (self.magZmin + self.magZmax) / 2

        # Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - self.prev_timestamp
        self.prev_timestamp = datetime.datetime.now()
        LP = b.microseconds / (1000000 * 1.0)

        # Apply low pass filter
        MAGx = MAGx * MAG_LPF_FACTOR + self.oldXMagRawValue * (1 - MAG_LPF_FACTOR)
        MAGy = MAGy * MAG_LPF_FACTOR + self.oldYMagRawValue * (1 - MAG_LPF_FACTOR)
        MAGz = MAGz * MAG_LPF_FACTOR + self.oldZMagRawValue * (1 - MAG_LPF_FACTOR)
        ACCx = ACCx * ACC_LPF_FACTOR + self.oldXAccRawValue * (1 - ACC_LPF_FACTOR)
        ACCy = ACCy * ACC_LPF_FACTOR + self.oldYAccRawValue * (1 - ACC_LPF_FACTOR)
        ACCz = ACCz * ACC_LPF_FACTOR + self.oldZAccRawValue * (1 - ACC_LPF_FACTOR)

        self.oldXMagRawValue = MAGx
        self.oldYMagRawValue = MAGy
        self.oldZMagRawValue = MAGz
        self.oldXAccRawValue = ACCx
        self.oldYAccRawValue = ACCy
        self.oldZAccRawValue = ACCz

        # Median filter for accelerometer
        for x in range(ACC_MEDIANTABLESIZE - 1, 0, -1):
            self.acc_medianTable1X[x] = self.acc_medianTable1X[x - 1]
            self.acc_medianTable1Y[x] = self.acc_medianTable1Y[x - 1]
            self.acc_medianTable1Z[x] = self.acc_medianTable1Z[x - 1]

        self.acc_medianTable1X[0] = ACCx
        self.acc_medianTable1Y[0] = ACCy
        self.acc_medianTable1Z[0] = ACCz

        self.acc_medianTable2X = self.acc_medianTable1X[:]
        self.acc_medianTable2Y = self.acc_medianTable1Y[:]
        self.acc_medianTable2Z = self.acc_medianTable1Z[:]

        self.acc_medianTable2X.sort()
        self.acc_medianTable2Y.sort()
        self.acc_medianTable2Z.sort()

        ACCx = self.acc_medianTable2X[int(ACC_MEDIANTABLESIZE / 2)]
        ACCy = self.acc_medianTable2Y[int(ACC_MEDIANTABLESIZE / 2)]
        ACCz = self.acc_medianTable2Z[int(ACC_MEDIANTABLESIZE / 2)]

        # Median filter for magnetometer
        for x in range(MAG_MEDIANTABLESIZE - 1, 0, -1):
            self.mag_medianTable1X[x] = self.mag_medianTable1X[x - 1]
            self.mag_medianTable1Y[x] = self.mag_medianTable1Y[x - 1]
            self.mag_medianTable1Z[x] = self.mag_medianTable1Z[x - 1]

        self.mag_medianTable1X[0] = MAGx
        self.mag_medianTable1Y[0] = MAGy
        self.mag_medianTable1Z[0] = MAGz

        self.mag_medianTable2X = self.mag_medianTable1X[:]
        self.mag_medianTable2Y = self.mag_medianTable1Y[:]
        self.mag_medianTable2Z = self.mag_medianTable1Z[:]

        self.mag_medianTable2X.sort()
        self.mag_medianTable2Y.sort()
        self.mag_medianTable2Z.sort()

        MAGx = self.mag_medianTable2X[int(MAG_MEDIANTABLESIZE / 2)]
        MAGy = self.mag_medianTable2Y[int(MAG_MEDIANTABLESIZE / 2)]
        MAGz = self.mag_medianTable2Z[int(MAG_MEDIANTABLESIZE / 2)]

        # Convert Gyro raw to degrees per second
        rate_gyr_x = GYRx * G_GAIN
        rate_gyr_y = GYRy * G_GAIN
        rate_gyr_z = GYRz * G_GAIN

        # Calculate the angles from the gyro.
        self.gyroXangle += rate_gyr_x * LP
        self.gyroYangle += rate_gyr_y * LP
        self.gyroZangle += rate_gyr_z * LP

        # Convert Accelerometer values to degrees
        AccXangle = math.atan2(ACCy, ACCz) * RAD_TO_DEG
        AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG

        # Adjust AccYangle
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0

        # Complementary filter used to combine the accelerometer and gyro values.
        self.CFangleX = AA * (self.CFangleX + rate_gyr_x * LP) + (1 - AA) * AccXangle
        self.CFangleY = AA * (self.CFangleY + rate_gyr_y * LP) + (1 - AA) * AccYangle

        # Kalman filter used to combine the accelerometer and gyro values.
        self.kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y, LP)
        self.kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x, LP)

        # Calculate heading
        heading = 180 * math.atan2(MAGy, MAGx) / M_PI

        if heading < 0:
            heading += 360

        # Tilt compensated heading
        accXnorm = ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm / math.cos(pitch))

        if IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3:  # LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
            magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) - MAGz * math.sin(roll) * math.cos(pitch)
        else:  # LSM9DS1
            magXcomp = MAGx * math.cos(pitch) - MAGz * math.sin(pitch)
            magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) + MAGz * math.sin(roll) * math.cos(pitch)

        tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp) / self.M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360

        return heading, tiltCompensatedHeading
    