from XRPLib.board import Board
from XRPLib.encoder import Encoder
from XRPLib.motor import Motor
from XRPLib.imu import IMU
from XRPLib.differential_drive import DifferentialDrive
import time, math
from machine import Timer

board = Board.get_default_board()
imu = IMU.get_default_imu()
diffDrive = DifferentialDrive.get_default_differential_drive()

hardware_timer_period = 0.05

x_positions = []
y_positions = []
xkf_positions = []
ykf_positions = []

def matrix_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

class PositionEstimation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

        self.x_kf = 0
        self.y_kf = 0
        self.theta_kf = 0
        self.prev_theta_kf = 0

        # Note: Only using kalman filter for theta in this lab
        self.P = [
            [0.1,0.0,0.0],
            [0.0,0.1,0.0],
            [0.0,0.0,0.1]
        ]
        self.Q = [
            [0.01,0.0,0.0],
            [0.0,0.01,0.0],
            [0.0,0.0,0.01]
        ]
        self.R = [ #20 0.01
            [0.05,0.0,0.0],
            [0.0,0.05,0.0],
            [0.0,0.0,10]
        ]

        self.track_width = 15.5 #cm
        self.wheel_diameter = 6
        self.RPMtoCMPS = (math.pi* self.wheel_diameter) / 60
        self.CMPStoRPM = 60 / (math.pi*self.wheel_diameter)
    
    def get_filtered_imu(self):
        readings = []
        for i in range(5):
            d = imu.get_gyro_z_rate()*(math.pi / 180000)
            readings.append(d)

        readings.sort()
        return readings[2]  # Return the median
    
    def kalman_filter(self, w_enc, w_imu):
        # Prediction step
        self.prev_theta_kf = self.theta_kf
        theta_measure = self.theta_kf + w_imu*hardware_timer_period 
        theta_pred = self.prev_theta_kf + w_enc * hardware_timer_period 
        self.P = matrix_add(self.P,self.Q)

        # Update step
        P_theta = self.P[2][2]
        R_theta = self.R[2][2]
        K = P_theta/(P_theta + R_theta)

        self.theta_kf = theta_pred + K*(theta_measure - theta_pred) 
        self.P[2][2] = (1-K)*self.P[2][2]

    def update_coord(self):
        right_motor_speed = diffDrive.right_motor.get_speed()*self.RPMtoCMPS
        left_motor_speed = diffDrive.left_motor.get_speed()*self.RPMtoCMPS
        w_enc  = (right_motor_speed-left_motor_speed)/self.track_width
        
        w_imu = self.get_filtered_imu()

        # State estimate with forward kinematics
        if w_enc == 0: 
            V = (left_motor_speed+right_motor_speed)*0.5
            self.x += V*math.cos(self.theta)*hardware_timer_period
            self.y += V*math.sin(self.theta)*hardware_timer_period
        else:
            R = self.track_width*0.5*(right_motor_speed+left_motor_speed)/(right_motor_speed-left_motor_speed)
            self.x += -R*math.sin(self.theta) + R*math.sin(self.theta+w_enc*hardware_timer_period)
            self.y += R*math.cos(self.theta) - R*math.cos(self.theta+w_enc*hardware_timer_period)
            self.theta += w_enc*hardware_timer_period

        # Kalman filter theta prediction + update
        self.kalman_filter(w_enc, w_imu)
        
        # Kalman filter estimates for x and y
        if w_enc == 0:
            V = (left_motor_speed + right_motor_speed) * 0.5
            self.x_kf += V * math.cos(self.prev_theta_kf) * hardware_timer_period
            self.y_kf += V * math.sin(self.prev_theta_kf) * hardware_timer_period
        else:
            R = self.track_width * 0.5 * (right_motor_speed + left_motor_speed) / (right_motor_speed - left_motor_speed)
            self.x_kf += -R * math.sin(self.prev_theta_kf) + R * math.sin(self.theta_kf)
            self.y_kf += R * math.cos(self.prev_theta_kf) - R * math.cos(self.theta_kf)

        x_positions.append(self.x)
        y_positions.append(self.y)
        xkf_positions.append(self.x_kf)
        ykf_positions.append(self.y_kf)

        if self.display:
            print('FK')
            print(self.x, self.y)
            print('KF')
            print(self.x_kf, self.y_kf, self.theta_kf)

    def go(self):
        timer = Timer()
        self.display = False
        board.wait_for_button()

        timer.init(period=int(hardware_timer_period * 1000),
           mode=Timer.PERIODIC, 
           callback=lambda t: kinematics.update_coord())

        # Set up combo of straight trajectories and point turns
        diffDrive.set_speed(20,20)
        time.sleep(3)
        self.display = True # Set this variable to True for 50 ms to print current state estimate
        time.sleep(0.05)
        self.display = False
        diffDrive.set_speed(-80,80)
        time.sleep(4) 
        self.display = True
        time.sleep(0.05)
        self.display = False
        diffDrive.set_speed(40,40)
        time.sleep(1)
        self.display = True
        time.sleep(0.05)
        self.display = False
        diffDrive.set_speed(0,0)
        time.sleep(3)

kinematics = PositionEstimation()
kinematics.go()

print('X POSITIONS')
print(x_positions)
print('Y POSITIONS')
print(y_positions)
print('X KF POSITIONS')
print(xkf_positions)
print('Y KF POSITIONS')
print(ykf_positions)
