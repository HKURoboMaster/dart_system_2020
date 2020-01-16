# missile - By: Jiarui - 周三 1月 15 2020

import sensor, image, time, math, imu
from pyb import Servo
from mpu9250 import MPU9250
IMU = MPU9250('X')

time_shut_down = 5 #the time to shut down under no condition (in unit of s)
green_threshold   = ( 85, 90,-75,-55, 5, 35)
s1 = Servo(1) #aileron 1 horizontal left
s2 = Servo(2) #aileron 2 horizontal right
s3 = Servo(3) #aileron 3 vertical tail
K = 50 #the value should be measured
X = 160 #total pixels in x
Y = 120 #total pixels in y
KP = 1 #the value should be measured
k_pitch = 1 #the value should be measured
k_yaw = 1 #the value should be measured
k_roll = 1 #the value should be measured


def initialize_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time = 100)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    return


def readin_imu():
    acceleration, gyro, mag = IMU.sensors()

    acceleration_overall = 0.0
    acceleration_overall += acceleration.x ** 2
    acceleration_overall += acceleration.y ** 2
    acceleration_overall += acceleration.z ** 2
    acceleration_overall = acceleration_overall ** 0.5

    return gyro, acceleration_overall


def initialize_imu():
    IMU.wake()
    return


def readin_sensor():
    img = sensor.snapshot()

    blobs = img.find_blobs([green_threshold])
    area_total, x_pix, y_pix = (0.0, 0.0, 0.0)

    if blobs:
        for b in blobs:
            ROI = (b[0],b[1],b[2],b[3])
            img.draw_rectangle(b[0:4]) # rect
            img.draw_cross(b[5], b[6]) # cx, cy
            area = b[2] * b[3]
            x_pix += b[5]*area
            y_pix += b[6]*area
            area_total += area

    x_pix /= area_total
    y_pix /= area_total
    x_angle = math.atan(K * x_pix)
    y_angle = math.atan(K * y_pix)
    angle = (x_angle, y_angle)

    if len(blobs) == 1:
        b = blobs[0]
        Lm = (b[2]+b[3])/2
        distance = K/Lm

    return distance, angle


def initialize_servo():
    s1.angle(0)
    s2.angle(0)
    s3.angle(0)
    return


def servo_act(s1_angle, s2_angle, s3_angle):
    s1.angle(s1_angle)
    s2.angle(s2_angle)
    s3.angle(s3_angle)
    return

'''
def calculate_kp(flydata):
    kp = 1
    return kp
'''

def imu_stablize(s1_angle, s2_angle, s3_angle, flydata):
    angular_velocity = flydata[0] #[pitch, roll, yaw]

    #pitch Correction
    s1_angle += k_pitch * angular_velocity[0]
    s2_angle -= k_pitch * angular_velocity[0]

    #roll Correction
    s1_angle += k_roll * angular_velocity[1]
    s2_angle += k_roll * angular_velocity[1]

    #yaw Correction
    s3_angle += k_yaw * angular_velocity[2]
    return s1_angle, s2_angle, s3_angle


def servo_change_camera_correction(differ, flydata):
    x_angle, y_angle = differ

    #convert the angle between flight route and the target to the rotate
    #angle of the servos.
    '''KP = calculate_kp(flydata)'''
    s1_angle = KP * y_angle
    s2_angle = -KP * y_angle
    s3_angle =  KP * x_angle

    #prevent abnormal rotation
    s1_angle, s2_angle, s3_angle = imu_stablize(s1_angle, s2_angle, s3_angle, flydata)

    servo_act(s1_angle, s2_angle, s3_angle)
    return


def servo_change_imu_stabilize(differ, flydata):
    s1_angle, s2_angle, s3_angle = (0.0, 0.0, 0.0)
    s1_angle, s2_angle, s3_angle = imu_stablize(s1_angle, s2_angle, s3_angle, flydata)

    servo_act(s1_angle, s2_angle, s3_angle)
    return


def target_found():
    img = sensor.snapshot()
    blobs = img.find_blobs([green_threshold])
    if (len(blobs) >= 1):
        green_block_found = True
    else:
        green_block_found = False
    return green_block_found


def imu_correction():
    flydata = readin_imu()
    servo_change_imu_stabilize(differ, flydata)
    return


def camera_correction():
    target_position = readin_sensor()
    flydata = readin_imu()
    differ = target_position[1]
    servo_change_camera_correction(differ, flydata)
    return


def main():
    initialize_imu()
    initialize_servo()

#待机状态 以每秒30帧检测加速度 判断是否起飞
    flydata = readin_imu()
    while (flydata[1] < 0.5): # acceleration < 0.5g
        clock.tick(30)
        flydata = readin_imu()

#起飞后 未找到目标 以大约每秒10帧画面搜索目标
    initialize_sensor()
    t = time.clock()

    f = target_found()
    while (f == False):
        clock.tick(10)
        f = target_found()

#第一次找到目标后 以每秒30帧更新目标状态并调整舵机
    while (time.clock() < time_shut_down):
        clock.tick(30)
        if (target_found()):
            camera_correction()
        else:
            imu_correction()

    IMU.sleep()
    initialize_servo()
    return


main()
