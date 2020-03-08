# missile - By: Jiarui - 周三 1月 15 2020

import sensor, image, time, math, imu
from pyb import Servo
from mpu9250 import MPU9250
IMU = MPU9250('X')

time_shut_down = 5 #the time to shut down under no condition (in unit of s)
limit_ready_to_launch = 5 #the longest time between ready and launch (in unit of s)
limit_found_to_hit = 5 #the longest time between found and hit (in unit of s)
green_threshold   = ( 85, 90,-75,-55, 5, 35)
s1 = Servo(1) #aileron 1 horizontal left
s2 = Servo(2) #aileron 2 horizontal right
s3 = Servo(3) #aileron 3 vertical tail
time_found = 0.0
time_ready = 0.0
time_launch = 0.0 #use global variables to write down switch points
K = 50 #the value should be measured
X = 160 #total pixels in x
Y = 120 #total pixels in y
KP = 1 #the value should be measured
k_pitch = 1 #the value should be measured
k_yaw = 1 #the value should be measured
k_roll = 1 #the value should be measured

'''
def calculate_kp(flydata):
    kp = 1
    return kp


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
    KP = calculate_kp(flydata)
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
'''

def initialize_servo():
    s1.angle(0)
    s2.angle(0)
    s3.angle(0)
    return


def initialize_imu():
    IMU.wake()
    return


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

    return gyro, acceleration_overall, acceleration.x, acceleration.y, acceleration.z


def readin_sensor(img):
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


def check_ready():
    global last_y_accel=0
    global last_z_accel=0
    global last_state="stop"
    flydata = readin_imu()
    y_accel= flydata[2]
    z_accel= flydata[3]
    if (-0.05<y_accel<0.05 and -0.05<z_accel<0.05):#not orbiting, can be modified
        if (last_y_accel<=0 and last_z_accel=>0 and last_state="move"):#check whether move to lunch position
            ifready=True
        else:
            last_y_accel=0
            last_z_accel=0
            last_state="stop"
            ifready=False
    else:#orbiting
        last_y_accel=y_accel
        last_z_accel=z_accel
        last_state="move"
        ifready=False
    return ifready


def check_launch():
    flydata = readin_imu()
    iflaunch = (flydata[1] < 0.5)
    return iflaunch


def check_imu_data(flydata):
    flydata = readin_imu()
    if ( flydata[1] <= 2 && flydata[2] <= 1 && flydata[3] <= 1 && flydata[4] <= 1):
        if_imu_correct = True
    else:
        if_imu_correct = False
    return if_imu_correct


def check_target_found(img):
    blobs = img.find_blobs([green_threshold])
    if (len(blobs) >= 1):
        green_block_found = True
    else:
        green_block_found = False
    return green_block_found


def check_hit():
    check_target_found(img)
    if ( green_block_found == 1 ):
        blobs = img.find_blobs([green_threshold])
        area_total = 0.0
        if blobs:
            for b in blobs:
                ROI = (b[0],b[1],b[2],b[3])
                area = b[2] * b[3]
                area_total += area
        if ( area >= 1300 ):
            ifhit = True
        else:
            ifhit = False
    else:
        ifhit = False
    return ifhit


def calculate_kp():
    flydata = readin_imu()

    return KP


def calculate_imu_change(flydata):
    KP = calculate_kp()
    s1_angle = -KP * flydata[4]
    s2_angle = KP * flydata[4]
    s3_angle = -KP * flydata[3]
    return s1_angle, s2_angle, s3_angle


def calculate_camera_change(img):
    target_position = readin_sensor(img)
    differ = target_position[1]
    x_angle, y_angle = differ

    #convert the angle between flight route and the target to the rotate
    #angle of the servos.
    KP = 10 * calculate_kp()
    s1_angle = KP * y_angle
    s2_angle = -KP * y_angle
    s3_angle = KP * x_angle
    return s1_angle, s2_angle, s3_angle


def calculate_imu_adapt(s1_angle, s2_angle, s3_angle):
    flydata = readin_imu()
    if (check_imu_data(flydata)):
    
    return s1_angle, s2_angle, s3_angle


def act_servo(s1_angle, s2_angle, s3_angle):
    s1.angle(s1_angle)
    s2.angle(s2_angle)
    s3.angle(s3_angle)
    return


def control_shutdown():
    IMU.sleep()
    initialize_servo()
    #command needed: turn off camera
    '''
    control_rest() #should restart or not?
    '''
    return 0


def control_aim(img0):
    global time_found
    img = img0
    t2 = time.clock()
    while ( t2 - time_found < limit_found_to_hit ):
        clock.tick(50)
        s1_angle, s2_angle, s3_angle = calculate_camera_change(img)
        s1_angle, s2_angle, s3_angle = calculate_imu_adapt(s1_angle, s2_angle, s3_angle)
        act_servo(s1_angle, s2_angle, s3_angle)
        img = sensor.snapshot()
        iffound = check_target_found(img)
        if (! iffound):
            i = 0
            while (i < 3) and (! iffound):
                i += 1
                img = sensor.snapshot()
                iffound = check_target_found(img)
            if (i == 3):
                control_stablize() #check for 3 times
        if (check_hit()):
            break
    control_shutdown()
    return 0


def control_stablize():
    global time_launch
    img = sensor.snapshot()
    iffound = check_target_found(img)
    while (! iffound):
        clock.tick(30)
        flydata = readin_imu()
        if (check_imu_data(flydata)):
            s1_angle, s2_angle, s3_angle = calculate_imu_change(flydata)
            act_servo(s1_angle, s2_angle, s3_angle)
        img = sensor.snapshot()
        iffound = check_target_found(img)
    global time_found
    time_found = time.clock()
    control_aim(img)
    return 0


def control_launch():
    global time_ready
    iflaunch = check_launch()
    while (! iflaunch):
        clock.tick(50)
        t1 = time.clock()
        if (t1 - time_ready > limit_ready_to_launch):
            control_ready()
        iflaunch = check_launch()
    global time_launch
    time_launch = time.clock()
    control_stablize()
    return 0


def control_ready():
    ifready = check_ready()
    while (! ifready):
        clock.tick(10)
        ifready = check_ready()
    initialize_sensor()
    global time_ready
    time_ready = time.clock()
    control_launch()
    return 0


def control_rest():
    initialize_servo()
    initialize_imu()
    t = time.clock()
    global time_found
    time_found = time.clock()
    global time_launch
    time_launch = time.clock()
    global time_ready
    time_ready = time.clock()
    return t


def main():
    t0 = control_rest()
    control_ready()
    return 0


main()
