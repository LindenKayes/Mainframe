from machine import Pin, PWM, I2C
import time

#PWM is pulse width modulation

#Pins - OUT1 + OUT2
in1 = Pin(6,Pin.OUT) 
in2 = Pin(7,Pin.OUT)
enbA = PWM(Pin(8))

#OUT3 and OUT4
in3 = Pin(4, Pin.OUT)
in4 = Pin(3, Pin.OUT)
enbB = PWM(Pin(2))

#OUT 1 and OUT 2 for the second motor controller
#Second motor controller has suffix B

in1B = Pin(15, Pin.OUT)
in2B = Pin(14, Pin.OUT)
enbBA = PWM(Pin(12))

#OUT3 and OUT4 for the second motor controller
in3B = Pin(16, Pin.OUT)
in4B = Pin(19, Pin.OUT)
enbBB= PWM(Pin(18))

#Set directions of polarity for each magnet 
in1.low()
in2.high()    #Magnet 1

in3.high()
in4.low()    #Magnet 2

in1B.high()
in2B.low()    #Magnet 3 

in3B.low()
in4B.high()     #Magnet 4 

enbBB.freq(1000)    #Set Frequency of each to base of 1htz
enbBA.freq(1000)
enbA.freq(1000)
enbB.freq(1000)


def Strong(x):   #Strong / varies from magnet to magnet 
    x.duty_u16(65535)
    

def Medium():             #Maintain semi-strong balance
    enbA.duty_u16(40000)
    enbB.duty_u16(40000)
    enbBA.duty_u16(40000)
    enbBB.duty_u16(40000)
    
def Weak():                 #Maintain weak balance (easy to offset)
    enbA.duty_u16(25000)
    enbB.duty_u16(25000)
    enbBA.duty_u16(25000)
    enbBB.duty_u16(25000)

def Off():                      #Stop magnetic output 
    enbA.duty_u16(0)
    enbB.duty_u16(0)
    enbBA.duty_u16(0)
    enbBB.duty_u16(0)


#CODE FROM REPOSITORY STARTS HERE
#this code was gotten at a repository: linked here https://github.com/ellenrapps/Road-to-Autonomous-Drone-Using-Raspberry-Pi-Pico/blob/main/pico_mpu6050.py
#import PIN and I2C from machine library 


#Define I2C bus 
i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1))

#Device address on the I2C bus
MPU6050_ADDR = 0x68

#PWR_MGMT_1 memory address
MPU6050_PWR_MGMT_1 = 0x6B

#Accelerometer and Gyroscope's high and low register for each axis
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_XOUT_L = 0x3C
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_YOUT_L = 0x3E
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_ACCEL_ZOUT_L = 0x40
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_XOUT_L = 0x44
MPU6050_GYRO_YOUT_H = 0x45
MPU6050_GYRO_YOUT_L = 0x46
MPU6050_GYRO_ZOUT_H = 0x47
MPU6050_GYRO_ZOUT_L = 0x48

#Accelerometer's LSB/g (least significant bits per gravitational force) sensitivity
MPU6050_LSBG = 16384.0

#Gyroscope's LSB/g sensitivity
MPU6050_LSBDS = 131.0 


#Set all bits in the PWR_MGMT_1 register to 0
def mpu6050_init(i2c):
    i2c.writeto_mem(MPU6050_ADDR, MPU6050_PWR_MGMT_1, bytes([0]))


def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) |  (l[0] ^ 255) + 1


#Get Accelerometer values
def mpu6050_get_accel(i2c):
    accel_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1)
    accel_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L, 1)
    accel_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, 1)
    accel_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L, 1)
    accel_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 1)
    accel_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L, 1)
    
    return [combine_register_values(accel_x_h, accel_x_l) / MPU6050_LSBG,
            combine_register_values(accel_y_h, accel_y_l) / MPU6050_LSBG,
            combine_register_values(accel_z_h, accel_z_l) / MPU6050_LSBG]


#Get Gyroscope values
def mpu6050_get_gyro(i2c):
    gyro_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1)
    gyro_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_L, 1)
    gyro_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_H, 1)
    gyro_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_L, 1)
    gyro_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 1)
    gyro_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_L, 1)
    
    return [combine_register_values(gyro_x_h, gyro_x_l) / MPU6050_LSBDS,
            combine_register_values(gyro_y_h, gyro_y_l) / MPU6050_LSBDS,
            combine_register_values(gyro_z_h, gyro_z_l) / MPU6050_LSBDS]


if __name__ == "__main__":
    i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1))
    mpu6050_init(i2c)
    
#CODE FROM REPOSITORY ENDS HERE

#Adjustment - Strength 
while True:
    gx, gy, gz = mpu6050_get_gyro(i2c)
    Weak()    #set to weak as base line for stability 
    if gy > 40:   #when y axis shifts down on right side  
        Strong(enbA)  #push up on right side 
        Strong(enbB)
    if gy < -40:	#when y axis shifts down on left side
        Strong(enbBA)   #push up on right side
        Strong(enbBB)
    if gy > -40 and gy < 40:  #if mostly stable, keep mostly stable / reset to weak
        Weak()
        
        
    