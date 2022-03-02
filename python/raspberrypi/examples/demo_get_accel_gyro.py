# -*- coding:utf-8 -*-

'''
   @file demo_get_accel_gyro.py
   @brief Get BMI160 6-axis sensor data including gyroscope, accelerometer, etc.
   @n ----------------------------------------------------------------------------------------
   @n  Sensor      |                                     raspberry pi                         |
   @n -----------------------------------------------------------------------------------------
   @n  +           | 3V3/VCC                                                   |   3V3/VCC    |
   @n  -           | GND                                                       |      GND     |
   @n  C           | connected to scl of MCU's IIC                             |  SCL/3(BCM)  |
   @n  D           | connected to sda of MCU's IIC                             |  SDA/2(BCM)  |
   @n  INT1        | connected to the external interrupt IO pin of MCU         |  floating    |
   @n  INT2        | connected to the external interrupt IO pin of MCU         |  floating    |
   @n  SDO         | connected to the IO pin of MCU or connected to GND or VCC |  floating    |
   @n ----------------------------------------------------------------------------------------
  
   @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
   @licence     The MIT License (MIT)
   @author [Arya](xue.peng@dfrobot.com)
   @version  V1.0
   @date  2021-06-11
   @url from https://github.com/DFRobot/DFRobot_BME280
'''

import sys
import os
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_BMI160 import *

'''
  @brief The constructor of the BMI160 sensor using IIC communication.
  @param addr:  7-bit IIC address, controlled by SDO pin.
  @n     BMI160_IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
  @n     BMI160_IIC_ADDR_SDO_L or 0x68:  SDO pull down.
'''
bmi = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_H)

if __name__ == "__main__":
  '''
    @brief Initialize sensor
    @return Error code:
    BMI160_OK                         or  0 : init succeeded, no error
    BMI160_E_NULL_PTR                 or -1 : the parameters is empty
    BMI160_E_COM_FAIL                 or -2 : interaction failed
    BMI160_E_DEV_NOT_FOUND            or -3 : device not connected
    BMI160_E_OUT_OF_RANGE             or -4 : out of sensor range
    BMI160_E_INVALID_INPUT            or -5 : invalid input
    BMI160_E_ACCEL_ODR_BW_INVALID     or -6 : accelerometer data output rate is invalid
    BMI160_E_GYRO_ODR_BW_INVALID      or -7 : gyroscope data output rate is invalid
    BMI160_E_LWP_PRE_FLTR_INT_INVALID or -8 : low-power interrupt filter is invalid
    BMI160_E_LWP_PRE_FLTR_INVALID     or -9 : low-power filter is invalid
    BMI160_FOC_FAILURE                or -11: crystal oscillator failed
  '''
  while bmi.begin() != BMI160_OK:
    print("Initialization 6-axis sensor failed.")
    time.sleep(1)
  print("Initialization 6-axis sensor sucess.")
  
  
  while True:
    '''
      @brief Get sensor data, including data of gyroscope, accelerometer, etc.
      @return Return data of dictionary type, the format is as follows：
      @n      {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}}
      @n Note: it's raw data, process it to get the correct data：
      @n   Gyroscope: each axis of gyroscope data *3.14/180.0, after calculation, unit is rad/s
      @n   Accelerometer: each axis of accelerometer data//16384.0, after calculation, unit is g
    '''
    data = bmi.get_sensor_data()
    print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(data['gyro']['x']*3.14/180.0, data['gyro']['y']*3.14/180.0, data['gyro']['z']*3.14/180.0))
    print("accel :  x: %.3f g    ,  y: %.3f g    ,  z: %.3f g    "%(data['accel']['x']/16384.0, data['accel']['y']/16384.0, data['accel']['z']/16384.0))
    print("\n")
    time.sleep(0.1)
