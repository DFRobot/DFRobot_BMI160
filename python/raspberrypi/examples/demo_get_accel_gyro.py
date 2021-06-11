# -*- coding:utf-8 -*-

'''
  # @file demo_get_accel_gyro.py
  # @brief 获取BMI160 6轴传感器的陀螺仪、加速度等数据。
  # @n ----------------------------------------------------------------------------------------
  # @n  Sensor      |                                     raspberry pi                         |
  # @n -----------------------------------------------------------------------------------------
  # @n  +           | 3V3/VCC                                                   |   3V3/VCC    |
  # @n  -           | GND                                                       |      GND     |
  # @n  C           | connected to scl of MCU's IIC                             |  SCL/3(BCM)  |
  # @n  D           | connected to sda of MCU's IIC                             |  SDA/2(BCM)  |
  # @n  INT1        | connected to the external interrupt IO pin of MCU         |      悬空    |
  # @n  INT2        | connected to the external interrupt IO pin of MCU         |      悬空    |
  # @n  SDO         | connected to the IO pin of MCU or connected to GND or VCC |      悬空    |
  # @n ----------------------------------------------------------------------------------------
  #
  # @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  # @licence     The MIT License (MIT)
  # @author [Arya](xue.peng@dfrobot.com)
  # @version  V1.0
  # @date  2021-06-11
  # @get from https://www.dfrobot.com
  # @url from https://github.com/DFRobot/DFRobot_BME280
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
    @brief 初始化传感器
    @return 错误代码:
    BMI160_OK                         or  0 : 初始化成功，无错误
    BMI160_E_NULL_PTR                 or -1 : 参数为空
    BMI160_E_COM_FAIL                 or -2 : 交互失败
    BMI160_E_DEV_NOT_FOUND            or -3 : 设备未连接
    BMI160_E_OUT_OF_RANGE             or -4 : 量程超出传感器范围
    BMI160_E_INVALID_INPUT            or -5 : 无效的输入
    BMI160_E_ACCEL_ODR_BW_INVALID     or -6 : 加速度数据输出速率无效
    BMI160_E_GYRO_ODR_BW_INVALID      or -7 : 陀螺仪数据输出速率无效
    BMI160_E_LWP_PRE_FLTR_INT_INVALID or -8 : 低功耗中断滤波器无效
    BMI160_E_LWP_PRE_FLTR_INVALID     or -9 : 低功耗滤波器无效
    BMI160_FOC_FAILURE                or -11: 晶振失败
  '''
  while bmi.begin() != BMI160_OK:
    print("Initialization 6-axis sensor failed.")
    time.sleep(1)
  print("Initialization 6-axis sensor sucess.")
  
  
  while True:
    '''
      @brief 获取传感器数据，包括陀螺仪和加速度等数据
      @return 返回字典类型的数据，格式如下：
      @n      {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}}
      @n 注意，这是原始数据，得到正确数据还需要处理：
      @n   陀螺仪： 陀螺仪的每轴数据需要*3.14/180.0，计算后，单位rad/s
      @n   加速度： 加速度的每轴数据需要//16384.0, 计算后，单位g
    '''
    data = bmi.get_sensor_data()
    print("gyro  :  x: %.3f rad/s,  y: %.3f rad/s,  z: %.3f rad/s"%(data['gyro']['x']*3.14/180.0, data['gyro']['y']*3.14/180.0, data['gyro']['z']*3.14/180.0))
    print("accel :  x: %.3f g    ,  y: %.3f g    ,  z: %.3f g    "%(data['accel']['x']/16384.0, data['accel']['y']/16384.0, data['accel']['z']/16384.0))
    print("\n")
    time.sleep(0.1)