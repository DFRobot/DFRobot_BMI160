# -*- coding:utf-8 -*-

'''
  # @file demo_step_counter_normalpy
  # @brief Through the example, you can get the sensor data which means step counter.
  # @n set step counter power mode by setStepPowerMode(stepNormalPowerMode means normal model,stepLowPowerMode means low power model)
  # @n upload interrupt number by set_int (choose int1 or int2)  
  # @n data from int1 and int2 read in readStepCounter
  #
  # @n Hardware conneted table in IIC
  # @n Hardware conneted table in IIC
  # @n ----------------------------------------------------------------------------------------
  # @n  Sensor      |                                     raspberry pi                         |
  # @n -----------------------------------------------------------------------------------------
  # @n  +           | 3V3/VCC                                                   |   3V3/VCC    |
  # @n  -           | GND                                                       |      GND     |
  # @n  C           | connected to scl of MCU's IIC                             |  SCL/3(BCM)  |
  # @n  D           | connected to sda of MCU's IIC                             |  SDA/2(BCM)  |
  # @n  INT1        | connected to the external interrupt IO pin of MCU         |     悬空     |
  # @n  INT2        | connected to the external interrupt IO pin of MCU         |   22(BCM)    |
  # @n  SDO         | connected to the IO pin of MCU or connected to GND or VCC |      悬空    |
  # @n ----------------------------------------------------------------------------------------
  #
  # @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  # @licence     The MIT License (MIT)
  # @author [Arya](xue.peng@dfrobot.com)
  # @version  V1.0
  # @date  2021-06-10
  # @get from https://www.dfrobot.com
  # @url from https://github.com/DFRobot/DFRobot_BME280
'''

import sys
import os
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_BMI160 import *

'''
  @brief The constructor of the BMI160 sensor using IIC communication.
  @param addr:  7-bit IIC address, controlled by SDO pin.
  @n     BMI160_IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
  @n     BMI160_IIC_ADDR_SDO_L or 0x68:  SDO pull down.
'''
bmi = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_H)

read_step = False
def step_change(channel):
  global read_step
  read_step = True #once the step conter is changed, the value can be read 


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

  '''
    @brief 配置中断引脚
    @param intNum: 传感器的INT引脚，INT1或INT2:
    @n     1 : 传感器的INT1引脚
    @n     2 : 传感器的INT2引脚
    @return 错误代码:
    BMI160_OK     or  0 : 配置成功
    others value        : 配置失败
  '''
  if bmi.set_int(2) != BMI160_OK:
    print("set interrput fail")

  '''
    @brief 设置计步计数
    @return 错误代码:
    @n      BMI160_OK     or  0 : 设置成功
    @n      others value        : 设置失败
  '''
  if bmi.set_step_counter() != BMI160_OK:
    print("set step fail")

  '''
    @brief 设置计步模式
    @param model: 模式类型
    @n     step_normal_power_mode:  普通模式下计步
    @n     step_lower_power_mode :  低功耗模式下计步
    @return 错误代码:
    BMI160_OK     or  0 : 设置成功
    others value        : 设置失败
  '''
  if bmi.set_step_power_mode(bmi.step_normal_power_mode) != BMI160_OK:
    print("set step_normal_power_mode fail")
  GPIO.setup(22, GPIO.IN) #22: the exteral pin of raspberry pi which is connected to INT2
  GPIO.add_event_detect(22, GPIO.FALLING, step_change)

  while True:
    if read_step:
      '''
        @brief 读取计步计数数据
        @return 字典类型数据， 格式为错误代码+当前的计步数据:
        @n      {'error':0, 'step':0}
        @n 注意： 当'error'的值为 BMI160_OK 时，表示'step'为有效数据，否则为无效数据
      '''
      rslt = bmi.read_step_counter()
      if rslt['error'] == BMI160_OK:
        print("step counter = %d"%rslt['step'])
      read_step = False