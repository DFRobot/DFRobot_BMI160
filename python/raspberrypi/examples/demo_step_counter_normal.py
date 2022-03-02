# -*- coding:utf-8 -*-

'''
   @file demo_step_counter_normalpy
   @brief Through the example, you can get the sensor data which means step counter.
   @n Set step counter power mode by setStepPowerMode(stepNormalPowerMode means normal model, stepLowPowerMode means low power model)
   @n Upload interrupt number by set_int (choose int1 or int2)  
   @n data from int1 and int2 read in readStepCounter
  
   @n Hardware conneted table in IIC
   @n Hardware conneted table in IIC
   @n ----------------------------------------------------------------------------------------
   @n  Sensor      |                                     raspberry pi                         |
   @n -----------------------------------------------------------------------------------------
   @n  +           | 3V3/VCC                                                   |   3V3/VCC    |
   @n  -           | GND                                                       |      GND     |
   @n  C           | connected to scl of MCU's IIC                             |  SCL/3(BCM)  |
   @n  D           | connected to sda of MCU's IIC                             |  SDA/2(BCM)  |
   @n  INT1        | connected to the external interrupt IO pin of MCU         |   floating   |
   @n  INT2        | connected to the external interrupt IO pin of MCU         |   22(BCM)    |
   @n  SDO         | connected to the IO pin of MCU or connected to GND or VCC |   floating   |
   @n ----------------------------------------------------------------------------------------
  
   @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
   @licence     The MIT License (MIT)
   @author [Arya](xue.peng@dfrobot.com)
   @version  V1.0
   @date  2021-06-10
   @url from https://github.com/DFRobot/DFRobot_BMI160
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

  '''
    @brief Configure interrupt pin
    @param intNum: The INT pin of sensor, INT1 or INT2:
    @n     1 : The INT1 pin of sensor
    @n     2 : The INT2 pin of sensor
    @return Error code:
    BMI160_OK     or  0 : Config succeeded
    others value        : Config failed
  '''
  if bmi.set_int(2) != BMI160_OK:
    print("set interrput fail")

  '''
    @brief Set step count
    @return Error code:
    @n      BMI160_OK     or  0 : Setting succeeded
    @n      others value        : Setting failed
  '''
  if bmi.set_step_counter() != BMI160_OK:
    print("set step fail")

  '''
    @brief Set pedometer power mode
    @param model: Power mode type
    @n     step_normal_power_mode:  Count step in normal power mode
    @n     step_lower_power_mode :  Count step in low power mode
    @return Error code:
    BMI160_OK     or  0 : setting succeeded
    others value        : setting failed
  '''
  if bmi.set_step_power_mode(bmi.step_normal_power_mode) != BMI160_OK:
    print("set step_normal_power_mode fail")
  GPIO.setup(22, GPIO.IN) #22: the exteral pin of raspberry pi which is connected to INT2
  GPIO.add_event_detect(22, GPIO.FALLING, step_change)

  while True:
    if read_step:
      '''
        @brief Read step count data
        @return Data of dictionary type, the format is error code + current step count data:
        @n      {'error':0, 'step':0}
        @n Note: when 'error' value is BMI160_OK, it means that 'step' is valid data, otherwise it is invalid data.
      '''
      rslt = bmi.read_step_counter()
      if rslt['error'] == BMI160_OK:
        print("step counter = %d"%rslt['step'])
      read_step = False
