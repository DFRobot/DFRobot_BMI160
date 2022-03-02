# -*- coding:utf-8 -*-

'''
  @file DFRobot_BMI160.py
  @brief The BMI160 6-axis inertial motion sensor integrates accelerometer and gyroscope into one, and uses I2C communication.
  @n Features：
  @n 1. Accelerometer scale options: ±2g/±4g/±8g/±16g
  @n 2. Gyroscope scale options: ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
  @n 3. Accelerometer zero drift: ±40mg
  @n 4. Gyroscope zero drift: ±10°/s
  @n 5. I2C address is controlled by SDO：
  @n   BMI160_IIC_ADDR_SDO_H or 0x69: SDO turns to high level (SDO default to be high)
  @n   BMI160_IIC_ADDR_SDO_L or 0x68：SDO turns to low level
  @n
  @n Hardware conneted table in IIC
  @n --------------------------------------------------------------------------
  @n  Sensor      |                          raspberry pi                     |
  @n --------------------------------------------------------------------------
  @n  +           | 3V3/VCC                                                   |
  @n  -           | GND                                                       |
  @n  C           | connected to scl of MCU's IIC                             |
  @n  D           | connected to sda of MCU's IIC                             |
  @n  INT1        | connected to the external interrupt IO pin of MCU         |
  @n  INT2        | connected to the external interrupt IO pin of MCU         |
  @n  SDO         | connected to the IO pin of MCU or connected to GND or VCC |
  @n --------------------------------------------------------------------------
  @
  @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author [Arya](xue.peng@dfrobot.com)
  @version  V1.0
  @date  2021-06-10
  @url from https://github.com/DFRobot/DFRobot_BMI160
'''
import sys
import smbus
import time
import RPi.GPIO as GPIO
import spidev
import numpy as np

BMI160_IIC_ADDR_SDO_H = 0x69
BMI160_IIC_ADDR_SDO_L = 0x68

''' Mask definitions '''
BMI160_ACCEL_BW_MASK              = 0x70
BMI160_ACCEL_ODR_MASK             = 0x0F
BMI160_ACCEL_UNDERSAMPLING_MASK   = 0x80
BMI160_ACCEL_RANGE_MASK           = 0x0F
BMI160_GYRO_BW_MASK               = 0x30
BMI160_GYRO_ODR_MASK              = 0x0F
BMI160_GYRO_RANGE_MSK             = 0x07

BMI160_ERR_REG_MASK   = 0x0F

''' BMI160 Register map '''
BMI160_CHIP_ID_ADDR             = 0x00
BMI160_ERROR_REG_ADDR           = 0x02
BMI160_AUX_DATA_ADDR            = 0x04
BMI160_GYRO_DATA_ADDR           = 0x0C
BMI160_ACCEL_DATA_ADDR          = 0x12
BMI160_STATUS_ADDR              = 0x1B
BMI160_INT_STATUS_ADDR          = 0x1C
BMI160_FIFO_LENGTH_ADDR         = 0x22
BMI160_FIFO_DATA_ADDR           = 0x24
BMI160_ACCEL_CONFIG_ADDR        = 0x40
BMI160_ACCEL_RANGE_ADDR         = 0x41
BMI160_GYRO_CONFIG_ADDR         = 0x42
BMI160_GYRO_RANGE_ADDR          = 0x43
BMI160_AUX_ODR_ADDR             = 0x44
BMI160_FIFO_DOWN_ADDR           = 0x45
BMI160_FIFO_CONFIG_0_ADDR       = 0x46
BMI160_FIFO_CONFIG_1_ADDR       = 0x47
BMI160_AUX_IF_0_ADDR            = 0x4B
BMI160_AUX_IF_1_ADDR            = 0x4C
BMI160_AUX_IF_2_ADDR            = 0x4D
BMI160_AUX_IF_3_ADDR            = 0x4E
BMI160_AUX_IF_4_ADDR            = 0x4F
BMI160_INT_ENABLE_0_ADDR        = 0x50
BMI160_INT_ENABLE_1_ADDR        = 0x51
BMI160_INT_ENABLE_2_ADDR        = 0x52
BMI160_INT_OUT_CTRL_ADDR        = 0x53
BMI160_INT_LATCH_ADDR           = 0x54
BMI160_INT_MAP_0_ADDR           = 0x55
BMI160_INT_MAP_1_ADDR           = 0x56
BMI160_INT_MAP_2_ADDR           = 0x57
BMI160_INT_DATA_0_ADDR          = 0x58
BMI160_INT_DATA_1_ADDR          = 0x59
BMI160_INT_LOWHIGH_0_ADDR       = 0x5A
BMI160_INT_LOWHIGH_1_ADDR       = 0x5B
BMI160_INT_LOWHIGH_2_ADDR       = 0x5C
BMI160_INT_LOWHIGH_3_ADDR       = 0x5D
BMI160_INT_LOWHIGH_4_ADDR       = 0x5E
BMI160_INT_MOTION_0_ADDR        = 0x5F
BMI160_INT_MOTION_1_ADDR        = 0x60
BMI160_INT_MOTION_2_ADDR        = 0x61
BMI160_INT_MOTION_3_ADDR        = 0x62
BMI160_INT_TAP_0_ADDR           = 0x63
BMI160_INT_TAP_1_ADDR           = 0x64
BMI160_INT_ORIENT_0_ADDR        = 0x65
BMI160_INT_ORIENT_1_ADDR        = 0x66
BMI160_INT_FLAT_0_ADDR          = 0x67
BMI160_INT_FLAT_1_ADDR          = 0x68
BMI160_FOC_CONF_ADDR            = 0x69
BMI160_CONF_ADDR                = 0x6A
BMI160_IF_CONF_ADDR             = 0x6B
BMI160_SELF_TEST_ADDR           = 0x6D
BMI160_OFFSET_ADDR              = 0x71
BMI160_OFFSET_CONF_ADDR         = 0x77
BMI160_INT_STEP_CNT_0_ADDR      = 0x78
BMI160_INT_STEP_CONFIG_0_ADDR   = 0x7A
BMI160_INT_STEP_CONFIG_1_ADDR   = 0x7B
BMI160_COMMAND_REG_ADDR         = 0x7E
BMI160_SPI_COMM_TEST_ADDR       = 0x7F
BMI160_INTL_PULLUP_CONF_ADDR    = 0x85

''' Maximum limits definition '''
BMI160_ACCEL_ODR_MAX         = 15
BMI160_ACCEL_BW_MAX          = 2
BMI160_ACCEL_RANGE_MAX       = 12
BMI160_GYRO_ODR_MAX          = 13
BMI160_GYRO_BW_MAX           = 2
BMI160_GYRO_RANGE_MAX        = 4

''' Delay in ms settings '''
BMI160_ACCEL_DELAY_MS         = 0.005
BMI160_GYRO_DELAY_MS          = 0.081
BMI160_ONE_MS_DELAY           = 0.001
BMI160_AUX_COM_DELAY          = 0.01
BMI160_GYRO_SELF_TEST_DELAY   = 0.02
BMI160_ACCEL_SELF_TEST_DELAY  = 0.05

''' Soft reset command '''
BMI160_SOFT_RESET_CMD         = 0xb6
BMI160_SOFT_RESET_DELAY_MS    = 0.015
BMI160_COMMAND_REG_ADDR       = 0x7E

'''BMI160 unique chip identifier '''
BMI160_CHIP_ID        = 0xD1

''' Bandwidth settings '''
''' Accel Bandwidth '''
BMI160_ACCEL_BW_OSR4_AVG1     = 0x00
BMI160_ACCEL_BW_OSR2_AVG2     = 0x01
BMI160_ACCEL_BW_NORMAL_AVG4   = 0x02
BMI160_ACCEL_BW_RES_AVG8      = 0x03
BMI160_ACCEL_BW_RES_AVG16     = 0x04
BMI160_ACCEL_BW_RES_AVG32     = 0x05
BMI160_ACCEL_BW_RES_AVG64     = 0x06
BMI160_ACCEL_BW_RES_AVG128    = 0x07

BMI160_GYRO_BW_OSR4_MODE      = 0x00
BMI160_GYRO_BW_OSR2_MODE      = 0x01
BMI160_GYRO_BW_NORMAL_MODE    = 0x02

''' Output Data Rate settings '''
''' Accel Output data rate '''
BMI160_ACCEL_ODR_RESERVED   = 0x00
BMI160_ACCEL_ODR_0_78HZ     = 0x01
BMI160_ACCEL_ODR_1_56HZ     = 0x02
BMI160_ACCEL_ODR_3_12HZ     = 0x03
BMI160_ACCEL_ODR_6_25HZ     = 0x04
BMI160_ACCEL_ODR_12_5HZ     = 0x05
BMI160_ACCEL_ODR_25HZ       = 0x06
BMI160_ACCEL_ODR_50HZ       = 0x07
BMI160_ACCEL_ODR_100HZ      = 0x08
BMI160_ACCEL_ODR_200HZ      = 0x09
BMI160_ACCEL_ODR_400HZ      = 0x0A
BMI160_ACCEL_ODR_800HZ      = 0x0B
BMI160_ACCEL_ODR_1600HZ     = 0x0C
BMI160_ACCEL_ODR_RESERVED0  = 0x0D
BMI160_ACCEL_ODR_RESERVED1  = 0x0E
BMI160_ACCEL_ODR_RESERVED2  = 0x0F

''' Gyro Output data rate '''
BMI160_GYRO_ODR_RESERVED    = 0x00
BMI160_GYRO_ODR_25HZ        = 0x06
BMI160_GYRO_ODR_50HZ        = 0x07
BMI160_GYRO_ODR_100HZ       = 0x08
BMI160_GYRO_ODR_200HZ       = 0x09
BMI160_GYRO_ODR_400HZ       = 0x0A
BMI160_GYRO_ODR_800HZ       = 0x0B
BMI160_GYRO_ODR_1600HZ      = 0x0C
BMI160_GYRO_ODR_3200HZ      = 0x0D

''' Power mode settings '''
''' Accel power mode '''
BMI160_ACCEL_NORMAL_MODE     = 0x11
BMI160_ACCEL_LOWPOWER_MODE   = 0x12
BMI160_ACCEL_SUSPEND_MODE    = 0x10

''' Gyro power mode '''
BMI160_GYRO_SUSPEND_MODE     = 0x14
BMI160_GYRO_NORMAL_MODE      = 0x15
BMI160_GYRO_FASTSTARTUP_MODE = 0x17

''' Range settings '''
''' Accel Range '''
BMI160_ACCEL_RANGE_2G      = 0x03
BMI160_ACCEL_RANGE_4G      = 0x05
BMI160_ACCEL_RANGE_8G      = 0x08
BMI160_ACCEL_RANGE_16G     = 0x0C

''' Gyro Range '''
BMI160_GYRO_RANGE_2000_DPS  = 0x00
BMI160_GYRO_RANGE_1000_DPS  = 0x01
BMI160_GYRO_RANGE_500_DPS   = 0x02
BMI160_GYRO_RANGE_250_DPS   = 0x03
BMI160_GYRO_RANGE_125_DPS   = 0x04

''' Error code definitions '''
BMI160_OK                         = 0
BMI160_E_NULL_PTR                 = -1
BMI160_E_COM_FAIL                 = -2
BMI160_E_DEV_NOT_FOUND            = -3
BMI160_E_OUT_OF_RANGE             = -4
BMI160_E_INVALID_INPUT            = -5
BMI160_E_ACCEL_ODR_BW_INVALID     = -6
BMI160_E_GYRO_ODR_BW_INVALID      = -7
BMI160_E_LWP_PRE_FLTR_INT_INVALID = -8
BMI160_E_LWP_PRE_FLTR_INVALID     = -9
BMI160_E_AUX_NOT_FOUND            = -10
BMI160_FOC_FAILURE                = -11
BMI160_ERR_CHOOSE                 = -12

'''bmi160 active state of any & sig motion interrupt.'''
'''enum bmi160_any_sig_motion_active_interrupt_state'''
eBMI160_BOTH_ANY_SIG_MOTION_DISABLE = -1
eBMI160_ANY_MOTION_ENABLE = 0
eBMI160_SIG_MOTION_ENABLE = 1

''' Sensor & time select definition'''
BMI160_ACCEL_SEL  = 0x01
BMI160_GYRO_SEL   = 0x02
BMI160_TIME_SEL   = 0x04

''' Sensor select mask'''
BMI160_SEN_SEL_MASK   = 0x07

eBMI160_ACCEL_ONLY          = 1
eBMI160_GYRO_ONLY           = 2
eBMI160_BOTH_ACCEL_AND_GYRO = 3

BMI160_RAW_DATA_LENGTH = 15

BMI160_INT_CHANNEL_NONE = 0
BMI160_INT_CHANNEL_1    = 1
BMI160_INT_CHANNEL_2    = 2
BMI160_INT_CHANNEL_BOTH = 3

BMI160_ACC_ANY_MOTION_INT          = 0
BMI160_ACC_SIG_MOTION_INT          = 1
BMI160_STEP_DETECT_INT             = 2
BMI160_ACC_DOUBLE_TAP_INT          = 3
BMI160_ACC_SINGLE_TAP_INT          = 4
BMI160_ACC_ORIENT_INT              = 5
BMI160_ACC_FLAT_INT                = 6
BMI160_ACC_HIGH_G_INT              = 7
BMI160_ACC_LOW_G_INT               = 8
BMI160_ACC_SLOW_NO_MOTION_INT      = 9
BMI160_ACC_GYRO_DATA_RDY_INT       = 10
BMI160_ACC_GYRO_FIFO_FULL_INT      = 11
BMI160_ACC_GYRO_FIFO_WATERMARK_INT = 12

''' Enable/disable bit value '''
BMI160_ENABLE           = 0x01
BMI160_DISABLE          = 0x00

''' Latch Duration '''
BMI160_LATCH_DUR_NONE               = 0x00
BMI160_LATCH_DUR_312_5_MICRO_SEC    = 0x01
BMI160_LATCH_DUR_625_MICRO_SEC      = 0x02
BMI160_LATCH_DUR_1_25_MILLI_SEC     = 0x03
BMI160_LATCH_DUR_2_5_MILLI_SEC      = 0x04
BMI160_LATCH_DUR_5_MILLI_SEC        = 0x05
BMI160_LATCH_DUR_10_MILLI_SEC       = 0x06
BMI160_LATCH_DUR_20_MILLI_SEC       = 0x07
BMI160_LATCH_DUR_40_MILLI_SEC       = 0x08
BMI160_LATCH_DUR_80_MILLI_SEC       = 0x09
BMI160_LATCH_DUR_160_MILLI_SEC      = 0x0A
BMI160_LATCH_DUR_320_MILLI_SEC      = 0x0B
BMI160_LATCH_DUR_640_MILLI_SEC      = 0x0C
BMI160_LATCH_DUR_1_28_SEC           = 0x0D
BMI160_LATCH_DUR_2_56_SEC           = 0x0E
BMI160_LATCHED                      = 0x0F

BMI160_STEP_DETECT_NORMAL       = 0
BMI160_STEP_DETECT_SENSITIVE    = 1
BMI160_STEP_DETECT_ROBUST       = 2
BMI160_STEP_DETECT_USER_DEFINE  = 3

class DFRobot_BMI160:
  _dev_accel_cfg_bw    = BMI160_ACCEL_BW_NORMAL_AVG4
  _dev_accel_cfg_odr   = BMI160_ACCEL_ODR_100HZ
  _dev_accel_cfg_power = BMI160_ACCEL_SUSPEND_MODE
  _dev_accel_cfg_range = BMI160_ACCEL_RANGE_2G
  _dev_gyro_cfg_bw     = BMI160_GYRO_BW_NORMAL_MODE
  _dev_gyro_cfg_odr    = BMI160_GYRO_ODR_100HZ
  _dev_gyro_cfg_power  = BMI160_GYRO_SUSPEND_MODE
  _dev_gyro_cfg_range  = BMI160_GYRO_RANGE_2000_DPS

  _dev_pre_accel_cfg_bw    = _dev_accel_cfg_bw
  _dev_pre_accel_cfg_odr   = _dev_accel_cfg_odr
  _dev_pre_accel_cfg_power = _dev_accel_cfg_power
  _dev_pre_accel_cfg_range = _dev_accel_cfg_range
  _dev_pre_gyro_cfg_bw     = _dev_gyro_cfg_bw
  _dev_pre_gyro_cfg_odr    = _dev_gyro_cfg_odr
  _dev_pre_gyro_cfg_power  = _dev_gyro_cfg_power
  _dev_pre_gyro_cfg_range  = _dev_gyro_cfg_range
  
  _dev_any_sig_sel  = eBMI160_ANY_MOTION_ENABLE
  _dev_chip_id = BMI160_CHIP_ID
  
  _raw_data = [0]*BMI160_RAW_DATA_LENGTH
  _update = 0;
  
  _gyro_x  = 0
  _gyro_y  = 0
  _gyro_z  = 0
  _accel_x = 0
  _accel_y = 0
  _accel_z = 0
  _accel_sensor_time = 0
  _gyro_sensor_time  = 0
  
  step_normal_power_mode = 0
  step_lower_power_mode = 1
  
  '''Enum status mode'''
  eSTATUS_OK                   = 0     #Normal status, no error
  eSTATUS_ERR                  = 1     #Error status
  eSTATUS_ERR_DEV_NOT_DETECTED = 2     #Device not detected
  eSTATUS_ERR_PARAM            = 3     #Parameter error

  def __init__(self):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

  def begin(self, mode = 0):
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
    if self.soft_reset() != BMI160_OK:
      print("soft reset failed")
      return -1
    rslt = self._get_regs(BMI160_CHIP_ID_ADDR, 1)
    if rslt[0] == BMI160_OK and rslt[1] == BMI160_CHIP_ID:
      self._dev_any_sig_sel  = eBMI160_ANY_MOTION_ENABLE
      self._dev_chip_id = rslt[1]
      rslt = self.soft_reset()
      if rslt == BMI160_OK:
        rslt == self._set_sens_conf()
        
    else:
      rslt = BMI160_E_DEV_NOT_FOUND
    return rslt

  def set_int(self, intNum):
    '''
      @brief Configure interrupt pin
      @param intNum: The INT pin of sensor, INT1 or INT2:
      @n     1 : The INT1 pin of sensor
      @n     2 : The INT2 pin of sensor
      @return Error code:
      BMI160_OK     or  0 : Config succeeded
      others value        : Config failed
    '''
    if intNum == 1:
      self._write_bytes(0x52, [8,0x0B,0,1,0,0])
      self._write_bytes(0x7A, [0x15,3])
    elif intNum == 2:
      self._write_bytes(0x52, [8,0xB0,0,0,0,1])
      self._write_bytes(0x7A, [0x15,3])
    else:
      return BMI160_E_NULL_PTR
    return BMI160_OK
    
  def set_step_power_mode(self, model):
    '''
      @brief Set pedometer power mode
      @param model: Power mode type
      @n     step_normal_power_mode:  Count step in normal power mode
      @n     step_lower_power_mode :  Count step in low power mode
      @return Error code:
      BMI160_OK     or  0 : setting succeeded
      others value        : setting failed
    '''
    if model == self.step_normal_power_mode:
      _dev_accel_cfg_odr   = BMI160_ACCEL_ODR_1600HZ
      _dev_accel_cfg_power = BMI160_ACCEL_NORMAL_MODE
      _dev_gyro_cfg_odr    = BMI160_GYRO_ODR_3200HZ
      _dev_gyro_cfg_power  = BMI160_GYRO_NORMAL_MODE
    elif model == self.step_lower_power_mode:
      _dev_accel_cfg_odr   = BMI160_ACCEL_ODR_50HZ
      _dev_accel_cfg_power = BMI160_ACCEL_LOWPOWER_MODE
      _dev_gyro_cfg_odr    = BMI160_GYRO_ODR_50HZ
      _dev_gyro_cfg_power  = BMI160_GYRO_SUSPEND_MODE
    else:
      _dev_accel_cfg_odr   = BMI160_ACCEL_ODR_1600HZ
      _dev_accel_cfg_power = BMI160_ACCEL_NORMAL_MODE
      _dev_gyro_cfg_odr    = BMI160_GYRO_ODR_3200HZ
      _dev_gyro_cfg_power  = BMI160_GYRO_NORMAL_MODE
    _dev_accel_cfg_bw    = BMI160_ACCEL_BW_NORMAL_AVG4
    _dev_accel_cfg_range = BMI160_ACCEL_RANGE_2G
    _dev_gyro_cfg_bw     = BMI160_GYRO_BW_NORMAL_MODE
    _dev_gyro_cfg_range  = BMI160_GYRO_RANGE_2000_DPS
    
    
    rslt = self._set_accel_conf()
    if rslt == BMI160_OK:
      rslt = self._set_gyro_conf()
      if rslt == BMI160_OK:
        rslt = self._set_power_mode()
        if rslt == BMI160_OK:
          rslt = self._check_invalid_settg()
    return rslt

  def soft_reset(self):
    '''
      @brief Soft reset
      @param model: Power mode type
      @n     step_normal_power_mode:  Count step in normal power mode
      @n     step_lower_power_mode :  Count step in low power mode
      @return Error code:
      @n      BMI160_OK     or  0 : Soft reset succeeded
      @n      others value        : Soft reset failed
    '''
    rslt = self._set_regs(BMI160_COMMAND_REG_ADDR, [BMI160_SOFT_RESET_CMD])
    time.sleep(BMI160_SOFT_RESET_DELAY_MS)
    if rslt == BMI160_OK:
      self._default_param_settg()
    return rslt

  def get_sensor_data(self):
    '''
      @brief Get sensor data, including data of gyroscope, accelerometer, etc.
      @return Return data of dictionary type, the format is as follows：
      @n      {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}}
      @n Note: it's raw data, process it to get the correct data：
      @n   Gyroscope: gyroscope data of each axis *3.14/180.0, after calculation, unit is rad/s
      @n   Accelerometer: accelerometer data of each axis //16384.0, after calculation, unit is g
    '''
    sensor = {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}}
    rslt = self._get_raw_data()
    self._update = 0
    if rslt == BMI160_OK:
      sensor['accel']['x'] = self._accel_x
      sensor['accel']['y'] = self._accel_y
      sensor['accel']['z'] = self._accel_z
      sensor['gyro']['x'] = self._gyro_x
      sensor['gyro']['y'] = self._gyro_y
      sensor['gyro']['z'] = self._gyro_z
    return sensor

  def get_accel_data(self):
    '''
      @brief Get accelerometer data
      @return Return data of dictionary type, the format is as follows：
      @n      {'accel':{'x':0, 'y':0, 'z':0}}
      @n Note: it's raw data, process it to get the correct data：
      @n   Accelerometer: accelerometer data of each axis//16384.0, after calculation, unit is g
    '''
    senor = {'accel':{'x':0, 'y':0, 'z':0}}
    if self._update & 0x01 != 0x01:
      rslt = self._get_raw_data()
    sensor['accel']['x'] = self._accel_x
    sensor['accel']['y'] = self._accel_y
    sensor['accel']['z'] = self._accel_z
    self._update &= 0xFE
    return sensor

  def get_gyro_data(self):
    '''
      @brief Get gyroscope data
      @return Return data of dictionary type, the format is as follows：
      @n      {'gyro':{'x':0, 'y':0, 'z':0}}
      @n Note: it's raw data, process it to get the correct data：
      @n   Gyroscope: gyroscope data of each axis *3.14/180.0, after calculation, unit is rad/s
    '''
    senor = {'gyro':{'x':0, 'y':0, 'z':0}}
    if self._update & 0x02 != 0x02:
      rslt = self._get_raw_data()
    sensor['gyro']['x'] = self._gyro_x
    sensor['gyro']['y'] = self._gyro_y
    sensor['gyro']['z'] = self._gyro_z
    self._update &= 0xFD
    return sensor

  def set_step_counter(self):
    '''
      @brief Set step count
      @return Error code:
      @n      BMI160_OK     or  0 : Setting succeeded
      @n      others value        : Setting failed
    '''
    rslt = self._get_regs(BMI160_INT_STEP_CONFIG_1_ADDR, 1)
    if rslt[0] == BMI160_OK:
      rslt[1] |= ((1 << 3) & 0xFF)
      rslt = self._set_regs(BMI160_INT_STEP_CONFIG_1_ADDR, [rslt[1]])
    else:
      rslt = rslt[0]
    return rslt

  def read_step_counter(self):
    '''
      @brief Read step count data
      @return Data of dictionary type, the format is error code + current step count data:
      @n      {'error':0, 'step':0}
      @n Note: when 'error' value is BMI160_OK, it means that 'step' is valid data, otherwise it is invalid data.
    '''
    val = {'error':0, 'step':0}
    rslt = self._get_regs(BMI160_INT_STEP_CNT_0_ADDR, 2)
    if rslt[0] == BMI160_OK:
      lsb = rslt[1]
      msb = (rslt[2] << 8)
      val['step'] = msb | lsb
    val['error']  =  rslt[0]
    return val

  def _get_raw_data(self):
    self._raw_data = [0]*BMI160_RAW_DATA_LENGTH
    rslt = self._get_regs(BMI160_GYRO_DATA_ADDR, BMI160_RAW_DATA_LENGTH)
    self._update = 0x03
    idex = 1
    _gyro_x  = 0
    _gyro_y  = 0
    _gyro_z  = 0
    _accel_x = 0
    _accel_y = 0
    _accel_z = 0
    _accel_sensor_time = 0
    _gyro_sensor_time  = 0
    if rslt[0] == BMI160_OK:
      lsb = rslt[idex]
      msb = rslt[idex+1]
      self._gyro_x = np.int16((msb)*256 + lsb)
      idex += 2
      
      lsb = rslt[idex]
      msb = rslt[idex+1]
      self._gyro_y = np.int16((msb)*256 + lsb)
      idex += 2
      
      lsb = rslt[idex]
      msb = rslt[idex+1]
      self._gyro_z = np.int16((msb)*256 + lsb)
      idex += 2
      
      lsb = rslt[idex]
      msb = rslt[idex+1]
      self._accel_x = np.int16((msb)*256 + lsb)
      idex += 2
      
      lsb = rslt[idex]
      msb = rslt[idex+1]
      self._accel_y = np.int16((msb)*256 + lsb)
      idex += 2
      
      lsb = rslt[idex]
      msb = rslt[idex+1]
      self._accel_z = np.int16((msb)*256 + lsb)
      idex += 2
      
      time_0 = rslt[idex]
      time_1 = rslt[idex+1] << 8
      time_2 = rslt[idex+2] << 16
      
      self._accel_sensor_time = time_2|time_1|time_0
      self._gyro_sensor_time = time_2|time_1|time_0
      return BMI160_OK
    return BMI160_E_COM_FAIL
    
  def _set_regs(self, reg, dataList):
    rslt = BMI160_OK
    count = 0
    rslt = self._write_bytes(reg, dataList)
    time.sleep(0.001)
    if rslt == len(dataList):
      return BMI160_OK
    else:
      return BMI160_E_COM_FAIL
  
  def _get_regs(self, reg, length):
    rslt = self._read_bytes(reg,length)
    time.sleep(0.001)
    if len(rslt) != length:
      rslt.insert(0, BMI160_E_COM_FAIL)
    else:
      rslt.insert(0, BMI160_OK)
    return rslt

  def _update_reg_bit_value(self, reg_value, bit, offset, value):
    reg_value &= ((~(offset << bit)) & 0xFF)
    reg_value |= (value << bit)
    return reg_value & 0xFF

  def _get_reg_bit_value(self, reg_value, bit, offset):
    reg_value >>= bit
    reg_value &= offset
    return reg_value & 0xFF

  def _default_param_settg(self):
    self._dev_accel_cfg_bw    = BMI160_ACCEL_BW_NORMAL_AVG4
    self._dev_accel_cfg_odr   = BMI160_ACCEL_ODR_100HZ
    self._dev_accel_cfg_power = BMI160_ACCEL_SUSPEND_MODE
    self._dev_accel_cfg_range = BMI160_ACCEL_RANGE_2G
    self._dev_gyro_cfg_bw     = BMI160_GYRO_BW_NORMAL_MODE
    self._dev_gyro_cfg_odr    = BMI160_GYRO_ODR_100HZ
    self._dev_gyro_cfg_power  = BMI160_GYRO_SUSPEND_MODE
    self._dev_gyro_cfg_range  = BMI160_GYRO_RANGE_2000_DPS

    self._dev_pre_accel_cfg_bw    = self._dev_accel_cfg_bw
    self._dev_pre_accel_cfg_odr   = self._dev_accel_cfg_odr
    self._dev_pre_accel_cfg_power = self._dev_accel_cfg_power
    self._dev_pre_accel_cfg_range = self._dev_accel_cfg_range
    self._dev_pre_gyro_cfg_bw     = self._dev_gyro_cfg_bw
    self._dev_pre_gyro_cfg_odr    = self._dev_gyro_cfg_odr
    self._dev_pre_gyro_cfg_power  = self._dev_gyro_cfg_power
    self._dev_pre_gyro_cfg_range  = self._dev_gyro_cfg_range

  def _set_sens_conf(self):
    self._dev_accel_cfg_odr   = BMI160_ACCEL_ODR_1600HZ
    self._dev_accel_cfg_range = BMI160_ACCEL_RANGE_2G
    self._dev_accel_cfg_bw    = BMI160_ACCEL_BW_NORMAL_AVG4
    
    self._dev_accel_cfg_power = BMI160_ACCEL_NORMAL_MODE
    
    self._dev_gyro_cfg_odr    = BMI160_GYRO_ODR_3200HZ
    self._dev_gyro_cfg_range  = BMI160_GYRO_RANGE_2000_DPS
    self._dev_gyro_cfg_bw     = BMI160_GYRO_BW_NORMAL_MODE
    
    self._dev_gyro_cfg_power  = BMI160_GYRO_NORMAL_MODE
    
    rslt = self._set_accel_conf()
    if rslt == BMI160_OK:
      rslt = self._set_gyro_conf()
      if rslt == BMI160_OK:
        rslt = self._set_power_mode()
        if rslt == BMI160_OK:
          rslt = self._check_invalid_settg()
    return rslt
    
    
  def _set_accel_conf(self):
    rslt = self._check_accel_config()
    if rslt[0] == BMI160_OK:
      rslt1 = self._set_regs(BMI160_ACCEL_CONFIG_ADDR, [rslt[1]])
      if rslt1 == BMI160_OK:
        self._dev_pre_accel_cfg_bw    = self._dev_accel_cfg_bw
        self._dev_pre_accel_cfg_odr   = self._dev_accel_cfg_odr
        time.sleep(BMI160_ONE_MS_DELAY)
        rslt1 = self._set_regs(BMI160_ACCEL_RANGE_ADDR, [rslt[2]])
        if rslt1 == BMI160_OK:
          self._dev_pre_accel_cfg_range = self._dev_accel_cfg_range
    else:
      rslt1 = rslt[0]
    return rslt1
    
  def _set_gyro_conf(self):
    value = [0]*2
    rslt = self._check_gyro_config()
    if rslt[0] == BMI160_OK:
      value[0] = rslt[1]
      value[1] = rslt[2]
      rslt = self._set_regs(BMI160_GYRO_CONFIG_ADDR, [value[0]])
      if rslt == BMI160_OK: 
        self._dev_pre_gyro_cfg_bw    = self._dev_gyro_cfg_bw
        self._dev_pre_gyro_cfg_odr   = self._dev_gyro_cfg_odr
        time.sleep(BMI160_ONE_MS_DELAY)
        rslt = self._set_regs(BMI160_GYRO_RANGE_ADDR, [value[1]])
        if rslt == BMI160_OK:
          self._dev_pre_gyro_cfg_range = self._dev_gyro_cfg_range
    else:
      rslt = rslt[0]
    return rslt

  def _set_power_mode(self):
    rslt = self._set_accel_pwr()
    if rslt == BMI160_OK:
      rslt = self._set_gyro_pwr()
    return rslt
    
  def _set_accel_pwr(self):
    rslt = BMI160_OK
    if (self._dev_accel_cfg_power >= BMI160_ACCEL_SUSPEND_MODE) and (self._dev_accel_cfg_power <= BMI160_ACCEL_LOWPOWER_MODE):
      if self._dev_accel_cfg_power != self._dev_pre_accel_cfg_power:
        rslt = self._process_under_sampling()
        if rslt == BMI160_OK:
          rslt = self._set_regs(BMI160_COMMAND_REG_ADDR, [self._dev_accel_cfg_power])
          if self._dev_pre_accel_cfg_power == BMI160_ACCEL_SUSPEND_MODE:
            time.sleep(BMI160_ACCEL_DELAY_MS)
          self._dev_pre_accel_cfg_power = self._dev_accel_cfg_power
          return BMI160_OK
    return rslt

  def _process_under_sampling(self):
    value = 0
    rslt = self._get_regs(BMI160_ACCEL_CONFIG_ADDR, 1)
    if rslt[0] == BMI160_OK:
      value = rslt[1]
      if self._dev_accel_cfg_power == BMI160_ACCEL_LOWPOWER_MODE:
        temp = (value & (~BMI160_ACCEL_UNDERSAMPLING_MASK))&0xff
        value = (temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK))&0xff
        rslt = self._set_regs(BMI160_ACCEL_CONFIG_ADDR, [value])
        if rslt == BMI160_OK:
          rslt = self._set_regs(BMI160_INT_DATA_0_ADDR, [0])
          return rslt
      else:
        if value & BMI160_ACCEL_UNDERSAMPLING_MASK:
          temp = (value & (~BMI160_ACCEL_UNDERSAMPLING_MASK)) & 0xFF
          value = temp
          rslt = self._set_regs(BMI160_ACCEL_CONFIG_ADDR, [value])
          return rslt
    rslt = rslt[0]
    return rslt

  def _set_gyro_pwr(self):
    rslt = BMI160_OK
    if self._dev_gyro_cfg_power == BMI160_GYRO_SUSPEND_MODE or self._dev_gyro_cfg_power == BMI160_GYRO_NORMAL_MODE or self._dev_gyro_cfg_power == BMI160_GYRO_FASTSTARTUP_MODE:
      if self._dev_gyro_cfg_power != self._dev_pre_gyro_cfg_power:
        rslt = self._set_regs(BMI160_COMMAND_REG_ADDR, [self._dev_gyro_cfg_power])
        if self._dev_pre_gyro_cfg_power == BMI160_GYRO_SUSPEND_MODE:
          time.sleep(BMI160_GYRO_DELAY_MS)
        elif self._dev_pre_gyro_cfg_power == BMI160_GYRO_FASTSTARTUP_MODE and self._dev_gyro_cfg_power == BMI160_GYRO_NORMAL_MODE:
          time.sleep(0.01)
        self._dev_pre_gyro_cfg_power = self._dev_gyro_cfg_power
        return BMI160_OK
    
    return rslt

  def _check_invalid_settg(self):
    value = 0
    rslt = self._get_regs(BMI160_ERROR_REG_ADDR, 1)
    if len(rslt) == 2:
      value = rslt[1]
    value = (value >> 1)&0xff
    value = value & BMI160_ERR_REG_MASK
    rslt = rslt[0]
    if value == 1:
      rslt = BMI160_E_ACCEL_ODR_BW_INVALID
    elif value == 2:
      rslt = BMI160_E_GYRO_ODR_BW_INVALID
    elif value == 3:
      rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID
    elif value == 7:
      rslt = BMI160_E_LWP_PRE_FLTR_INVALID
    return rslt
    
  def _check_gyro_config(self):
    value = [0]*3
    rslt = self._get_regs(BMI160_GYRO_CONFIG_ADDR, 2)
    if rslt[0] == BMI160_OK:
      value[1] = rslt[1]
      value[2] = rslt[2]
      rslt = self._process_gyro_odr(value[1])
      if rslt[0] == BMI160_OK:
        value[1] = rslt[1]
        rslt = self._process_gyro_bw(value[1])
        if rslt[0] == BMI160_OK:
          rslt = self._process_gyro_range(value[2])
          if rslt[0] == BMI160_OK:
            value[2] = rslt[1]
    value[0] = rslt[0]
    return value
  
  def _process_gyro_odr(self, data):
    rslt = [0]*2
    rslt[1] = data
    if self._dev_gyro_cfg_odr <= BMI160_GYRO_ODR_MAX:
      if self._dev_gyro_cfg_odr != self._dev_pre_gyro_cfg_odr:
        odr = self._dev_gyro_cfg_odr & 0xFF
        temp = (data & (~BMI160_GYRO_ODR_MASK)) & 0xff
        rslt[1] = temp | (odr & BMI160_GYRO_ODR_MASK)
        rslt[0] = BMI160_OK
    else:
      rslt[0] = BMI160_E_OUT_OF_RANGE
    return rslt

  def _process_gyro_bw(self, data):
    rslt = [0]*2
    rslt[1] = data
    if self._dev_gyro_cfg_bw <= BMI160_GYRO_BW_MAX:
      #if self._dev_gyro_cfg_bw != self._dev_pre_gyro_cfg_bw:
      bw = self._dev_gyro_cfg_bw & 0xFF
      temp = (data & (~BMI160_GYRO_BW_MASK)) & 0xff
      rslt[1] = (temp | ( (bw << 4) & BMI160_GYRO_BW_MASK))&0xff
      rslt[0] = BMI160_OK
    else:
      rslt[0] = BMI160_E_OUT_OF_RANGE
    return rslt

  def _process_gyro_range(self, data):
    rslt = [0]*2
    rslt[1] = data
    if self._dev_gyro_cfg_range <= BMI160_ACCEL_RANGE_MAX:
      if self._dev_gyro_cfg_range != self._dev_pre_gyro_cfg_range:
        range = self._dev_gyro_cfg_range & 0xFF
        temp = (data & (~BMI160_GYRO_RANGE_MSK)) & 0xff
        rslt[1] = (temp | ( range & BMI160_GYRO_RANGE_MSK))&0xff
        rslt[0] = BMI160_OK
    else:
      rslt[0] = BMI160_E_OUT_OF_RANGE
    return rslt

  def _check_accel_config(self):
    value = [0]*3
    rslt = self._get_regs(BMI160_ACCEL_CONFIG_ADDR, 2)
    if rslt[0] == BMI160_OK:
      value[1] = rslt[1]
      value[2] = rslt[2]
      rslt = self._process_accel_odr(value[1])
      if rslt[0] == BMI160_OK:
        value[1] = rslt[1]
        rslt = self._process_accel_bw(value[1])
        if rslt[0] == BMI160_OK:
          rslt = self._process_accel_range(value[2])
          if rslt[0] == BMI160_OK:
            value[2] = rslt[1]
    value[0] = rslt[0]
    return value
  def _process_accel_odr(self, data):
    rslt = [0]*2
    rslt[1] = data
    if self._dev_accel_cfg_odr <= BMI160_ACCEL_ODR_MAX:
      if self._dev_accel_cfg_odr != self._dev_pre_accel_cfg_odr:
        odr = self._dev_accel_cfg_odr & 0xFF
        temp = (data & (~BMI160_ACCEL_ODR_MASK)) & 0xff
        rslt[1] = temp | (odr & BMI160_ACCEL_ODR_MASK)
        rslt[0] = BMI160_OK
    else:
      rslt[0] = BMI160_E_OUT_OF_RANGE
    return rslt

  def _process_accel_bw(self, data):
    rslt = [0]*2
    rslt[1] = data
    if self._dev_accel_cfg_bw <= BMI160_ACCEL_BW_MAX:
      if self._dev_accel_cfg_bw != self._dev_pre_accel_cfg_bw:
        bw = self._dev_accel_cfg_bw & 0xFF
        temp = (data & (~BMI160_ACCEL_BW_MASK)) & 0xff
        rslt[1] = (temp | ( (bw << 4) & BMI160_ACCEL_BW_MASK))&0xff
        rslt[0] = BMI160_OK
    else:
      rslt[0] = BMI160_E_OUT_OF_RANGE
    return rslt

  def _process_accel_range(self,data):
    rslt = [0]*2
    rslt[1] = data
    if self._dev_accel_cfg_range <= BMI160_ACCEL_RANGE_MAX:
      if self._dev_accel_cfg_range != self._dev_pre_accel_cfg_range:
        range = self._dev_accel_cfg_range & 0xFF
        temp = (data & (~BMI160_ACCEL_RANGE_MASK)) & 0xff
        rslt[1] = (temp | ( range & BMI160_ACCEL_RANGE_MASK))&0xff
        rslt[0] = BMI160_OK
    else:
      rslt[0] = BMI160_E_OUT_OF_RANGE
    return rslt

  def _write_bytes(self, reg, buf):
    pass

  def _read_bytes(self, reg, length):
    pass

class DFRobot_BMI160_IIC(DFRobot_BMI160):
  def __init__(self,addr = BMI160_IIC_ADDR_SDO_H):
    '''
      @brief The constructor of the BMI160 sensor using IIC communication.
      @param addr:  7-bit IIC address, controlled by SDO pin.
      @n     BMI160_IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
      @n     BMI160_IIC_ADDR_SDO_L or 0x68:  SDO pull down.
    '''
    self._addr = addr
    self._bus = smbus.SMBus(1)
    DFRobot_BMI160.__init__(self)


  def _write_bytes(self, reg, buf):
    try:
      self._bus.write_i2c_block_data(self._addr, reg, buf)
      return len(buf)
    except:
      return 0

  def _read_bytes(self, reg, length):
    try:
      rslt = self._bus.read_i2c_block_data(self._addr, reg, length)
      return rslt
    except:
      return [0]*length


