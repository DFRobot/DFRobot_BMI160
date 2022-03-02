# DFRobot_BMI160

* [中文](./README_CN.md)

The BMI160 is a highly integrated, low power inertial measurement unit (IMU) that provides precise acceleration and angular rate (gyroscopic) measurement.<br>
The BMI160 contains 16 bit digtial,triaxial accelerometer and 16 bit digital, triaxial gyroscope.<br>
This example is for BMI160 sensor and it oprated via Arduino I2C.<br>

![产品效果图](../../resources/images/SEN0250.png)

## Product Link（[https://www.dfrobot.com/product-1716.html](https://www.dfrobot.com/product-1716.html)）
    SKU: SEN0250  
   
## Table of Contents

* [Summary](#summary)
* [Connected](#connected)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This is a 6-axis MEMS sensor BMI160 library. It can only support IIC communication.<br>
It can detect accelerometer, gyroscope, and onboard temperature.<br>

## Connected



| Sensor      |                          raspberry pi                     |
|------------ |:---------------------------------------------------------:|
|  +           | 3V3/VCC                                                   |
|  -           | GND                                                       |
|  C           | connected to scl of MCU's IIC                             |
|  D           | connected to sda of MCU's IIC                             |
|  INT1        | connected to the external interrupt IO pin of MCU         |
|  INT2        | connected to the external interrupt IO pin of MCU         |
|  SDO         | connected to the IO pin of MCU or connected to GND or VCC |

## Installation
1. To use this library, first download the library file<br>
```python
sudo git clone https://github.com/DFRobot/DFRobot_BMI160
```
2. Open and run the routine. To execute a routine demo_x.py, enter python demo_x.py in the command line. For example, to execute the demo_sleep.py routine, you need to enter :<br>

```python
python demo_sleep.py 
or
python2 demo_sleep.py 
or
python3 demo_sleep.py
```


## Methods

```python 
  '''
    @brief The constructor of the BMI160 sensor using IIC communication.
    @param addr:  7-bit IIC address, controlled by SDO pin.
    @n     BMI160_IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
    @n     BMI160_IIC_ADDR_SDO_L or 0x68:  SDO pull down.
  '''
  class DFRobot_BMI160_IIC(DFRobot_BMI160):
  def __init__(self,addr = BMI160_IIC_ADDR_SDO_H):
  
  '''
    @brief Initialize sensor
    @return Error code:
    BMI160_OK                         or  0 : init succeeded, no error
    BMI160_E_NULL_PTR                 or -1 : the parameters is empty
    BMI160_E_COM_FAIL                 or -2 : interaction failed
    BMI160_E_DEV_NOT_FOUND            or -3 : device is not connected
    BMI160_E_OUT_OF_RANGE             or -4 : range is out of sensor range
    BMI160_E_INVALID_INPUT            or -5 : invalid input
    BMI160_E_ACCEL_ODR_BW_INVALID     or -6 : accelerometer data output rate is invalid
    BMI160_E_GYRO_ODR_BW_INVALID      or -7 : gyroscope data output rate is invalid
    BMI160_E_LWP_PRE_FLTR_INT_INVALID or -8 : low-power interrupt filter is invalid
    BMI160_E_LWP_PRE_FLTR_INVALID     or -9 : low-power filter is invalid
    BMI160_FOC_FAILURE                or -11: crystal oscillator failed
  '''
  def begin(self, mode = 0):
  
  '''
    @brief Configure interrupt pin
    @param intNum: The INT pin of sensor, INT1 or INT2:
    @n     1 : The INT1 pin of sensor
    @n     2 : The INT2 pin of sensor
    @return Error code:
    BMI160_OK     or  0 : config succeeded
    others value        : config failed
  '''
  def set_int(self, intNum):
  
  '''
    @brief Set pedometer power mode
    @param model: Power mode type
    @n     step_normal_power_mode:  Count step in normal power mode
    @n     step_lower_power_mode :  Count step in low power mode
    @return Error code:
    BMI160_OK     or  0 : setting succeeded
    others value        : setting failed
  '''
  def set_step_power_mode(self, model):
  
  '''
    @brief Soft reset, after reset
    @param model: Power mode type
    @n     step_normal_power_mode:  Count step in normal power mode
    @n     step_lower_power_mode :  Count step in low power mode
    @return Error code:
    @n      BMI160_OK     or  0 : soft reset succeeded
    @n      others value        : soft reset failed
  '''
  def soft_reset(self):
  
  '''
    @brief Get sensor data, including data of gyroscope, accelerometer, etc.
    @return Return data of dictionary type, the format is as follows：
    @n      {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}}
    @n Note: it's the raw data, process it to get the correct data：
    @n   Gyroscope: gyroscope data per axis need *3.14/180.0, after calculation, unit is rad/s
    @n   Accelerometer: accelerometer data per axis required //16384.0, after calculation, unit is g
  '''
  def get_sensor_data(self):
  
  '''
    @brief Get accelerometer data
    @return Return data of dictionary type, the format is as follows：
    @n      {'accel':{'x':0, 'y':0, 'z':0}}
    @n Note: it's the raw data, process it to get the correct data：
    @n   Accelerometer: accelerometer data per axis required //16384.0, after calculation, unit is g
  '''
  def get_accel_data(self):
  
  '''
    @brief Get gyroscope data
    @return Return data of dictionary type, the format is as follows：
    @n      {'gyro':{'x':0, 'y':0, 'z':0}}
    @n Note: it's the raw data, process it to get the correct data：
    @n   Gyroscope: gyroscope data per axis required *3.14/180.0, after calculation, unit is rad/s
  '''
  def get_gyro_data(self):
  
  '''
    @brief Set step count
    @return Error code:
    @n      BMI160_OK     or  0 : Setting succeeded
    @n      others value        : Setting failed
  '''
  def set_step_counter(self):
  
  '''
    @brief Read step count data
    @return Data of dictionary type, the format is error code + current step count data:
    @n      {'error':0, 'step':0}
    @n Note: when 'error' value is BMI160_OK, it means that 'step' is valid data, otherwise it is invalid data.
  '''
  def read_step_counter(self):
```

## Compatibility

| MCU         | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :--: | :----: | :----: | :--: |
| RaspberryPi2 |      |        |   √    |      |
| RaspberryPi3 |      |        |   √    |      |
| RaspberryPi4 |  √   |        |        |      |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :--: | :----: | :----: | ---- |
| Python2 |  √   |        |        |      |
| Python3 |  √   |        |        |      |

## History

- 2021-06-11 - Version 1.0.0 released.s

## Credits

Written by Arya(xue.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





