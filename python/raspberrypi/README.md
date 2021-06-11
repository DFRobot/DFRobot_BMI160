# DFRobot_BMI160.py
The BMI160 is a highly integrated, low power inertial measurement unit (IMU) that provides precise acceleration and angular rate (gyroscopic) measurement.
The BMI160 contains 16 bit digtial,triaxial accelerometer and 16 bit digital, triaxial gyroscope.
This example is for BMI160 sensor and it oprated via raspberry pi I2C.
Features:<br>
 * 1.加速度可选标尺：±2g/±4g/±8g/±16g<br>
 * 2.陀螺仪可选标尺：±125°/s,±250°/s,±500°/s,±1000°/s,±2000°/s<br>
 * 3.加速度计零点漂移:±40mg<br>
 * 4.陀螺仪零点漂移:±10°/s<br>
 * 5.IIC地址通过SDO控制：<br>
 *   BMI160_IIC_ADDR_SDO_H or 0x69: SDO拉高（SDO默认拉高）<br>
 *   BMI160_IIC_ADDR_SDO_L or 0x68：SDO拉低<br>

![正反面svg效果图](https://github.com/Arya11111/DFRobot_MCP23017/blob/master/resources/images/SEN0245svg1.png)


## Product Link（链接到英文商城）
    
   
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
Hardware conneted table<br>
### hardware conneted table in IIC
--------------------------------------------------------------------------
 Sensor      |                          raspberry pi                     |
------------ |:---------------------------------------------------------:|
 +           | 3V3/VCC                                                   |
 -           | GND                                                       |
 C           | connected to scl of MCU's IIC                             |
 D           | connected to sda of MCU's IIC                             |
 INT1        | connected to the external interrupt IO pin of MCU         |
 INT2        | connected to the external interrupt IO pin of MCU         |
 SDO         | connected to the IO pin of MCU or connected to GND or VCC |

## Installation
To use this library, first download the library file, then open the examples folder and run the demo in the folder Proceed as follows:
* sudo git clone https://github.com/DFRobot/DFRobot_BMI160
* cd python
* cd raspberrypi
* cd examples
* python demo_*


## Methods

```C++
'''
  @brief The constructor of the BMI160 sensor using IIC communication.
  @param addr:  7-bit IIC address, controlled by SDO pin.
  @n     BMI160_IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
  @n     BMI160_IIC_ADDR_SDO_L or 0x68:  SDO pull down.
'''
class DFRobot_BMI160_IIC(DFRobot_BMI160):
def __init__(self,addr = BMI160_IIC_ADDR_SDO_H):

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
def begin(self, mode = 0):

'''
  @brief 配置中断引脚
  @param intNum: 传感器的INT引脚，INT1或INT2:
  @n     1 : 传感器的INT1引脚
  @n     2 : 传感器的INT2引脚
  @return 错误代码:
  BMI160_OK     or  0 : 配置成功
  others value        : 配置失败
'''
def set_int(self, intNum):

'''
  @brief 设置计步模式
  @param model: 模式类型
  @n     step_normal_power_mode:  普通模式下计步
  @n     step_lower_power_mode :  低功耗模式下计步
  @return 错误代码:
  BMI160_OK     or  0 : 设置成功
  others value        : 设置失败
'''
def set_step_power_mode(self, model):

'''
  @brief 软件复位，复位后
  @param model: 模式类型
  @n     step_normal_power_mode:  普通模式下计步
  @n     step_lower_power_mode :  低功耗模式下计步
  @return 错误代码:
  @n      BMI160_OK     or  0 : 软件复位成功
  @n      others value        : 软件复位失败
'''
def soft_reset(self):

'''
  @brief 获取传感器数据，包括陀螺仪和加速度等数据
  @return 返回字典类型的数据，格式如下：
  @n      {'accel':{'x':0, 'y':0, 'z':0}, 'gyro':{'x':0, 'y':0, 'z':0}}
  @n 注意，这是原始数据，得到正确数据还需要处理：
  @n   陀螺仪： 陀螺仪的每轴数据需要*3.14/180.0，计算后，单位rad/s
  @n   加速度： 加速度的每轴数据需要//16384.0, 计算后，单位g
'''
def get_sensor_data(self):

'''
  @brief 获取加速度数据
  @return 返回字典类型的数据，格式如下：
  @n      {'accel':{'x':0, 'y':0, 'z':0}}
  @n 注意，这是原始数据，得到正确数据还需要处理：
  @n   加速度： 加速度的每轴数据需要//16384.0, 计算后，单位g
'''
def get_accel_data(self):

'''
  @brief 获取陀螺仪数据
  @return 返回字典类型的数据，格式如下：
  @n      {'gyro':{'x':0, 'y':0, 'z':0}}
  @n 注意，这是原始数据，得到正确数据还需要处理：
  @n   陀螺仪： 陀螺仪的每轴数据需要*3.14/180.0，计算后，单位rad/s
'''
def get_gyro_data(self):

'''
  @brief 设置计步计数
  @return 错误代码:
  @n      BMI160_OK     or  0 : 设置成功
  @n      others value        : 设置失败
'''
def set_step_counter(self):

'''
  @brief 读取计步计数数据
  @return 字典类型数据， 格式为错误代码+当前的计步数据:
  @n      {'error':0, 'step':0}
  @n 注意： 当'error'的值为BMI160_OK时，表示'step'为有效数据，否则为无效数据
'''
def read_step_counter(self):


```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    |
------------------ | :----------: | :----------: | :---------: |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
Arduino uno        |      √       |              |             |
raspberry          |      √       |              |             |

## History

- Data 2021-06-11
- Version V1.0

## Credits

Written by Arya(xue.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))





