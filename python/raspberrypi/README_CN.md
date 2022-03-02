# DFRobot_BMI160

* [English Version](./README.md)

BMI160 是一种高度集成的低功耗惯性测量单元 (IMU)，可提供精确的加速度和角速率（陀螺仪）测量。<br>
BMI160 包含 16 位数字三轴加速度计和 16 位数字三轴陀螺仪。<br>
此示例适用于 BMI160 传感器，它通过 Arduino I2C 进行操作。<br>

![产品效果图](../../resources/images/SEN0250.png)


## 产品链接（[https://www.dfrobot.com.cn/goods-1693.html](https://www.dfrobot.com.cn/goods-1693.html)）
    SKU: SEN0250  

## 目录

  * [概述](#概述)
  * [连接](#连接)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

提供 Arduino 库，通过 I2C 通信控制 bmi160 获取 加速度、陀螺仪和计步器数据。<br>

## 连接

| Sensor      |                          raspberry pi                     |
| ------------ | :--: |
|  +           | 3V3 or VCC                                                   |
|  -           | GND                                                       |
|  C           | 连接到树莓派的scl引脚                                       |
|  D           | 连接到树莓派的sda引脚                                       |
|  INT1        | 连接到树莓派的外部中断引脚                                   |
|  INT2        | 连接到树莓派的外部中断引脚                                   |
|  SDO         | 连接到树莓派的IO引脚或连接到VCC或GND                         |


## 库安装
1. 下载库至树莓派，要使用这个库，首先要将库下载到Raspberry Pi，命令下载方法如下:<br>
```python
sudo git clone https://github.com/DFRobot/DFRobot_BMI160
```
2. 打开并运行例程，要执行一个例程demo_x.py，请在命令行中输入python demo_x.py。例如，要执行 demo_sleep.py例程，你需要输入:<br>

```python
python demo_sleep.py 
或 
python2 demo_sleep.py 
或 
python3 demo_sleep.py
```

## 方法

```python
  '''
    @brief 使用 IIC 通信的 BMI160 传感器的构造函数。
    @param addr:  7位IIC地址，通过SDO引脚控制
    @n     BMI160_IIC_ADDR_SDO_H or 0x69:  SDO引脚拉高
    @n     BMI160_IIC_ADDR_SDO_L or 0x68:  SDO引脚拉低
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
    @brief 软件复位
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

## 兼容性

| 主板         | 通过 | 未通过 | 未测试 | 备注 |
| ------------ | :--: | :----: | :----: | :--: |
| RaspberryPi2 |      |        |   √    |      |
| RaspberryPi3 |      |        |   √    |      |
| RaspberryPi4 |  √   |        |        |      |

* Python 版本

| Python  | 通过 | 未通过 | 未测试 | 备注 |
| ------- | :--: | :----: | :----: | ---- |
| Python2 |  √   |        |        |      |
| Python3 |  √   |        |        |      |

## 历史

- 2021/06/01 - 1.0.0 版本

## 创作者

Written by Arya(xue.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))






