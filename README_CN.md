# DFRobot_BMI160

* [English Version](./README.md)

BMI160 是一种高度集成的低功耗惯性测量单元 (IMU)，可提供精确的加速度和角速率（陀螺仪）测量。<br>
BMI160 包含 16 位数字三轴加速度计和 16 位数字三轴陀螺仪。<br>
此示例适用于 BMI160 传感器，它通过 Arduino I2C 进行操作。<br>

![产品效果图](./resources/images/SEN0250.png)


## 产品链接（[https://www.dfrobot.com.cn/goods-1693.html](https://www.dfrobot.com.cn/goods-1693.html)）
    SKU: SEN0250  
   
## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述
提供 Arduino 库，通过 I2C 通信控制 bmi160 获取 加速度、陀螺仪和计步器数据。<br>

## 库安装

使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```C++
  /**
   * @fn I2cInit
   * @brief 设置I2C地址，并初始化
   * @param i2c_addr  bmi160 I2C地址
   * @n     0x68: 传感器SDIO拉低时，IIC地址为0x68
   * @n     0x69: 传感器SDIO引脚拉高时，IIC地址为0x69
   * @return BMI160_OK(0) 返回成功
   */
  int8_t I2cInit(int8_t i2c_addr = BMI160_I2C_ADDR);
  
  /**
   * @brief 设置获取数据的类型，并保存获取的数据
   * @param type  3种类型
   * @n     onlyAccel    :   只获取加速度数据
   * @n     onlyGyro     :   只获取陀螺仪数据
   * @n     bothAccelGyro:   既获取加速度数据，又获取陀螺仪数据
   * @param data  指向保存获取的数据的缓存
   * @return BMI160_OK(0) 获取成功
   */
  int8_t getSensorData(uint8_t type,int16_t* data);
  
  /**
   * @fn getAccelData
   * @brief 获取加速度数据
   * @param data 指向存储加速度数据的缓存的指针
   * @return BMI160_OK(0) 获取成功
   */
  int8_t getAccelData(int16_t* data);
  
  /**
   * @fn getGyroData
   * @brief 获取陀螺仪数据
   * @param data 指向存储陀螺仪数据缓存的指针
   * @return BMI160_OK(0) 获取成功
   */
  int8_t getGyroData(int16_t* data);
  
  /**
   * @fn getAccelGyroData
   * @brief 获取加速度陀螺仪数据
   * @param data 指向存储加速度陀螺仪数据缓存的指针
   * @return BMI160_OK(0) 获取成功
   */
  int8_t getAccelGyroData(int16_t* data);

  /**
   * @fn softReset
   * @brief 软件复位BMI160传感器
   * @return BMI160_OK(0) 复位成功
   */
  int8_t softReset();
  
  /**
   * @fn setInt
   * @brief 设置中断引脚
   * @param intNum 选择中断引脚号
   * @return BMI160_OK(0) 设置成功
   */
  int8_t setInt(int intNum);
  
  /**
   * @fn setStepCounter
   * @brief 使能计步
   * @return BMI160_OK(0) 成功
   */
  int8_t setStepCounter();
  
  /**
   * @fn readStepCounter
   * @brief 读取计步数据
   * @param stepVal 指向存储计步数据缓存的指针 
   * @return BMI160_OK(0) 读取成功
   */
  int8_t readStepCounter(uint16_t *stepVal);
  /**
   * @fn setStepPowerMode
   * @brief 设置步进功率模型
   * @param model 模型类型
   * @return BMI160_OK(0) 设置成功
   */
  int8_t setStepPowerMode(uint8_t model);
```

## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32   |      √       |              |             | 
FireBeetle-ESP8266 |      √       |              |             | 
Arduino uno        |      √       |              |             | 

## 历史

- 2017-12-01 - 1.0.0 版本

## 创作者

Written by DFRobot_haoJ(hao.jiang@dfrobot.com), 2017. (Welcome to our [website](https://www.dfrobot.com/))




