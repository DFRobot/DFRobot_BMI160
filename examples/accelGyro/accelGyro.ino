 /*!
  * DFRobot_BMI160.ino
  *
  * I2C addr:
  *   0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
  *   0x69: set I2C address by parameter
  *
  * Through the example, you can get the sensor data by using getSensorData:
  * get acell by paremeter onlyAccel;
  * get gyro by paremeter onlyGyro;
  * get both acell and gyro by paremeter bothAccelGyro.
  * 
  * With the rotation of the sensor, data changes are visible.
  *
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V1.0
  * date  2017-11-27
  */

#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("=============================");
  Serial.println("=============================");
  Serial.println("=============================");
  Serial.println("=============================");
  Serial.println("=============================");
  
  if (bmi160.softReset()==BMI160_OK){
    Serial.println("reset ok");
  }
  
  if (bmi160.I2cInit(i2c_addr)==BMI160_OK){
    Serial.println("init ok");
  }
}

void loop(){
  
  int i = 0;
  int rslt;
  int16_t accelGyro[6]={0}; 
  rslt = bmi160.getSensorData(bmi160.bothAccelGyro, accelGyro);
  if(rslt == 0){
    for(i=0;i<6;i++){
      if (i<3){
        //Serial.print(accelGyro[i]);Serial.print("\t");  
        Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
      }else{
        //Serial.print(accelGyro[i]);Serial.print("\t");
        Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
      }
    }
    Serial.println();
  }else{
    Serial.println("err");
  }
  
  /*
   * int16_t onlyAccel[3]={0};
   * bmi160.getSensorData(bmi160.onlyAccel, onlyAccel);
   */

  /*
   * int16_t onlyGyro[3]={0};
   * bmi160.getSensorData(bmi160.onlyGyro, onlyGyro);
   */
}










