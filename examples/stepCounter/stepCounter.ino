/*!
  * interrupt.ino
  *
  * I2C addr:
  *   0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
  *   0x69: set I2C address by parameter
  *
  * Through the example, you can get the sensor data which means step counter.
  * upload interrupt number by setInt (choose int1 or int2)  
  * data from int1 and int2 read in stepChange
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
int pbIn = 13; 
enum intnum {
  int1 = 1,
  int2 = 2
};

void stepChange()
{
  int8_t rslt = BMI160_OK;
  uint16_t step_count = 0;
  if (bmi160.readStepCounter(&step_count)==BMI160_OK){
    Serial.println("stepeChange ok");
    Serial.println(step_count);
  }else{
    Serial.println("stepeChange false");
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("=============================");
  Serial.println("=============================");
  Serial.println("=============================");
  Serial.println("=============================");
  Serial.println("=============================");
  
  if (bmi160.I2cInit(i2c_addr)==BMI160_OK){
    if (bmi160.setInt(int1)==BMI160_OK){
      if (bmi160.setStepCounter()==BMI160_OK){
        attachInterrupt(pbIn, stepChange, CHANGE);
      }else{
        Serial.println("set step fail"); 
      }
    }else{
      Serial.println("set interrput fail");
    }
  }else{
    Serial.println("i2c init fail");
  }
 
}

void loop(){
}










