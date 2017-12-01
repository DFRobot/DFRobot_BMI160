/*!
  * stepCounter.ino
  *
  * I2C addr:
  *   0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
  *   0x69: set I2C address by parameter
  *
  * Through the example, you can get the sensor data which means step counter.
  * upload interrupt number by setInt (choose int1 or int2)  
  * data from int1 and int2 read in readStepCounter
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
bool readStep = false;

#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560
  int pbIn = 2;
#elif ARDUINO_AVR_LEONARDO
  int pbIn = 3; 
#else
  int pbIn = 13;
#endif
/*the bmi160 have two interrput interface*/
int int1 = 1;
int int2 = 2;

void stepChange()
{
  readStep = true;
}

void setup(){
  Serial.begin(115200);
  delay(100);
    
  while (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("i2c init fail");
    delay(1000); 
  }
  if (bmi160.setInt(int1) != BMI160_OK){
    Serial.println("set interrput fail");
    while(1);
  }
  if (bmi160.setStepCounter() != BMI160_OK){
    Serial.println("set step fail");
    while(1);   
  }
#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_LEONARDO
  attachInterrupt(digitalPinToInterrupt(pbIn), stepChange, FALLING);
#else
  attachInterrupt(pbIn, stepChange, FALLING);
#endif
}

void loop(){
  if (readStep){
    int8_t rslt = BMI160_OK;
    uint16_t stepCounter = 0;
    if (bmi160.readStepCounter(&stepCounter)==BMI160_OK){
      Serial.print("step counter = ");Serial.println(stepCounter);
    }
    readStep = false;
  }
}


