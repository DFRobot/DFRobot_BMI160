/*!
 * @file stepCounterNormal.ino
 * @brief I2C addr:
 * @n  0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
 * @n  0x69: set I2C address by parameter
 * @n Through the example, you can get the sensor data which means step counter. Set step counter power mode by setStepPowerMode(stepNormalPowerMode means normal
 * @n mode, stepLowPowerMode means low power model) upload interrupt number by setInt (choose int1 or int2)  data from int1 and int2 read in readStepCounter
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author  DFRobot_haoJ(hao.jiang@dfrobot.com)
 * @version V1.0
 * @date 2017-12-01
 * @url https://github.com/DFRobot/DFRobot_BMI160
 */

#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
bool readStep = false;

#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_PRO
  //interrupt number of uno and mega2560 is 0
  int pbIn = 2;
#elif ARDUINO_AVR_LEONARDO
  //interrupt number of uno and leonardo is 0
  int pbIn = 3; 
#else
  int pbIn = 13;
#endif
/*the bmi160 have two interrput interfaces*/
int int1 = 1;
int int2 = 2;

void stepChange()
{
  //once the step conter is changed, the value can be read 
  readStep = true;
}

void setup(){
  Serial.begin(115200);
  delay(100);
  
  //set and init the bmi160 i2c address  
  while (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("i2c init fail");
    delay(1000); 
  }
  
  //set interrput number to int1 or int2
  if (bmi160.setInt(int2) != BMI160_OK){
    Serial.println("set interrput fail");
    while(1);
  }

  //set the bmi160 mode to step counter
  if (bmi160.setStepCounter() != BMI160_OK){
    Serial.println("set step fail");
    while(1);
  }
  
  //set the bmi160 power model, contains:stepNormalPowerMode,stepLowPowerMode
  if (bmi160.setStepPowerMode(bmi160.stepNormalPowerMode) != BMI160_OK){
    Serial.println("set setStepPowerMode fail");
    while(1);
  }
  
#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_LEONARDO || defined ARDUINO_AVR_PRO
  //set the pin in the board to connect to int1 or int2 of bmi160
  attachInterrupt(digitalPinToInterrupt(pbIn), stepChange, FALLING);
#else
  attachInterrupt(pbIn, stepChange, FALLING);
#endif

  Serial.println(pbIn);
}

void loop(){
  if (readStep){
    uint16_t stepCounter = 0;
    //read step counter from hardware bmi160 
    if (bmi160.readStepCounter(&stepCounter)==BMI160_OK){
      Serial.print("step counter = ");Serial.println(stepCounter);
    }
    readStep = false;
  }
}


