#include <EEPROM.h>
#define MOTOR_CLOCKWISE      0
#define MOTOR_ANTICLOCKWISE  1
/******Pins definitions*************/
#define MOTORSHIELD_IN1	8
#define MOTORSHIELD_IN2	11
#define MOTORSHIELD_IN3	12
#define MOTORSHIELD_IN4	13
#define CTRLPIN_A		9
#define CTRLPIN_B		10

const unsigned char stepper_ctrl[]={
  0x27,0x36,0x1e,0x0f};
struct MotorStruct
{
  int8_t speed;
  uint8_t direction;
};
MotorStruct stepperMotor;
unsigned int number_of_steps = 200;

int flexRes = A0;
int flexResReading = 0;

const int numReadings = 10;

int readingsOne[numReadings];      // the readings from the analog input
int readingsTwo[numReadings];
int i = 0;  // the index of the current reading
int totalOne = 0;                  // the running total
int totalTwo = 0;
int rmsOne = 0;
int rmsTwo = 0;
int averageOne = 0;                // the average
int averageTwo = 0;
int avgOne = 0;
int avgTwo = 0;
int totOne = 0; 
int totTwo = 0;
int count = 0;
int outputFinal = 0;
int position = 0;
int inputPinOne = A0;
int inputPinTwo = A1;

void initialize()
{
  pinMode(MOTORSHIELD_IN1,OUTPUT);
  pinMode(MOTORSHIELD_IN2,OUTPUT);
  pinMode(MOTORSHIELD_IN3,OUTPUT);
  pinMode(MOTORSHIELD_IN4,OUTPUT);
  pinMode(CTRLPIN_A,OUTPUT);
  pinMode(CTRLPIN_B,OUTPUT);
  stop();
  stepperMotor.speed = 100;
  stepperMotor.direction = MOTOR_CLOCKWISE;
}
/*******************************************/
void stop()
{
  /*Unenble the pin, to stop the motor. */
  digitalWrite(CTRLPIN_A,LOW);
  digitalWrite(CTRLPIN_B,LOW);
}

void setup(){
  EEPROM.write(4, 0);
  position = EEPROM.read(4);
  // initialize serial communication with computer:
  Serial.begin(9600); 
  setPwmSwizzler(6, 9, 10, 11);
  initialize();  
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++){
    readingsOne[thisReading] = 0;
    readingsTwo[thisReading] = 0;   
  } 
}

void loop() {
  // subtract the last reading:
  totalOne = totalOne - (readingsOne[i]*readingsOne[i]); 
  totalTwo = totalTwo - (readingsTwo[i]*readingsTwo[i]); 
  totOne = totOne - readingsOne[i];
  totTwo = totTwo - readingsTwo[i]; 
  // read from the sensor:  
  readingsOne[i] = analogRead(inputPinOne);
  readingsTwo[i] = analogRead(inputPinTwo);
  
  readingsOne[i] = map(readingsOne[i], 0, 1023, 0, 1023);
  readingsTwo[i] = map(readingsTwo[i], 0, 1023, 0, 1023);
  
  totOne = totOne + readingsOne[i];
  totTwo = totTwo + readingsTwo[i];
  //readingsOne[i] = readingsOne[i]*readingsOne[i];
  //readingsTwo[i] = readingsTwo[i]*readingsTwo[i];
  // add the reading to the total:
  totalOne = totalOne + (readingsOne[i]*readingsOne[i]);      
  totalTwo = totalTwo + (readingsTwo[i]*readingsTwo[i]);
  // advance to the next position in the array:  
  i = i + 1;                    

  // if we're at the end of the array...
  if (i >= numReadings) {             
    // ...wrap around to the beginning:
    i = 0;   
  }    

  // calculate the average:
  avgOne = totOne / numReadings;
  avgTwo = totTwo / numReadings;
  averageOne = totalOne / numReadings;
  averageTwo = totalTwo / numReadings;
  rmsOne = sqrt(averageOne);// - avgOne;
  rmsTwo = sqrt(averageTwo);// - avgTwo;
  if (rmsOne >= 400 || rmsTwo >= 400)
  {
    if (rmsOne > rmsTwo)
    {
      if (position < 1600)
      {
        stepperMotor.direction= MOTOR_CLOCKWISE;
         while(count < 100)
         {
            int millis_delay = 60L * 1000L /number_of_steps/(stepperMotor.speed);
            digitalWrite(MOTORSHIELD_IN1,1);
            digitalWrite(MOTORSHIELD_IN2,0);
            digitalWrite(MOTORSHIELD_IN3,0);
            digitalWrite(MOTORSHIELD_IN4,0);
            digitalWrite(CTRLPIN_A,1);
            digitalWrite(CTRLPIN_B,0);  
            delay(millis_delay);     
        
            digitalWrite(MOTORSHIELD_IN1,0);
            digitalWrite(MOTORSHIELD_IN2,0);
            digitalWrite(MOTORSHIELD_IN3,1);
            digitalWrite(MOTORSHIELD_IN4,0);
            digitalWrite(CTRLPIN_A,0);
            digitalWrite(CTRLPIN_B,1);                               
            delay(millis_delay); 
            
            digitalWrite(MOTORSHIELD_IN1,0);
            digitalWrite(MOTORSHIELD_IN2,1);
            digitalWrite(MOTORSHIELD_IN3,0);
            digitalWrite(MOTORSHIELD_IN4,0);
            digitalWrite(CTRLPIN_A,1);
            digitalWrite(CTRLPIN_B,0);
            delay(millis_delay);     
        
            digitalWrite(MOTORSHIELD_IN1,0);
            digitalWrite(MOTORSHIELD_IN2,0);
            digitalWrite(MOTORSHIELD_IN3,0);
            digitalWrite(MOTORSHIELD_IN4,1);
            digitalWrite(CTRLPIN_A,0);
            digitalWrite(CTRLPIN_B,1);
            delay(millis_delay);  
            count++;
            position++;
          }
          EEPROM.write(4, position);
      }
    }
    else if (rmsTwo > rmsOne)
    {
      if (position > 0)
      {
        stepperMotor.direction= MOTOR_ANTICLOCKWISE;
        while(count < 100)
        {
          int millis_delay = 60L * 1000L /number_of_steps/(stepperMotor.speed);
          digitalWrite(MOTORSHIELD_IN1,0);
          digitalWrite(MOTORSHIELD_IN2,0);
          digitalWrite(MOTORSHIELD_IN3,0);
          digitalWrite(MOTORSHIELD_IN4,1);
          digitalWrite(CTRLPIN_A,0);
          digitalWrite(CTRLPIN_B,1);
          delay(millis_delay); 
          
          digitalWrite(MOTORSHIELD_IN1,0);
          digitalWrite(MOTORSHIELD_IN2,1);
          digitalWrite(MOTORSHIELD_IN3,0);
          digitalWrite(MOTORSHIELD_IN4,0);
          digitalWrite(CTRLPIN_A,1);
          digitalWrite(CTRLPIN_B,0);
          delay(millis_delay);  
          
          digitalWrite(MOTORSHIELD_IN1,0);
          digitalWrite(MOTORSHIELD_IN2,0);
          digitalWrite(MOTORSHIELD_IN3,1);
          digitalWrite(MOTORSHIELD_IN4,0);
          digitalWrite(CTRLPIN_A,0);
          digitalWrite(CTRLPIN_B,1);
          delay(millis_delay);  
          
          digitalWrite(MOTORSHIELD_IN1,1);
          digitalWrite(MOTORSHIELD_IN2,0);
          digitalWrite(MOTORSHIELD_IN3,0);
          digitalWrite(MOTORSHIELD_IN4,0);
          digitalWrite(CTRLPIN_A,1);
          digitalWrite(CTRLPIN_B,0); 
          delay(millis_delay);
          count++;
          position--;
        }
        EEPROM.write(4, position);
      }
    }
  else
    {
      count = 0;
    }
  }   
  count = 0;
  //outputFinal = abs(rmsOne)-abs(rmsTwo);  
  // send it to the computer as ASCII digits
  Serial.print(rmsOne);
  Serial.print(", ");
  Serial.print(rmsTwo);  
  Serial.print(", ");
  Serial.println(position);
  //Serial.print(", ");  
  //Serial.println(outputFinal);  
  //delay(1); 
}
