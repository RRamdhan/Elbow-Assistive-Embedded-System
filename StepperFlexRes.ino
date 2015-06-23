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

void setup() {
  Serial.begin(9600); 
  setPwmSwizzler(6, 9, 10, 11);
  initialize();//Initialization for the stepper motor.
}

void loop() {
  // put your main code here, to run repeatedly: 

  flexResReading = analogRead(flexRes);
  //flexResReading = map(flexResReading, 0, 1024, 0, 1024);
  if (flexResReading >= 512) 
  {
    stepperMotor.direction= MOTOR_CLOCKWISE;
  }
  else if (flexResReading <= 400) 
  {
    stepperMotor.direction= MOTOR_ANTICLOCKWISE;
  }
  while (flexResReading >= 512){
    flexResReading = analogRead(flexRes);
    //stepperMotor.speed = 100;
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
  }
  
  while (flexResReading <= 400){
    flexResReading = analogRead(flexRes);
    //stepperMotor.speed = 100;
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
  }
  stop();
  Serial.println(flexResReading);
}






