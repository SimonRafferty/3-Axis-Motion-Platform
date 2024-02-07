//*****************************************************************************************
// Simple kinematic controller for 3 Axis 'Office Chair'
//
// It used 3x NEMA23 Stepper motors with step/direction drivers
// Motors are coupled to NMRV030 Worm Reduction gearboxes
// 30:1 reduction for up to 100kg load, fast movement
// 50:1 reduction for 150kg, reasonably fast.
//
// Intended to run on an Adafruit Huzzah32 controller
//
// Simon Rafferty 2022 - feel free to copy, hack, distribute as you wish!
// SimonSFX@outlook.com
//*****************************************************************************************



#include <AccelStepper.h>

#define JOYSTICK_X A3           //Yellow - Roll
#define JOYSTICK_Y A2           //Green - Pitch
#define JOYSTICK_Z A4           //Blue - Heave
#define JOYSTICK_BUTTON 25      //White - STOP

#define Stepper1_STEP 13        //Yellow - Pin for Step
#define Stepper1_DIR 27         //Blue - Pin for Direction
#define Stepper1_ENA 33         //White - Pin for Drive Enable
#define Stepper1_HOME 15        //Green - Pin for Home sensor
#define Stepper1_MAXPOS 4200   //Maximum steps from Home position

#define Stepper2_STEP 32        //Yellow - Pin for Step
#define Stepper2_DIR 14         //Blue - Pin for Direction
#define Stepper2_ENA 22         //White - Pin for Drive Enable
#define Stepper2_HOME 23        //Green - Pin for Home sensor
#define Stepper2_MAXPOS 4200   //Maximum steps from Home position

#define Stepper3_STEP 19        //Yellow - Pin for Step
#define Stepper3_DIR 18         //Blue - Pin for Direction
#define Stepper3_ENA 5          //White - Pin for Drive Enable
#define Stepper3_HOME 4         //Green - Pin for Home sensor
#define Stepper3_MAXPOS 4000   //Maximum steps from Home position

//Settings for all three motors
#define ACCELERATION 10000
#define SMOOTH 2000
#define MAX_SPEED 1000
#define HOME_ACCELERATION 100000
#define HOME_SPEED 150000 //Speed for homing the motors

//Joystick Range Values - You will need to edit these!
//The ADC has a 12bit resolution, but the joystick is unlikely to give the full 0 to 4096 range
#define JOYMIN_X 0
#define JOYMAX_X 4096

#define JOYMIN_Y 0
#define JOYMAX_Y 4096

#define JOYMIN_Z 0
#define JOYMAX_Z 4096

#define READ_INTERVAL 100             //This is the interval in ms between position updates
#define POWERDOWN_INTERVAL 300000     //This is the inactivity time in ms before the motors de-power (5 mins)

long Timeout = millis(); //This is used to de-power the steppers after 5 mins inactivity
long ReadTimer = millis(); //This is used to slow down the update rate & give time for the steppers to move before an update

/*
//Not used, worked better without filter
float gx0, gx1, gx2, gx3, gx4, gx5, gx6, gx7, gx8, gx9;  //Sav Golay variables.  Savitsky Golay filter.
float gy0, gy1, gy2, gy3, gy4, gy5, gy6, gy7, gy8, gy9;  //Sav Golay variables.  Savitsky Golay filter.
float gz0, gz1, gz2, gz3, gz4, gz5, gz6, gz7, gz8, gz9;  //Sav Golay variables.  Savitsky Golay filter.
*/

// Define two steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, Stepper1_STEP, Stepper1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, Stepper2_STEP, Stepper2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, Stepper3_STEP, Stepper3_DIR);



void setup()
{ 
  Serial.begin(115200);
  
  pinMode(Stepper1_ENA, OUTPUT);
  pinMode(Stepper2_ENA, OUTPUT);
  pinMode(Stepper3_ENA, OUTPUT);
  pinMode(Stepper1_HOME, INPUT_PULLUP);
  pinMode(Stepper2_HOME, INPUT_PULLUP);
  pinMode(Stepper3_HOME, INPUT_PULLUP);

  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  
  //pinMode(JOYSTICK_X, INPUT);
  //pinMode(JOYSTICK_Y, INPUT);
  //pinMode(JOYSTICK_Z, INPUT);
  
  
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper1.setEnablePin(Stepper1_ENA);
  stepper1.setPinsInverted  (true,false,true); 
       
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(ACCELERATION);
  stepper2.setEnablePin(Stepper2_ENA);
  stepper2.setPinsInverted  (true,false,true); 


  stepper3.setMaxSpeed(MAX_SPEED);
  stepper3.setAcceleration(ACCELERATION);
  stepper3.setEnablePin(Stepper3_ENA);
  stepper3.setPinsInverted  (true,false,true); 


  //Switch on motors
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  stepper3.enableOutputs();

  //Test the home sensors
  //Uncomment this first - make sure sensors work before you do anything else.
/*
  while(true) {
    Serial.print("Home 1 :"); Serial.print(digitalRead(Stepper1_HOME)); 
    Serial.print("  Home 2 :"); Serial.print(digitalRead(Stepper2_HOME)); 
    Serial.print("  Home 3 :"); Serial.print(digitalRead(Stepper3_HOME)); 
    Serial.println("");
    delay(500);
    //repeat forever  
  }
*/
  //First Home the motors
  stepper1.setAcceleration(HOME_ACCELERATION);
  stepper1.setMaxSpeed(HOME_SPEED); //Slow Speed
  while(digitalRead(Stepper1_HOME)) {
    stepper1.move(-1);
    stepper1.run();
  }
  stepper1.setCurrentPosition(0);  //Set this position as home
  stepper1.setAcceleration(ACCELERATION);
  stepper1.setMaxSpeed(MAX_SPEED);
  Serial.println("Stepper 1 HOMED");
  
  stepper2.setAcceleration(HOME_ACCELERATION);
  stepper2.setMaxSpeed(HOME_SPEED); //Slow Speed
  while(digitalRead(Stepper2_HOME)) {
    stepper2.move(-1);
    stepper2.run();
  }
  stepper2.setCurrentPosition(0);  //Set this position as home
  stepper2.setAcceleration(ACCELERATION);
  stepper2.setMaxSpeed(MAX_SPEED);
  Serial.println("Stepper 2 HOMED");

  
  stepper3.setAcceleration(HOME_ACCELERATION);
  stepper3.setMaxSpeed(HOME_SPEED); //Slow Speed
  while(digitalRead(Stepper3_HOME)) {
    stepper3.move(-1);
    stepper3.run();
  }
  stepper3.setCurrentPosition(0);  //Set this position as home
  stepper3.setAcceleration(ACCELERATION);
  stepper3.setMaxSpeed(MAX_SPEED);
  Serial.println("Stepper 3 HOMED");
  
  stepper3.setCurrentPosition(0);  //Set this position as home

/*
  //This is a pre-programmed test sequence
  for(int Repeat=0; Repeat<2; Repeat++) {
    for(int Angle=0; Angle<361; Angle++) {
      stepper1.moveTo((sin(float(Angle % 360)/57.32)+2) * Stepper1_MAXPOS/4);
      stepper2.moveTo((sin(float((Angle + 120) % 360)/57.32)+2) * Stepper2_MAXPOS/4); 
      stepper3.moveTo((sin(float((Angle + 240) % 360)/57.32)+2) * Stepper3_MAXPOS/4); 
      Timeout = millis();
      //while(millis()-Timeout<20){
      while((stepper1.distanceToGo()>50) || (stepper2.distanceToGo()>50) || (stepper3.distanceToGo()>50) ){
        stepper1.run();
        stepper2.run();
        stepper3.run();
        //Serial.print("Stepper1 Pos= "); Serial.println(stepper1.currentPosition());
      }
    }
  }
*/
  //stepper1.moveTo(0);
  //stepper2.moveTo(0);
  //stepper3.moveTo(0);
  while((stepper1.distanceToGo()!=0) || (stepper2.distanceToGo()!=0) || (stepper3.distanceToGo()!=0) ){  
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
/*
  //Disable motors for testing
  stepper1.disableOutputs();
  stepper2.disableOutputs();
  stepper3.disableOutputs();
*/
}


void loop()
{
  //If button pressed, make everything react faster
  if(!digitalRead(JOYSTICK_BUTTON)) {
    stepper1.setAcceleration(ACCELERATION);
    stepper2.setAcceleration(ACCELERATION);
    stepper3.setAcceleration(ACCELERATION);
} else {
    stepper1.setAcceleration(SMOOTH);
    stepper2.setAcceleration(SMOOTH);
    stepper3.setAcceleration(SMOOTH);
}  
  
  //Read Joystick, scale values & move platform
  if((millis()-ReadTimer)>READ_INTERVAL) {
    readJoystick();
    ReadTimer = millis();
  }
  
  if((millis()-Timeout) > POWERDOWN_INTERVAL){
    //Platform has not moved for 5 mins, disable motors
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    stepper3.disableOutputs();
  }

  //See if platform is moving & reset timeout
  if(stepper1.distanceToGo()!=0 || stepper2.distanceToGo()!=0 || stepper3.distanceToGo()!=0){
     Timeout = millis();
  }
  
  if(!Stepper1_HOME) {
    //Home triggered, reset home position. This will happen when steps are missed
    stepper1.setCurrentPosition(0);
  }
  if(!Stepper2_HOME) {
    //Home triggered, reset home position This will happen when steps are missed
    stepper2.setCurrentPosition(0);
  }
  if(!Stepper3_HOME) {
    //Home triggered, reset home position This will happen when steps are missed
    stepper3.setCurrentPosition(0);
  }
  
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

void readJoystick(){
  float JRoll = 0;
  float JPitch = 0;
  float JHeave = 0;
  
  for(int Sample=1; Sample<=100; Sample++) {
    JRoll = JRoll + analogRead(JOYSTICK_X);
    JPitch = JPitch + analogRead(JOYSTICK_Y);
    JHeave = JHeave + analogRead(JOYSTICK_Z);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }


  //SG_filter_result is the accelerometer value from the rolling SG filter on the 0-1023 scale
  JRoll = JRoll /100;
  JPitch = JPitch /100;
  JHeave = JHeave /100;
  
  
  //Serial.print("Joystick Raw Roll="); Serial.print(JRoll);
  //Serial.print("  Pitch="); Serial.print(JPitch);
  //Serial.print("  Heave="); Serial.println(JHeave);

  JRoll = mapf(JRoll, JOYMIN_X, JOYMAX_X, -5000, 5000);
  JPitch = mapf(JPitch, JOYMIN_Y, JOYMAX_Y, -5000, 5000);
  JHeave = mapf(JHeave, JOYMIN_Z, JOYMAX_Z, -5000, 5000);

  //Serial.print("Scaled Roll="); Serial.print(JRoll);
  //Serial.print("  Pitch="); Serial.print(JPitch);
  //Serial.print("  Heave="); Serial.println(JHeave);
  //Serial.println("");
  //Serial.println("");

  platformMovePRH(JPitch, JRoll, JHeave);
}  

void platformMovePRH(float Pitch, float Roll, float Heave) {
// This is a very simple inverse kinematic routine to turn PRH values into stepper positions
// Motor 1 at front, 2 Left Rear, 3 Right Rear
// Values as percentages, 0% is mid travel, +/-50% full travel.  For each motor range is 0 to Stepper?_MAXPOS
float MotPos1 = 0;
float MotPos2 = 0;
float MotPos3 = 0;
float MaxVal = -1000;
float MinVal = 1000;
float RemainingHeave = 100.0-Heave;

  MotPos1 = MotPos1 + Pitch;
  MotPos2 = MotPos2 - Pitch;
  MotPos3 = MotPos3 - Pitch;
  
  MotPos2 = MotPos2 + Roll;
  MotPos3 = MotPos3 - Roll;


  //Add in Heave
  MotPos1 = MotPos1 + Heave;
  MotPos2 = MotPos2 + Heave;
  MotPos3 = MotPos3 + Heave;

  //Make sure values are in range
  if(MotPos1>5000) MotPos1=5000;
  if(MotPos2>5000) MotPos2=5000;
  if(MotPos3>5000) MotPos3=5000;
  if(MotPos1<-5000) MotPos1=-5000;
  if(MotPos2<-5000) MotPos2=-5000;
  if(MotPos3<-5000) MotPos3=-5000;
  MaxVal = 5000;
  MinVal = -5000;
/*
  //Find the max & min values
  if(MotPos1 > MaxVal) MaxVal = MotPos1;
  if(MotPos2 > MaxVal) MaxVal = MotPos2;
  if(MotPos3 > MaxVal) MaxVal = MotPos3;
  
  if(MotPos1 < MinVal) MinVal = MotPos1;
  if(MotPos2 < MinVal) MinVal = MotPos2;
  if(MotPos3 < MinVal) MinVal = MotPos3;
*/  
/*
  Serial.print("Mot Pos 1="); Serial.print(MotPos1);
  Serial.print("  2="); Serial.print(MotPos2);
  Serial.print("  3="); Serial.println(MotPos3);
  Serial.println("");
  Serial.println("");
*/  
/*
  //Give priority to pitch & roll over heave
  if((MaxVal-MinVal)>100){
    //Need to scale the values to fit
    float Scale = 100/(MaxVal-MinVal);
    MotPos1 = MotPos1 * Scale;
    MotPos2 = MotPos2 * Scale;
    MotPos3 = MotPos3 * Scale;
    //No values have been scaled, we can reset the max & min
    MaxVal = 50;
    MinVal = -50;
  }

  if(MinVal<-50){
    //Bump up the values to fit in -50 to +50
    MotPos1 = MotPos1 - MinVal + 50;
    MotPos2 = MotPos2 - MinVal + 50;
    MotPos3 = MotPos3 - MinVal + 50;    
  }
  if(MaxVal>50){
    //Reduce the values to fit in -50 to +50
    MotPos1 = MotPos1 - MaxVal - 50;
    MotPos2 = MotPos2 - MaxVal - 50;
    MotPos3 = MotPos3 - MaxVal - 50;    
  }
*/
  //Scale the values into steps
  int MotSteps1 = int(mapf(MotPos1,MinVal,MaxVal, Stepper1_MAXPOS/30, Stepper1_MAXPOS)); //This will turn -50 to +50 range into 3% to Stepper?_MAXPOS
  int MotSteps2 = int(mapf(MotPos2,MinVal,MaxVal, Stepper2_MAXPOS/30, Stepper2_MAXPOS)); //This will turn -50 to +50 range into 3% to Stepper?_MAXPOS
  int MotSteps3 = int(mapf(MotPos3,MinVal,MaxVal, Stepper3_MAXPOS/30, Stepper3_MAXPOS)); //This will turn -50 to +50 range into 3% to Stepper?_MAXPOS

  if(MotSteps1<Stepper1_MAXPOS/10) MotSteps1=Stepper1_MAXPOS/30;
  if(MotSteps2<Stepper2_MAXPOS/10) MotSteps2=Stepper2_MAXPOS/30;
  if(MotSteps3<Stepper3_MAXPOS/10) MotSteps3=Stepper3_MAXPOS/30;

  if(MotSteps1>Stepper1_MAXPOS) MotSteps1=Stepper1_MAXPOS;
  if(MotSteps2>Stepper2_MAXPOS) MotSteps2=Stepper2_MAXPOS;
  if(MotSteps3>Stepper3_MAXPOS) MotSteps3=Stepper3_MAXPOS;
/*
  Serial.print("Stepper Pos 1="); Serial.print(MotSteps1);
  Serial.print("  2="); Serial.print(MotSteps2);
  Serial.print("  3="); Serial.println(MotSteps3);
  Serial.println("");
  Serial.println("");
*/
  //Move steppers to new position
  //Uncomment below when values look realistic
  platformMoveABC(MotSteps1, MotSteps2, MotSteps3);
}

void platformMoveABC(int Motor1, int Motor2, int Motor3) {
//Move motors, non blocking
static int Last1, Last2, Last3; 
  if(Motor1!=Last1) { stepper1.moveTo(Motor1); Last1=Motor1; }
  if(Motor2!=Last2) { stepper2.moveTo(Motor2); Last2=Motor2; }
  if(Motor3!=Last3) { stepper3.moveTo(Motor3); Last3=Motor3; }
  stepper1.run();
  stepper2.run();
  stepper3.run();
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  // the perfect map fonction, with constraining and float handling
  x = constrain(x, in_min, in_max);
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
