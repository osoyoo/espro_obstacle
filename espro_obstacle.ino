/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Espro Robot Car Object Avoidace project
 * Tutorial URL https://osoyoo.com/?p=61263
 * CopyRight www.osoyoo.com
 * This project will show you how to make ESPRO robot car in auto drive mode and avoid obstacles
 */
#include <ESP32Servo.h>
#define speedPinR 16    //  RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  23    //Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2  25    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 17    // Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  26    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  27   //Left Motor direction pin 1 to MODEL-X IN4
#define SPEED 100
#define LPT 2 // scan loop coumter
#define SERVO_PIN    32  //servo connect to D32
#define Echo_PIN    19 // Ultrasonic Echo pin connect to D19
#define Trig_PIN    18  // Ultrasonic Trig pin connect to D18

#define BUZZ_PIN     12
#define FAST_SPEED  220     //both sides of the motor speed
#define SPEED  120     //both sides of the motor speed
#define TURN_SPEED  160     //both sides of the motor speed
#define BACK_SPEED1  200     //back speed
#define BACK_SPEED2  90     //back speed

// Motor PWM configuration - use lower frequency to reduce servo interference
#define MOTOR_PWM_FREQ 1000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_L_CHANNEL 14
#define MOTOR_R_CHANNEL 15

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends turning (miliseconds)

int thereis;
Servo head;
/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Left()  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(0,0);
}

/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  ledcWrite(speedPinL, speed_L);
  ledcWrite(speedPinR, speed_R);
}

void buzz_ON()   //open buzzer
{
  
  for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
  }
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, LOW);
  
}
void alarm(){
   buzz_ON();
   buzz_OFF();
}

/*detection of ultrasonic distance*/
int watch(){
  long echo_distance;
  long pulse_duration;
  // Ensure trigger is LOW
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(2);

  // Send 10us pulse to trigger
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN,LOW);

  // Read echo pulse with timeout (30ms = ~5m max range)
  pulse_duration = pulseIn(Echo_PIN, HIGH, 30000);
  if (pulse_duration == 0) {
    // Timeout - no echo received
    return 0;
  }

  // Calculate distance in cm (speed of sound = 343m/s)
  // Distance = (pulse_duration * 0.0343) / 2
  echo_distance = pulse_duration * 0.01715; // in cm

  return round(echo_distance);
}
//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
String watchsurrounding(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */

  // Detach motor PWM and re-initialize servo to reclaim timers
  ledcDetach(speedPinL);
  ledcDetach(speedPinR);
  delay(50);  // Wait for motor PWM to fully stop

  // Re-attach servo to ensure it has control of its timers
  head.detach();
  delay(10);
  head.setPeriodHertz(50);
  head.attach(SERVO_PIN, 500, 2400);
  delay(50);

int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | B100;
    }

  head.write(150);
  delay(400);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
    alarm();
     obstacle_status  =obstacle_status | B1000;
    }
  head.write(120); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(400);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
    alarm();
     obstacle_status  =obstacle_status | B10000;
    }

  head.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(400);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | B100;
    }
  head.write(60);
  delay(400);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | B10;
    }
  head.write(30);
  delay(400);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | 1;
    }
  head.write(90); //Finish looking around (look forward again)
  delay(400);

  // Detach servo before re-attaching motor PWM to avoid conflicts
  head.detach();
  delay(10);

  // Re-attach motor PWM after servo movements complete
  ledcAttach(speedPinL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttach(speedPinR, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  delay(10);

  // Re-attach servo for next scan
  head.setPeriodHertz(50);
  head.attach(SERVO_PIN, 500, 2400);
  head.write(90);  // Keep at center position
  delay(50);

  String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);

  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
}

void auto_avoidance(){

  ++numcycles;
  if(numcycles>=LPT){ //Watch if something is around every LPT loops while moving forward 
     stop_Stop();
    String obstacle_sign=watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
      Serial.print("begin str=");
        Serial.println(obstacle_sign);
                    if( obstacle_sign=="10000"){
     Serial.println("SLIT right");
          set_Motorspeed(FAST_SPEED,SPEED);
     go_Advance();
 
      delay(turntime);
      stop_Stop();
    }
        else    if( obstacle_sign=="00001"  ){
     Serial.println("SLIT LEFT");
       set_Motorspeed(SPEED,FAST_SPEED);
      go_Advance();
  
      delay(turntime);
      stop_Stop();
    }
    else if( obstacle_sign=="11100" || obstacle_sign=="01000" || obstacle_sign=="11000"  || obstacle_sign=="10100"  || obstacle_sign=="01100" ||obstacle_sign=="00100"  ||obstacle_sign=="01000" ){
     Serial.println("hand right");
	    go_Right();
      set_Motorspeed(TURN_SPEED,TURN_SPEED);
      delay(turntime);
      stop_Stop();
    } 
    else if( obstacle_sign=="00010" || obstacle_sign=="00111" || obstacle_sign=="00011"  || obstacle_sign=="00101" || obstacle_sign=="00110" || obstacle_sign=="01010" ){
    Serial.println("hand left");
     go_Left();//Turn left
     set_Motorspeed(TURN_SPEED,TURN_SPEED);
      delay(turntime);
      stop_Stop();
    }
 
    else if(  obstacle_sign=="01111" ||  obstacle_sign=="10111" || obstacle_sign=="11111"  ){
    Serial.println("hand back right");
	  go_Left();
		set_Motorspeed( FAST_SPEED,SPEED);
       delay(backtime);
          stop_Stop();
        } 
         else if( obstacle_sign=="11011"  ||    obstacle_sign=="11101"  ||  obstacle_sign=="11110"  || obstacle_sign=="01110"  ){
    Serial.println("hand back left");
    go_Right();
    set_Motorspeed( SPEED,FAST_SPEED);
       delay(backtime);
          stop_Stop();
        }    
  
        else Serial.println("no handle");
    numcycles=0; //Restart count of cycles
  } else {
     set_Motorspeed(SPEED,SPEED);
     go_Advance();  // if nothing is wrong go forward using go() function above.
        delay(backtime);
          stop_Stop();
  }
  
  //else  Serial.println(numcycles);
  
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
 Serial.println("go right");
    go_Right();
    set_Motorspeed( SPEED,SPEED);
  delay(backtime);
      ++thereis;}
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 25){
  Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis=0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start!");

  /*init servo FIRST and test it BEFORE setting up motor PWM*/
  Serial.println("Initializing servo...");
  head.setPeriodHertz(50);
  head.attach(SERVO_PIN, 500, 2400);
  Serial.println("Servo initialized");

  delay(100);

  Serial.println("Testing servo...");
  head.write(30);//rotate to right
  delay(1000);
  head.write(150);//rotate to left
  delay(1000);
  head.write(90);//center
  delay(1000);
  Serial.println("Servo positioned at 90 degrees (center)");

  /*setup L298N motor pins*/
  pinMode(RightDirectPin1, OUTPUT);
  pinMode(RightDirectPin2, OUTPUT);
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT);

  /*setup motor PWM AFTER servo test completes*/
  Serial.println("Setting up motor PWM...");
  ledcAttach(speedPinL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttach(speedPinR, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

  // Set initial speed to 0
  ledcWrite(speedPinL, 0);
  ledcWrite(speedPinR, 0);

  Serial.println("Motor PWM initialized");

  // Re-assert servo position after motor PWM setup to prevent drift
  delay(100);
  head.write(90);
  delay(500);
  Serial.println("Servo re-positioned at 90 degrees");

  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN,INPUT);
  digitalWrite(Trig_PIN,LOW);

  Serial.println("Testing ultrasonic sensor...");
  delay(100);

  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);
  buzz_OFF();
  set_Motorspeed(200,200);
}

void loop() {
   auto_avoidance();
}
