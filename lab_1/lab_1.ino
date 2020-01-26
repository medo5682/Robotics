#include <Sparki.h>

#define STATE_ROTATE 0
#define STATE_DRIVE_AND_CAPTURE 1
#define STATE_DRIVE 2
#define STATE_TURN_180_DRIVE 3
#define STATE_FOLLOW_LINE 4
#define STATE_STOP_BEEP_DROP 5
#define STATE_DONE 6

// Set up some global variables with default values to be replaced during operation
int current_state = STATE_ROTATE;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int cm_distance = 1000;
int lineLeft = 1000;
int lineCenter = 1000;
int lineRight = 1000;


void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
//  sparki.gripperSt/op(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance= sparki.ping();
  lineLeft= sparki.lineLeft();
  lineRight= sparki.lineRight();
  lineCenter= sparki.lineCenter();
}


void turn_180_drive(int current_state) {
  sparki.moveRight(180); // rotate right 180 degrees
  if((lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold)){
    sparki.moveForward();
  } else {
    sparki.moveStop();
  }
}

void rotate() {  
    sparki.moveRight();
    if(cm_distance < 30 && cm_distance >0){
          sparki.print("Object found at ");
          sparki.print(cm_distance);
          sparki.println("cm");
          current_state = STATE_DRIVE_AND_CAPTURE;
          sparki.print("CHANGING STATE TO DRIVE");
          sparki.println(current_state);
          sparki.updateLCD();
          sparki.moveStop();
          delay(1000);
    }
}

void drive_and_capture() { 
    sparki.println("DRIVE CALLED"); 
    sparki.updateLCD();
    if (cm_distance > 7){
      sparki.moveForward();  
    }
    else{
      sparki.moveStop();
      sparki.gripperClose();
      current_state = STATE_TURN_180_DRIVE;
      sparki.print("CHANGING STATE TO STATE_TURN_180_DRIVE");
      sparki.println(current_state);
      sparki.updateLCD();
      delay(5000);
    }
}


void turn_180_drive() {
  sparki.moveRight(180); // rotate right 180 degrees
  if((lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold)){
    sparki.moveForward();
  } else {
    current_state = STATE_FOLLOW_LINE;
    sparki.moveStop();
  }
}


void line_to_start() {
  if ( lineLeft < threshold ) // if line is below left line sensor
  {  
    sparki.moveLeft(); // turn left
  }
 
  if ( lineRight < threshold ) // if line is below right line sensor
  {  
    sparki.moveRight(); // turn right
  }
 
  // if the center line sensor is the only one reading a line
  if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
  {
    sparki.moveForward(); // move forward
  }
  else if(false){
    current_state = STATE_STOP_BEEP_DROP;
  }  
  delay(100); // wait 0.1 seconds

  
  
}

void stop_beep_drop(){
  sparki.moveStop();
  delay(1000);
  sparki.beep();
  delay(1000);
  sparki.gripperOpen();
  delay(5000);  
  current_state = STATE_DONE;
}




void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

//  sparki.clearLCD();
//  sparki.print("STATE: ");/
//  sparki.println(current_state);/
//  sparki.updateLCD();/


  // Your state machine code goes here

  switch(current_state){
    case STATE_ROTATE:
      rotate();
      break;
    case STATE_DRIVE_AND_CAPTURE:
      drive_and_capture();
      break;
    case STATE_TURN_180_DRIVE:
      turn_180_drive();
      break;
    case STATE_FOLLOW_LINE:
      line_to_start();
      break;
    case STATE_STOP_BEEP_DROP:
      stop_beep_drop();
      break;
    default:
      break;
  }
  
  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}
