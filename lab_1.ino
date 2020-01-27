#include <Sparki.h>

#define STATE_ROTATE 0
#define STATE_DRIVE_AND_CAPTURE 1
#define STATE_DRIVE 2
#define STATE_TURN_180 3
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
//  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance= sparki.ping();
  lineLeft= sparki.lineLeft();
  lineRight= sparki.lineRight();
  lineCenter= sparki.lineCenter();
}


void rotate() {  
    sparki.moveRight();
    if(cm_distance < 30 && cm_distance >0){
          sparki.print("Object found at ");
          sparki.print(cm_distance);
          sparki.println("cm");
          current_state = STATE_DRIVE_AND_CAPTURE;
          sparki.print("CHANGING STATE TO");
          sparki.println(current_state);
          sparki.updateLCD();
          sparki.moveStop();
    }
}

void drive_and_capture() { 
    if (cm_distance > 3){
      sparki.moveForward();  
    }
    else{
      sparki.moveStop();
      sparki.gripperClose();
      current_state = STATE_TURN_180;
      sparki.println("CHANGING STATE TO");
      sparki.println(current_state);
      sparki.updateLCD();
      delay(5000);
      sparki.gripperStop();
    }
}

void drive(){
  if( (lineLeft > threshold) && (lineRight > threshold) ){
    sparki.println("HAVENT FOUND THE LINE YET");
    sparki.updateLCD();
    sparki.moveForward();
    current_state = STATE_DRIVE;
  } 
  else {
    sparki.println("FOUND THE LINE");
    current_state = STATE_FOLLOW_LINE;
    sparki.updateLCD();
    sparki.moveStop();
  }
  
}


void turn_180() {
  sparki.println("TURN 180 and DRIVE CALLED");
  sparki.updateLCD();
  sparki.moveRight(180); // rotate right 180 degrees
  current_state= STATE_DRIVE;
}


void line_to_start() {
  sparki.println("LINE TO START");
  sparki.println(lineLeft);
  sparki.println(lineRight);
  sparki.println(lineCenter);
  sparki.updateLCD();
  delay(1000);
  if( (lineCenter <threshold) && (lineLeft <threshold) && (lineRight < threshold) ){
    current_state = STATE_STOP_BEEP_DROP;
  }  
  
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
  
  delay(100); // wait 0.1 seconds

  
}

void stop_beep_drop(){
  sparki.moveStop();
  sparki.beep();
  delay(1000);
  sparki.gripperOpen();
  delay(5000);  
  current_state = STATE_DONE;
}




void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);
  sparki.updateLCD();


  // Your state machine code goes here

  switch(current_state){
    case STATE_ROTATE:
      rotate();
      break;
    case STATE_DRIVE_AND_CAPTURE:
      drive_and_capture();
      break;
    case STATE_TURN_180:
      turn_180();
      break;
    case STATE_FOLLOW_LINE:
      line_to_start();
      break;
    case STATE_STOP_BEEP_DROP:
      stop_beep_drop();
      break;
   case STATE_DRIVE:
      drive();
      break;
    default:
      break;
  }
  
  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}
