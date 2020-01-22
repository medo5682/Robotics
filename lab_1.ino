#include <Sparki.h>

#define STATE_ROTATE
#define STATE_CAPTURE
#define STATE_DRIVE
#define STATE_FOLLOW_LINE
#define STATE_STOP_BEEP_DROP

// Set up some global variables with default values to be replaced during operation
int current_state = STATE_ROTATE;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int cm_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;


void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance= sparki.ping();
  line_left= sparki.lineLeft();
  line_right= sparki.lineRight();
  line_center= sparki.lineCenter();
  }


void turn_180_drive(current_state) {
  sparki.moveRight(180); // rotate right 180 degrees
  if((line_center < threshold) && (line_left < threshold) && (line_right < threshold)){
    sparki.moveForward();
  } else {
    spark.moveStop();
  }
}



//void updateState(current_state) {
//  //check if transition is met and return
//  
//}


void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  sparki.clearLCD();
  sparki.print("STATExx: ");
  sparki.println(current_state);

  

  // Your state machine code goes here

  switch(current_state){
    case STATE_ROTATE:
      break;
    case STATE_CAPTURE:
      break;
    case STATE_DRIVE:
      break;
    case STATE_FOLLOW_LINE:
      break;
    case STATE_STOP_BEEP_DROP:
      break;
    default:
      break;
  }

  //current_state = updateState(current_state);

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}
