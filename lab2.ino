#include <Sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define MOVE_FORWARD 3
#define MOVE_LEFT 4
#define MOVE_RIGHT 5
#define MOVE_STOP 6


int current_state = CONTROLLER_DISTANCE_MEASURE; // Change this variable to determine which controller to run
int type_move;
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
float time_1, time_2, tot_time, time_start, time_end, final_time, tim, speed_30;
float d_x, pr, r, wheel_sep_r, d_theta;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  time_1 = (float)millis();
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  // distance = sparki.ping();
}

void measure_30cm_speed() {
  if ((line_left < threshold) && (line_right < threshold) && (line_center < threshold)){
    sparki.moveStop();
    time_2 = (float)millis();
    tot_time = time_2 - time_1;
    sparki.clearLCD();
    sparki.println(tot_time);
    sparki.updateLCD();
    current_state = CONTROLLER_FOLLOW_LINE;
    type_move = MOVE_STOP;
    speed_30 = (.3/(tot_time/1000)); //m/s
    sparki.print("speed: ");
    sparki.println(speed_30);
    sparki.updateLCD();
    
  } else {
    sparki.moveForward();
    type_move = MOVE_FORWARD;
  }
  
}

void lineFollow() {
  int threshold = 500;

  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor

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

  sparki.clearLCD(); // wipe the screen

  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);

  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);

  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);

  sparki.updateLCD(); // display all of the information written to the screen

  delay(100); // wait 0.1 seconds
}


void updateOdometry() {
  // TODO
  //speed_30 = v = r * theta_l/r
  //xi = cos(theta) * (v) = v
  //yi = sin(theta) * (v) = 0
  //theta = (2v)/d
  sparki.print("move type: ");
  sparki.println(type_move);
  sparki.updateLCD();
  if (type_move == MOVE_FORWARD){
    d_x = (speed_30*100)*0.1; //cm/s
    sparki.print("d_x");
    sparki.println(d_x);
    pose_x = pose_x + d_x;
  }
  if (type_move == MOVE_RIGHT) {
    //pr = 0.003;
    //r = (8/3.1415926535);
    wheel_sep_r = (8.7/2); //cm
    d_theta = ((2*speed_30))/(wheel_sep_r*10);
    pose_theta = pose_theta + d_theta;
  }
  if (type_move == MOVE_LEFT) {
    //pr = 0.003;
    //r = (8/3.1415926535);
    wheel_sep_r = (8.7/2);
    d_theta = (2*speed_30)/(wheel_sep_r*10);
    pose_theta = pose_theta - d_theta;
  }
  displayOdometry();
}

void displayOdometry() {
  //sparki.clearLCD();
  sparki.print("pose x:");
  sparki.println(pose_x);
  sparki.print("pose y:");
  sparki.println(pose_y);
  sparki.print("pose theta:");
  sparki.println(pose_theta);
  sparki.updateLCD();
}

void loop() {
  time_start = (float)millis();

  // TODO: Insert loop timing/initialization code here
  readSensors();
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
    sparki.moveForward(16);
    type_move = MOVE_FORWARD;
    //sparki.moveRight();
    //type_move = MOVE_RIGHT;
    updateOdometry();
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      //break;
  }
  time_end = (float)millis();
  final_time = (time_end - time_start);

  delay(1000*CYCLE_TIME - final_time);
}
