#include <Sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2

#define MOVE_FORWARD 3
#define MOVE_LEFT 4
#define MOVE_RIGHT 5
#define MOVE_STOP 6


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
int type_move;
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
float time_1, time_2, tot_time, time_start, time_end, final_time, tim;
float d_x, pr, r, wheel_sep_r, d_theta;
double speed_30 = 0.0277520814;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  time_1 = (double)millis();
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
    time_2 = (double)millis();
    tot_time = (double)((time_2 - time_1)/1000.0); //in seconds
    sparki.clearLCD();
    sparki.println(tot_time);
    sparki.updateLCD();
    current_state = CONTROLLER_FOLLOW_LINE;
//    type_move = MOVE_STOP;
    double distance = 0.30;
    speed_30 = (double)(distance/tot_time); //m/s
    sparki.print("speed: ");
    sparki.println(speed_30);
    sparki.updateLCD();    
  } else {
    sparki.moveForward();
//    type_move = MOVE_FORWARD;
  }
  
}

void lineFollow() {  
  if ( line_left < threshold ) // if line is below left line sensor
  {  
    sparki.moveLeft(); // turn left
    type_move = MOVE_LEFT;
    
  }

  if ( line_right < threshold ) // if line is below right line sensor
  {  
    sparki.moveRight(); // turn right
    type_move = MOVE_RIGHT;
  }

  // if the center line sensor is the only one reading a line
  if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
  {
    sparki.moveForward(); // move forward
    type_move = MOVE_FORWARD;
  }  
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
      lineFollow();
      updateOdometry();
      break;
      
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }
  time_end = (float)millis();
  final_time = (time_end - time_start);

  delay(1000*CYCLE_TIME - final_time);
}
