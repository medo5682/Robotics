#include <Sparki.h>
#include <math.h>

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
double time_1, time_2, tot_time, time_start, time_end, final_time, tim;
double d_x,d_y, pr, r, wheel_sep_r, d_theta;
double speed_30 = 0.0277520814; // m/s
//double speed_30;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
//  time_1 = (double)millis();
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void measure_30cm_speed() {
  if ((line_left < threshold) && (line_right < threshold) && (line_center < threshold)){
    sparki.moveStop();
    time_2 = millis();
    tot_time = time_2/1000.0; //in seconds
    sparki.clearLCD();
    current_state = CONTROLLER_FOLLOW_LINE;
    speed_30 = 30.0/tot_time; //m/s
    sparki.print("speed: ");
    sparki.println(speed_30);
    sparki.updateLCD();    
  } else {
    sparki.moveForward();
  }
  
}

void lineFollow() {  

  if ((line_left<threshold) && (line_right<threshold)&& (line_center<threshold)){
//      sparki.println("START");
//      sparki.updateLCD();
//      pose_x = 0;
//      pose_y = 0;
//      pose_theta= 0;
      sparki.moveForward();
      type_move = MOVE_FORWARD;
  }
  else{
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
}


void updateOdometry() {

  wheel_sep_r = 0.087;//                                          m/s    * 100 cm/m  * cycle time
  d_x = cos(pose_theta) * speed_30*100 * CYCLE_TIME;// equivalent to (speed_30 * 100) *    0.1
  d_y = sin(pose_theta) * speed_30*100 * CYCLE_TIME;
  if (type_move == MOVE_FORWARD){
    d_theta=0;
  }
  else if(type_move == MOVE_LEFT){
     d_theta = ((2*speed_30*100)/(wheel_sep_r*100))*CYCLE_TIME;
     pose_theta += d_theta;
  }
  else if (type_move == MOVE_RIGHT){
    d_theta =   ((2*speed_30*100)/(wheel_sep_r*100))*CYCLE_TIME;
    pose_theta -= d_theta; 
  }

  pose_x += d_x;
  pose_y += d_y;
  displayOdometry();
}


void displayOdometry() {
  sparki.clearLCD();
  sparki.print("move type: ");
  sparki.println(type_move);
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
