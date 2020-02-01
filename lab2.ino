#include <Sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_DISTANCE_MEASURE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
float time_1, time_2, tot_time, time_start, time_end, final_time, tim, speed_30;

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
  // TODO
  if ((line_left < threshold) && (line_right < threshold) && (line_center < threshold)){
    sparki.moveStop();
    time_2 = (float)millis();
    tot_time = time_2 - time_1;
    sparki.clearLCD();
    sparki.println(tot_time);
    sparki.updateLCD();
    current_state = CONTROLLER_FOLLOW_LINE;
    speed_30 = (.3/(tot_time/1000));
    sparki.print("speed: ");
    sparki.println(speed_30);
    sparki.updateLCD();
    
  } else {
    sparki.moveForward();
  }
  
}


void updateOdometry() {
  // TODO
}

void displayOdometry() {
  // TODO
}

void loop() {
  time_start = (float)millis();

  // TODO: Insert loop timing/initialization code here
  readSensors();
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // TODO
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }
  time_end = (float)millis();
  final_time = (time_end - time_start);
  //sparki.clearLCD();
//  sparki.print("final time: ");
//  sparki.println(final_time);
//  sparki.updateLCD();

  delay(1000*CYCLE_TIME - final_time);
//  tim = millis();
//  sparki.print("after cycle");
//  sparki.println(tim-time_start);
//  sparki.updateLCD();
}
