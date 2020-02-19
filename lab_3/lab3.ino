#include <Sparki.h>
#include <math.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART2;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0., dY = 0.;

float d_x, d_y, d_theta, euc_dist, wheel_sep_r;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15,0.05, to_radians(135));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void positionError() {
  sparki.println("in position error");
  sparki.updateLCD();
  dX = dest_pose_x - pose_x;
  dY = dest_pose_y - pose_y;
  euc_dist = sqrt((dX*dX) + (dY*dY));
  sparki.moveForward();
  delay((euc_dist/ROBOT_SPEED)*1000);
  sparki.moveStop();
  pose_x = dest_pose_x;
  pose_y = dest_pose_y;
}

void bearingError() {
  sparki.println("In bearing Error");
  sparki.updateLCD();
  dest_pose_theta = to_degrees(dest_pose_theta);
  sparki.moveLeft((atan2f((dest_pose_y -pose_y),(dest_pose_x- pose_x))) - dest_pose_theta);  //radians

  delay(1000);
  dest_pose_theta = to_radians(dest_pose_theta);
  pose_theta = dest_pose_theta;
}

void headingError() {
  sparki.println("In heading error");
  sparki.updateLCD();
  dest_pose_theta = to_degrees(dest_pose_theta);
  sparki.moveRight(dest_pose_theta - pose_theta);
  delay(3000);
  dest_pose_theta = to_radians(dest_pose_theta);
  pose_theta = dest_pose_theta;
  current_state = CONTROLLER_GOTO_POSITION_PART3;
}

void inverseKinematics() {
  phi_l = ((2*ROBOT_SPEED) - (AXLE_DIAMETER * d_theta))/(2*WHEEL_RADIUS);
  phi_r = ((2*ROBOT_SPEED) + (AXLE_DIAMETER * d_theta))/(2*WHEEL_RADIUS);
}


void updateOdometry() {
  // TODO: Update pose_x, pose_y, pose_theta

  wheel_sep_r = AXLE_DIAMETER;                                     
  d_x = cos(pose_theta) * ROBOT_SPEED*100 * CYCLE_TIME;// equivalent to (speed_30 * 100) *    0.1
  d_y = sin(pose_theta) * ROBOT_SPEED*100 * CYCLE_TIME;
  d_theta = 0;

  if(left_wheel_rotating == 1){
     d_theta = ((2*ROBOT_SPEED*100)/(wheel_sep_r*100))*CYCLE_TIME;
     pose_theta += d_theta;
  }
  else if (right_wheel_rotating == 1){
    d_theta =   ((2*ROBOT_SPEED*100)/(wheel_sep_r*100))*CYCLE_TIME;
    pose_theta -= d_theta; 
  } 

  pose_x += d_x;
  pose_y += d_y;

  // Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
  
  displayOdometry();
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      
      bearingError();
      positionError();
      headingError();
      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      //updateOdometry();
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

      break;
  }

  //sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
