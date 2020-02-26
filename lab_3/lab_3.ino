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
//int current_state = CONTROLLER_FOLLOW_LINE;/
int current_state = CONTROLLER_GOTO_POSITION_PART3;

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

float xr, theta_dot;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  delay(10000);
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.3, 0.15, to_radians(50));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  calcDistanceError();
  orig_dist_to_goal = d_err; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void calcDistanceError(){
  dX = dest_pose_x - pose_x;
  dY = dest_pose_y - pose_y;
  d_err = sqrt((dX*dX) + (dY*dY));
}

void positionError() {
  Serial.println("in position error");
  sparki.updateLCD();
  calcDistanceError();
  
  sparki.moveForward();
  delay((d_err/ROBOT_SPEED)*1000);
  
  sparki.moveStop();
  pose_x = dest_pose_x;
  pose_y = dest_pose_y;
}

void calcBearingError(){
  //atan2f returns radians, dest_pose_theta is in radians
  b_err = atan2f((dest_pose_y -pose_y),(dest_pose_x- pose_x)) - dest_pose_theta;  //radians
}

void bearingError() {
  Serial.println("In bearing Error");
  sparki.updateLCD();
  calcBearingError();
  sparki.moveLeft(to_degrees(b_err));
  delay(1000);
  dest_pose_theta = to_radians(dest_pose_theta);
  pose_theta = dest_pose_theta;
}

void calcHeadingError(){
  h_err = dest_pose_theta - pose_theta;
}
 
void headingError() {
  calcHeadingError();
  
  Serial.println("In heading error");
  sparki.updateLCD();
   
  sparki.moveRight(to_degrees(dest_pose_theta) - to_degrees(pose_theta));
  delay(3000);
  
  pose_theta = dest_pose_theta;
  current_state = -1; //stop robot after fixingheading error
}

void inverseKinematics() {  //calculate necessary wheel speed
  phi_l = (xr - ((AXLE_DIAMETER * theta_dot) / 2.0))/(WHEEL_RADIUS);
  phi_r = (xr + ((AXLE_DIAMETER * theta_dot) / 2.0))/(WHEEL_RADIUS);
}


void find_speed_theta() {
  xr = d_err*0.05;
  if (xr > 0.03) {
    xr = 0.03;
  }
  float p1 = (d_err/orig_dist_to_goal)*b_err;  
  float p2 = (1-(d_err/orig_dist_to_goal))*h_err;
  theta_dot = p1 + p2;
}

void checkStop(){
  if (d_err <0.05 && to_degrees(h_err)<10){
    Serial.print("in check stop");
    sparki.updateLCD();
    sparki.moveStop();
    left_speed_pct = 0.;
    right_speed_pct = 0.;
    b_err = 0;
    d_err = 0;
    h_err = 0;
    current_state = -1; //which state is this?
  }
}


void updateOdometry() {
  // TODO: Update pose_x, pose_y, pose_theta
                                   
  dX = ((cos(pose_theta) * ROBOT_SPEED * CYCLE_TIME)/2) * (left_speed_pct + right_speed_pct);// m/s
  dY = ((sin(pose_theta) * ROBOT_SPEED * CYCLE_TIME)/2) * (left_speed_pct + right_speed_pct); //m/s
  dTheta = ((ROBOT_SPEED * CYCLE_TIME)/AXLE_DIAMETER) * (right_speed_pct - left_speed_pct);

  pose_x += dX;
  pose_y += dY;
  pose_theta += dTheta;

  // Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
  
  displayOdometry();
}

void displayOdometry() {  
  Serial.print("X: ");
  Serial.print(pose_x);
  Serial.print(" Xg: ");
  Serial.println(dest_pose_x);
  Serial.print("Y: ");
  Serial.print(pose_y);
  Serial.print(" Yg: ");
  Serial.println(dest_pose_y);
  Serial.print("T:");
  Serial.print(to_degrees(pose_theta));
  Serial.print(" Tg:");
  Serial.println(to_degrees(dest_pose_theta));

  Serial.print("dX : ");
  Serial.print(dX );
  Serial.print(" dT: ");
  Serial.println(dTheta);
  Serial.print("phl: "); Serial.print(phi_l); Serial.print(" phr: "); Serial.println(phi_r);
  Serial.print("p: "); Serial.print(d_err); Serial.print(" a: "); Serial.println(to_degrees(b_err));
  Serial.print("h: "); Serial.println(to_degrees(h_err)); Serial.print(left_speed_pct); Serial.println(right_speed_pct);
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if ((line_center < threshold) && (line_left > threshold) && (line_right > threshold) ) {
        //move forward
        left_wheel_rotating = 1;
        right_wheel_rotating = 1;
        left_dir = DIR_CCW;
        right_dir = DIR_CW;
        left_speed_pct = 1;
        right_speed_pct = 1;
        sparki.motorRotate(MOTOR_LEFT, left_dir, left_speed_pct*100);
        sparki.motorRotate(MOTOR_RIGHT, right_dir, right_speed_pct*100);
      } else if (line_left < threshold) {
        //move left
         left_wheel_rotating = 1;
        right_wheel_rotating = 1;
        left_dir = DIR_CW;
        right_dir = DIR_CW;
        left_speed_pct = -1;
        right_speed_pct = 1;
        
        sparki.motorRotate(MOTOR_LEFT, left_dir, abs(left_speed_pct*100));
        sparki.motorRotate(MOTOR_RIGHT, right_dir, right_speed_pct*100);
        
      } else if (line_right < threshold) {
        // move right
        left_wheel_rotating = 1;
        right_wheel_rotating = 1;
        left_dir = DIR_CCW;
        right_dir = DIR_CCW;
        left_speed_pct = 1;
        right_speed_pct = -1;
        
        sparki.motorRotate(MOTOR_LEFT, left_dir, left_speed_pct*100);
        sparki.motorRotate(MOTOR_RIGHT, right_dir, abs(right_speed_pct*100));
        
      } 
      else {
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
      
      bearingError();
      positionError();
      headingError();
      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      updateOdometry();
      
      calcDistanceError();
      calcBearingError();
      calcHeadingError();
      checkStop();
      find_speed_theta();
      inverseKinematics();

      // scale wheel speed percentages by largest phi
      float denominator= phi_r;
      if (phi_l >= phi_r){
        denominator = phi_l;
      }
      
      left_speed_pct = phi_l/denominator; 
      right_speed_pct = phi_r/denominator;
      Serial.print(left_speed_pct);
      Serial.print(right_speed_pct);
      
      left_dir = DIR_CCW;
      if (left_speed_pct < 0) {   // negative left phi -> moving left wheel backwards
        left_dir = DIR_CW;
      }
      right_dir = DIR_CW;
      if (right_speed_pct < 0) {    // negative right phi -> moving right wheel backwards
        right_dir = DIR_CCW;
      }

      sparki.motorRotate(MOTOR_LEFT, left_dir, abs(int(left_speed_pct*100.)));
      sparki.motorRotate(MOTOR_RIGHT, right_dir, abs(int(right_speed_pct*100.)));
      
      break;
  }

  sparki.clearLCD();
  //updateOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
