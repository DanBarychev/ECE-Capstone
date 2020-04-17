String BirdPOV = "Bird's Eye View";

int width = 600; 
int height = 600;

int userX = width / 2; 
int userY = height;

int robotReachX = width / 2; 
int robotReachY =  height / 2;
float robotReachDiameter = (2 / 3.0) * width;

int robotX = width / 2;
int robotY = height / 2;
float robotDiameter = (1/15.0) * width;

float landingLocDiam = 20;

float vy = 6;        // horizontal velocity in forward direction (m s^-1)
float vx = 0.2;      // horizontal velocity in left/right direction (m s^-1)
float vz = 2;        // vertical velocity (m s^-1)

// TODO: how to use this info?
//float thetaDeg = 45;                   // vertical angle (deg)
//float theta = (PI / 180) * thetaDeg;   // vertical angle (rad)

float phiDeg = -10;                    // horizontal angle (deg)
float phi = (PI / 180) * phiDeg;       // horizontal angle (rad)

float hThrow = 1.20;                   // height ball at release (m)

float g = 9.81;                        // gravitational field strength (m s^-2)

float t = getTimeOfFlight ();

float dispX = getDisplacementX(t);
float dispY = getDisplacementY(t);

float alpha = atan(dispX / dispY);             // angle of ball's trajectory due to vx
float horizDist = sqrt(sq(dispX) + sq(dispY)); // horizontal distance of landing location from the user

float predictedX = horizDist * sin(alpha + phi);
float predictedY = horizDist * cos(alpha + phi);
  
float mappedX = (width / 2) + (predictedX / 6) * width;  // predicted x and y landing locations mapped onto screen
float mappedY = height * (1 - (predictedY / 6));

float getTimeOfFlight () {
  /* derived from equation s = ut + 1/2 at^2 
  and quadratic formula */

  return (vz + sqrt(sq(vz) + 2 * g * hThrow)) / g;
}

float getDisplacementY (float t) {
  return vy * t; 
}

float getDisplacementX (float t) {
  return vx * t; 
}

void setup(){
  size(600, 600);
}

void draw(){ 
  background(76, 187, 23);
  
  fill(250, 250, 250);
  circle(userX, userY, 20);  // user at bottom of screen
  
  fill(76, 187, 23); 
  circle(robotReachX, robotReachY, robotReachDiameter); // circular robot reach area 
  
  fill(211,211,211);
  circle(robotX, robotY, robotDiameter); // robot
 
  textSize(25);
  
  fill(250, 250, 250);
  text(BirdPOV, 30, 50);
  
  fill(0, 0, 0);
  text("R", robotX-7, robotY+8); // R symbol on robot
  
  fill(0, 0, 255);
  circle(mappedX, mappedY, landingLocDiam); // predicted landing location
  
  fill(255, 0, 0);
  textSize(18);
  text("L", mappedX-5, mappedY+7); // L symbol at landing location
}
