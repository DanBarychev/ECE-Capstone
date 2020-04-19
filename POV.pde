int fps = 60;

int width = 600; 
int height = 600;

float landingLocDiam; 

float actualX = -1;  // actual landing location coordinates (m)
float actualY = 3;
float actualT = 0.8; // actual time of flight (s)
float mappedActualT = actualT * fps;   // in frames


/*
float vy = 3.9; 
float vx = -1.2;
float vz = 2; 
*/

float vy = 3.9;        // horizontal velocity in forward direction (m s^-1)
float vx = -1.2;        // horizontal velocity in left/right direction (m s^-1)
float vz = 2;        // vertical velocity (m s^-1)

// TODO: how to use this info?
// float thetaDeg = 45;                   // vertical angle (deg)
// float theta = (PI / 180) * thetaDeg;   // vertical angle (rad)

float phiDeg = 0;                    // horizontal angle (deg)
float phi = (PI / 180) * phiDeg;       // horizontal angle (rad)

float hThrow = 1.20;                   // height ball at release (m)

float g = 9.81;                        // gravitational field strength (m s^-2)

float t;
float dispX;
float dispY;
float alpha;             // angle of ball's trajectory due to vx, TODO: edge case?
float horizDist;         // horizontal distance of landing location from the user
float predictedX;
float predictedY;
float mappedX;  // predicted x and y landing locations mapped onto screen
float mappedY;
float mappedActualX;
float mappedActualY;

int userX; 
int userY;

float ballDiam;
float ballX;
float ballY;
float ballMappedSpeedX;
float ballMappedSpeedY;
boolean isLanded;
boolean ballCaught;

int robotReachX; 
int robotReachY;
float robotReachDiameter;
float robotStartX;
float robotStartY;
float robotX;
float robotY;
float robotDiameter;
float robotSpeed = 1.5;   // m s^-1
float robotMappedV;
float robotMoveAngle; 
float robotMappedVx;
float robotMappedVy;
float robotTravelTime;
float robotMappedVxMag;
float robotMappedVyMag;
boolean isLocReached;
boolean dirChanged;
int dirChangeFrame;

String BirdPOV = "Bird's Eye View";

int signNum (float num) {
  if (num < 0) return -1;
  else return 1;
}

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

void predictLandingLocation () {
  landingLocDiam = 20; 
  t = getTimeOfFlight ();
  dispX = getDisplacementX(t);
  dispY = getDisplacementY(t);
  if (dispY == 0) dispY = 0.000000001;     // TODO: fix this
  alpha = atan(dispX / dispY);             // angle of ball's trajectory due to vx, TODO: edge case?
  horizDist = sqrt(sq(dispX) + sq(dispY)); // horizontal distance of landing location from the user

  predictedX = horizDist * sin(alpha + phi);
  predictedY = (signNum(dispY)) * (horizDist * cos(alpha + phi));

  mappedX = (width / 2) + (predictedX / 6) * width;  // predicted x and y landing locations mapped onto screen
  mappedY = height * (1 - (predictedY / 6));

  mappedActualX = (width / 2) + (actualX / 6) * width;
  mappedActualY= height * (1 - (actualY / 6));
}

void initBallData() {
  ballDiam = 5;
  ballX = width / 2;
  ballY = height;
  isLanded = false;
  ballCaught = false;
}

void initBallSpeed() {
  ballMappedSpeedX = signNum(actualX) * ((dist(ballX, 0, mappedActualX, 0) / actualT) / fps);
  ballMappedSpeedY = signNum(actualY) * ((dist(0, ballY, 0, mappedActualY) / actualT) / fps);
}

void initBall () {
  initBallData();
  initBallSpeed();
}

void initUser() {
  userX = width / 2; 
  userY = height;
}

void initRobotData() {
  robotReachX = width / 2; 
  robotReachY =  height / 2;
  robotReachDiameter = (2 / 3.0) * width;
  robotStartX = width / 2;
  robotStartY = height / 2;
  robotX = robotStartX;
  robotY = robotStartY;
  robotDiameter = (1/15.0) * width;
  isLocReached = false;
  dirChanged = false;
}

int getQuadrant () {
  if ((mappedX <= robotStartX) && (mappedY > robotStartY)) return 1;
  if ((mappedX >= robotStartX) && (mappedY <= robotStartY)) return 3;
  else if ((mappedX < robotStartX) && (mappedY <= robotStartY)) return 4;
  else return 2;
}

float[] getRobotVelocities(float robotMappedVxMag, float robotMappedVyMag) {
  float[] velocities = new float[2];
  int quadrant = getQuadrant();
  println(quadrant);
  if (quadrant == 1) {
    velocities[0] = -robotMappedVxMag;
    velocities[1] = robotMappedVyMag;
  }
  else if (quadrant == 2) {
    velocities[0] = robotMappedVxMag;
    velocities[1] = robotMappedVyMag;
  }
  else if (quadrant == 3) {
    velocities[0] = robotMappedVxMag;
    velocities[1] = -robotMappedVyMag;
  }
  else {
    velocities[0] = -robotMappedVxMag;
    velocities[1] = -robotMappedVyMag;
  }
  return velocities;
}

void initRobotNavigation () {
  robotMappedV = ((width * robotSpeed) / 6) / fps;
  robotMoveAngle = atan((abs(robotY - mappedY)) / (abs(robotX - mappedX)));
  robotMappedVxMag = robotMappedV * cos(robotMoveAngle);
  robotMappedVyMag = robotMappedV * sin(robotMoveAngle);
  float[] robotVelocities = getRobotVelocities(robotMappedVxMag, robotMappedVyMag);
  robotMappedVx = robotVelocities[0];
  robotMappedVy = robotVelocities[1];
  robotTravelTime = (dist(robotX, robotY, mappedX, mappedY) / robotMappedV);
}

void initRobot() {
  initRobotData();
  initRobotNavigation ();
}

void init () {
  predictLandingLocation();
  initUser();
  initBall ();
  initRobot();
}

void setup(){
  size(600, 600);
  init();  
}

void moveBall() {
  ballX += ballMappedSpeedX;
  ballY -= ballMappedSpeedY;
}

void moveRobot() {
  robotX += robotMappedVx;
  robotY += robotMappedVy;
}

void changeDirection () {
  robotMappedVx = -robotMappedVx;
  robotMappedVy = -robotMappedVy;
}

void isBallCaught () {
  if (dist(robotX, robotY, ballX, ballY) < ((ballDiam / 2)  + (robotDiameter / 2))) {
    ballCaught = true;
  }
}

void timerFired() {
  if (frameCount < mappedActualT) {
    moveBall();
  }
  else if (!isLanded) {
    isLanded = true;
  }
 
  if (frameCount < robotTravelTime) { 
    moveRobot();
  }
  else if (!isLocReached){
    isLocReached = true;
  }
  
  if (isLanded && isLocReached) {
    // robot goes back to start location
    if (!dirChanged) {
      dirChanged = true;
      dirChangeFrame = frameCount;
      changeDirection();
      isBallCaught();
    }
    if ((frameCount - dirChangeFrame) < robotTravelTime) {
      moveRobot();
    }
  } 
}

void drawLandings() {
   fill(0, 0, 255);
   circle(mappedX, mappedY, landingLocDiam); // predicted landing location
  
   fill(255, 0, 0);
   textSize(15);
   text("Lp", mappedX-8, mappedY+5); // Lp symbol at predicted landing location 

   fill(255, 255, 0);
   circle(mappedActualX, mappedActualY, landingLocDiam); // actual landing location
  
   fill(255, 0, 0);
   textSize(15);
   text("La", mappedActualX-8, mappedActualY+5); // La symbol at predicted landing location
}

void drawScreen () {
  background(76, 187, 23);
  
  fill(250, 250, 250);
  circle(userX, userY, 20);  // user at bottom of screen
  
  fill(76, 187, 23); 
  circle(robotReachX, robotReachY, robotReachDiameter); // circular robot reach area 
  
  drawLandings();
  
  fill(211,211,211);
  circle(robotX, robotY, robotDiameter); // robot
  
  textSize(25);
  
  fill(250, 250, 250);
  text(BirdPOV, 30, 50);
  
  fill(0, 0, 0);
  text("R", robotX-7, robotY+8); // R symbol on robot
  
  drawball ();
}

void drawball () {
  fill(250, 0, 0);
  if (!ballCaught) {
    circle(ballX, ballY, ballDiam);  
  }
  else {
    circle(robotX, robotY, ballDiam); 
  }
} 

void draw(){ 
  // default frame rate: 60 frames per second
  timerFired();
  drawScreen();
}
