int fps = 60;

int width = 1200; 
int height = 600;

int viewWidth = width / 2;

float landingLocDiam;

float actualX = 0.3;
float actualY = 1.1;
float actualT = 0.9;

/*
float actualX = 2.5;  // actual landing location coordinates (m)
float actualY = 1.5;
float actualT = 0.8; // actual time of flight (s)
*/

/*
float actualX = -1;  // actual landing location coordinates (m)
float actualY = 3;
float actualT = 0.8; // actual time of flight (s)
*/
float mappedActualT = actualT * fps;   // in frames


float vy = 1.37;     // vx of AHRS
float vx = 0.22;     // vy of AHRS
float vz = 0.36;     // vz of AHRS


/*
float vy = 3.9; 
float vx = -1.2;
float vz = 2; 
*/

/*
float vy = 2;        // horizontal velocity in forward direction (m s^-1)
float vx = 4;        // horizontal velocity in left/right direction (m s^-1)
float vz = 2;        // vertical velocity (m s^-1)
*/

// TODO: how to use this info?
// float thetaDeg = 45;                   // vertical angle (deg)
// float theta = (PI / 180) * thetaDeg;   // vertical angle (rad)

float phiDeg = 19;                    // horizontal angle (deg)
float phi = (PI / 180) * phiDeg;       // horizontal angle (rad)

float hThrow = 1;                   // height ball at release (m)

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
int robotSteps;
int robotReturnSteps;

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
  landingLocDiam = 60;  // before: 20
  t = getTimeOfFlight ();
  dispX = getDisplacementX(t);
  dispY = getDisplacementY(t);
  if (dispY == 0) dispY = 0.000000001;     // TODO: fix this
  alpha = atan(dispX / dispY);             // angle of ball's trajectory due to vx, TODO: edge case?
  horizDist = sqrt(sq(dispX) + sq(dispY)); // horizontal distance of landing location from the user

  predictedX = horizDist * sin(alpha + phi);
  predictedY = (signNum(dispY)) * (horizDist * cos(alpha + phi));

  mappedX = (viewWidth / 2) + (predictedX / 2) * viewWidth; // before: / 6  // predicted x and y landing locations mapped onto screen
  mappedY = height * (1 - (predictedY / 2));   // before: /6

  mappedActualX = (viewWidth / 2) + (actualX / 2) * viewWidth; // before: /6
  mappedActualY= height * (1 - (actualY / 2));         // before: /6
} 

void initBallData() {
  ballDiam = 15;  // before: 5
  ballX = viewWidth / 2;
  ballY = height;
  isLanded = false;
  ballCaught = false;
}

// TODO: scale?
void initBallSpeed() {
  ballMappedSpeedX = signNum(actualX) * ((dist(ballX, 0, mappedActualX, 0) / actualT) / fps);
  ballMappedSpeedY = signNum(actualY) * ((dist(0, ballY, 0, mappedActualY) / actualT) / fps);
}

void initBall () {
  initBallData();
  initBallSpeed();
}

void initUser() {
  userX = viewWidth / 2; 
  userY = height;
}

void initRobotData() {
  robotReachX = viewWidth / 2; 
  robotReachY =  height / 2;
  robotReachDiameter = viewWidth; // before:(2 / 3.0) * width
  robotStartX = viewWidth / 2;
  robotStartY = height / 2;
  robotX = robotStartX;
  robotY = robotStartY;
  robotDiameter = (1/5.0) * viewWidth; // before:1 / 15.0
  isLocReached = false;
  dirChanged = false;
  robotSteps = 0;
  robotReturnSteps = 0;
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
  robotMappedV = ((viewWidth * robotSpeed) / 2) / fps; // before: /6
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
  size(1200, 600);
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
  if ((!isLocReached) && (frameCount < mappedActualT)) {
    robotSteps += 1;
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
    robotReturnSteps+=1;
    if (robotReturnSteps < robotSteps) {
      moveRobot();
    }
    
  } 
}

void drawLandings() {
   fill(0, 0, 255);
   circle(mappedX, mappedY, landingLocDiam); // predicted landing location
  
   fill(255, 0, 0);
   textSize(20);
   text("Lp", mappedX-10, mappedY+8); // Lp symbol at predicted landing location 

   fill(255, 255, 0);
   circle(mappedActualX, mappedActualY, landingLocDiam); // actual landing location
  
   fill(255, 0, 0);
   textSize(20);
   text("La", mappedActualX-10, mappedActualY+8); // before: -8, + 5 // La symbol at predicted landing location
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

void drawScreen () {
  background(76, 187, 23);
  
  fill(76, 187, 23); 
  circle(robotReachX, robotReachY, robotReachDiameter); // circular robot reach area 
  
  fill(250, 250, 250);
  circle(userX, userY, 60);  // before:20 // user at bottom of screen
  
  drawLandings();
  
  fill(211,211,211);
  circle(robotX, robotY, robotDiameter); // robot
  
  textSize(25);
  
  fill(250, 250, 250);
  text(BirdPOV, 30, 50);
  
  fill(0, 0, 0);
  text("R", robotX-21, robotY+24); // before: -7, +8 // R symbol on robot
  
  drawball ();
}

void draw(){ 
  // default frame rate: 60 frames per second
  timerFired();
  drawScreen();
}
