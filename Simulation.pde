PImage img;        //source: https://unsplash.com/photos/Qgq7j_QCYtw
PImage target;     //source: http://www.clker.com/clipart-red-snipper-target.html
PImage sky;        //source: https://www.freepik.com/free-photo/white-cloud-blue-sky-sea_3962982.htm#page=1&query=sky&position=0
PImage aboveP;     //source: https://www.pinterest.com/pin/267753140317410600/
PImage basket;     //source: https://www.shutterstock.com/image-photo/empty-fruit-wicker-brown-basket-bowl-266927345
PImage tennis;     //source: https://www.stickpng.com/img/sports/tennis/ball-tennis
PImage manSide;    //source: http://immediate-entourage.blogspot.com/2011/04/man-standing-side-view.html
PImage basketSide; //source: https://d2gg9evh47fn9z.cloudfront.net/800px_COLOURBOX3327394.jpg
// Consulted the Carnegie Mellon University 18349 Fall 2019 Lecture 21 for help with PID control

int fps = 60;

int width = 1200; 
int height = 600;

int viewWidth = width / 2;

float actualX = -0.5;  // actual landing location coordinates (m)
float actualY = 1.42;
float actualT = 0.8; // actual time of flight (s)

float computationTime = 0.3;

float mappedActualT = actualT * fps;   // in frames

float vy = 2.5;    // horizontal velocity in forward direction (m s^-1)       vx of AHRS
float vx = -0.8;   // horizontal velocity in left/right direction (m s^-1)    vy of AHRS
float vz = 1.3;    // vertical velocity (m s^-1)                              vz of AHRS

// TODO: how to use this info?
// float thetaDeg = 45;                   // vertical angle (deg)
// float theta = (PI / 180) * thetaDeg;   // vertical angle (rad)

float phiDeg = 0;                    // horizontal angle (deg)
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

float landingLocDiam;

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
float robotMappedV;
float robotMoveAngle; 
float robotMappedVx;
float robotMappedVy;
float robotTravelTime;
float robotMappedVxMag;
float robotMappedVyMag;
boolean isLocReached;
boolean dirChanged;
int robotSteps;
int robotReturnSteps;

boolean move; // robot moves after computation time ellapsed

float robotSideY;  

float sideStartZ;
float sideBallZ;
float sideBallY;

float actualVz;
float tFromRelease;

String BirdPOV = "Bird's Eye View";

// PID
float maxVoltage = 8;       // V
float maxRobotSpeed = 2;    // m s^-1
float motorError = 0.1;  
float maxDutyCycle = 1;
float minDutyCycle = 0.6;
float vTarget = 2;          // m s^-1
float Kp = 0.17;
float Kd = 0.17;
float Ki = 0.17;

float vActual;
float prevError;
FloatList listErrors;
float vActualError;
float vActualErrorSign;
float error;
float dutyCycle;
float voltage;

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
  mappedActualY = height * (1 - (actualY / 2));         // before: /6 //<>//
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

void initPIDdata() {
  vActual = 0;
  prevError = 0;
  listErrors = new FloatList();
  vActualError = random(motorError);
  if (random(1) < 0.5) {
    vActualErrorSign = -1;
  }
  else {
    vActualErrorSign = 1;
  }
}

void initRobotData() {
  robotReachX = viewWidth / 2; 
  robotReachY =  height / 2;
  robotReachDiameter = viewWidth; // before:(2 / 3.0) * width
  robotStartX = viewWidth / 2;
  robotStartY = height / 2;
  robotX = robotStartX;
  robotY = robotStartY;
  robotDiameter = (1/8.0) * viewWidth; // before:1 / 15.0
  isLocReached = false;
  dirChanged = false;
  robotSteps = 0;
  robotReturnSteps = 0;
  move = false;
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

float sumElems(FloatList lst) {
  float sum = 0;
  for (int i = 0; i < lst.size(); i++) {
    sum += lst.get(i);
  }
  return sum;
}

void PIDSim () {
  error = vTarget - vActual;
  dutyCycle = Kp * abs(error) + Kd * abs(error - prevError) + Ki * (sumElems(listErrors));
  voltage = max(min(dutyCycle, maxDutyCycle), minDutyCycle) * maxVoltage;
  vActual = ((voltage * maxRobotSpeed) / maxVoltage) * (1 + vActualErrorSign * vActualError);
  listErrors.append(error);
  prevError = error;
}

void getRobotSpeed() {
  PIDSim();
  robotMappedV = ((viewWidth * vActual) / 2) / fps; // before: /6
  robotMoveAngle = atan((abs(robotY - mappedY)) / (abs(robotX - mappedX)));
  robotMappedVxMag = robotMappedV * cos(robotMoveAngle);
  robotMappedVyMag = robotMappedV * sin(robotMoveAngle);
  float[] robotVelocities = getRobotVelocities(robotMappedVxMag, robotMappedVyMag);
  robotMappedVx = robotVelocities[0];
  robotMappedVy = robotVelocities[1];
  if (dirChanged) {
    changeDirection();
  } 
}

void initRobotNavigation () {
  getRobotSpeed();
  robotTravelTime = (dist(robotX, robotY, mappedX, mappedY) / robotMappedV);
}

void initRobot() {
  initRobotData();
  initRobotNavigation ();
}

void calcVertInitVelocity() {
  actualVz = (-hThrow + ((g / 2) * (pow(actualT, 2)))) / actualT;
}

void initSideRobot() {
  robotSideY = 0.75 * width - 48;
}

void initSideView() {
  sideStartZ = 570 - (hThrow * (height / 2));
  sideBallZ = sideStartZ;
  sideBallY = viewWidth;
  calcVertInitVelocity();
  initSideRobot();
}

void init () {
  predictLandingLocation();
  initBall ();
  initUser();
  initPIDdata();
  initRobot();
  initSideView();
}

void setup() {
  img = loadImage("grass.png");
  target = loadImage("target.png");
  sky = loadImage("sky.jpg");
  aboveP = loadImage("above.png");
  basket = loadImage("basket.png");
  tennis = loadImage("tennis.png");
  manSide = loadImage("man.png");
  basketSide = loadImage("basketSide.png");
  
  size(1200, 600);
  init();  
}

void moveBall() {
  ballX += ballMappedSpeedX;
  ballY -= ballMappedSpeedY;
}

void moveSideBall() {
  sideBallY += ballMappedSpeedY;
  tFromRelease += (1.0 / fps);
  sideBallZ = sideStartZ - (height / 2.0) * (actualVz * tFromRelease - (g / 2) * pow(tFromRelease, 2));
}

void moveRobot() {
  getRobotSpeed();
  robotX += robotMappedVx;
  robotY += robotMappedVy;
}

void moveSideRobot() {
  robotSideY -= robotMappedVy;
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
  if (!isLanded) {
    moveBall();
    moveSideBall();
  }
  
  if (frameCount >= (computationTime * fps)) {
    move = true;
  }
  
  if (move && (!isLocReached)) {
    robotSteps += 1;
    moveRobot();   // speed is updated in moveRobot(), the change also affects side view robot
    moveSideRobot();
  }
   
  if ((!isLanded) && (frameCount >= mappedActualT)) {
    isLanded = true;
    isBallCaught();
  }
   
  // TODO: no need to add move??
  if ((!isLocReached) && (dist(robotX, robotY, mappedX, mappedY) < ((landingLocDiam / 16)  + (robotDiameter / 16)))) {
    isLocReached = true;
  }
  
  // TODO: no need to add move??
  if (isLanded && isLocReached) {
    // robot goes back to start location
    if (!dirChanged) {
      dirChanged = true;
      initPIDdata();      // reset PID, error on the way back is probably different  
      changeDirection();  // TODO: is this line necessary???
    }
    
    if (robotReturnSteps < robotSteps) {
      robotReturnSteps+=1;
      moveRobot();
      moveSideRobot();
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
    image(tennis, ballX - 9, ballY - 9); 
  }
  else {
    image(tennis, robotX - 9, robotY - 9);
  }
} 

// calculations for on screen printing

void drawSideView() {
  float altitude = (actualVz * tFromRelease - (g / 2) * pow(tFromRelease, 2)) + hThrow;
  //float ballSpeedX = (ballMappedSpeedX  *  2) / viewWidth;
  //float ballSpeedY = (ballMappedSpeedY  *  2) / height;
  float verticalDisplacement = altitude - hThrow;
  String alt = str(round(altitude*10)*.1);
  String heightD = str(round(verticalDisplacement*100)*.01);
  image(sky, viewWidth, 0); 
  sky.resize(viewWidth, height);
  image(img, viewWidth, 570);
  img.resize((ceil((4 / 3.0) * viewWidth)), ceil((4/3.0) * height));
  image(manSide, 320, 90);
  //details
  //String Range = str(ballX);
  fill(200,200,200);
  textSize(21);
  //text("Distance: " + Range + " m", 630, 100);
  fill(255,255,255);
  text("Side View", 620, 30);
  textSize(17);
  text("Flight Time: " + str(round(tFromRelease*10)*.1) + "s", 1050, 30);
  textSize(16);
  text("Altitude: " + alt + "m", 630, 50);
  text("Ball Y Displacement: " + heightD + "m", 630, 70);
  fill(50,50,50);
  text("H: 1.0m", 601, 300);
  fill(255,255,255);
  text("0m", 600, height -10);
  text("1m", 890, height -10);
  text("2m", width - 23, height-10);
   
  line(600,0, 600, 600);
  
  if (!ballCaught) {
    image(tennis, sideBallY, sideBallZ);
  }
  
  //robot and wheels
  image(basketSide, robotSideY, 520);
  fill(70,70,70);
  rect(robotSideY + 10, 558, 74, 5);
  circle(robotSideY + 28, height-35, 15);
  circle(robotSideY + 68, height-35, 15);
 
}

void drawScreen () {
  float altitude = (actualVz * tFromRelease - (g / 2) * pow(tFromRelease, 2)) + hThrow;
  float ballSpeedX = (ballMappedSpeedX  *  2) / viewWidth;
  float ballSpeedY = (ballMappedSpeedY  *  2) / height;
  float verticalDisplacement = altitude - hThrow;
  float ballSpeedZ = sqrt(sq(actualVz) - 2 * g * verticalDisplacement); // derived from equation of motion
  float ballSpeed = sqrt(sq(ballSpeedX) + sq(ballSpeedY)  + sq(ballSpeedZ));
  String bSZ = str(round(actualVz*1000)*.001);
  if (tFromRelease == t) {
    bSZ = "0";
  }
  drawSideView();
  //bird's view
  image(img, -200, -200);
  image(target, 1, -1); 
  image(aboveP, 150, height-50);
  textSize(15);
  fill(240,240,240);
  //forward markers
  text("0 m", 287, height - 24);
  text(".5 m", 287, height - 163);
  text("1 m", 287, 306);
  text("1.5 m", 282, 178);
  text("2 m", 287, 36);
  //side markers
  fill(255,255,255);
  text("+.5 m", 415, 306);
  text("-.5 m", 160, 306);
  text("+1 m", 550, 306);
  text("-1 m", 20, 306);
  textSize(17);
  text("Flight Time: " + str(round(tFromRelease*10)*.1) + "s", 450, 30);
  textSize(16);
  text("Ball Vtotal: " + str(round(ballSpeed*100)*.01) + "m/s", 25, 50);
  text("Ball Vi x: " + str(round(ballSpeedX*10000)*.0001) + "m/s", 25, 550);
  text("Ball Vi y: " +str(round(ballSpeedY*10000)*.0001) + "m/s", 25, 570);
  text("Ball Vi z: " + bSZ + "m/s", 25, 590);
  drawLandings();
  
  textSize(21);
  fill(250, 250, 250);
  text(BirdPOV, 20, 30);

  image(basket, robotX-38, robotY-38);  // robot
  
  drawball (); 
}

void draw(){ 
  // default frame rate: 60 frames per second
  timerFired();
  drawScreen();
}
