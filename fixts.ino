#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <constants.h>

// Motors                         
NoU_Motor frontLeftMotor(8);      
NoU_Motor frontRightMotor(1);     
NoU_Motor backLeftMotor(4);       
NoU_Motor backRightMotor(5);      
NoU_Motor intakeMotor(3);         

// Servos
NoU_Servo claw(2);
NoU_Servo armServo(1);

float armAngle = 45;
float clawAngle = 0;

enum State { MANUAL, AUTO };
State mode;

float angular_scale;
float measured_angle;
float Time_Elapsed;
float z_yaw;
bool alt;

// Drivetrain

void setup() {
  PestoLink.begin("Minifrc11.5");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_ENABLED);

  frontLeftMotor.setBrakeMode(true);
  frontRightMotor.setBrakeMode(true);
  backLeftMotor.setBrakeMode(true);
  backRightMotor.setBrakeMode(true);

  measured_angle = -28.42726; // Tune this by spinning 5 full times
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  armServo.write(armAngle);
  claw.write(clawAngle);
  NoU3.calibrateIMUs(); // Takes exactly 1 second
  mode = MANUAL;
  z_yaw = (NoU3.yaw * angular_scale);

}
}


void loop() {
  if (mode==MANUAL) {
    manualcode();
  } else {
    autocode();
  }

  }




void manualcode() {
  if (PestoLink.buttonHeld(12)){
    PestoLink.printTerminal("Auto Mode");
    mode = AUTO;
  } else {
    chassis();
    arm();
    claw();
    claw.write(clawAngle);
    armServo.write(armAngle);
  }
}


void chassis() {
  if (PestoLink.update()) {
    float y = -PestoLink.getAxis(2);
    float x = PestoLink.getAxis(3);
    float rx = PestoLink.getAxis(1); //yaw


    float botHeading = NoU3.yaw * angular_scale;

    float rotX = -1 * (x * cos(-botHeading) - y * sin(-botHeading));
    float rotY = -1 * (x * sin(-botHeading) + y * cos(-botHeading));
    rotX = rotX * 1.1;

    float calc = fabs(rotY) + fabs(rotX) + fabs(rx);
    float denominator = (calc >= 1) ? calc : 1.0;

    float frontLeftPower  = (rotY + rotX + rx) / denominator;
    float backLeftPower   = (rotY - rotX + rx) / denominator;
    float frontRightPower = (rotY - rotX - rx) / denominator;
    float backRightPower  = (rotY + rotX - rx) / denominator;

    frontLeftMotor.set(frontLeftPower);
    backLeftMotor.set(backLeftPower);
    frontRightMotor.set(frontRightPower);
    backRightMotor.set(backRightPower);
    NoU3.setServiceLight(LIGHT_ENABLED);

    // Battery voltage telemetry
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if(PestoLink.buttonHeld(MID_LEFT)){
      NoU3.yaw=0; 
    }
  } else {
    frontLeftMotor.set(0);
    backLeftMotor.set(0);
    frontRightMotor.set(0);
    backRightMotor.set(0);
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}


void arm() {
  if (PestoLink.buttonHeld(7)) {
      if (armAngle + 15 > 90) {
          armAngle = 90;   // upper bound
      } else {
          armAngle = armAngle + 15;
      }
      
  }

  if (PestoLink.buttonHeld(6)) {
      if (armAngle - 15 <= 0) {     // lower bound
          armAngle = 0;
      } else {
          armAngle = armAngle - 15;
      }
  }



void claw() {
  if (PestoLink.buttonHeld(5)) {
    clawAngle = 10;
  }

  if (PestoLink.buttonHeld(4)) {
    clawAngle = 0;
  }
}


struct vector2f {
    float x;
    float y;
};

float circumference = 6.28; // 
unsigned long started = millis();

// ask stone if russel can bring his guitar and play it tmrw at lunch
void STOP() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    backLeftMotor.set(0);
    backRightMotor.set(0);
}

vector2f position = {0, 0};

float vx = 0;
float vy = 0;
float lastT = millis() * 0.001f;

void getPosition(float lastT){
    float ax = NoU3.acceleration_x;
    float ay = NoU3.acceleration_y;

    float t0 = millis() * 0.001f;
    //first x then y

    float t = t0-lastT;
    lastT = t0;

    //stupid conversion to radians and global acceleration because with rotation, the values MAY flip. nah, not may, WILL.
    //ex: rotating 90* will make previous acceleration x the new acceleration y. 90* rotation.
    float yaw_RAD = ((NoU3.yaw * angular_scale) - z_yaw) * 0.017f;
    float g_ax = ax * cos(yaw_RAD) - ay * sin(yaw_RAD); //global_acceleration(x)
    float g_ay = ax * sin(yaw_RAD) + ay * cos(yaw_RAD); // HAHHA NAMING SCHEME WAS NOT INTENDED, I SWEAR LMAOOO

    
    position.x += (vx * t) + (0.5f * g_ax * t*t);
    vx += g_ax*t;
    
    position.y += (vy * t) + (0.5f * g_ay * t*t);
    vy += g_ay*t;

}

void oneDimensionalMove(float power, char direction) { // PLEASE mess with these

    int mult_fwd = 1;
    int mult_back = -1;
    switch (direction) {
        case 'F' :
                mult_fwd = 1;
                mult_back = 1;
                break;
        case 'B' :
                mult_fwd = -1;
                mult_back = -1;
                break;
        
        case 'L' :
                mult_fwd = -1;
                mult_back = 1;
                break;
        
        case 'R' :
                mult_fwd = 1;
                mult_back = -1;
                break;
            
        default:
            mult_fwd = 0;
            mult_back = 0;
            break;
    }

    frontLeftMotor.set(power * mult_fwd);
    frontRightMotor.set(-1 * power * mult_fwd);
    backLeftMotor.set(-1 * power * mult_back);
    backRightMotor.set(power * mult_back);
}

float rot_error = 2; // change to get result wanted

void rotWheels(float power) {
    frontLeftMotor.set(power);
    frontRightMotor.set(power);
    backLeftMotor.set(-1 * power);
    backRightMotor.set(-1 * power);
}

// note to self :  .yaw exists

void rotate(float degrees, float power, bool absRotate = false) {

    float yaw0 = (NoU3.yaw * angular_scale) - z_yaw; //account for starting yaw, probably zero but cant be too safe (it SHOULD be zero)
    const float yaw_c = yaw0;
    int mult = 0;
    if (absRotate) {
        float thetaDiff = fmod(degrees - yaw0 + 540.f, 360.f) - 180.f; // funny trick to get rotation between (-180,180]
        mult = (thetaDiff >= 0) ? 1 : -1; // switch if wrong pls ( i mean +-1 )
        while (fabs(thetaDiff) > rot_error) {
            rotWheels(mult * power);
            yaw0 = (NoU3.yaw * angular_scale) - z_yaw;
            thetaDiff = fmod(degrees - yaw0 + 540.f, 360.f) - 180.f;
        }
    }
    else {
        float targetYaw = yaw_c + degrees;
        float thetaDiff = fmod(targetYaw + 540.f, 360.f) - 180.f;
        mult = (thetaDiff >= 0) ? 1 : -1;
        //mult = (degrees == 0) ? 0 : (fabs(degrees) / degrees);
        while (fabs(thetaDiff) > rot_error) {
            rotWheels(mult * power);
            yaw0 = (NoU3.yaw * angular_scale) - z_yaw;
            thetaDiff = fmod(degrees - yaw0 + 540.f, 360.f) - 180.f;

        }
    }
    rotWheels(0);
}


void autoMode(){
    z_yaw = (NoU3.yaw * angular_scale); // reiterate because we are waiting for keypress and robot may have been moved
    getPosition(millis());
    if (position.x != 0 || position.y != 0 || vx != 0 || vy != 0){
        position.x = 0;
        position.y = 0;
        vx = 0;
        vy = 0;
    }

    float dist = 0;
    vector2f oldFolks = {position.x, position.y};

    oneDimensionalMove(1, 'F');
    lastT = millis();
    while(dist < 45){
        getPosition(lastT);
        dist = sqrt(pow((position.x - oldFolks.x), 2) + pow((position.y - oldFolks.y), 2));
    }
    STOP();

    rotate(32, 1, true);
    oneDimensionalMove(1,'F');
    oldFolks.x = position.x;
    oldFolks.y = position.y;
    lastT = millis();
    while(dist < 18) {
        getPosition(lastT);
        dist = sqrt(pow((position.x - oldFolks.x), 2) + pow((position.y - oldFolks.y), 2));
    }
    STOP();

    rotate(122, 1, true);
    oneDimensionalMove(1,'F');
    oldFolks.x = position.x;
    oldFolks.y = position.y;
    lastT = millis();
    while(dist < 17) {
        getPosition(lastT);
        dist = sqrt(pow((position.x - oldFolks.x), 2) + pow((position.y - oldFolks.y), 2));
    }
    STOP();

    rotate(243, 1, true);
    oneDimensionalMove(1,'F');
    oldFolks.x = position.x;
    oldFolks.y = position.y;
    lastT = millis();
    while(dist < 24){
        getPosition(lastT);
        dist = sqrt(pow((position.x - oldFolks.x), 2) + pow((position.y - oldFolks.y), 2));
    }
    STOP();


    hasRun = true;
}




void autocode() {
   if (PestoLink.buttonHeld(D_DOWN)) {
    PestoLink.printTerminal("Manual mode");
     mode = MANUAL;
     return;
   }
  autoMode();
  mode = MANUAL;

  }
