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
NoU_Servo stageI(1);
NoU_Servo stageII(2);
NoU_Servo armServo(3);

// Global State
float intakeT = 0;
float angleI = 45.0; 
float angleII = 130.0;
float armAngle = 90;

enum State { MANUAL, AUTO };
State mode;

float angular_scale;
float measured_angle;
float Time_Elapsed;

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
  NoU3.calibrateIMUs(); // Takes exactly 1 second
  mode = MANUAL;
}


void loop() {
  if (mode==MANUAL) {
    manualcode();
  } else {
    autocode();
  }

  }




void manualcode() {
  if (PestoLink.buttonHeld(D_UP)){
    PestoLink.printTerminal("Auto Mode");
    mode = AUTO;
  } else {
    chassis();
    arm();
    armServo.write(armAngle);
  }
}


void chassis() {
  if (PestoLink.update()) {
    float y = -PestoLink.getAxis(1);
    float x = PestoLink.getAxis(0);
    float rx = PestoLink.getAxis(2); //yaw


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
  while (PestoLink.buttonHeld(RIGHT_TRIGGER)) {
      delay(150);
      armAngle += 15;
  }

  while (PestoLink.buttonHeld(LEFT_TRIGGER)) {
      delay(150);
      armAngle -= 15;

  }
  
}


// void drive_cuh() {
//     float y = 10.0;
//     float x = 0.0;
//     float rx = 0.0;

//     float botHeading = NoU3.yaw * angular_scale;

//     float rotX = x * cos(-botHeading) - y * sin(-botHeading);
//     float rotY = x * sin(-botHeading) + y * cos(-botHeading);
//     rotX = rotX * 1.1;

//     float calc = fabs(rotY) + fabs(rotX) + fabs(rx);
//     float denominator = (calc >= 1) ? calc : 1.0;

//     float frontLeftPower  = (rotY + rotX + rx) / denominator;
//     float backLeftPower   = (rotY - rotX + rx) / denominator;
//     float frontRightPower = (rotY - rotX - rx) / denominator;
//     float backRightPower  = (rotY + rotX - rx) / denominator;

//     frontLeftMotor.set(frontLeftPower);
//     backLeftMotor.set(backLeftPower);
//     frontRightMotor.set(frontRightPower);
//     backRightMotor.set(backRightPower);
//     NoU3.setServiceLight(LIGHT_ENABLED);

//     // Battery voltage telemetry
//     float batteryVoltage = NoU3.getBatteryVoltage();
//     PestoLink.printBatteryVoltage(batteryVoltage);
//   }


