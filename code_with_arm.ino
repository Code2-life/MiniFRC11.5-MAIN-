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

float armAngle = -90;
float clawAngle = 0;
bool alt = false;


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

  measured_angle = 32.528; // Tune this by spinning 5 full times
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  armServo.write(armAngle);
  claw.write(clawAngle);
  NoU3.calibrateIMUs(); // Takes exactly 1 second
  mode = MANUAL;
}


void loop() {
  char buffer[20];
  char clawd[20];



  dtostrf(armAngle, 0, 2, buffer);
  dtostrf(clawAngle, 0, 2, clawd);
  PestoLink.printfTerminal("buffer ", buffer);
  PestoLink.printfTerminal(clawd);
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
    clawfunc();
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

    float rotX = (x * cos(-botHeading) - y * sin(-botHeading));
    float rotY = (x * sin(-botHeading) + y * cos(-botHeading));
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
      if (armAngle + 15 > 180) {
          armAngle = 180;   // upper bound
      } else {
          armAngle = armAngle + 15;
      }
      
  }

  if (PestoLink.buttonHeld(6)) {
      if (armAngle - 15 < 0) {     // lower bound
          armAngle = 100;
      } else {
          armAngle = armAngle - 15;
      }
  }
}


void clawfunc() {
  if (PestoLink.buttonHeld(RIGHT_BUMPER)) {
    clawAngle = 15;
  }

  if (PestoLink.buttonHeld(LEFT_BUMPER)) {
    clawAngle = 0;
  }
}

void drive_cuh() {
    float y = 0.0;
    float x = 0.0;
    float rx = 1.0;

    float botHeading = NoU3.yaw * angular_scale;

    float rotX = x * cos(-botHeading) - y * sin(-botHeading);
    float rotY = x * sin(-botHeading) + y * cos(-botHeading);
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
  }

void autocode() {
  // Run drive_cuh() for 5 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 1500) {
    drive_cuh();
  }

  // Stop motors after 5 seconds
  frontLeftMotor.set(0);
  backLeftMotor.set(0);
  frontRightMotor.set(0);
  backRightMotor.set(0);

  // Return to manual mode after auto
  mode = MANUAL;
}

