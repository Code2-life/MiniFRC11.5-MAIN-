#include <constant_s.h> //IMPORTANT

//do the other includes yourself, constant_s.h is important to this file only.

// Motors                         
NoU_Motor frontLeftMotor(4);      
NoU_Motor frontRightMotor(1);     
NoU_Motor backLeftMotor(8);       
NoU_Motor backRightMotor(5);      
NoU_Motor intakeMotor(3);         

// Servos
NoU_Servo stageI(1);
NoU_Servo stageII(2);
NoU_Servo clawServo(3);

// Global State
float intakeT = 0;
float angleI = 45.0; 
float angleII = 130.0;
float clawAngle = grab;

unsigned long time_since_start = millis();

void setup() {
  PestoLink.begin("Minifrc11.5");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_ENABLED);

  frontLeftMotor.setBrakeMode(true);
  frontRightMotor.setBrakeMode(true);
  backLeftMotor.setBrakeMode(true);
  backRightMotor.setBrakeMode(true);

  measured_angle = 32.8275; // Tune this by spinning 5 full times
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  NoU3.calibrateIMUs(); // Takes exactly 1 second
}

void autoMode(){
    setup();

    

}