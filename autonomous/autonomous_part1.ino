#include <constant_s.h> //IMPORTANT
#include <src/Alfredo_NoU3.h> // cool thing to have

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

unsigned float r_angle = 0;

void setup() {
  PestoLink.begin("Minifrc11.5"); //damn i WANT to change the name
  Serial.begin(115200); // wow so fast

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_ENABLED);

  frontLeftMotor.setBrakeMode(true);
  frontRightMotor.setBrakeMode(true);
  backLeftMotor.setBrakeMode(true);
  backRightMotor.setBrakeMode(true);

  measured_angle = 32.8275; // Tune this by spinning 5 full times << check this
  angular_scale = (5.0 * 2.0 * PI) / measured_angle;
  NoU3.calibrateIMUs(); // Takes exactly 1 second << couldnt tell
}

struct vector2f {
    float x;
    float y;
};

struct vector3f {
    float x;
    float y;
    float z;
};

//alias for gyro and accelerometer initial position

float i_pitch =NoU3.gyroscope_x;
float i_roll = NoU3.gyroscope_y;
float i_yaw = NoU3.gyroscope_z; // one most useful here, hopefully its from 0-360. but we can just modulo with 360

float i_accX = NoU3.acceleration_x;
float i_accY = NoU3.acceleration_y;
float i_accZ = NoU3.acceleration_z;

float circumference = 4.85; //inches?? fix this!! 2 * 3.14 * radius and round to nearest 100ths place!!!


unsigned long started = millis();


// ask stone if russel can bring his guitar and play it tmrw at lunch


void oneDimentional(float power) {
    frontLeftMotor.set(power);
    frontRightMotor.set(-1 * power);
    backLeftMotor.set(-1 * power);
    backLeftMotor.set(power);
}

float rot_error = 2;

void rotPower(float power) {
    frontLeftMotor.set(power);
    frontRightMotor.set(power);
    backLeftMotor.set(power);
    backLeftMotor.set(power);
}

void rotate(float degrees, float power, bool absRotate = false) {
    unsigned long d0 = millis()
    float yaw0 = NoU3.gyroscope_z - yaw;
    int mult = abs(degrees) / degrees;
    if (absRotate) {
        if ((abs(degrees - yaw0)) > 180) { 
            mult = -1; 
        }
        while (NoU3.gyroscope_z != degrees) {

        }
    }

}

void moveNorth(int cm) {
    for 


}


void autoMode(){
    setup();

    

}