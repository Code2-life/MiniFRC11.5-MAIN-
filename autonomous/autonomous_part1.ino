//angel

#include <constant_s.h> //IMPORTANT
#include <src/Alfredo_NoU3.h> // cool thing to have
#include <cmath> // wonderful maths

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
float clawAngle = 0;

float angular_scale;
float measured_angle;

// const float ang_scale = 0.957; // not sure why you need this if you're using imu.calibrate
const float ang_scale = 1.f; //try with either keep whichever is more accurate

float z_yaw = 0;

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

    z_yaw = (NoU3.yaw * ang_scale);

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
    float yaw_RAD = ((NoU3.yaw * ang_scale) - z_yaw) * 0.017f;
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

    float yaw0 = (NoU3.yaw * ang_scale) - z_yaw; //account for starting yaw, probably zero but cant be too safe (it SHOULD be zero)
    const float yaw_c = yaw0;
    int mult = 0;
    if (absRotate) {
        float thetaDiff = fmod(degrees - yaw0 + 540.f, 360.f) - 180.f; // funny trick to get rotation between (-180,180]
        mult = (thetaDiff >= 0) ? 1 : -1; // switch if wrong pls ( i mean +-1 )
        while (fabs(thetaDiff) > rot_error) {
            rotWheels(mult * power);
            yaw0 = (NoU3.yaw * ang_scale) - z_yaw;
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
            yaw0 = (NoU3.yaw * ang_scale) - z_yaw;
            thetaDiff = fmod(degrees - yaw0 + 540.f, 360.f) - 180.f;

        }
    }
    rotWheels(0);
}

bool hasRun = false;

void autoMode(){
    if (hasRun) return;
    z_yaw = (NoU3.yaw * ang_scale); // reiterate because we are waiting for keypress and robot may have been moved
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

void loop(){
    autoMode();
}