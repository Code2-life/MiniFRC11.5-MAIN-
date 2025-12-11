#include <PestoLink-Receive.h>
#include <Alfredo_NoU2.h>
#include <NewPing.h> // no clue what this is
 

// god this is terrible code

long AUTO_START_TIME = 0;
long SHOOTER_START_TIME = 0;


void handleAutoMode() {
  if (PestoLink.buttonHeld(MID_RIGHT)) {
    ROBOT_STATE = MANUAL;
    return;
  }
  TSA = (millis() - AUTO_START_TIME);
    if(TSA < 14000){
      if(TSA < 1000){ //aim and spin up shooter
        servo.write(33);//subwoofer angle
        shooterMotor.set(1);
      }
      if(TSA > 1000 && TSA < 1500){ //shoot
        indexerMotor.set(1);
      }
      if(TSA > 1500 && TSA < 2000){//reset motor
        resetMotors();
      }
      if(TSA > 2100 && TSA < 2200){//reset servo
        servo.write(0);
      }
      if(TSA > 2500 && TSA < 4000){//intake
        indexerMotor.set(1);
        shooterMotor.set(-1);
        if(TSA > 2600 && TSA < 3100/*change for distance*/){
          drivetrain.arcadeDrive(1, 0);
        } else{
          drivetrain.arcadeDrive(0, 0);
        }
      }
      if(TSA > 4000 && TSA < 4250){//reset motor
        resetMotors();
      }
      if(TSA > 4000 && TSA < 4500/*change for distance*/){//drive to subwoofer
        drivetrain.arcadeDrive(-1, 0);
      }
      if(TSA > 4500 && TSA < 5600){//stop
        drivetrain.arcadeDrive(0,0);
      }
      if(TSA > 5600 && TSA < 7000){ //aim and spin up shooter
        drivetrain.arcadeDrive(0, 0);
        servo.write(33);//subwoofer angle
        shooterMotor.set(1);
      }
      if(TSA > 7000 && TSA < 7500){ //shoot
        indexerMotor.set(1);
      }
      if(TSA > 7500 && TSA < 8000){//reset motor
        resetMotors();
      }
      if(TSA > 8000){//reset servo
        servo.write(0);
      }
    }
    else {
      drivetrain.arcadeDrive(0, 0);
      return;      
    }
}