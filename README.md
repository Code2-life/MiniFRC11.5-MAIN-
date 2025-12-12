these are formulas used to calculate stuff

since the sensors on NoU3 provides only measures over time, like gyro gives degrees/s and accelerometer gives
s^2 you cant really just do like comparison. you gotta do accumulation TWICE. (2 integrals). light work.

to integrate twice, you can just take displacement += acceleration * time^2

you start at {0, 0} ofc

i initially thought you should be able to just get revolution of the motor and multiply it with 2pi radius of the wheel to get
the distance travelled. oh how naive i was. anyways thats most of the calculation

the acceleration vector should give the position but we aint doing all that. 

i gave up (too resource intensive)

just decided to use pre timed steps.


for rotations, though:

we track the rotation of the robot since time 0, yaw specifically. then, while it rotates, we can integrate with time: since i 
dont know if the rotation is in degrees/ ms or seconds, i chose it to be degrees per seconds

since millis() returns time in milliseconds, i need to convert it to seconds via / 100. i could, also bitshift it. but naaa
its fast enough.

rotation is calculated just like acceleration except its the rate of change over ONE time; angle += gyro_y (yaw) * time