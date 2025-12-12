# <h1 style="color:red">README!!</h1>

## 1-51
<p>initial setups of variables and load some libraries</p>
<br>

## 52-56
<p>create a structure for easy position vector</p>
<br>

## 57-58
<p>other variables</p>
<br>

## 61-66: <p style="color:#dbdcaa">STOP()</p>
<p>sets all motor to zero (rolling stop)</p>
<br>

## 68-72
<p>other variables, for position</p>
<br>

## 74-97: 
### <span><p style="color:#dbdcaa">getPosition(</p><p style="color:#a6def6">float lasT</p><p style="color:#dbdcaa">)</p></span>
<p>float ax, ay are the instantanious acceleration values.</p>
<p>float t0 is current time at start of calling this function.</p>

86-88 is normalizing the rotation to be what it really is with the ground.

91-95 kinematics to accumilate (integrate) the position.

## 99-133: 
### oneDimensionalMove(float power, char direction)

switch case for direction to strafe the bot. needs calibration,
but should be right. right now:

B = -1\*F<br>
R = -1\*L

<p style="color:red">make sure that whenever calling this function, in direction to use single quotes('') and not double quotes("") as c++ reserves them for char and strings respectively</p>

final 4 lines are setting motor to power.

## 135-142: rotWheels(float power)

<p>135: set rotation error</p> <p style="color:red; font-weight: bold;">⚠️ mess around with this to get better rotation accuracy.</p>

set power to the motors.</p> <a style="color:red; font-weight: bold;"> make sure to check values and we are getting rotation we want</a>

## 146-166
### void rotate(float degrees, float power, bool absRotate = false)

perhaps the most complex part of the code.

degrees is in degrees. it has 2 possibilities: either absolute (true), where the robot makes it's yaw equal to the provided degree, or relative position (false), where it rotates degrees provided. does not align itself.

float yaw0 is the initial position when starting rotation, for rotations. im sure the absolue rotation and relative if/else could be merged to one but im too lazy to be doing all that

mult is either -1 or 1, i wish it was a 2 bit variable, but noooo

<p>theta diff:</p> 
<p>degrees - yaw0 : gets offset needed.</p>
<p>+ 540.f : think of this: 

min rotation we can get is -360. add 360 to make minimum of 0.

fmod(360.f)

modulo for getting any values over 360 and kind of "normalizing" the rotation to be a difference between 0-360

-180 is finally to get this from -180 to 180, in the least theta rotates. 

theta >= 0 is right wise rotation, therefore mult is 1

else, theta is <0 therefore its a left wise rotation, mult is -1 (opposite rotation)

loop for getting to gotten difference of angles, and recomputing the values

else is just rotating n degrees, with almost the same logic above. except this time, you need to also add your starting yaw to get proper offset.


## 177-236 
autoMode()
pretty self explanitory, but the route it takes is as in the image below. measuerments given. 100% power so it may be too fast.

to test the functionality, one make a simulated area of given dimentions with markers for positions given below. 

this is the text path:

start at {0, 0}, yaw 0
go to {46, 0}, translation {46, 0}      , dist: 46; 
go to {62, 10}, translation {16, 10}    , dist: 19; yaw: 32     , transform: 32
go to {62, -8}, translation {0, -18}    , dist: 18; yaw: 122    , transform: 90
go to {41, 5}, translation {-20, 13}    , dist: 25; yaw: 243    , transform: 121

since all transforms are < 180, i can use abs rotate

<p style="color:red; font-weight:bold; font-size: 24px"> PLEASE CHECK ALL AND DO PROPER CORRECTIONS BEFORE UPLOADING!!! THIS IS BOUND TO HAVE MEASUREMENT ISSUES!!