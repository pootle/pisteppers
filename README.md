# pisteppers
simple code to drive stepper motors from a raspberry pi with good performance and minimal extra hardware.

This code use pigpio waves to drive gpio pins allowing fine cntrol of rampup speed on the steppers.

It is set up to drive steppers through a basic controller such as a Pololu A4988 chip or for unipolar motors via (for example ULN2003).

By using pigpio waves, jitter and timing glitches are minimal allowing the motors to be driven at pretty much their best possible speed, depending on the type of stepper and the load.

More background in blog post here: http://morepootling.blogspot.co.uk/2017/04/final-updates-on-driving-stepper-motors.html

There is a simple web server / web front end that can be used to test and tune parameters.

See the wiki for installation and setup instructions

To do a very simple test:

Set the next action mode for a motor to fast goto

Set target position to an integer number. This counts in smallest microsteps defined in the json file, so use quite large numbers - 10000 should do

Click goto now - this finds motors with a next action mode set, and starts the relevent routine.

## Current issues

There are still a couple of issues in this new version:

The most important is:
The motor threads are using too much CPU. 1 motor will run OK, but a second motor may cause pigpio to barf as it hasn't had enough CPU time
to recycle resources.
