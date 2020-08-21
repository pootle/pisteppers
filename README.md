# pisteppers
Code to drive stepper motors from a raspberry pi with good performance and minimal extra hardware.

This code optioanlly uses pigpio waves to drive gpio pins allowinf precise timings.

It is set up to drive steppers through a basic controller such as a Pololu A4988 chip or for unipolar motors via (for example ULN2003).

By using pigpio waves, jitter and timing glitches are minimal allowing the motors to be driven at pretty much their best possible speed, depending on the type of stepper and the load.

There is a simple web server / web front end that can be used to test and tune parameters.

See the wiki for installation and setup instructions

To do a very simple test:

Set the next action mode for a motor to fast goto

Set target position to an integer number. This counts in smallest microsteps defined in the json file, so use quite large numbers - 10000 should do

Click goto now - this finds motors with a next action mode set, and starts the relevent routine.
