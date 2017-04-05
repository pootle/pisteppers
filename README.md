# pisteppers
simple demo code to drive stepper motors from a raspberry pi with good performance and minimal extra hardware

This code use pigpio waves to drive gpio pins allowing fine cntrol of rampup speed on the steppers.

It is set up to drive steppers through a basic controller such as a Pololu A4988 as descrined in blog post here....

By using pigpio waves jitter and timing glitches are minimal allowing the motors to be driven at pretty much their best possible speed, depending on the type of stepper and the load.

More background in blog post here: http://morepootling.blogspot.co.uk/2017/04/final-updates-on-driving-stepper-motors.html
