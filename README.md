# pisteppers
simple code to drive stepper motors from a raspberry pi with good performance and minimal extra hardware.

This code use pigpio waves to drive gpio pins allowing fine cntrol of rampup speed on the steppers.

It is set up to drive steppers through a basic controller such as a Pololu A4988 as described in blog post linked below.

By using pigpio waves, jitter and timing glitches are minimal allowing the motors to be driven at pretty much their best possible speed, depending on the type of stepper and the load.

More background in blog post here: http://morepootling.blogspot.co.uk/2017/04/final-updates-on-driving-stepper-motors.html

This version is a radical update on the earlier version and includes a simple web server / web front end that can be used to test and tune parameters.

## installation

Clone the utilities repository and this repository from github:
> git clone https://github.com/pootle/pootles_utils.git

> git clone https://github.com/pootle/pisteppers.git

Then switch to pootles_utils folder and run setup to install utilities

> cd pootles_utils

> sudo python3 setup.py install

## setup

The hardware configuration is defined in the file motorset.json.
Different configuration files can be used, either by changing the entry in the config file, or using the -s parameter whern running the app 

This json file defines the various pins for each function. Any pin that is not relevant set 'pinno' to -1 and the pin is ignored.

pigpio daemon needs to be running to support this software. You can set pigpio daemon to run automatically at boot:

> sudo systemctl enable pigpiod

test run the app:

> ./app.py -c config_full.py

Then use a web browser on any local machine to access the web service on port 8000

To do a very simple test:

Set the next action mode for a motor to fast goto

Set target position to an integer number. This counts in smallest microsteps defined in the json file, so use quite large numbers - 10000 should do

Click goto now - this finds motors with a next action mode set, and starts the relevent routine.

## Current issues

There are still a couple of issues in this new version:

The most important is:
The motor threads are using too much CPU. 1 motor will run OK, but a second motor may cause pigpio to barf as it hasn't had enough CPU time
to recycle resources.