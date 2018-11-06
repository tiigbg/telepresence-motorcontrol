# telepresence-motorcontrol

This is an experimental setup of a telepresence robot that is supposed to drive around a museum, remote controlled by a user on a different location using a website. This repository handles the controlling of the motors, servos and sensors. The main parts are a Teensy 3.6 board which receives the user input, sensor input, and controls servos to aim the webcam, and a MakeBlock Orion board which controls the four motors via two motor driver boards (with custom firmware).

Because this is an experimental prototype, this readme is bound to get outdated. Sorry about that :)

Documentation about Orion board
http://learn.makeblock.com/en/makeblock-orion/

Documentation Teensy 3.6 board
https://www.pjrc.com/teensy/pinout.html


The input for controlling the direction of driving has 3 parameters; driveAngle, driveSpeed and rotationSpeed. These are combined in an algorithm to compute howeach motor should spin. 

driveAngle to translate at: [0, 2pi] (float)
   PI*1.5 = right direction
   0      = forward direction
   PI*0.5 = left direction
   PI     = backwards direction
driveSpeed: [-1,1] (float)
   1 = maximum forward (in the chosen direction)
   -1 = maximum backward (in the chosen direction)
rotationSpeed: [-1,1] (float)
   -1 = turn maximum left
   1 = turn maximum right

## Other repositories used in this project
The web interface (both user and robot side) https://github.com/Dealerpriest/robot-webapp
Sending input from the web to a serialport https://github.com/Dealerpriest/websocket-to-serial
Custom firmware for the motor driver boards https://github.com/Dealerpriest/me-high-power-motorboard-firmware
Signalling server for webRTC https://github.com/Dealerpriest/distance-visit-signaling-server

## Contact
Feel free to post an issue if you have any questions.