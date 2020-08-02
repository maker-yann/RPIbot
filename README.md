# RPIbot
Simple and cheap servo controlled robot with raspberry pi zero

This robot is a project to build a simple and cheap robot. It can be extended with functionalities.


The hardware is based on: 
- a raspberry pi zero board for the high level calculation
- a NodeMcu D1 mini for real time calculations like enoder and ultrasonic range detection (connected by the UART)
- two mini continuous rotation servo motors for the propulsion
- two mini servo for the camera and ultrasonic sensor pan and tilt mounting
- a PCA9685 16-Channel Servo Driver
- two pololu QTR-1RC reflectance sensors
- two home made encoder wheels with 16 black and white segments
- an USB power pack with 2 outputs. Supports charge during use.
- a free wheel
- Chassis wood based with 3 long screws for distance

![Alt HW diagram](/images/RPIbot_Hardware.jpg)
