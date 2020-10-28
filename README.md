# RPIbot
Simple and cheap servo controlled robot with raspberry pi zero

This robot is a project to build a simple and cheap robot. It can be extended with functionalities.

<img src="/images/IMG_20201025_105224.jpg?raw=true" width="480">

The hardware is based on: 
- a raspberry pi zero board for the high level calculation
- an optional NodeMcu D1 mini for real time calculations like enoder and ultrasonic range detection (connected by the UART)
- two DC gear motors and optical encoder (replacement for previous mini continuous rotation servo motors for the propulsion)
- two mini servo for the camera and ultrasonic sensor pan and tilt mounting
- a PCA9685 16-Channel Servo Driver
- an USB power pack with 2 outputs. Supports charge during use.
- a free wheel
- Chassis wood based with 3 long screws for distance

![HW diagram](/images/RPIbot_Hardware_V2.png?raw=true)
