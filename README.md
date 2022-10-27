# test PID controller
 
This repository contains an implementation of PID controllers. And some demos showing them at work. The creation of this project was inspired by this [youtube video](https://youtu.be/y3K6FUgrgXw).


https://user-images.githubusercontent.com/16965931/198413430-a115bcb1-4c88-469d-b25d-a143baff1fd8.mp4

The video above shows a 2D simulation of a drone, which is programmed to move to a target. The white dot is the drone, and the red dot is a target that changes position every few seconds. The drone has two axis of control. Each controlled by a PID controller. The drone can tilt right and left, and adjust how hard it's thrusting. 

One PID controller controlls the tilt angle of the drone, and attempts to get the drone horizontally aligned with the target. The other PID controller adjusts the thrust power of the drone, and attemtps to get the drone vertically level with the target. The thruster is limited to one direction, it can not thrust the drone downwards, the drone must rely on gravity to decrease it's altitude.

An angle of 0 indicates the drone is aimed straight upwards, and would thrust straight downwards. And angle of -45 would indicate that the drone's top is pointed to the top left, and it's thrusters push down right.

 
