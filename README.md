# Udacity-GoCHaseIt-Repo
Second udacity  Robotics Software Engineer Nanodegree Program project

#### This folder contains the basic structure files generated is the Udacity Ros essential modules

Developed by **Rodrigo Montebello Saboya Brito** and the help of the internet including *Ros wiki and Udacity git hub repos*

This project objective was to model the robot's world and the robot.

Also write the code so the robot can chase a white ball placed in front of his camera, when the ball leaves the camera sight the robot stops.

The drive_bot publish directions to the robot's motor whenever it recieves a request, informing the linear and angular position, publishes geometry messages.

The process_image "looks" for white pixels on the screen and publish the cordinates if there's a ball, calling the drive_bot function and pass velocities to it, it stops the bot when the white ball leaves the screen.

### Reference sources:
#### Ros wiki: (http://wiki.ros.org/ROS/Tutorials)
> Udacity peer repos: 
>> (https://github.com/kevingasik/GoChaseIt)
>> (https://github.com/furrypython/RoboND-GoChaseIt-Project)
>> (https://github.com/drachins/UdemyGoChaseIt)
>> (https://github.com/llu0120/Udacity-Go-Chase-It)
