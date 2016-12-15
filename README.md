# Robotics-Final-Project
Final project involving programming an AR Drone and TurtleBot to navigate and interact with vision. Built with ROS, written in Python.

Youtube URL for demo video: https://youtu.be/gbDNrzqeofI

### CptS 483 Final Project Proposal #################################################
Eric Chen

Group Members: Trevor Mozingo and I

Minimum success conditions and gist of the project: 
	- We want to have the TurtleBot going in a loop (or square) using vision and have a drone track and follow the TurtleBot above it. We should end up with two robots going in loops, the drone slightly behind the TurtleBot. We initially wanted to have the TurtleBot go to a marker and then land the drone on the TurtleBot but we decided to be more realistic after hearing how difficult the drones can be to work with. At the minimum we want the drone to at least be in the air and able to see the TurtleBot. We don't yet know if we can actually get the drone to follow the TurtleBot. 

How will we meet these conditions:
	Timeline:

11/20-11/30 Since we already have code for making the TurtleBot visually go in a square we will use this time to research what drones we have access to, what it is capable of doing, and how we can attain that tracking functionality we need. We will work with the TAs to accomplish this.

12/1 Lab 5 is on drones! We'll (hopefully) learn a lot about drone localization from this lab.

12/1-12/8 Work on the drone part of the final project

12/9-12/11 Record the demo for extra credit and in case the live demo doesn't work

12/11 Final presentations

What modules are needed:
	- For the turtleBot portion we already know what modules and libraries we will need from doing Lab 4.
	- We will need to research what modules the drone needs to see color, and then go toward it up to a certian point. Essentially what the TurtleBot did in that that vision guided square lab but the drone as well.  

### Write Up

Original Final Project Proposal:
“Minimum success conditions and gist of the project:  
 	We want to have the TurtleBot going in a loop (or square) using vision and have a drone track and follow the TurtleBot above it. We should end up with two robots going in loops, the drone slightly behind the TurtleBot. We initially wanted to have the TurtleBot go to a marker and then land the drone on the TurtleBot but we decided to be more realistic after hearing how difficult the drones can be to work with. At the minimum we want the drone to at least be in the air and able to see the TurtleBot. We don't yet know if we can actually get the drone to follow the TurtleBot.”

Steps Taken:
Planning
Gazebo virtual testing with both robots
Learning how to program/connect to the AR Drone
Calibration of physical drone’s vision/color filter
Finalizing markers/environment
Run with both physical robots/fine tuning

What we tried to do: 
	Allow the TurtleBot to navigate on its own using vision and simultaneously have an AR Drone track and follow it.

What worked, what didn’t, and why: 
	The drones were very unpredictable, making debugging take more time than anticipated. We originally intended for the drone to track and follow the TurtleBot more smoothly by also enabling its bottom camera. Thereby hovering over it once it had gotten close enough using its front-facing camera. Due to time limitations, we relied solely on the front camera instead and modified our algorithm accordingly. We then replaced the TurtleBot’s  hat  (a marker designed for the bottom camera), with a 360 degree marker that was propped up to the drone’s level of hovering. Despite how unpredictable the drones were, I’m proud we got it to work. Every we told our proposal to told us to reconsider due to it being to difficult and even the TAs wished us the best of luck. 

Why it’s cool:
	We think our project is “cool” because it shows off what awesome things can be done with only a semester of an introductory robotics course. We were able to have two drones moving based on vision, and interacting with each other to a certain degree. None of the motion and kinematics were hard coded, which we were very proud of. This meant that under ideal conditions and once set up, the robots could move all on their own for a while without human intervention or assistance. In fact, our TurtleBot Stewie was running laps around the work area for most of the night, sometimes getting in the way of passerby’s and cutting corners every now and then.
