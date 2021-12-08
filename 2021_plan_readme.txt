NOTES ON SIMULATION PROGRAMMING WITH THE STUDENTS

1) Introduce everyone to the Python robot code (Plan: 1/14/2021)
2) Get everyone set up with an environment for programming (Plan: 1/15/2021)
3) Do a python progaming review for everyone after we see the code (1/19/2021)
4) Need to get everyone able to retrieve the code (github tutorial (done) )
5) Visualization tutorial - how to plot the location and orientation of the robot (done)
6) Start programming the autonomous routes (done)
7) https://docs.wpilib.org/en/stable/ is where we  go 


FRC ToDo:

Pathweaver and Trajectory libraries (done)
Localization = external measurements (encoder + gyro) to get agent pose and orientation
Can also use FRC vision for perspective (something)

show off the controller (done)
try to get a better tuning (done)
try to get a better way of looking at the data (done)


*** To do with the students:  ***

*** LOOKS LIKE PATHWEAVER IS ABSOLUTE POSITION - need to find a way to set the current pose of the robot to the beginning of the trajectory for it to work right (Getting there - just set the pose when it starts)

To Do Today:

1) Get everyone to try the rev libraries - as of last week they are supposed to be mac-compatible, and I would like to switch to emulating CAN devices 

2) show off the drivetrain characterization tool and how it is different from before

3) show off the new pathweaver recipes

4) field any questions


Shooter FRC ToDo:

Motors: 
1 Neo (free running to a belt) - flywheels
1 775 (versaplanetary on sprocket and chain) - angle on the outer hood
1 775 feed motor - delivers to the flywheel

Subsystem - 'shooter'
	Controllers:
	Sparkmax on the neo  "sparkmax_flywheel"
	Spark on the 775 (SPARK PWM) that runs the hood elevation "spark_hood"
	Spark on the feed motor - plain old SPARK PWM  "spark_feed"
	
	For instance:  self.spark_hood = Spark(6)

	Digital IO: Limit switches  (stretch goal)
	"limit_hi" and "limit_low"
	

	Encoders:
	Velocity, built-in to the sparkmax "sparkneo_flywheel_encoder"
	Position, as an encoder on the 775.  What did we do before? ""

	Functions:
	set_flywheel(velocity)
	stop_flywheel()
	set_feed_motor(speed)  * 
	stop_feed_motor() 
	set_elevation(angle)  - need to be tied into an encoder
	calibrate() - periodically poll to see if limit_hi or limit_low and set angle accordingly


Commands:
Button to fire a ball (feed motor) - "shooter_shoot"
hood angle up and down (joystick) - "shooter_aim"
toggle to turn flywheel on and off "shooter_enable"

OI:
bind a few buttons to these commands

Steps:  look at https://robotpy.readthedocs.io/en/stable/
1) Make a new subsystem
	- Define motor controllers
	- add functions to the motor controllers
2) Make a new command for each of the commands in the list above
3) Bind the commands to joystick buttons