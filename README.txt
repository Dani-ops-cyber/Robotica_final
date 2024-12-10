1 step
	*please check the wifi direction
	
	wifi:	ROSNET
	pass:	ROSNET2024
2 step
	*Login to Jetson via SSH
	
	ssh jetson@boot.local
	pass: jetson
3 step
	*To activate the project go to:
	
	cd ponyo_ws
		
4 step

	*initialization for the subscriber program
	
	* Please apply "colcon"
	
	colcon build
	source install/setup.bash
	
	*It should be noted that these commands are inside the folder.
	*Next activate the program
	ros2 run moveMotor MotorTopic
	
5 step
	* in other window we activate the publishing program
	* Please apply "colcon"
	
	colcon build
	source install/setup.bash
	
	*It should be noted that these commands are inside the folder.
	*Next activate the program
	ros2 run moveMotor Qcamera
	
	
#Recommendations: It is suggested to position the vehicle on a surface with some roughness since the tracks of the vehicle tend to slip.
	
	
	
	
