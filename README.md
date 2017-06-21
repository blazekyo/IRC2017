# IRC2017
-created by Delon - 17/6/17

(Delon) Will try seperate all the classes into different library files if have time.

Current Autobot code consist of 3 main classes >> Arm, Leg and Sequencer.
Arm class will handle the servo arm and claw,
Leg will handle the two motor driver and the motors,
Sequencer is important in controlling a sequence of which (Arm/Leg) is (used/called) first. In other words, which action to perform.

I'd tried to seperate encoder and pid controller into new classes and fcked up smewhere so I kept them outside.

IMPORTANT NOTES!!!
-Create own repository to edit/change/save the code.
-Don't pull the code and merge with the one in the Master Repository UNLESS it is tested with NO BUGS(currently), and it's TESTED on the real bot and it WORKS. 
-Have fun playing with the code.

## Version V1.1

#### Feature:
1.	Make the code into different modules.

#### Notes:
1.	There will be three new files needed for the 
	IRC_AutobotV1.1.ino to work correctly which are:
	(A).	Autobot_arm.h
	(B).	Autobot_settings.h
	(C).	Autobot_wheel.h
