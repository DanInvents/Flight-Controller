# Flight-Controller

Release date: 03.10.2020

The Main folder contains the required Arduino files for the Flight Controller software to work.
Remember to install the sensor libraries as well as the modified libraries for the code to work properly.

Software changes:

* Now the blue LED blinks to indicate the time setting both in the automatic and the timer mode. For example, if
	the LED blinks 4 times in the timer mode, the deploy time is 4 seconds. Please refer to the user manual for more information
	on the operation modes.

* The buzzer beeps repeatedly when the flight controller is armed. This is useful to know that the controller is ready for flight.
	As in the original version, the controller beeps repeatedly once the deployment time has passed, both in the automatic and the timer mode.
	
* An issue with the altitude offset has been corrected. Now the Flight Controller sets the baseline to zero and not to an unknown value.

* For more precise timing of the initial conditions, I have redefined the trigger criterion. Now the flight controller triggers both when
	the jumper has been removed and when the acceleration along the y axis reaches -0.2 g.
	
* I have removed the Kalman filter acting on the acceleration data. The data is not as smooth, but it reflects the real world values more accurately.
	This is important when measuring short peak accelerations, such in a rocket launch.
	
Dan Invents
