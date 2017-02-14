package org.usfirst.frc.team1089.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This {@link Subsystem} contains all the hardware for
 * giving the robot some senses
 */
public class Sensors extends Subsystem {
	public final AnalogInput RANGEFINDER;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public Sensors() {
		RANGEFINDER = new AnalogInput(1);
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
	}
}
