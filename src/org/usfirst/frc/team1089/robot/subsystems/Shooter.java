package org.usfirst.frc.team1089.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This {@link Subsystem} handles the shooting mechanism, 
 * including controlling the speed of the shooting motors, and more.
 */
public class Shooter extends Subsystem {
	//private CANTalon motor;
	
	public Shooter(){
		//motor = new CANTalon(1); // TODO Replace X4 talon
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
