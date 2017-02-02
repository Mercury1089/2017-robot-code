package org.usfirst.frc.team1089.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * 
 * @author Jared Tulayan
 */
public class Shooter extends Subsystem {
	private CANTalon motor;
	
	public Shooter(){
		motor = new CANTalon(1); //placeholder till I replace for X4 talon
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
