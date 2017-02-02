package org.usfirst.frc.team1089.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This {@link Subsystem} handles the shooting mechanism, 
 * including controlling the speed of the shooting motors, and more.
 */
public class Shooter extends Subsystem implements PIDOutput{
	private CANTalon motor;
	
	public Shooter(){
		motor = new CANTalon(1); // TODO Replace X4 talon
	}
	
	public void initDefaultCommand() {
		//Default Command should be that the wheels are stopped
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		motor.set(output);
	}
}
