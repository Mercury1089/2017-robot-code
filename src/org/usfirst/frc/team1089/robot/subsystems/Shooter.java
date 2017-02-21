package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.RunShooter;
import org.usfirst.frc.team1089.robot.commands.ShootWithDistance;
import org.usfirst.frc.team1089.robot.commands.TestShooter;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Subsystem} handles the shooting mechanism, 
 * including controlling the speed of the shooting motors, and more.
 */
public class Shooter extends Subsystem implements PIDOutput{
	public CANTalon motor;
	private boolean isVbus;
	public enum ShooterEnum {
		NO_SHOOTER,
		LEFT_SHOOTER,
		RIGHT_SHOOTER,
		DUAL_SHOOTER,
		DUAL_STAGGERED_SHOOTER
		
	}
	public Shooter(int ID) {
		motor = new CANTalon(ID);	
	}
	
	public void initDefaultCommand() {
		//setDefaultCommand(new RunShooter(this));
		setDefaultCommand(new ShootWithDistance(this));
		//setDefaultCommand(new TestShooter(this));
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		motor.set(output);
	}
}
