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
 * specifically controlling the speed of the motor of the shooter.
 */
public class Shooter extends Subsystem implements PIDOutput{
	public CANTalon motor;
	private double magVal;
	private double highest, lowest;
	public enum ShooterEnum {
		NO_SHOOTER,
		LEFT_SHOOTER,
		RIGHT_SHOOTER,
		DUAL_SHOOTER,
		DUAL_STAGGERED_SHOOTER
		
	}
	
	public Shooter(int ID) {
		motor = new CANTalon(ID);
		resetHighLow();
	}
	
	public void initDefaultCommand() {
		//setDefaultCommand(new RunShooter(this));
		setDefaultCommand(new ShootWithDistance(this));
		//setDefaultCommand(new TestShooter(this));
	}
	
	//For testing PID of the shooter
	public double getHighest(){
		magVal = SmartDashboard.getNumber("Shooter ID " + motor.getDeviceID() + ": Encoder Value", 2900) * -1;
		if (magVal > highest) {
    		highest = magVal;
    	}
    	return highest;
		
	}
	//For testing PID of the shooter
	public double getLowest(){
		magVal = SmartDashboard.getNumber("Shooter ID " + motor.getDeviceID() + ": Encoder Value", 2900) * -1;
		if (magVal < lowest) {
    		lowest = magVal;
    	}
		return lowest;
	}
	
    public void resetHighLow() {
		highest = 0;
		lowest = 20000;
	}
	
	

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		motor.set(output);
	}
}
