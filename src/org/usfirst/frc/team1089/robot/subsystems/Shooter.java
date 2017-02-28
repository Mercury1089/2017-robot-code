package org.usfirst.frc.team1089.robot.subsystems;

import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.RunShooter;
import org.usfirst.frc.team1089.robot.commands.ShootWithDistance;
import org.usfirst.frc.team1089.robot.commands.TestShooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Subsystem} handles the shooting mechanism,
 * specifically controlling the speed of the motor of the shooter.
 */
public class Shooter extends Subsystem implements PIDOutput{
	
	//Programming is like sex:
	//One mistake and you have to support it for the rest of your life.
	
	//Moral of the Story: Shoot your shots my guys
	
	public CANTalon motor;
	private double highest, lowest;

	public static final double P = 0.7;
	public static final double I = 0.0;
	public static final double D = 0.2;

	public enum ShooterEnum {
		NO_SHOOTER,
		LEFT_SHOOTER,
		RIGHT_SHOOTER,
		DUAL_SHOOTER,
		DUAL_STAGGERED_SHOOTER
	}
	
	public Shooter(int ID) {
		motor = new CANTalon(ID);
    	motor.enableBrakeMode(false);
		motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		motor.reverseSensor(false);

	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new RunShooter(this));
		//setDefaultCommand(new ShootWithDistance(this));
		//setDefaultCommand(new TestShooter(this));
	}
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		motor.set(output);
	}

	public CANTalon getMotor() {
		return motor;
	}
	//For testing PID of the shooter
	public double getHighest() {
    	return highest;		
	}
	//For testing PID of the shooter
	public double getLowest() {
		return lowest;
	}
	
	public void setToVbus() {
    	motor.setPID(P, I, D);
    	motor.changeControlMode(CANTalon.TalonControlMode.Speed);
    	motor.configPeakOutputVoltage(12, -12);
		motor.configNominalOutputVoltage(0,0);
		motor.enableControl();
	}
	
	public void setToSpeed() {
		motor.disableControl();
    	motor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	public void updateHighLow() {
    	double magVal = motor.getEncPosition();
		if (magVal > highest) {
    		highest = magVal;
    	}
		if (magVal < lowest) {
    		lowest = magVal;
    	}	
	}

    public void resetHighLow() {
		highest = 0;
		lowest = 20000;
	}
}
