package org.usfirst.frc.team1089.robot.subsystems;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.ExampleCommand;
import org.usfirst.frc.team1089.robot.commands.RunShooter;
import org.usfirst.frc.team1089.robot.commands.ShootWithDistance;
import org.usfirst.frc.team1089.robot.commands.TestShooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Subsystem} handles the shooting mechanism,
 * specifically controlling the speed of the motor of the shooter.
 */
public class Shooter extends Subsystem {
	
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
		//setDefaultCommand(new RunShooter(this));
		setDefaultCommand(new ShootWithDistance(this, Robot.rightFeeder));
		//setDefaultCommand(new TestShooter(this));
		//setDefaultCommand(new ExampleCommand());
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
	
	public void setToSpeed() {
    	motor.setPID(P, I, D);
    	motor.changeControlMode(CANTalon.TalonControlMode.Speed);
    	motor.configPeakOutputVoltage(12, -12);
		motor.configNominalOutputVoltage(0,0);
		motor.enableControl();
		MercLogger.logMessage(Level.INFO, "Shooter " + motor.getDeviceID() + " in Speed mode.");
	}
	
	public void setToVbus() {
		motor.disableControl();
    	motor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		MercLogger.logMessage(Level.INFO, "Shooter " + motor.getDeviceID() + " in Vbus mode.");
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
