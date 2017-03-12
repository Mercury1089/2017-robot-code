package org.usfirst.frc.team1089.robot.subsystems;

import java.util.concurrent.CountDownLatch;
import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.RobotMap;
import org.usfirst.frc.team1089.robot.commands.ExampleCommand;
import org.usfirst.frc.team1089.robot.commands.RunShooter;
import org.usfirst.frc.team1089.robot.commands.ShootWithDistance;
import org.usfirst.frc.team1089.robot.commands.StopShooter;
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
	
	public CANTalon shooterMotor;
	private CANTalon feederMotor;
	private double highest, lowest;
	private double setSpeed;

	public static final double F = 0.2;
	public static final double P = 0.45; // 0.7
	public static final double I = 0.0;
	public static final double D = 0.2; // 0.2

	private static final double QUAD_ENC_TICKS_PER_ROTATION = 200;	//Includes x4 for QUAD
	private static final double HUNDRED_MS_PER_MINUTE = 600;
	
	public enum ShooterEnum {
		NO_SHOOTER,
		LEFT_SHOOTER,
		RIGHT_SHOOTER,
		DUAL_SHOOTER,
		DUAL_STAGGERED_SHOOTER
	}
	
	public Shooter(int shooterID, int feederID) {
		shooterMotor = new CANTalon(shooterID);
    	shooterMotor.enableBrakeMode(false);
    	
    	// setting feedback as quad encoder does NOT enable unit scaling by default
    	// when set to speed mode the speed will be expressed in pulses per 100 ms
		shooterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooterMotor.reverseSensor(false);
		setToSpeed();
		setSpeed = 0.0;
		
		feederMotor = new CANTalon(feederID);
		feederMotor.enableBrakeMode(true);
	}
	
	public void initDefaultCommand() {
		//setDefaultCommand(new RunShooter(this));
		//setDefaultCommand(new ShootWithDistance(this));
		//setDefaultCommand(new TestShooter(this));
		setDefaultCommand(new StopShooter(this));
	}

	public CANTalon getMotor() {
		return shooterMotor;
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
		shooterMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
    	shooterMotor.setP(P);
    	shooterMotor.setI(I);
    	shooterMotor.setD(D);
    	shooterMotor.setF(F);
    	shooterMotor.configPeakOutputVoltage(12, -12);
		shooterMotor.configNominalOutputVoltage(0,0);
		shooterMotor.enableControl();
		MercLogger.logMessage(Level.INFO, "Shooter " + shooterMotor.getDeviceID() + " in Speed mode.");
	}
	
	public void setToVbus() {
		//shooterMotor.disableControl();
    	shooterMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		MercLogger.logMessage(Level.INFO, "Shooter " + shooterMotor.getDeviceID() + " in Vbus mode.");
	}

	public void updateHighLow() {
    	double magVal = shooterMotor.getEncPosition();
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
    
    public void setSpeed(double rpm) {
    	setSpeed = rpm;
    	double pulsesPer100ms = rpm * QUAD_ENC_TICKS_PER_ROTATION / HUNDRED_MS_PER_MINUTE;
    	shooterMotor.set(pulsesPer100ms);
    	MercLogger.logMessage(Level.INFO, "Shooter: " + shooterMotor.getDeviceID() + ": setSpeed(" + rpm + "): shooterMotor.set(" + pulsesPer100ms + ")");
    }
    
    public double getSpeed() {
    	double pulsesPer100ms = shooterMotor.get();
    	return pulsesPer100ms * HUNDRED_MS_PER_MINUTE / QUAD_ENC_TICKS_PER_ROTATION;
    }
    
    public double getSetSpeed() {
    	return setSpeed;
    }
    
    public void runFeeder(boolean run) {
    	feederMotor.set(run ? -1 : 0);
    } 
}
