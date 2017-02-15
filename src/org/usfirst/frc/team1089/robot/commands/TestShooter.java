package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TestShooter extends Command {

	//Testing Purposes Only
	double highest, lowest;

	public TestShooter() {
    	
    	requires(Robot.shooter);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.motor.setPID(0.5, 0.0, 0.5);
		Robot.shooter.motor.configPeakOutputVoltage(12, -12);
		Robot.shooter.motor.configNominalOutputVoltage(0,0);
    	Robot.shooter.motor.enableControl();
		Robot.shooter.motor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	Robot.shooter.motor.setProfile(0);
    	Robot.shooter.motor.reverseSensor(false);
		SmartDashboard.putNumber("shooterVolts", 0.0);
    	SmartDashboard.putBoolean("shooterIsRunning", false);
    	SmartDashboard.putBoolean("enableHighLow", false);

    	resetHighLow();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shooter.motor.changeControlMode(CANTalon.TalonControlMode.Speed);
    	
    	double volts = SmartDashboard.getBoolean("shooterIsRunning", false) ? SmartDashboard.getNumber("shooterVolts", 0) : 0.0;
    	
    	if (SmartDashboard.getBoolean("shooterIsRunning", false)) {
    		System.out.println(volts);
    	}
    	
    	double magVal = SmartDashboard.getNumber("Mag Enc Val", 2900);
    	
    	
    	if (magVal < lowest) {
    		lowest = magVal;
    	}
    	if (magVal > highest) {
    		highest = magVal;
    	}
    	
    	if (!SmartDashboard.getBoolean("enableHighLow", false)) {
    		resetHighLow();
    	}

    	SmartDashboard.putNumber("LOWEST", lowest);
		SmartDashboard.putNumber("HIGHEST", highest);
    		
    	Robot.shooter.motor.set(volts);
    	System.out.println(Robot.shooter.motor.get());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    public void resetHighLow() {
		highest = 0;
		lowest = 10000;
	}
}
