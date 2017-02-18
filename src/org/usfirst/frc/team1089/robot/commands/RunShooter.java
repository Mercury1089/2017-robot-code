	package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RunShooter extends Command {

	private double highest, lowest;
	private Shooter shooter;
	private final double LOWEST_RPM = 3500;		//TODO Change this
	private final double HIGHEST_RPM  = 5000;   //TODO CHange this
	
	public RunShooter(Shooter s) {
    	requires(s);
    	shooter = s;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	shooter.motor.changeControlMode(CANTalon.TalonControlMode.Speed);
    	shooter.motor.setPID(0.5, 0.0, 0.5);
		shooter.motor.configPeakOutputVoltage(12, -12);
		shooter.motor.configNominalOutputVoltage(0,0);
    	shooter.motor.enableControl();
		shooter.motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	shooter.motor.setProfile(0);
    	shooter.motor.reverseSensor(false);
		SmartDashboard.putNumber("shooterVolts", 0.0);
    	SmartDashboard.putBoolean("shooterIsRunning", false);
    	SmartDashboard.putBoolean("enableHighLow", false);
    	//Selectable
    	resetHighLow();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	double speed = SmartDashboard.getBoolean("shooterIsRunning", false) ? SmartDashboard.getNumber("shooterVolts", 0) : 0.0;
    	
    	if (SmartDashboard.getBoolean("shooterIsRunning", false)) {
    		System.out.println(speed);
    	}
    	
    	double magVal = SmartDashboard.getNumber("Encoder Value", 2900);
    	
    	
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
    		
    	shooter.motor.set(speed);
    	if (!inRange(speed))
    		end();
    	
    	System.out.println(shooter.motor.get());
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
    
    public boolean inRange(double speed) {
    	return (speed > LOWEST_RPM && speed < HIGHEST_RPM);
    }
    
}
