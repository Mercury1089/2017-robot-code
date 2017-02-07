package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RunShooter extends PIDCommand {

	private double _heading;
	double highest = 0;
	double lowest = 0;
	
    public RunShooter(double heading) {
    	super(0.3, 0, 0.7); //TODO Test these values
    	requires(Robot.shooter);
    	_heading = heading;
    	getPIDController().setContinuous(true);
    	getPIDController().setAbsoluteTolerance(0.1);
    	getPIDController().setInputRange(0, 14);
    	getPIDController().setOutputRange(-.5, .5);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.motor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	getPIDController().setSetpoint(_heading);
		MercLogger.logMessage(Level.INFO, "The Run Shooter Command has been initialized.");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	double magVal = SmartDashboard.getNumber("Mag Enc Val", 2900);
    	
    	
    	if (magVal < lowest) {
    		lowest = magVal;
    	}
    	if (magVal > highest) {
    		highest = magVal;
    	}
    	
    	boolean highLow = SmartDashboard.getBoolean("Enable High/Low", false);
    	
    	if (!highLow) {
    		resetHighLow();
    	}
    	else {
    		SmartDashboard.putNumber("LOWEST", lowest);
    		SmartDashboard.putNumber("HIGHEST", highest);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return getPIDController().onTarget();

    }

    // Called once after isFinished returns true
    protected void end() {
		MercLogger.logMessage(Level.INFO, "The Run Shooter Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	@Override
	protected double returnPIDInput() {
		// TODO Check if this is the correct input
		return Robot.shooter.motor.getBusVoltage();
	}

	@Override
	protected void usePIDOutput(double output) {
		Robot.shooter.pidWrite(output);
	}
	
	public void resetHighLow() {
		highest = 0;
		lowest = 10000;
	}
}
