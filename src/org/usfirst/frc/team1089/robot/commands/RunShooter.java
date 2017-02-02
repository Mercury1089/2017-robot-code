package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.Debug;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 *
 */
public class RunShooter extends PIDCommand {

	private double _heading;
	
    public RunShooter(double heading) {
    	super(0, 0, 0); //TODO Test these values
    	requires(Robot.driveTrain);
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
    	getPIDController().setSetpoint(_heading);
		Debug.logMessage(Level.INFO, "The Run Shooter Command has been initialized.");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return getPIDController().onTarget();

    }

    // Called once after isFinished returns true
    protected void end() {
		Debug.logMessage(Level.INFO, "The Run Shooter Command has ended.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	@Override
	protected double returnPIDInput() {
		// TODO Put in code from vision that gets our distance from goal
		return 0;
	}

	@Override
	protected void usePIDOutput(double output) {
		Robot.shooter.pidWrite(output);
	}
}
