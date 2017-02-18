package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 *
 */
public class DriveToWall extends PIDCommand {

	private double distanceFromWall;
	
    public DriveToWall(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(0.7, 0, 0.65);
    	Robot.driveTrain.disableRobotDrive();
    	requires(Robot.driveTrain);
    	getPIDController().setContinuous(true);
    	getPIDController().setAbsoluteTolerance(0.1);
    	getPIDController().setInputRange(0, 10);
    	getPIDController().setOutputRange(-.4, .4);
    	distanceFromWall = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.ultrasonic.getUltrasonic().resetAccumulator();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        /*return Robot.ultrasonic.getRange() <= distanceFromWall;*/
    	return getPIDController().onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	MercLogger.logMessage(Level.INFO, "We have driven to the wall");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return Robot.ultrasonic.getRange() - distanceFromWall;
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		Robot.driveTrain.pidWrite(output);
	}
}
