package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This {@link Command} simply enables the ability to drive with the joysticks
 * with arcade drive.
 */
public class DriveWithJoysticks extends Command {

    public DriveWithJoysticks() {
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setToVbus();
    	MercLogger.logMessage(Level.INFO, "DriveWithJoysticks: Enabled");
    	//MercLogger.logMessage(Level.INFO, "The robot has returned to Driving With Joysticks.");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.joystickDrive(Robot.oi.getLeftStick(), Robot.oi.getRightStick());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
		//Debug.logMessage(Level.INFO, "The robot is no longer using Drive With Joysticks.");
    	//MercLogger.logMessage(Level.INFO, "The robot is no longer using Drive With Joysticks.");
    	MercLogger.logMessage(Level.INFO, "DriveWithJoysticks: Disabled");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	MercLogger.logMessage(Level.INFO, "DriveWithJoysticks: Interrupted");
    	end();
    }
}
