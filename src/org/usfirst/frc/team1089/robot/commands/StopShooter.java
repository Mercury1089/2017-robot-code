package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.Debug;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This {@link Command} stops the shooter
 */
public class StopShooter extends Command {
	// TODO Finish this entire class
	public StopShooter(){
		requires(Robot.shooter);
	}
	
	@Override
	protected void initialize() {
		Debug.logMessage(Level.INFO, "The Stop Shooter Command has been initialized.");
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Debug.logMessage(Level.INFO, "The Stop Shooter Command has ended.");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
