package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This {@link Command} stops the shooter.
 */
public class StopShooter extends Command {
	
	private Shooter shooter;
	
	/**
	 * <pre>
	 * public StopShooter(Shooter s)
	 * </pre>
	 * Creates this {@code StopShooter} command to stop the specified
	 * {@Link Shooter}
	 * 
	 * @param s the {@code Shooter} to stop
	 */
	public StopShooter(Shooter s){
		requires(s);
		shooter = s;
	}
	
	@Override
	protected void initialize() {
		MercLogger.logMessage(Level.INFO, "StopShooter: Initialized");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		end();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		shooter.getMotor().disable();
		MercLogger.logMessage(Level.INFO, "StopShooter: Completed");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		MercLogger.logMessage(Level.INFO, "StopShooter: Interrupted");
	}
}
