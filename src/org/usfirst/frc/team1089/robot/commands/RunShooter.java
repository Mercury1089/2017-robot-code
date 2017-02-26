	package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.subsystems.Shooter;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This {@link Command} runs a {@link Shooter} using SmartDashboard input.
 */
public class RunShooter extends Command {

	private Shooter shooter;
	private final double LOWEST_RPM = 3500;		//TODO Change this
	private final double HIGHEST_RPM  = 5000;   //TODO CHange this

	/**
	 * <pre>
	 * public RunShooter(Shooter s)
	 * </pre>
	 * Creates this {@code RunShooter} command with the specific
	 * {@code Shooter} to control
	 * 
	 * @param s the {@code Shooter} that this command will control 
	 */
	public RunShooter(Shooter s) {
    	requires(s);
    	shooter = s;
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
    	shooter.setToSpeed();
    	shooter.resetHighLow();
    	MercLogger.logMessage(Level.INFO, "RunShooter: Initialized");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	double speed = SmartDashboard.getBoolean("Shooter ID " + shooter.getMotor().getDeviceID() + ": shooterIsRunning", false) ? SmartDashboard.getNumber("Shooter ID " + shooter.getMotor().getDeviceID() + ": shooterRPM", 0) : 0.0;
 
    	if (speed == 0.0)
    		shooter.getMotor().disableControl();
    	else 
    		shooter.getMotor().enableControl();

    	if (!SmartDashboard.getBoolean("Shooter ID " + shooter.getMotor().getDeviceID() + ": enableHighLow", false)) {
    		shooter.resetHighLow();
    	}
    
    	shooter.updateHighLow();
    	shooter.getMotor().set(speed);
    	if (!inRange(speed))
    		end();
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	MercLogger.logMessage(Level.INFO, "RunShooter: Completed");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	MercLogger.logMessage(Level.INFO, "RunShooter: Interrupted");
    }
    
   
    public boolean inRange(double speed) {
    	return (speed > LOWEST_RPM && speed < HIGHEST_RPM);
    }

}
