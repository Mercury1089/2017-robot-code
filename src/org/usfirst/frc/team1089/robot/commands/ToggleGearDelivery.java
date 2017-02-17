package org.usfirst.frc.team1089.robot.commands;

import java.util.logging.Level;

import org.usfirst.frc.team1089.robot.Robot;
import org.usfirst.frc.team1089.robot.util.MercLogger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A command used to either open or close the gear delivery mechanism
 * 
 * @author Luke Letourneau
 * @version 1
 * 
 * {@link}
 */
public class ToggleGearDelivery extends InstantCommand {

	private static final long MOVE_DURATION_MILLISECONDS = 1000;
	private double position;
	private long startTimeMillis;
	/**
	 * @param open Whether or not to open the gear deliveryS
	 */
    public ToggleGearDelivery(boolean open) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.gear);
        if(open)
    		position = Robot.gear.getOpenPosition();
    	else
    		position = Robot.gear.getClosedPosition();
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTimeMillis = Robot.gear.getServoPosition() == position ? System.currentTimeMillis() - MOVE_DURATION_MILLISECONDS : System.currentTimeMillis();
    	Robot.gear.setServoPosition(position);
    	MercLogger.logMessage(Level.INFO, "Initialize complete");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Servo Pos", Robot.gear.getServoPosition());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.gear.getServoPosition() == position && System.currentTimeMillis() > startTimeMillis + MOVE_DURATION_MILLISECONDS;
    	//return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	MercLogger.logMessage(Level.INFO, "End called.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	MercLogger.logMessage(Level.INFO, "Interrupted called.");
    }
}
