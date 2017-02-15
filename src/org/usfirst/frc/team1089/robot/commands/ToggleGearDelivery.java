package org.usfirst.frc.team1089.robot.commands;

import org.usfirst.frc.team1089.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * A command used to either open or close the gear delivery mechanism
 * 
 * @author Luke Letourneau
 * @version 1
 * 
 * {@link}
 */
public class ToggleGearDelivery extends InstantCommand {

	private boolean open;
	
	/**
	 * @param open Whether or not to open the gear deliveryS
	 */
    public ToggleGearDelivery(boolean open) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.gear);
        this.open = open;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(open)
    		Robot.gear.setServo(Robot.gear.getOpenPosition());
    	else
    		Robot.gear.setServo(Robot.gear.getClosedPosition());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
}
